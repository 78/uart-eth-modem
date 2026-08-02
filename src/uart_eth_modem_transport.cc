// Copyright 2025 Terrence
// SPDX-License-Identifier: Apache-2.0

#include "uart_eth_modem.h"

#include <chrono>
#include <cstdio>
#include <cstring>

#include <esp_check.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <hal/gpio_ll.h>
#include <lwip/dns.h>
#include <lwip/tcpip.h>


// TX task: unified task for all frame transmission
// All frames (AT commands, handshake, Ethernet) go through this single path,
// eliminating resource contention between SendFrame and direct UHCI access.
void UartEthModem::TxTaskRun() {
    ESP_LOGD(kTag, "TX task started");

    while (!stop_flag_.load()) {
        TxFrame frame;
        // Wait for frame in queue (blocks here, not in LWIP context)
        if (xQueueReceive(tx_queue_, &frame, portMAX_DELAY) == pdTRUE) {
            if (stop_flag_.load()) {
                // Cleanup frame if stopping
                if (frame.data) {
                    free(frame.data);
                }
                // Notify waiter if any
                if (frame.done_sem) {
                    if (frame.result) {
                        *frame.result = ESP_ERR_INVALID_STATE;
                    }
                    xSemaphoreGive(frame.done_sem);
                }
                break;
            }

            esp_err_t ret = ESP_OK;

            // Enter active state if needed (via MainTask)
            if (working_state_.load() != WorkingState::Active) {
                Event event = {.type = EventType::TxRequest, .rx_buffer = nullptr};
                xQueueSend(event_queue_, &event, pdMS_TO_TICKS(10));
                
                // Wait for active state
                EventBits_t bits = xEventGroupWaitBits(
                    event_group_,
                    kEventActiveState | kEventStop,
                    pdFALSE,
                    pdFALSE,
                    pdMS_TO_TICKS(200) // Increased from 50ms for slow wakeup
                );

                if (bits & kEventStop) {
                    ret = ESP_ERR_INVALID_STATE;
                    goto done;
                }

                if (!(bits & kEventActiveState)) {
                    ESP_LOGE(kTag, "TX task: timeout waiting for active state");
                    ret = ESP_ERR_TIMEOUT;
                    goto done;
                }
            }

            // Update activity time
            last_activity_time_us_ = esp_timer_get_time();
            
            // Clear event bits before sending
            //ConfigureSrdyInterrupt(kSrdyInterruptForAck);
            gpio_set_intr_type(config_.srdy_pin, GPIO_INTR_NEGEDGE);
            gpio_intr_enable(config_.srdy_pin);
            xEventGroupClearBits(event_group_, kEventSrdyHigh);

            // Transmit via UHCI (Synchronous FIFO mode)
            ret = uart_uhci_.Transmit(frame.data, frame.length);
            if (ret != ESP_OK) {
                ESP_LOGE(kTag, "TX task: UHCI transmit failed: %s", esp_err_to_name(ret));
                goto done;
            }

            // Wait for ACK (SRDY high)
            {
                EventBits_t bits = xEventGroupWaitBits(
                    event_group_,
                    kEventSrdyHigh | kEventStop,
                    pdTRUE,   // clear on exit
                    pdFALSE,  // wait for any bit
                    pdMS_TO_TICKS(kAckTimeoutMs)
                );

                if (bits & kEventStop) {
                    ret = ESP_ERR_INVALID_STATE;
                    goto done;
                }

                if (!(bits & kEventSrdyHigh)) {
                    // ACK timeout - assume data was received
                    ESP_LOGW(kTag, "TX task: ACK timeout in %ld us", (long)(esp_timer_get_time() - last_activity_time_us_));
                } else if (debug_enabled_.load()) {
                    ESP_LOGI(kTag, "TX task: frame sent, %d bytes, acked in %ld us", 
                             frame.length, (long)(esp_timer_get_time() - last_activity_time_us_));
                }
            }

done:
            ConfigureSrdyInterrupt(kSrdyInterruptForAck);
            // Cleanup and notify
            free(frame.data);
            if (frame.done_sem) {
                if (frame.result) {
                    *frame.result = ret;
                }
                xSemaphoreGive(frame.done_sem);
            }
        }
    }

    ESP_LOGD(kTag, "TX task exiting");
    xEventGroupSetBits(event_group_, kEventTxTaskDone);
}

// Main task: handles all events (using UHCI DMA, no blocking I/O)
void UartEthModem::MainTaskRun() {
    ESP_LOGD(kTag, "Main task started (UHCI DMA mode)");

    // Initialize MRDY to high (not busy, allow slave to sleep)
    SetMrdy(MrdyLevel::High);
    working_state_.store(WorkingState::Idle);

    while (!stop_flag_.load()) {
        // Calculate next timeout based on current state
        TickType_t wait_ticks = CalculateNextTimeout();

        Event event;
        if (xQueueReceive(event_queue_, &event, wait_ticks) == pdTRUE) {
            HandleEvent(event);
        } else {
            // Timeout occurred
            HandleIdleTimeout();
        }
    }

    // Ensure we're in idle state before exiting
    if (working_state_.load() != WorkingState::Idle) {
        EnterIdleState();
    }

    ESP_LOGD(kTag, "Main task exiting");
    xEventGroupSetBits(event_group_, kEventMainTaskDone);
}

// UHCI RX callback static wrapper (called from ISR context)
bool IRAM_ATTR UartEthModem::UhciRxCallbackStatic(const UartUhci::RxEventData& data, void* user_data) {
    auto* self = static_cast<UartEthModem*>(user_data);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send RxData event to main task with buffer pointer
    Event event = {
        .type = EventType::RxData,
        .rx_buffer = data.buffer,
    };

    xQueueSendFromISR(self->event_queue_, &event, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken == pdTRUE;
}

// Start continuous DMA receive using buffer pool
void UartEthModem::StartDmaReceive() {
    // Only start DMA receive if in Active or PendingIdle state
    // In Idle state, don't start receive to release UHCI's NO_LIGHT_SLEEP lock
    WorkingState state = working_state_.load();
    if (state == WorkingState::Idle) {
        ESP_LOGD(kTag, "Skipping DMA receive in idle state");
        return;
    }

    // Start continuous receive with buffer pool
    esp_err_t ret = uart_uhci_.StartReceive();
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to start DMA receive: %s", esp_err_to_name(ret));
    }
}

// Calculate next timeout based on current working state
TickType_t UartEthModem::CalculateNextTimeout() {
    WorkingState state = working_state_.load();

    if (state == WorkingState::Idle) {
        // In idle state, wait indefinitely for events
        return portMAX_DELAY;
    }

    if (state == WorkingState::PendingActive) {
        // In pending active state, wait for SRDY low with timeout
        return pdMS_TO_TICKS(100);  // Increased from 50ms for slow slave wakeup
    }

    if (state == WorkingState::Active) {
        // In active state, calculate remaining time until idle timeout
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_us = now_us - last_activity_time_us_;
        int64_t elapsed_ms = elapsed_us / 1000;
        int64_t remaining_ms = kIdleTimeoutMs - elapsed_ms;

        if (remaining_ms <= 0) {
            return 0;  // Already timed out
        }
        return pdMS_TO_TICKS(remaining_ms);
    }

    if (state == WorkingState::PendingIdle) {
        // In pending idle state, check SRDY periodically
        return pdMS_TO_TICKS(10);  // Check every 10ms
    }

    return portMAX_DELAY;
}

// Handle incoming event
void UartEthModem::HandleEvent(const Event& event) {
    // Wake the TX task from normal task context. Event-group operations from
    // an ISR are deferred through the FreeRTOS timer queue; one of those
    // deferred commands can otherwise outlive Stop() and target a deleted
    // event group during a fast modem hand-off.
    if (event.type == EventType::SrdyLow || event.type == EventType::SrdyHigh) {
        xEventGroupSetBits(event_group_, kEventSrdyHigh);
    }

    switch (event.type) {
        case EventType::TxRequest: {
            // Master wants to send data
            WorkingState state = working_state_.load();
            if (state == WorkingState::Idle || state == WorkingState::PendingIdle) {
                // Need to wake up slave first (or ensure slave stays awake)
                // Always go through PendingActive to avoid race condition:
                // In PendingIdle, MRDY is high. Even if SRDY is low now,
                // slave could see MRDY high and start sleeping before we
                // call EnterActiveState(). So always set MRDY low first,
                // then wait for SRDY low confirmation.
                EnterPendingActiveState();
            }
            // If already Active or PendingActive, nothing to do
            break;
        }

        case EventType::SrdyLow:
            HandleSrdyLow();
            break;

        case EventType::SrdyHigh:
            HandleSrdyHigh();
            break;

        case EventType::RxData:
            HandleRxData(event.rx_buffer);
            break;

        case EventType::Stop:
            stop_flag_.store(true);
            break;

        default:
            break;
    }
}

// Handle data received via DMA buffer pool
void UartEthModem::HandleRxData(UartUhci::RxBuffer* buffer) {
    if (!buffer || buffer->size == 0) {
        if (buffer) {
            uart_uhci_.ReturnBuffer(buffer);
        }
        return;
    }

    // Ensure we're in active state
    if (working_state_.load() != WorkingState::Active) {
        EnterActiveState();
    }

    // Update activity time
    last_activity_time_us_ = esp_timer_get_time();

    uint8_t* data = buffer->data;
    size_t size = buffer->size;
    size_t offset = 0;

    // Process data, potentially multiple frames or partial frames
    while (offset < size) {
        if (reassembly_size_ == 0) {
            // Not currently reassembling, look for new frame header
            size_t remaining = size - offset;
            
            if (remaining < sizeof(FrameHeader)) {
                // Not enough data for header, copy to reassembly buffer
                memcpy(reassembly_buffer_, data + offset, remaining);
                reassembly_size_ = remaining;
                reassembly_expected_ = 0;  // Don't know expected size yet
                break;
            }

            // Parse header
            FrameHeader* header = reinterpret_cast<FrameHeader*>(data + offset);
            if (!header->ValidateChecksum()) {
                // Invalid header - likely end of valid data or corruption
                // Don't scan further, just discard remaining data
                if (debug_enabled_.load()) {
                    ESP_LOGI(kTag, "Invalid checksum at offset %d, discarding %d remaining bytes",
                             offset, size - offset);
                }
                break;
            }

            uint16_t payload_len = header->GetPayloadLength();
            size_t frame_size = sizeof(FrameHeader) + payload_len;

            if (frame_size > kMaxFrameSize) {
                ESP_LOGW(kTag, "Frame too large: %d bytes", frame_size);
                offset++;
                continue;
            }

            if (remaining >= frame_size) {
                // Complete frame available, process directly
                ProcessReceivedFrame(data + offset, frame_size);
                offset += frame_size;
                
                // Send ACK after processing complete frame
                SendAckPulse();
            } else {
                // Partial frame, copy to reassembly buffer
                memcpy(reassembly_buffer_, data + offset, remaining);
                reassembly_size_ = remaining;
                reassembly_expected_ = frame_size;
                break;
            }
        } else {
            // Currently reassembling a frame
            size_t remaining = size - offset;
            
            if (reassembly_expected_ == 0) {
                // Still need to determine frame size (was missing header bytes)
                size_t need_for_header = sizeof(FrameHeader) - reassembly_size_;
                size_t copy_size = std::min(remaining, need_for_header);
                memcpy(reassembly_buffer_ + reassembly_size_, data + offset, copy_size);
                reassembly_size_ += copy_size;
                offset += copy_size;

                if (reassembly_size_ >= sizeof(FrameHeader)) {
                    // Now we have the header
                    FrameHeader* header = reinterpret_cast<FrameHeader*>(reassembly_buffer_);
                    if (!header->ValidateChecksum()) {
                        if (debug_enabled_.load()) {
                            ESP_LOGI(kTag, "Invalid reassembled header checksum, discarding");
                        }
                        reassembly_size_ = 0;
                        reassembly_expected_ = 0;
                        break;  // Stop processing this buffer
                    }
                    uint16_t payload_len = header->GetPayloadLength();
                    reassembly_expected_ = sizeof(FrameHeader) + payload_len;

                    if (reassembly_expected_ > kMaxFrameSize) {
                        ESP_LOGW(kTag, "Reassembled frame too large: %d bytes", reassembly_expected_);
                        reassembly_size_ = 0;
                        reassembly_expected_ = 0;
                        continue;
                    }
                }
                continue;
            }

            // We know the expected size, continue collecting data
            size_t need = reassembly_expected_ - reassembly_size_;
            size_t copy_size = std::min(remaining, need);
            
            if (reassembly_size_ + copy_size > kMaxFrameSize) {
                ESP_LOGW(kTag, "Reassembly buffer overflow");
                reassembly_size_ = 0;
                reassembly_expected_ = 0;
                break;
            }

            memcpy(reassembly_buffer_ + reassembly_size_, data + offset, copy_size);
            reassembly_size_ += copy_size;
            offset += copy_size;

            if (reassembly_size_ >= reassembly_expected_) {
                // Frame complete, process it
                ProcessReceivedFrame(reassembly_buffer_, reassembly_expected_);
                reassembly_size_ = 0;
                reassembly_expected_ = 0;
                
                // Send ACK after processing complete frame
                SendAckPulse();
            }
        }
    }

    // Return buffer to pool
    uart_uhci_.ReturnBuffer(buffer);
}

// Handle SRDY low event: Slave wants to send data or is ready to receive
void UartEthModem::HandleSrdyLow() {
    WorkingState state = working_state_.load();

    if (state == WorkingState::PendingActive) {
        // Slave acknowledged our wakeup, now enter active state
        if (debug_enabled_.load()) {
            ESP_LOGI(kTag, "Slave ready, entering active state");
        }
        EnterActiveState();
    } else if (state == WorkingState::Idle || state == WorkingState::PendingIdle) {
        // Slave is waking us up (slave-initiated), enter active state directly
        if (debug_enabled_.load()) {
            ESP_LOGI(kTag, "Slave wakeup detected, entering active state");
        }
        EnterActiveState();
    }
}

// Handle SRDY high event: Slave ACK or entering sleep
void UartEthModem::HandleSrdyHigh() {
    WorkingState state = working_state_.load();

    if (state == WorkingState::PendingIdle) {
        // Both sides are now idle, enter idle state
        if (debug_enabled_.load()) {
            ESP_LOGI(kTag, "Slave also idle, entering idle state");
        }
        EnterIdleState();  // This will configure interrupt for wakeup
    }
}

// Handle idle timeout
void UartEthModem::HandleIdleTimeout() {
    WorkingState state = working_state_.load();

    if (state == WorkingState::PendingActive) {
        // Timeout waiting for slave to wake up
        // Check if SRDY is already low (we might have missed the interrupt)
        if (IsSrdyLow()) {
            if (debug_enabled_.load()) {
                ESP_LOGI(kTag, "Slave ready (polled), entering active state");
            }
            EnterActiveState();
        } else {
            // Slave not responding, signal timeout but enter active state anyway
            // (the TX will likely fail, but that's handled at higher level)
            ESP_LOGW(kTag, "Slave not responding (SRDY still high), forcing active state");
            EnterActiveState();
        }
    } else if (state == WorkingState::Active) {
        // Timeout in active state, enter pending idle
        if (debug_enabled_.load()) {
            ESP_LOGI(kTag, "Idle timeout, entering pending idle state");
        }
        EnterPendingIdleState();
    } else if (state == WorkingState::PendingIdle) {
        // Check if SRDY is high
        if (!IsSrdyLow()) {
            if (debug_enabled_.load()) {
                ESP_LOGI(kTag, "Slave is idle, entering idle state");
            }
            EnterIdleState();
        }
        // If SRDY is still low, slave has data to send
        // Stay in pending idle and wait for SRDY high event
    }
}

// Enter pending active state: Master initiates wakeup, wait for slave
void UartEthModem::EnterPendingActiveState() {
    WorkingState prev_state = working_state_.load();
    if (prev_state != WorkingState::Idle && prev_state != WorkingState::PendingIdle) {
        return;  // Only valid from Idle or PendingIdle state
    }

    // Set MRDY low first to prevent slave from sleeping
    SetMrdy(MrdyLevel::Low);

    // Check if SRDY is already low (slave still active or already responded)
    // This avoids waiting for an interrupt that will never come
    if (IsSrdyLow()) {
        if (debug_enabled_.load()) {
            ESP_LOGI(kTag, "Slave already ready, entering active state directly");
        }
        // Slave is already ready, go directly to active state
        EnterActiveState();
        return;
    }

    if (debug_enabled_.load()) {
        ESP_LOGI(kTag, "Entering pending active state (waking up slave)");
    }

    // Update state
    working_state_.store(WorkingState::PendingActive);

    // Configure SRDY interrupt to detect slave wakeup (falling edge -> low)
    ConfigureSrdyInterrupt(kSrdyInterruptForAck);
}

// Enter active working state
void UartEthModem::EnterActiveState() {
    WorkingState prev_state = working_state_.load();
    if (prev_state == WorkingState::Active) {
        // Already active, just signal in case someone is waiting
        xEventGroupSetBits(event_group_, kEventActiveState);
        return;
    }

    ESP_LOGD(kTag, "Entering active state");

    // Start DMA receive if not already running
    // (DMA keeps running during PendingIdle, so check before starting)
    if (!uart_uhci_.IsReceiving()) {
        esp_err_t ret = uart_uhci_.StartReceive();
        if (ret != ESP_OK) {
            ESP_LOGE(kTag, "Failed to start UHCI receive: %s", esp_err_to_name(ret));
        }
    }

    // Set MRDY low (busy) - may already be low from PendingActive
    SetMrdy(MrdyLevel::Low);

    // Update state
    working_state_.store(WorkingState::Active);
    last_activity_time_us_ = esp_timer_get_time();

    // Signal that active state is ready (DMA receive started)
    xEventGroupSetBits(event_group_, kEventActiveState);
}

// Enter pending idle state
void UartEthModem::EnterPendingIdleState() {
    if (working_state_.load() != WorkingState::Active) {
        return;  // Not in active state
    }

    ESP_LOGD(kTag, "Entering pending idle state");

    // Set MRDY high (not busy, allow slave to sleep)
    SetMrdy(MrdyLevel::High);

    // Update state
    working_state_.store(WorkingState::PendingIdle);
}

// Enter idle/sleep state
void UartEthModem::EnterIdleState() {
    ESP_LOGD(kTag, "Entering idle state");

    // Clear active state bit (DMA will be stopped)
    xEventGroupClearBits(event_group_, kEventActiveState);

    // Ensure MRDY is high
    SetMrdy(MrdyLevel::High);

    // Update state
    working_state_.store(WorkingState::Idle);

    // Stop DMA receive to release UHCI's NO_LIGHT_SLEEP lock
    uart_uhci_.StopReceive();

    // Configure SRDY interrupt for wakeup (low level trigger)
    ConfigureSrdyInterrupt(kSrdyInterruptForWakeup);
}

// Enqueue TX frame for non-blocking transmission
esp_err_t UartEthModem::EnqueueTxFrame(const uint8_t* buf, size_t len) {
    // Allocate frame with header (DMA compatible memory)
    size_t total_len = sizeof(FrameHeader) + len;
    uint8_t* buffer = static_cast<uint8_t*>(heap_caps_malloc(total_len, MALLOC_CAP_DMA));
    if (!buffer) {
        ESP_LOGE(kTag, "Failed to allocate TX buffer for queue");
        return ESP_ERR_NO_MEM;
    }

    // Build header
    FrameHeader* header = reinterpret_cast<FrameHeader*>(buffer);
    *reinterpret_cast<uint32_t*>(header->raw) = 0;
    header->SetPayloadLength(len);
    header->SetSequence(seq_no_++);
    header->SetFlowControl(false);  // XON = 0 (permit to send)
    header->SetType(FrameType::kEthernet);
    header->UpdateChecksum();

    // Copy payload
    memcpy(buffer + sizeof(FrameHeader), buf, len);

    // Enqueue frame (non-blocking, no completion notification)
    TxFrame frame = {
        .data = buffer, 
        .length = total_len,
        .done_sem = nullptr,
        .result = nullptr
    };
    if (xQueueSend(tx_queue_, &frame, 0) != pdTRUE) {
        ESP_LOGW(kTag, "TX queue full, dropping frame");
        free(buffer);
        return ESP_ERR_NO_MEM;  // Queue full
    }

    return ESP_OK;
}

// Send frame (public interface): enqueue and wait for completion
// All transmission goes through TxTaskRun to avoid resource contention.
esp_err_t UartEthModem::SendFrame(const uint8_t* data, size_t length, FrameType type) {
    // Allocate frame with header (DMA compatible memory)
    size_t total_len = sizeof(FrameHeader) + length;
    uint8_t* buffer = static_cast<uint8_t*>(heap_caps_malloc(total_len, MALLOC_CAP_DMA));
    if (!buffer) {
        ESP_LOGE(kTag, "Failed to allocate TX buffer");
        return ESP_ERR_NO_MEM;
    }

    // Build header
    FrameHeader* header = reinterpret_cast<FrameHeader*>(buffer);
    *reinterpret_cast<uint32_t*>(header->raw) = 0;
    header->SetPayloadLength(length);
    header->SetSequence(seq_no_++);
    header->SetFlowControl(false);  // XON = 0 (permit to send), XOFF = 1 (shall not send)
    header->SetType(type);
    header->UpdateChecksum();

    // Copy payload
    memcpy(buffer + sizeof(FrameHeader), data, length);

    // Create binary semaphore for synchronous wait
    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (!done_sem) {
        ESP_LOGE(kTag, "Failed to create semaphore");
        free(buffer);
        return ESP_ERR_NO_MEM;
    }

    // Prepare frame with completion notification
    esp_err_t result = ESP_OK;
    TxFrame frame = {
        .data = buffer,
        .length = total_len,
        .done_sem = done_sem,
        .result = &result
    };

    // Enqueue frame (block for a short time if queue is full)
    if (xQueueSend(tx_queue_, &frame, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(kTag, "TX queue full, cannot send frame");
        vSemaphoreDelete(done_sem);
        free(buffer);
        return ESP_ERR_NO_MEM;
    }

    // Wait for transmission to complete (with timeout)
    // We wait long enough for TxTaskRun to finish its own internal timeouts (up to 1s for TX, 200ms for active state)
    if (xSemaphoreTake(done_sem, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGE(kTag, "SendFrame timeout waiting for completion");
        // WARNING: If we delete the semaphore here, TxTaskRun might still try to use it later,
        // causing a crash. However, after 2 seconds, it's very likely TxTaskRun has already
        // finished or timed out itself.
        vSemaphoreDelete(done_sem);
        // Note: buffer is freed by TxTaskRun, don't free here
        return ESP_ERR_TIMEOUT;
    }

    vSemaphoreDelete(done_sem);
    return result;
}

// Process frame data received via DMA
void UartEthModem::ProcessReceivedFrame(uint8_t* data, size_t size) {
    if (size < sizeof(FrameHeader)) {
        if (debug_enabled_.load() && size > 0) {
            ESP_LOGI(kTag, "Not enough data for header, size: %d", size);
        }
        return;
    }

    // Parse header from DMA buffer
    FrameHeader* header = reinterpret_cast<FrameHeader*>(data);

    // Validate checksum
    if (!header->ValidateChecksum()) {
        ESP_LOGW(kTag, "Invalid checksum, raw: %02x %02x %02x %02x", 
                 header->raw[0], header->raw[1], header->raw[2], header->raw[3]);
        return;
    }

    uint16_t payload_len = header->GetPayloadLength();
    if (sizeof(FrameHeader) + payload_len > size) {
        ESP_LOGW(kTag, "Incomplete frame: expected %d, got %d", 
                 sizeof(FrameHeader) + payload_len, size);
        return;
    }

    if (debug_enabled_.load()) {
        ESP_LOGI(kTag, "RX frame: type=%d, len=%d, seq=%d", 
                 static_cast<int>(header->GetType()), payload_len, header->GetSequence());
    }

    // Get payload pointer (data is in DMA buffer, need to copy for ownership)
    uint8_t* payload = data + sizeof(FrameHeader);

    // Handle frame based on type
    if (header->GetType() == FrameType::kEthernet) {
        // For Ethernet frames, we need to copy data since HandleEthFrame takes ownership
        uint8_t* payload_copy = static_cast<uint8_t*>(malloc(payload_len));
        if (payload_copy) {
            memcpy(payload_copy, payload, payload_len);
            HandleEthFrame(payload_copy, payload_len);
        } else {
            ESP_LOGE(kTag, "Failed to allocate RX buffer");
        }
    } else if (header->GetType() == FrameType::kAtCommand) {
        // For AT responses, we can use the data in place (it will be copied in HandleAtResponse)
        HandleAtResponse(reinterpret_cast<char*>(payload), payload_len);
    }
}

void UartEthModem::HandleEthFrame(uint8_t* data, size_t length) {
    if (!handshake_done_.load()) {
        // Check for handshake ACK
        if (length >= sizeof(kHandshakeAck) && memcmp(data, kHandshakeAck, sizeof(kHandshakeAck)) == 0) {
            ESP_LOGD(kTag, "Handshake ACK received");
            handshake_done_ = true;
            xEventGroupSetBits(event_group_, kEventHandshakeDone);
            // Mark as initialized, but wait for IP_EVENT_ETH_GOT_IP for network ready
            initialized_ = true;
        }
        free(data);
        return;
    }

    // Forward to iot_eth stack
    if (mediator_) {
        // Note: mediator->stack_input takes ownership of data (frees it)
        mediator_->stack_input(mediator_, data, length);
    } else {
        free(data);
    }
}

