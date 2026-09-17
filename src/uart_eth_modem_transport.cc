// Copyright 2025 Terrence
// SPDX-License-Identifier: Apache-2.0

#include "uart_eth_modem.h"
#include "uart_eth_memory.h"

#include <new>
#include <limits>
#include <utility>

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
        TxFrame* frame = nullptr;
        // Wait for frame in queue (blocks here, not in LWIP context)
        if (xQueueReceive(tx_queue_, &frame, portMAX_DELAY) == pdTRUE) {
            if (!frame) continue;  // Stop wakeup sentinel
            if (stop_flag_.load()) {
                CompleteTxFrame(frame, ESP_ERR_INVALID_STATE);
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
            ret = uart_uhci_.Transmit(frame->data, frame->length, 1000, &stop_flag_);
            if (ret != ESP_OK) {
                ESP_LOGE(kTag, "TX task: UHCI transmit failed: %s", esp_err_to_name(ret));
                goto done;
            }

            // Wait for ACK (SRDY high)
            {
                EventBits_t bits = xEventGroupWaitBits(
                    event_group_,
                    kEventSrdyHigh | kEventStop,
                    pdFALSE,  // Stop remains latched; ACK is cleared before TX
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
                             frame->length, (long)(esp_timer_get_time() - last_activity_time_us_));
                }
            }

done:
            ConfigureSrdyInterrupt(kSrdyInterruptForAck);
            CompleteTxFrame(frame, ret);
        }
    }

    // Join an enqueue that passed admission before cancellation was latched.
    { std::lock_guard<std::mutex> lock(tx_mutex_); }
    // Cancel queued work before publishing done so AT/init waiters wake now.
    TxFrame* pending = nullptr;
    while (xQueueReceive(tx_queue_, &pending, 0) == pdTRUE) {
        if (pending) CompleteTxFrame(pending, ESP_ERR_INVALID_STATE);
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
        uart_uhci_.ReclaimDeferredBuffers();
        TickType_t wait_ticks = CalculateNextTimeout();
        // A queue-full ISR may race the final dequeue. Bound the next wait
        // while RX is running so its deferred return never needs another event.
        if (uart_uhci_.IsReceiving() && wait_ticks > pdMS_TO_TICKS(100)) {
            wait_ticks = pdMS_TO_TICKS(100);
        }

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

    // StopReceive has joined the RX callbacks; drain all leases before Deinit.
    uart_uhci_.StopReceive();
    Event pending;
    while (xQueueReceive(event_queue_, &pending, 0) == pdTRUE) {
        if (pending.type == EventType::RxData) uart_uhci_.ReturnBuffer(pending.rx_buffer);
    }
    uart_uhci_.ReclaimDeferredBuffers();

    // Reuse this existing task as the cleanup coordinator: no extra SRAM stack.
    // Never destroy queues until RequestStop has finished publishing wakeups.
    xEventGroupWaitBits(event_group_, kEventTxTaskDone | kEventInitTaskDone | kEventStopPublished,
                       pdFALSE, pdTRUE, portMAX_DELAY);
    {
        std::lock_guard<std::timed_mutex> activation_lock(data_activation_mutex_);
        CleanupResources(true);
    }
    ESP_LOGD(kTag, "Main task cleanup complete");
    xEventGroupSetBits(event_group_, kEventMainTaskDone | kEventShutdownComplete);
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

    if (xQueueSendFromISR(self->event_queue_, &event, &xHigherPriorityTaskWoken) != pdTRUE) {
        self->uart_uhci_.DeferReturnBuffer(data.buffer);
    }

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
            // RequestStop already latched cancellation before sending wakeup.
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

    // A queued frame may outlive StopReceive. Process and return it before
    // attempting to remount the RX ring.

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

    // Return buffer to pool before restarting a stopped ring.
    uart_uhci_.ReturnBuffer(buffer);
    if (!stop_flag_.load() && working_state_.load() != WorkingState::Active) {
        EnterActiveState();
    }
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
        // A short wakeup may only be for deferred RX recycling.
        if (esp_timer_get_time() - last_activity_time_us_ < kIdleTimeoutMs * 1000) return;
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
            // Older RX events still own buffers. Their final return will
            // retry this transition; do not advertise Active before DMA starts.
            ESP_LOGD(kTag, "UHCI receive waiting for buffers: %s", esp_err_to_name(ret));
            SetMrdy(MrdyLevel::Low);
            working_state_.store(WorkingState::PendingActive);
            return;
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

void UartEthModem::TxPoolDeleter::operator()(UartEthTxPool* pool) const noexcept {
    pool->~UartEthTxPool();
    heap_caps_free(pool);
}

// All TX payloads are CPU-read by Transmit's FIFO writer, never DMA-read.
// The selected heap is strict: allocation failure never switches heaps.
esp_err_t UartEthModem::InitTxPool() {
    // Materialize the reusable pool mutex during initialization, not first send.
    std::lock_guard<std::mutex> lock(tx_mutex_);
    // Queue + active TX + completed synchronous frame awaiting its waiter.
    constexpr size_t max_slots = (SIZE_MAX - sizeof(UartEthTxPool)) / sizeof(TxFrame);
    if (config_.tx_queue_depth == 0 || config_.tx_queue_depth > max_slots - 2 ||
        config_.tx_queue_depth > std::numeric_limits<UBaseType_t>::max()) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t slots = config_.tx_queue_depth + 2;
    void* storage = uart_eth::memory::AllocateBuffer(
        sizeof(UartEthTxPool) + slots * sizeof(TxFrame), config_.use_psram);
    if (!storage) return ESP_ERR_NO_MEM;
    // One allocation contains both the pool metadata and its variable-size array.
    static_assert(sizeof(UartEthTxPool) % alignof(TxFrame) == 0);
    auto* frames = new (static_cast<uint8_t*>(storage) + sizeof(UartEthTxPool)) TxFrame[slots]{};
    TxPoolPtr pool{new (storage) UartEthTxPool{std::span{frames, slots}}};
    tx_done_ = xSemaphoreCreateBinary();
    if (!tx_done_) return ESP_ERR_NO_MEM;
    tx_queue_ = xQueueCreate(config_.tx_queue_depth, sizeof(TxFrame*));
    if (!tx_queue_) {
        vSemaphoreDelete(tx_done_);
        tx_done_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    tx_pool_ = std::move(pool);
    return ESP_OK;
}

void UartEthModem::DeinitTxPool() {
    // Workers are joined and admission is closed before this function runs.
    // Finish queued frames too, waking any synchronous waiter with cancellation.
    TxFrame* frame = nullptr;
    while (tx_queue_ && xQueueReceive(tx_queue_, &frame, 0) == pdTRUE) {
        if (frame) CompleteTxFrame(frame, ESP_ERR_INVALID_STATE);
    }
    // Join the waiter before deleting its shared semaphore or PSRAM slot.
    std::lock_guard<std::timed_mutex> wait_lock(at_mutex_);
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (tx_done_) {
        vSemaphoreDelete(tx_done_);
        tx_done_ = nullptr;
    }
    tx_pool_.reset();
    if (tx_queue_) {
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
    }
}

esp_err_t UartEthModem::QueueTxFrame(const uint8_t* data, size_t length,
                                    FrameType type, bool synchronous,
                                    TxFrame** queued_frame) {
    if (!data || length == 0) return ESP_ERR_INVALID_ARG;
    if (length > sizeof(TxFrame::data) - sizeof(FrameHeader)) return ESP_ERR_INVALID_SIZE;
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (stop_flag_.load() || !tx_pool_ || !tx_queue_) return ESP_ERR_INVALID_STATE;
    TxFrame* frame = tx_pool_->Acquire(sizeof(FrameHeader) + length, synchronous);
    if (!frame) return ESP_ERR_NO_MEM;
    FrameHeader header{};
    header.SetPayloadLength(length);
    header.SetSequence(seq_no_++);
    header.SetFlowControl(false);
    header.SetType(type);
    header.UpdateChecksum();
    memcpy(frame->data, &header, sizeof(header));
    memcpy(frame->data + sizeof(header), data, length);
    if (xQueueSend(tx_queue_, &frame, 0) != pdTRUE) {
        tx_pool_->Complete(*frame, ESP_ERR_NO_MEM);
        tx_pool_->ReleaseWaiter(*frame);
        return ESP_ERR_NO_MEM;
    }
    if (queued_frame) *queued_frame = frame;
    return ESP_OK;
}

void UartEthModem::CompleteTxFrame(TxFrame* frame, esp_err_t result) {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    tx_pool_->Complete(*frame, result);
    if (frame->waiter) {
        // Give under the same lock used to detach a timed-out waiter. No give
        // can arrive after that slot has been released or reused.
        xSemaphoreGive(tx_done_);
    }
}

esp_err_t UartEthModem::EnqueueTxFrame(const uint8_t* buf, size_t len) {
    return QueueTxFrame(buf, len, FrameType::kEthernet, false, nullptr);
}

esp_err_t UartEthModem::SendFrame(const uint8_t* data, size_t length, FrameType type) {
    // SendAt and the handshake caller both hold at_mutex_, so one reusable
    // completion semaphore is sufficient without another waiter mutex.
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        if (stop_flag_.load() || !tx_done_) return ESP_ERR_INVALID_STATE;
        xSemaphoreTake(tx_done_, 0);  // Drain a completion racing the last timeout.
    }
    TxFrame* frame = nullptr;
    const TickType_t start = xTaskGetTickCount();
    esp_err_t ret;
    do {
        ret = QueueTxFrame(data, length, type, true, &frame);
        if (ret != ESP_ERR_NO_MEM) break;
        if (xTaskGetTickCount() - start >= pdMS_TO_TICKS(100)) return ret;
        vTaskDelay(1);
    } while (true);
    if (ret != ESP_OK) return ret;

    xSemaphoreTake(tx_done_, pdMS_TO_TICKS(2000));
    std::lock_guard<std::mutex> lock(tx_mutex_);
    // Completion may win just after the timed wait expires; trust slot state.
    ret = frame->completed ? frame->result : ESP_ERR_TIMEOUT;
    tx_pool_->ReleaseWaiter(*frame);
    // Otherwise the worker still owns the slot and releases it on completion.
    return ret;
}

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
