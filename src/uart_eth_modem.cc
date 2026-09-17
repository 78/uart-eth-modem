// Copyright 2025 Terrence
// SPDX-License-Identifier: Apache-2.0

#include "uart_eth_modem.h"
#include "uart_eth_memory.h"

#include <algorithm>
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


// Static member definitions
constexpr uint8_t UartEthModem::kHandshakeRequest[];
constexpr uint8_t UartEthModem::kHandshakeAck[];

UartEthModem::UartEthModem(const Config& config) : config_(config) {
    // Generate MAC address
    esp_read_mac(mac_addr_, ESP_MAC_ETH);
    mac_addr_[5] ^= 0x01;  // Make unique

    // Create event group
    event_group_ = xEventGroupCreate();
    if (!event_group_) {
        ESP_LOGE(kTag, "Failed to create event group");
        abort();  // Constructor cannot fail gracefully
    }

    xEventGroupSetBits(event_group_, kEventShutdownComplete);

    // Initialize driver structure
    driver_.name = "uart_eth";
    driver_.init = [](iot_eth_driver_t* driver) -> esp_err_t {
        // Already initialized
        return ESP_OK;
    };
    driver_.deinit = [](iot_eth_driver_t* driver) -> esp_err_t {
        return ESP_OK;
    };
    driver_.start = [](iot_eth_driver_t* driver) -> esp_err_t {
        return ESP_OK;
    };
    driver_.stop = [](iot_eth_driver_t* driver) -> esp_err_t {
        return ESP_OK;
    };
    driver_.transmit = [](iot_eth_driver_t* driver, uint8_t* buf, size_t len) -> esp_err_t {
        // Get UartEthModem instance using container_of pattern
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
        auto* self = reinterpret_cast<UartEthModem*>(reinterpret_cast<char*>(driver) - offsetof(UartEthModem, driver_));
#pragma GCC diagnostic pop
        if (!self->handshake_done_.load()) {
            return ESP_ERR_INVALID_STATE;
        }
        
        // Non-blocking: enqueue frame for TX task to send
        return self->EnqueueTxFrame(buf, len);
    };
    driver_.get_addr = [](iot_eth_driver_t* driver, uint8_t* mac) -> esp_err_t {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
        auto* self = reinterpret_cast<UartEthModem*>(reinterpret_cast<char*>(driver) - offsetof(UartEthModem, driver_));
#pragma GCC diagnostic pop
        memcpy(mac, self->mac_addr_, 6);
        return ESP_OK;
    };
    driver_.set_mediator = [](iot_eth_driver_t* driver, iot_eth_mediator_t* mediator) -> esp_err_t {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
        auto* self = reinterpret_cast<UartEthModem*>(reinterpret_cast<char*>(driver) - offsetof(UartEthModem, driver_));
#pragma GCC diagnostic pop
        self->mediator_ = mediator;
        return ESP_OK;
    };
}

UartEthModem::~UartEthModem() {
    // Owners must successfully Stop before releasing the object. A timeout
    // cannot be followed by freeing storage still referenced by worker tasks.
    ESP_ERROR_CHECK(Stop());

    // Cleanup resources created in constructor
    if (event_group_) {
        vEventGroupDelete(event_group_);
        event_group_ = nullptr;
    }
}

esp_err_t UartEthModem::Start(StartMode mode) {
    const char* mode_name = "normal";
    if (mode == StartMode::kFlight) {
        mode_name = "flight";
    } else if (mode == StartMode::kRfTest) {
        mode_name = "RF test";
    }
    ESP_LOGI(kTag, "Starting UartEthModem (%s mode)...", mode_name);

    if (!IsStopped()) {
        ESP_LOGW(kTag, "Already started");
        return ESP_ERR_INVALID_STATE;
    }

    main_task_ = nullptr;
    init_task_ = nullptr;
    tx_task_ = nullptr;
    start_mode_ = mode;
    stop_flag_ = false;
    handshake_done_ = false;
    initializing_ = true;
    data_link_up_ = false;
    ip_ready_ = false;
    data_activation_blocked_ = false;
    // Stop() leaves wake-up bits set so every task can observe shutdown. A
    // UartEthModem instance is intentionally restartable (for example after
    // the module emits ECRDY), so clear all per-run bits before creating the
    // replacement tasks. Keeping kEventStop set would make TX and the control
    // task terminate immediately after an otherwise successful restart.
    xEventGroupClearBits(
        event_group_,
        kEventStart | kEventHandshakeDone | kEventStop | kEventNetworkReady |
            kEventAtResponse | kEventInitDone | kEventNetworkEventChanged |
            kEventSrdyHigh | kEventMainTaskDone | kEventInitTaskDone |
            kEventActiveState | kEventTxTaskDone | kEventRegistrationReady |
            kEventDataActivationBlocked | kEventShutdownComplete | kEventStopPublished | kEventAtReady);

    // Create event queue FIRST (before GPIO init, since ISR uses it)
    event_queue_ = xQueueCreate(32, sizeof(Event));
    if (!event_queue_) {
        ESP_LOGE(kTag, "Failed to create event queue");
        initializing_ = false;
        xEventGroupSetBits(event_group_, kEventShutdownComplete);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = InitTxPool();
    if (ret != ESP_OK) {
        stop_flag_ = true;
        DeinitTxPool();
        vQueueDelete(event_queue_);
        event_queue_ = nullptr;
        initializing_ = false;
        xEventGroupSetBits(event_group_, kEventShutdownComplete);
        return ret;
    }

    // Initialize UART
    ret = InitUart();
    if (ret != ESP_OK) {
        stop_flag_ = true;
        DeinitTxPool();
        vQueueDelete(event_queue_);
        event_queue_ = nullptr;
        initializing_ = false;
        xEventGroupSetBits(event_group_, kEventShutdownComplete);
        return ret;
    }

    // Initialize GPIO
    ret = InitGpio();
    if (ret != ESP_OK) {
        DeinitGpio();
        DeinitUart();
        stop_flag_ = true;
        DeinitTxPool();
        vQueueDelete(event_queue_);
        event_queue_ = nullptr;
        initializing_ = false;
        xEventGroupSetBits(event_group_, kEventShutdownComplete);
        return ret;
    }

    // Allocate frame reassembly buffer
    reassembly_buffer_ = static_cast<uint8_t*>(uart_eth::memory::AllocateBuffer(kMaxFrameSize, config_.use_psram));
    if (!reassembly_buffer_) {
        ESP_LOGE(kTag, "Failed to allocate reassembly buffer");
        DeinitGpio();
        stop_flag_ = true;
        DeinitTxPool();
        vQueueDelete(event_queue_);
        event_queue_ = nullptr;
        DeinitUart();
        initializing_ = false;
        xEventGroupSetBits(event_group_, kEventShutdownComplete);
        return ESP_ERR_NO_MEM;
    }
    reassembly_size_ = 0;
    reassembly_expected_ = 0;

    // Initialize UART UHCI DMA controller with buffer pool
    UartUhci::Config uhci_cfg = {
        .uart_port = config_.uart_num,
        .dma_burst_size = 32,
        .rx_pool = {
            .buffer_count = config_.rx_buffer_count,
            .buffer_size = config_.rx_buffer_size,
        },
    };

    ret = uart_uhci_.Init(uhci_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to init UHCI: %s", esp_err_to_name(ret));
        DeinitGpio();
        free(reassembly_buffer_);
        reassembly_buffer_ = nullptr;
        stop_flag_ = true;
        DeinitTxPool();
        vQueueDelete(event_queue_);
        event_queue_ = nullptr;
        DeinitUart();
        initializing_ = false;
        xEventGroupSetBits(event_group_, kEventShutdownComplete);
        return ret;
    }

    // Register UHCI callbacks
    uart_uhci_.SetRxCallback(UhciRxCallbackStatic, this);

    // Note: Don't start DMA receive here, will be started when entering Active state

    // Create main task (handles all events, no blocking operations)
    xTaskCreate([](void* arg) {
        static_cast<UartEthModem*>(arg)->MainTaskRun();
        vTaskDelete(nullptr);
    }, "uart_eth_main", 4096, this, 10, &main_task_);
    if (!main_task_) {
        stop_flag_ = true;
        CleanupResources(false);
        initializing_ = false;
        xEventGroupSetBits(event_group_, kEventShutdownComplete);
        return ESP_ERR_NO_MEM;
    }
    
    // Create init task
    xTaskCreate([](void* arg) {
        static_cast<UartEthModem*>(arg)->InitTaskRun();
        vTaskDelete(nullptr);
    }, "uart_eth_init", 4096, this, 4, &init_task_);

    // Create TX task (dedicated for non-blocking transmit from LWIP)
    xTaskCreate([](void* arg) {
        static_cast<UartEthModem*>(arg)->TxTaskRun();
        vTaskDelete(nullptr);
    }, "uart_eth_tx", 3072, this, 9, &tx_task_);  // Priority slightly lower than main

    if (!main_task_ || !init_task_ || !tx_task_) {
        ESP_LOGE(kTag, "Failed to create tasks");
        // Missing tasks cannot publish done bits. Wake the init task's start
        // wait and join every task that did start before freeing pool/queues.
        EventBits_t missing = 0;
        if (!main_task_) missing |= kEventMainTaskDone;
        if (!init_task_) missing |= kEventInitTaskDone;
        if (!tx_task_) missing |= kEventTxTaskDone;
        xEventGroupSetBits(event_group_, missing);
        const esp_err_t stopped = Stop();
        return stopped == ESP_OK ? ESP_ERR_NO_MEM : stopped;
    }

    // Signal start (initialization continues asynchronously in InitTaskRun)
    // Failures arrive asynchronously. ErrorNoSim retains the AT channel for
    // recovery; call Stop only when the owner wants to release that channel.
    xEventGroupSetBits(event_group_, kEventStart);

    ESP_LOGI(kTag, "UartEthModem starting asynchronously...");
    return ESP_OK;
}

void UartEthModem::RequestStop() {
    // Exactly one caller publishes wakeups. The cleanup task waits for the
    // final published bit before deleting any queue touched here.
    if (stop_flag_.exchange(true)) return;
    initialized_ = false;
    initializing_ = false;
    data_activation_blocked_ = true;
    xEventGroupClearBits(event_group_, kEventAtReady);
    xEventGroupSetBits(event_group_, kEventStop | kEventStart | kEventDataActivationBlocked);
    if (event_queue_) {
        Event event = {.type = EventType::Stop, .rx_buffer = nullptr};
        xQueueSend(event_queue_, &event, 0);
    }
    if (tx_queue_) {
        TxFrame* wakeup = nullptr;
        xQueueSend(tx_queue_, &wakeup, 0);
    }
    xEventGroupSetBits(event_group_, kEventStopPublished);
}

esp_err_t UartEthModem::Stop(uint32_t timeout_ms) {
    if (IsStopped()) return ESP_OK;
    const TaskHandle_t caller = xTaskGetCurrentTaskHandle();
    if (caller == main_task_ || caller == init_task_ || caller == tx_task_) {
        return ESP_ERR_INVALID_STATE;  // A callback cannot join its own task.
    }
    const int64_t deadline = esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000;
    RequestStop();
    const int64_t remaining_us = deadline - esp_timer_get_time();
    const TickType_t ticks = remaining_us > 0
        ? static_cast<TickType_t>(std::min<uint64_t>(
              (static_cast<uint64_t>(remaining_us) * configTICK_RATE_HZ) / 1000000,
              portMAX_DELAY - 1)) : 0;
    const EventBits_t bits = xEventGroupWaitBits(event_group_, kEventShutdownComplete,
                                                pdFALSE, pdTRUE, ticks);
    // Cleanup (including netif teardown and mutex acquisition) runs in the
    // existing main worker, so none of those operations can overrun this wait.
    return (bits & kEventShutdownComplete) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t UartEthModem::PrepareForShutdown(uint32_t timeout_ms) {
    data_activation_blocked_ = true;
    xEventGroupClearBits(event_group_, kEventRegistrationReady);
    xEventGroupSetBits(event_group_, kEventDataActivationBlocked);
    std::unique_lock<std::timed_mutex> lock(data_activation_mutex_, std::defer_lock);
    return lock.try_lock_for(std::chrono::milliseconds(timeout_ms)) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t UartEthModem::ExitRfTestMode() {
    std::string resp;

    ESP_LOGI(kTag, "Exiting RF test mode, restoring normal SIM mode...");
    esp_err_t ret = SendAtWithRetry("AT+ECSIMCFG=\"SimSimulator\",0", resp, 3000, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to restore normal SIM mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(kTag, "Rebooting modem after restoring normal SIM mode...");
    SendAt("AT+ECRST", resp, 500);
    vTaskDelay(pdMS_TO_TICKS(1500));

    ret = SendAtWithRetry("AT", resp, 500, 20);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Modem not responding after RF test exit reset");
        return ret;
    }

    ESP_LOGI(kTag, "Returning modem to CFUN=0 after RF test exit...");
    ret = SendAt("AT+CFUN=0", resp, 5000);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to enter CFUN=0 after RF test exit: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t UartEthModem::SendAt(const std::string& cmd, std::string& response, uint32_t timeout_ms) {
    std::unique_lock<std::timed_mutex> lock(at_mutex_, std::defer_lock);
    const auto lock_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!lock.try_lock_until(std::min(lock_deadline,
                std::chrono::steady_clock::now() + std::chrono::milliseconds(50)))) {
        if (std::chrono::steady_clock::now() >= lock_deadline) return ESP_ERR_TIMEOUT;
        if (stop_flag_.load()) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    // Check again after acquiring mutex in case Stop() was called while waiting
    if (stop_flag_.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    // AT detection runs during initialization; a missing SIM keeps the detected
    // control channel available without claiming network initialization succeeded.
    if (!IsAtReady() && !initializing_.load()) {
        ESP_LOGE(kTag, "Failed to send AT command: not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    at_command_response_.clear();
    waiting_for_at_response_ = true;
    xEventGroupClearBits(event_group_, kEventAtResponse);  // Clear any pending

    // Add \r if not present
    std::string cmd_with_cr = cmd;
    if (cmd_with_cr.empty() || cmd_with_cr.back() != '\r') {
        cmd_with_cr += '\r';
    }

    if (debug_enabled_.load()) {
        ESP_LOGI(kTag, "AT>>> %s", cmd.c_str());
    }

    // Send AT command frame
    esp_err_t ret = SendFrame(reinterpret_cast<const uint8_t*>(cmd_with_cr.c_str()), cmd_with_cr.size(), FrameType::kAtCommand);
    if (ret != ESP_OK) {
        waiting_for_at_response_ = false;
        return ret;
    }

    // Wait for response
    EventBits_t bits = xEventGroupWaitBits(
        event_group_,
        kEventAtResponse | kEventStop,
        pdFALSE,  // Stop is latched until the next Start()
        pdFALSE,  // Wait for any bit
        pdMS_TO_TICKS(timeout_ms)
    );
    if (bits & kEventStop) {
        waiting_for_at_response_ = false;
        return ESP_ERR_INVALID_STATE;
    }
    if (!(bits & kEventAtResponse)) {
        ESP_LOGW(kTag, "AT timeout: %s", cmd.c_str());
        waiting_for_at_response_ = false;
        return ESP_ERR_TIMEOUT;
    }

    waiting_for_at_response_ = false;
    response = at_command_response_;

    // Check for OK/ERROR
    if (response.find("OK") != std::string::npos) {
        return ESP_OK;
    } else if (response.find("ERROR") != std::string::npos) {
        return ESP_FAIL;
    }

    // Response contains neither OK nor ERROR (unexpected)
    return ESP_FAIL;
}

void UartEthModem::SetNetworkEventCallback(UartEthModemEventCallback callback) {
    network_event_callback_ = std::move(callback);
}

void UartEthModem::SetDebug(bool enabled) {
    debug_enabled_.store(enabled);
}

std::string UartEthModem::GetImei() {
    if (imei_.empty()) {
        std::string resp;
        if (SendAt("AT+CGSN=1", resp) == ESP_OK) {
            // Parse +CGSN: "IMEI" using sscanf, consistent with C implementation
            char imei[16] = {0};
            if (sscanf(resp.c_str(), "\r\n+CGSN: \"%15s", imei) == 1) {
                imei_ = imei;
            }
        }
    }
    return imei_;
}

std::string UartEthModem::GetImsi() {
    if (imsi_.empty()) {
        std::string resp;
        if (SendAt("AT+CIMI", resp) == ESP_OK) {
            // Parse response using sscanf, consistent with C implementation
            char imsi[16] = {0};
            if (sscanf(resp.c_str(), "\r\n%15s", imsi) == 1) {
                imsi_ = imsi;
            }
        }
    }
    return imsi_;
}

std::string UartEthModem::GetIccid() {
    if (iccid_.empty()) {
        std::string resp;
        if (SendAt("AT+ECICCID", resp) == ESP_OK) {
            // Parse +ECICCID: ICCID using sscanf, consistent with C implementation
            char iccid[21] = {0};
            if (sscanf(resp.c_str(), "\r\n+ECICCID: %20s", iccid) == 1) {
                iccid_ = iccid;
            }
        }
    }
    return iccid_;
}

std::string UartEthModem::GetCarrierName() {
    std::string resp;
    if (SendAt("AT+COPS?", resp) == ESP_OK) {
        // Parse +COPS: mode,format,"operator",act using sscanf
        int mode = 0, format = 0, act = 0;
        char operator_name[64] = {0};
        if (sscanf(resp.c_str(), "\r\n+COPS: %d,%d,\"%63[^\"]\",%d", &mode, &format, operator_name, &act) >= 3) {
            carrier_name_ = operator_name;
        }
    }
    return carrier_name_;
}

std::string UartEthModem::GetModuleRevision() {
    if (module_revision_.empty()) {
        std::string resp;
        if (SendAt("AT+CGMR", resp) == ESP_OK) {
            // Parse response using sscanf, consistent with C implementation
            char revision[128] = {0};
            if (resp.find("+CGMR:") != std::string::npos) {
                // Format: "\r\n+CGMR: \r\n<version>\r\n"
                if (sscanf(resp.c_str(), "\r\n+CGMR: \r\n%127[^\r\n]", revision) == 1) {
                    module_revision_ = revision;
                }
            } else {
                // Format: "\r\n<version>\r\n"
                if (sscanf(resp.c_str(), "\r\n%127[^\r\n]", revision) == 1) {
                    module_revision_ = revision;
                }
            }
        }
    }
    return module_revision_;
}

int UartEthModem::GetSignalStrength() {
    if (!initialized_.load()) {
        return 99;
    }
    if (cell_info_.stat == 2) {
        return 99;
    }

    std::string resp;
    if (SendAt("AT+CSQ", resp, 500) == ESP_OK) {
        // Parse +CSQ: rssi,ber
        auto pos = resp.find("+CSQ:");
        if (pos != std::string::npos) {
            int rssi = 99;
            sscanf(resp.c_str() + pos, "+CSQ: %d", &rssi);
            signal_strength_ = rssi;
        }
    }
    return signal_strength_;
}

esp_err_t UartEthModem::RestartRegistration() {
    if (!initialized_.load() || start_mode_ != StartMode::kNormal ||
        stop_flag_.load() || data_activation_blocked_.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Keep CFUN changes from racing ECNETDEVCTL activation after a late CEREG
    // URC. SendAt() supplies AT-channel serialization; this mutex covers the
    // larger registration/data-plane transition.
    std::lock_guard<std::timed_mutex> activation_lock(data_activation_mutex_);
    if (stop_flag_.load() || data_activation_blocked_.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(kTag, "Restarting cellular registration with CFUN=0/1");
    handshake_done_ = false;
    if (event_group_) xEventGroupClearBits(event_group_, kEventHandshakeDone);
    SetDataLinkUp(false);

    std::string resp;
    const esp_err_t cfun0 = SendAt("AT+CFUN=0", resp, 5000);
    if (cfun0 != ESP_OK) {
        ESP_LOGW(kTag, "CFUN=0 registration restart failed: %s",
                 esp_err_to_name(cfun0));
    }

    // Always attempt CFUN=1. A missing CFUN=0 response is ambiguous, and
    // leaving the modem outside full-function mode would make recovery worse.
    const esp_err_t cfun1 = SendAt("AT+CFUN=1", resp, 5000);
    if (cfun1 != ESP_OK) {
        ESP_LOGW(kTag, "CFUN=1 registration restart failed: %s",
                 esp_err_to_name(cfun1));
        return cfun1;
    }

    SetNetworkEvent(UartEthModemEvent::Connecting,
                    "registration restarted with CFUN=0/1");
    SendAt("AT+CEREG?", resp, 1000);
    return cfun0;
}

esp_err_t UartEthModem::RequestPlmnSearch() {
    // Retained for source compatibility with 0.6.2. ECPLMNS is absent on
    // deployed module firmware, so new and old callers receive an explicit
    // unsupported result without transmitting an AT command.
    return ESP_ERR_NOT_SUPPORTED;
}

const char* UartEthModem::GetNetworkEventName(UartEthModemEvent event) {
    switch (event) {
        case UartEthModemEvent::Connecting: return "Connecting";
        case UartEthModemEvent::Connected: return "Connected";
        case UartEthModemEvent::Disconnected: return "Disconnected";
        case UartEthModemEvent::ModemReset: return "ModemReset";
        case UartEthModemEvent::InFlightMode: return "InFlightMode";
        case UartEthModemEvent::RfTestReady: return "RfTestReady";
        case UartEthModemEvent::ErrorNoSim: return "ErrorNoSim";
        case UartEthModemEvent::ErrorRegistrationDenied: return "ErrorRegistrationDenied";
        case UartEthModemEvent::ErrorInitFailed: return "ErrorInitFailed";
        case UartEthModemEvent::ErrorNoCarrier: return "ErrorNoCarrier";
        case UartEthModemEvent::RequestingPdpContext: return "RequestingPdpContext";
        case UartEthModemEvent::RegistrationLost: return "RegistrationLost";
        case UartEthModemEvent::PlmnSearchFallback: return "PlmnSearchFallback";
        default: return "Unknown";
    }
}
