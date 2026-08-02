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
    Stop();

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

    if (initialized_.load()) {
        ESP_LOGW(kTag, "Already started");
        return ESP_ERR_INVALID_STATE;
    }

    start_mode_ = mode;
    stop_flag_ = false;
    handshake_done_ = false;
    initializing_ = true;
    application_managed_plmn_search_ = false;
    data_link_up_ = false;
    data_activation_blocked_ = false;
    xEventGroupClearBits(event_group_, kEventDataActivationBlocked);

    // Create event queue FIRST (before GPIO init, since ISR uses it)
    event_queue_ = xQueueCreate(32, sizeof(Event));
    if (!event_queue_) {
        ESP_LOGE(kTag, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }

    // Create TX queue for non-blocking transmit from LWIP
    tx_queue_ = xQueueCreate(kTxQueueDepth, sizeof(TxFrame));
    if (!tx_queue_) {
        ESP_LOGE(kTag, "Failed to create TX queue");
        vQueueDelete(event_queue_);
        event_queue_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    // Initialize UART
    esp_err_t ret = InitUart();
    if (ret != ESP_OK) {
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
        vQueueDelete(event_queue_);
        event_queue_ = nullptr;
        return ret;
    }

    // Initialize GPIO
    ret = InitGpio();
    if (ret != ESP_OK) {
        DeinitUart();
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
        vQueueDelete(event_queue_);
        event_queue_ = nullptr;
        return ret;
    }

    // Allocate frame reassembly buffer
    reassembly_buffer_ = static_cast<uint8_t*>(heap_caps_malloc(kMaxFrameSize, MALLOC_CAP_INTERNAL));
    if (!reassembly_buffer_) {
        ESP_LOGE(kTag, "Failed to allocate reassembly buffer");
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
        vQueueDelete(event_queue_);
        DeinitGpio();
        DeinitUart();
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
        free(reassembly_buffer_);
        reassembly_buffer_ = nullptr;
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
        vQueueDelete(event_queue_);
        DeinitGpio();
        DeinitUart();
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
        stop_flag_ = true;
        vTaskDelay(pdMS_TO_TICKS(100));
        uart_uhci_.Deinit();
        free(reassembly_buffer_);
        reassembly_buffer_ = nullptr;
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
        vQueueDelete(event_queue_);
        DeinitGpio();
        DeinitUart();
        return ESP_ERR_NO_MEM;
    }

    // Signal start (initialization continues asynchronously in InitTaskRun)
    // Failure will be notified via event callback (ErrorInitFailed, ErrorNoSim, etc.)
    // Caller should call Stop() after receiving failure event to cleanup resources
    xEventGroupSetBits(event_group_, kEventStart);

    ESP_LOGI(kTag, "UartEthModem starting asynchronously...");
    return ESP_OK;
}

esp_err_t UartEthModem::Stop() {
    // Check if there's anything to stop: event_queue_ is created in Start()
    // and destroyed in CleanupResources(). If it exists, tasks may be running.
    if (!event_queue_) {
        return ESP_OK;
    }

    ESP_LOGI(kTag, "Stopping UartEthModem...");

    PrepareForShutdown();
    stop_flag_ = true;
    initializing_ = false;
    if (event_group_) {
        xEventGroupSetBits(event_group_, kEventStop);
    }

    // Send Stop event to main task queue to wake it up from xQueueReceive
    // (MainTask may be blocked on portMAX_DELAY in Idle state)
    if (event_queue_) {
        Event event = {.type = EventType::Stop, .rx_buffer = nullptr};
        xQueueSend(event_queue_, &event, 0);
    }

    // Send dummy frame to tx_queue to wake up TxTask from xQueueReceive
    // (TxTask may be blocked on portMAX_DELAY waiting for frame)
    if (tx_queue_) {
        TxFrame dummy_frame = {};
        xQueueSend(tx_queue_, &dummy_frame, 0);
    }

    // Wait for every task to finish. Resource destruction while even one task
    // is alive is unsafe: task epilogues publish their done bit through this
    // event group. A partial non-zero mask must not be mistaken for all tasks.
    if (event_group_) {
        EventBits_t bits = 0;
        do {
            bits = xEventGroupWaitBits(event_group_, kEventAllTasksDone,
                                       pdTRUE, pdTRUE, pdMS_TO_TICKS(10000));
            if ((bits & kEventAllTasksDone) != kEventAllTasksDone) {
                ESP_LOGE(kTag, "Still waiting for modem tasks, completed mask=0x%lx",
                         static_cast<unsigned long>(bits & kEventAllTasksDone));
            }
        } while ((bits & kEventAllTasksDone) != kEventAllTasksDone);
    }

    // Cleanup all resources including iot_eth
    CleanupResources(true);

    initialized_ = false;

    ESP_LOGI(kTag, "UartEthModem stopped");
    return ESP_OK;
}

void UartEthModem::PrepareForShutdown() {
    data_activation_blocked_ = true;
    if (event_group_) {
        xEventGroupClearBits(event_group_, kEventRegistrationReady);
        xEventGroupSetBits(event_group_, kEventDataActivationBlocked);
    }

    // Setting the flag first prevents another activation from entering while
    // this waits for an already-running activation to observe the event and
    // leave. AT commands remain usable after the barrier is established.
    std::lock_guard<std::mutex> lock(data_activation_mutex_);
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
    while (!lock.try_lock_for(std::chrono::milliseconds(50))) {
        if (stop_flag_.load()) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    // Check again after acquiring mutex in case Stop() was called while waiting
    if (stop_flag_.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Allow AT commands during initialization even if not connected
    if (!handshake_done_.load() && !initializing_.load() && !initialized_.load()) {
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
        pdTRUE,   // Clear on exit
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

UartEthModem::CellInfo UartEthModem::GetCellInfo() {
    std::string resp;
    if (SendAt("AT+CEREG?", resp) == ESP_OK) {
        ParseAtResponse(resp);
    }
    return cell_info_;
}

esp_err_t UartEthModem::RequestPlmnSearch() {
    if (!application_managed_plmn_search_.load()) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    std::string resp;
    ESP_LOGI(kTag, "Requesting PLMN search with ECPLMNS");
    esp_err_t ret = SendAt("AT+ECPLMNS", resp, 5000);
    if (ret == ESP_OK) {
        SetNetworkEvent(UartEthModemEvent::Connecting);
        return ESP_OK;
    }

    ESP_LOGW(kTag, "ECPLMNS failed (%s); restoring modem-managed PLMN search",
             esp_err_to_name(ret));
    const bool restored = RestoreModemManagedPlmnSearch();
    if (!restored) {
        // A module that accepted level 3 but rejects both ECPLMNS and the
        // level-1 restore must not be left permanently out of service. CFUN
        // cycling is the conservative legacy escape hatch recommended by the
        // module vendor; it restarts registration without pretending that the
        // application still owns PLMN timing.
        std::string cfun_resp;
        const esp_err_t cfun0 = SendAt("AT+CFUN=0", cfun_resp, 5000);
        const esp_err_t cfun1 = cfun0 == ESP_OK
            ? SendAt("AT+CFUN=1", cfun_resp, 5000) : cfun0;
        ESP_LOGW(kTag, "PLMN fallback CFUN cycle result: %s",
                 esp_err_to_name(cfun1));
    }
    application_managed_plmn_search_ = false;
    SetNetworkEvent(UartEthModemEvent::PlmnSearchFallback,
                    restored ? "ECPLMNS failed; restored modem-managed search"
                             : "ECPLMNS and level restore failed; used CFUN restart");
    return ret;
}

const char* UartEthModem::GetNetworkEventName(UartEthModemEvent event) {
    switch (event) {
        case UartEthModemEvent::Connecting: return "Connecting";
        case UartEthModemEvent::Connected: return "Connected";
        case UartEthModemEvent::Disconnected: return "Disconnected";
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
