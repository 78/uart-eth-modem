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


void UartEthModem::InitTaskRun() {
    ESP_LOGD(kTag, "Init task started");

    // Wait for start signal
    xEventGroupWaitBits(event_group_, kEventStart, pdTRUE, pdTRUE, portMAX_DELAY);

    if (stop_flag_.load()) {
        goto exit;
    }

    // Run initialization sequence based on mode
    {
        esp_err_t init_ret = ESP_FAIL;
        switch (start_mode_) {
            case StartMode::kNormal:
                init_ret = RunNormalModeInitSequence();
                break;
            case StartMode::kFlight:
                init_ret = RunFlightModeInitSequence();
                break;
            case StartMode::kRfTest:
                init_ret = RunRfTestModeInitSequence();
                break;
        }
        if (init_ret != ESP_OK) {
            ESP_LOGE(kTag, "Initialization sequence failed");
            stop_flag_ = true;
            initializing_ = false;
            xEventGroupSetBits(event_group_, kEventStop);
            goto exit;
        }

        // Initialize iot_eth only in normal mode. Flight and RF test modes
        // keep the AT transport alive but never expose an Ethernet netif.
        if (start_mode_ == StartMode::kNormal) {
            if (InitIotEth() != ESP_OK) {
                ESP_LOGE(kTag, "Failed to initialize iot_eth");
                stop_flag_ = true;
                initializing_ = false;
                xEventGroupSetBits(event_group_, kEventStop);
                goto exit;
            }
        }

        ESP_LOGD(kTag, "Initialization complete");
        initializing_ = false;
        initialized_ = true;
        xEventGroupSetBits(event_group_, kEventInitDone);

        if (start_mode_ == StartMode::kNormal) {
            // Stay alive as the cellular control task. Registration may return
            // minutes later after an application-triggered ECPLMNS; activation
            // of ECNETDEVCTL and the Ethernet link must then happen in task
            // context rather than inside the AT response parser.
            while (!stop_flag_.load()) {
                if (cell_info_.stat == 1 || cell_info_.stat == 5) {
                    esp_err_t activate_ret = ActivateDataNetwork();
                    if (activate_ret != ESP_OK && activate_ret != ESP_ERR_INVALID_STATE) {
                        ESP_LOGW(kTag, "Data-network activation failed: %s",
                                 esp_err_to_name(activate_ret));
                    }
                }

                EventBits_t bits = xEventGroupWaitBits(
                    event_group_, kEventRegistrationReady | kEventStop,
                    pdTRUE, pdFALSE, portMAX_DELAY);
                if (bits & kEventStop) {
                    break;
                }
            }
        }
    }

exit:
    ESP_LOGD(kTag, "Init task exiting");
    xEventGroupSetBits(event_group_, kEventInitTaskDone);
}

void UartEthModem::HandleAtResponse(const char* data, size_t length) {
    std::string response(data, length);

    // Parse URC or response
    ParseAtResponse(response);

    // Only signal completion for final AT responses (OK/ERROR), not for URCs like ECRDY
    if (waiting_for_at_response_ &&
        (response.find("OK") != std::string::npos || response.find("ERROR") != std::string::npos)) {
        at_command_response_ = response;
        xEventGroupSetBits(event_group_, kEventAtResponse);
    }
}

esp_err_t UartEthModem::SendAtWithRetry(const std::string& cmd, std::string& response, uint32_t timeout_ms, int max_retries) {
    esp_err_t ret = ESP_FAIL;
    for (int i = 0; i < max_retries; i++) {
        // Check stop flag before each retry
        if (stop_flag_.load()) {
            return ESP_ERR_INVALID_STATE;
        }
        ret = SendAt(cmd, response, timeout_ms);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        if (i < max_retries - 1) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    return ret;
}

void UartEthModem::ParseAtResponse(const std::string& response) {
    if (debug_enabled_.load()) {
        ESP_LOGI(kTag, "AT<<< %s", response.c_str());
    }

    // Parse CEREG following at_modem.cc logic
    auto cereg_pos = response.find("+CEREG:");
    if (cereg_pos != std::string::npos) {
        const int previous_stat = cell_info_.stat;
        int n = 0, stat = 0;
        char tac[16] = {0}, ci[16] = {0};
        int act = 0;
        
        // Try format 1: +CEREG: n,stat,"tac","ci",act (with n parameter)
        if (sscanf(response.c_str() + cereg_pos, "+CEREG: %d,%d,\"%15[^\"]\",\"%15[^\"]\",%d", &n, &stat, tac, ci, &act) == 5) {
            cell_info_.stat = stat;
            cell_info_.tac = tac;
            cell_info_.ci = ci;
            cell_info_.act = act;
        }
        // Try format 2: +CEREG: stat,"tac","ci",act (without n parameter)
        else if (sscanf(response.c_str() + cereg_pos, "+CEREG: %d,\"%15[^\"]\",\"%15[^\"]\",%d", &stat, tac, ci, &act) == 4) {
            cell_info_.stat = stat;
            cell_info_.tac = tac;
            cell_info_.ci = ci;
            cell_info_.act = act;
        }
        // Try format 3: +CEREG: n,stat (unsolicited with n)
        else if (sscanf(response.c_str() + cereg_pos, "+CEREG: %d,%d", &n, &stat) == 2) {
            cell_info_.stat = stat;
        }
        // Try format 4: +CEREG: stat (query response)
        else if (sscanf(response.c_str() + cereg_pos, "+CEREG: %d", &stat) == 1) {
            cell_info_.stat = stat;
        }

        // Registration and IP readiness are separate. CONNECTED is emitted
        // only by IP_EVENT_ETH_GOT_IP after the persistent control task has
        // activated ECNETDEVCTL and raised the Ethernet link.
        bool new_network_ready = cell_info_.stat == 1 || cell_info_.stat == 5;
        if (cell_info_.stat == 2) {
            SetNetworkEvent(UartEthModemEvent::Connecting);
        } else if (cell_info_.stat == 3) {
            SetNetworkEvent(UartEthModemEvent::ErrorRegistrationDenied);
        } else if (new_network_ready) {
            if (event_group_) {
                xEventGroupSetBits(event_group_, kEventRegistrationReady);
            }
            if (previous_stat != 1 && previous_stat != 5) {
                SetNetworkEvent(UartEthModemEvent::Connecting,
                                "cellular registration restored");
            }
        } else if (cell_info_.stat == 0 || cell_info_.stat == 4) {
            if (previous_stat == 1 || previous_stat == 5 || data_link_up_.load()) {
                SetDataLinkUp(false);
                SetNetworkEvent(UartEthModemEvent::RegistrationLost,
                                "cellular out of service");
            }
        }
    } else if (response.find("+ECNETDEVCTL: 1") != std::string::npos) {
        // Network device ready (link up)
    } else if (response.find("+ECNETDEVCTL: 0") != std::string::npos) {
        const bool was_data_ready = handshake_done_ || data_link_up_.load();
        handshake_done_ = false;
        SetDataLinkUp(false);
        // ECNETDEVCTL? legitimately returns 0 before the first activation; it
        // is not an OOS transition. Only report a loss after the data plane
        // had previously completed its handshake or raised link.
        if (was_data_ready) {
            SetNetworkEvent(UartEthModemEvent::RegistrationLost,
                            "cellular data device down");
        }
        if ((cell_info_.stat == 1 || cell_info_.stat == 5) && event_group_) {
            xEventGroupSetBits(event_group_, kEventRegistrationReady);
        }
    }
}

esp_err_t UartEthModem::AtDetect() {
    std::string resp;
    esp_err_t ret;
    int baud_rates[] = {2000000, 3000000};
    ret = SendAtWithRetry("AT", resp, 500, 4);
    if (ret == ESP_OK) {
        detect_baud_rate_ = config_.baud_rate;
        return ESP_OK;
    }
    for (size_t i = 0; i < sizeof(baud_rates) / sizeof(baud_rates[0]); i++){
        uart_set_baudrate(config_.uart_num, baud_rates[i]);
        ESP_LOGI(kTag, "Trying baud rate: %d", baud_rates[i]);
        ret = SendAtWithRetry("AT", resp, 500, 4);
        if (ret == ESP_OK) {
            detect_baud_rate_ = baud_rates[i];
            ESP_LOGI(kTag, "Detected baud rate: %d", detect_baud_rate_);
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

esp_err_t UartEthModem::ConfigurePdp() {
    std::string resp;
    esp_err_t ret;

    // Notify upper layer that we are about to configure PDP context. If the
    // event callback runs synchronously, it may call SetPdpContext() to inject
    // apn_/pdp_type_ before we read them below.
    SetNetworkEvent(UartEthModemEvent::RequestingPdpContext);

    if (apn_.empty()) {
        ESP_LOGI(kTag, "APN not set, using default");
        return ESP_OK;
    }
    ret = SendAt("AT+CGDCONT?", resp, 1000);
    if (ret == ESP_OK) {
        // Use trailing comma to avoid matching CIDs like 11, 10, etc.
        auto cgcont_pos = resp.find("+CGDCONT: 1,");
        if (cgcont_pos != std::string::npos) {
            char pdp_type[16] = {0};
            char apn[128] = {0};
            if (sscanf(resp.c_str() + cgcont_pos,
                       "+CGDCONT: 1,\"%15[^\"]\",\"%127[^\"]\"",
                       pdp_type, apn) == 2) {
                if (std::strcmp(apn, apn_.c_str()) == 0 &&
                    std::strcmp(pdp_type, pdp_type_.c_str()) == 0) {
                    ESP_LOGI(kTag, "APN already set: %s (%s)", apn, pdp_type);
                    return ESP_OK;
                }
            }
        }
        ESP_LOGI(kTag, "setting APN: %s, PDP Type: %s", apn_.c_str(), pdp_type_.c_str());
        esp_err_t cfun0_ret = SendAt("AT+CFUN=0", resp, 5000);
        if (cfun0_ret != ESP_OK) {
            ESP_LOGW(kTag, "CFUN=0 failed: %s", esp_err_to_name(cfun0_ret));
        }
        // 3GPP TS 27.007 requires PDP_type and APN to be quoted strings.
        esp_err_t cgd_ret = SendAt(
            "AT+CGDCONT=1,\"" + pdp_type_ + "\",\"" + apn_ + "\"", resp);
        if (cgd_ret != ESP_OK) {
            ESP_LOGW(kTag, "CGDCONT failed: %s", esp_err_to_name(cgd_ret));
        }
        ret = SendAt("AT+CFUN=1", resp, 5000);
    }
    return ret;
}

bool UartEthModem::ConfigureApplicationManagedPlmnSearch() {
    std::string resp;
    esp_err_t ret = SendAt("AT+ECCFG=\"PlmnSearchPowerLevel\",3", resp, 2000);
    if (ret != ESP_OK) {
        ret = SendAt("AT+ECCFG=PlmnSearchPowerLevel,3", resp, 2000);
    }

    application_managed_plmn_search_ = (ret == ESP_OK);
    if (ret == ESP_OK) {
        ESP_LOGI(kTag, "Application-managed PLMN search enabled (level 3)");
        return true;
    }

    // An AT timeout is ambiguous: the modem may have applied level 3 while
    // losing only the response. Best-effort restore level 1 so a failed setup
    // cannot silently leave an old/unknown module with search disabled.
    const bool restored = RestoreModemManagedPlmnSearch();
    application_managed_plmn_search_ = false;
    ESP_LOGW(kTag, "PlmnSearchPowerLevel=3 unavailable; %s",
             restored ? "restored modem-managed level 1"
                      : "module rejected ECCFG, preserving its built-in behavior");
    return false;
}

bool UartEthModem::RestoreModemManagedPlmnSearch() {
    std::string resp;
    esp_err_t ret = SendAt("AT+ECCFG=\"PlmnSearchPowerLevel\",1", resp, 2000);
    if (ret != ESP_OK) {
        ret = SendAt("AT+ECCFG=PlmnSearchPowerLevel,1", resp, 2000);
    }
    if (ret == ESP_OK) {
        application_managed_plmn_search_ = false;
        return true;
    }
    ESP_LOGE(kTag, "Failed to restore modem-managed PLMN search");
    return false;
}

void UartEthModem::SetDataLinkUp(bool up) {
    const bool previous = data_link_up_.exchange(up);
    if (previous == up || !mediator_) {
        return;
    }

    iot_eth_link_t link_status = up ? IOT_ETH_LINK_UP : IOT_ETH_LINK_DOWN;
    mediator_->on_stage_changed(mediator_, IOT_ETH_STAGE_LINK, &link_status);
    ESP_LOGI(kTag, "Cellular data link %s", up ? "up" : "down");
}

esp_err_t UartEthModem::ActivateDataNetwork() {
    std::lock_guard<std::mutex> activation_lock(data_activation_mutex_);
    const auto activation_cancelled = [this]() {
        return stop_flag_.load() || data_activation_blocked_.load();
    };

    if (activation_cancelled() || data_link_up_.load()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cell_info_.stat != 1 && cell_info_.stat != 5) {
        return ESP_ERR_INVALID_STATE;
    }

    std::string resp;
    int state = 0;
    esp_err_t ret = SendAt("AT+ECNETDEVCTL?", resp, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    if (activation_cancelled()) {
        return ESP_ERR_INVALID_STATE;
    }
    sscanf(resp.c_str(), "\r\n+ECNETDEVCTL: %*d,%*d,%*d,%d", &state);

    if (state == 1) {
        ESP_LOGI(kTag, "Network device already started");
        handshake_done_ = true;
        xEventGroupSetBits(event_group_, kEventHandshakeDone);
    } else {
        handshake_done_ = false;
        xEventGroupClearBits(event_group_, kEventHandshakeDone);
        ESP_LOGI(kTag, "Starting network device...");
        ret = SendAt("AT+ECNETDEVCTL=2,1,1", resp, 5000);
        if (ret != ESP_OK) {
            return ret;
        }
        if (activation_cancelled()) {
            return ESP_ERR_INVALID_STATE;
        }
        ret = SendFrame(kHandshakeRequest, sizeof(kHandshakeRequest), FrameType::kEthernet);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    EventBits_t bits = xEventGroupWaitBits(
        event_group_, kEventHandshakeDone | kEventStop | kEventDataActivationBlocked,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(kHandshakeTimeoutMs));
    if ((bits & (kEventStop | kEventDataActivationBlocked)) || activation_cancelled()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(bits & kEventHandshakeDone)) {
        return ESP_ERR_TIMEOUT;
    }

    SetDataLinkUp(true);
    return ESP_OK;
}

esp_err_t UartEthModem::RunFlightModeInitSequence() {
    std::string resp;
    esp_err_t ret;

    // Step 1: AT test
    ESP_LOGI(kTag, "Detecting modem...");
    ret = AtDetect();
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Modem not detected");
        SetNetworkEvent(UartEthModemEvent::ErrorInitFailed, "Modem not detected (AT no response)");
        return ret;
    }

    // Enter flight mode (CFUN=4)
    ret = SendAt("AT+CFUN=4", resp, 3000);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to enter flight mode");
        SetNetworkEvent(UartEthModemEvent::ErrorInitFailed, "Failed to enter flight mode (CFUN=4)");
        return ret;
    }

    ESP_LOGI(kTag, "Checking SIM card...");
    if (!CheckSimCard()) {
        if (stop_flag_.load()) {
            ESP_LOGI(kTag, "SIM check cancelled during modem shutdown");
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGE(kTag, "SIM card not ready");
        SetNetworkEvent(UartEthModemEvent::ErrorNoSim);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(kTag, "Querying modem info...");
    QueryModemInfo();

    ESP_LOGI(kTag, "Flight mode initialization complete");
    initialized_ = true;
    SetNetworkEvent(UartEthModemEvent::InFlightMode);
    return ESP_OK;
}

esp_err_t UartEthModem::RunRfTestModeInitSequence() {
    std::string resp;
    esp_err_t ret;

    ESP_LOGI(kTag, "Detecting modem for RF test mode...");
    ret = AtDetect();
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Modem not detected");
        SetNetworkEvent(UartEthModemEvent::ErrorInitFailed, "Modem not detected (AT no response)");
        return ret;
    }

    ESP_LOGI(kTag, "Enabling SIM simulator for RF test mode...");
    ret = SendAtWithRetry("AT+ECSIMCFG=\"SimSimulator\",1", resp, 3000, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to enable SIM simulator");
        SetNetworkEvent(UartEthModemEvent::ErrorInitFailed,
                        "Failed to enable SIM simulator (ECSIMCFG)");
        return ret;
    }

    ESP_LOGI(kTag, "Rebooting modem after SIM simulator configuration...");
    SendAt("AT+ECRST", resp, 500);
    vTaskDelay(pdMS_TO_TICKS(1500));

    ret = SendAtWithRetry("AT", resp, 500, 20);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Modem not responding after RF test reset");
        SetNetworkEvent(UartEthModemEvent::ErrorInitFailed,
                        "Modem not responding after RF test reset");
        return ret;
    }

    ESP_LOGI(kTag, "Entering full functionality mode for RF test...");
    ret = SendAt("AT+CFUN=1", resp, 5000);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to enter full functionality mode for RF test");
        SetNetworkEvent(UartEthModemEvent::ErrorInitFailed,
                        "Failed to enter full functionality mode (CFUN=1)");
        return ret;
    }

    ESP_LOGI(kTag, "RF test mode initialization complete");
    initialized_ = true;
    SetNetworkEvent(UartEthModemEvent::RfTestReady);
    return ESP_OK;
}

esp_err_t UartEthModem::RunNormalModeInitSequence() {
    std::string resp;
    esp_err_t ret;
    bool modem_need_reset = false;

    // Step 1: AT test
    ESP_LOGI(kTag, "Detecting modem...");
    ret = AtDetect();
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Modem not detected");
        SetNetworkEvent(UartEthModemEvent::ErrorInitFailed, "Modem not detected (AT no response)");
        return ret;
    }

    const bool baud_changed = (detect_baud_rate_ != config_.baud_rate);

    // Check and configure network settings
    ESP_LOGI(kTag, "Checking network configuration...");
    ret = SendAt("AT+ECNETCFG?", resp, 1000);
    if (ret != ESP_OK || resp.find("+ECNETCFG: \"nat\",1") == std::string::npos) {
        // First-time configuration
        ESP_LOGI(kTag, "Configuring network (first-time setup)...");
        SendAtWithRetry("AT+ECPCFG=\"usbCtrl\",1", resp, 1000, 3);
        SendAtWithRetry("AT+ECNETCFG=\"nat\",1,\"192.168.10.2\"", resp, 1000, 3);
        modem_need_reset = true;
    }

    if (baud_changed) {
        ESP_LOGI(kTag, "Setting baud rate to configured value: %d", config_.baud_rate);
        SendAt("AT+XJCFG=netPortBaudRate," + std::to_string(config_.baud_rate), resp);
        modem_need_reset = true;
    }

    if (modem_need_reset) {
        modem_need_reset = false;
        // Reset after configuration
        xEventGroupClearBits(event_group_, kEventNetworkEventChanged);

        SendAt("AT+ECRST", resp, 500);
        if (baud_changed) {
            uart_set_baudrate(config_.uart_num, config_.baud_rate);
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
        // Wait for modem to respond
        ret = SendAtWithRetry("AT", resp, 500, 20);
        if (ret != ESP_OK) {
            ESP_LOGE(kTag, "Modem not responding after reset");
            SetNetworkEvent(UartEthModemEvent::ErrorInitFailed, "Modem not responding after reset");
            return ret;
        }
    }

    // Enter full functionality mode (CFUN=1)
    ret = SendAt("AT+CFUN=1", resp, 3000);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to enter full functionality mode");
        SetNetworkEvent(UartEthModemEvent::ErrorInitFailed, "Failed to enter full functionality mode (CFUN=1)");
        return ret;
    }

    ESP_LOGI(kTag, "Checking SIM card...");
    if (!CheckSimCard()) {
        if (stop_flag_.load()) {
            ESP_LOGI(kTag, "SIM check cancelled during modem shutdown");
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGE(kTag, "SIM card not ready");
        SetNetworkEvent(UartEthModemEvent::ErrorNoSim);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(kTag, "Querying modem info...");
    QueryModemInfo();

    ConfigurePdp();

    // Configure this before waiting for service. Older module revisions may
    // reject ECCFG; in that case their default level-1 periodic search remains
    // untouched and the application must not schedule ECPLMNS.
    ConfigureApplicationManagedPlmnSearch();

    // Set modem sleep parameters while the AT control plane is available.
    ret = SendAt("AT+ECSCLKEX=1," + std::to_string(kModemSleepTimeoutS) + ",30", resp, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to set modem sleep parameters");
        SetNetworkEvent(UartEthModemEvent::ErrorInitFailed, "Failed to set modem sleep parameters (ECSCLKEX)");
        return ret;
    }

    // Registration has no component-owned timeout. CEREG URCs drive the
    // persistent control task, and the top-level policy decides when ECPLMNS
    // should be retried.
    SendAt("AT+CEREG=2", resp);
    SetNetworkEvent(UartEthModemEvent::Connecting);
    GetCellInfo();

    ESP_LOGI(kTag, "Modem control plane initialized; waiting for registration");
    return ESP_OK;
}


bool UartEthModem::CheckSimCard() {
    std::string resp;
    for (int i = 0; i < 10; i++) {
        // Check stop flag before each iteration
        if (stop_flag_.load()) {
            ESP_LOGW(kTag, "CheckSimCard aborted due to stop flag");
            return false;
        }
        if (SendAt("AT+CPIN?", resp, 1000) == ESP_OK) {
            if (resp.find("+CPIN: READY") != std::string::npos) {
                return true;
            }
        }
        if (resp.find("+CME ERROR: 10") != std::string::npos) {
            // SIM not inserted
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}

bool UartEthModem::WaitForRegistration(uint32_t timeout_ms) {
    std::string resp;
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (!stop_flag_.load()) {
        if (SendAt("AT+CEREG?", resp, 1000) == ESP_OK) {
            ParseAtResponse(resp);
            if (cell_info_.stat == 1 || cell_info_.stat == 5) {
                return true;
            }
            if (cell_info_.stat == 3) {
                return false;  // Registration denied
            }
        }

        uint32_t elapsed = xTaskGetTickCount() * portTICK_PERIOD_MS - start;
        if (elapsed >= timeout_ms) {
            return false;
        }

        // Log progress
        if ((elapsed / 1000) % 10 == 0) {
            ESP_LOGI(kTag, "Waiting for registration... (%lu/%lu ms)", elapsed, timeout_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}

void UartEthModem::QueryModemInfo() {
    GetImei();
    GetIccid();
    GetModuleRevision();
    GetImsi();
    ESP_LOGD(kTag, "Modem Info - IMEI: %s, ICCID: %s, Rev: %s", imei_.c_str(), iccid_.c_str(), module_revision_.c_str());
}

