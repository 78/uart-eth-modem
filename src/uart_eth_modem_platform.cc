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


// Private methods

esp_err_t UartEthModem::InitUart() {
    // UHCI DMA mode: only configure UART params, don't install driver (UHCI takes over)
    // Zero-initialize and assign per-field so newly added uart_config_t members
    // across ESP-IDF versions (e.g. rx_glitch_filt_thresh in 6.2) stay defaulted
    // without tripping -Werror=missing-field-initializers.
    uart_config_t uart_config = {};
    uart_config.baud_rate = config_.baud_rate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    esp_err_t ret = uart_param_config(config_.uart_num, &uart_config);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to configure UART");

    ret = uart_set_pin(config_.uart_num, config_.tx_pin, config_.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to set UART pins");

    gpio_set_pull_mode(config_.rx_pin, GPIO_PULLUP_ONLY);

    return ESP_OK;
}

esp_err_t UartEthModem::InitGpio() {
    // MRDY pin (output) - Master Ready/Busy
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << config_.mrdy_pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    esp_err_t ret = gpio_config(&io_conf);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to configure MRDY pin");

    // Set MRDY high initially (idle)
    gpio_set_level(config_.mrdy_pin, 1);
    gpio_sleep_sel_dis(config_.mrdy_pin);

    // SRDY pin (input with interrupt) - Slave Ready/Busy
    io_conf.pin_bit_mask = (1ULL << config_.srdy_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ret = gpio_config(&io_conf);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to configure SRDY pin");

    // Enable GPIO wakeup on low level (for light sleep)
    ret = gpio_wakeup_enable(config_.srdy_pin, GPIO_INTR_LOW_LEVEL);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to enable GPIO wakeup");

    // Install ISR handler
    ret = gpio_isr_handler_add(config_.srdy_pin, SrdyIsrHandler, this);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to add ISR handler");

    // Initially configure for wakeup (low level trigger)
    ConfigureSrdyInterrupt(kSrdyInterruptForWakeup);

    return ESP_OK;
}

esp_err_t UartEthModem::InitIotEth() {
    // Install iot_eth driver
    iot_eth_config_t eth_cfg = {
        .driver = &driver_,
        .stack_input = nullptr,
        .stack_input_info = nullptr,
    };

    esp_err_t ret = iot_eth_install(&eth_cfg, &eth_handle_);
    ESP_RETURN_ON_ERROR(ret, kTag, "Failed to install iot_eth driver");

    // Create netif with GARP disabled
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    // Copy base config to modify flags (original is const)
    esp_netif_inherent_config_t base_cfg = *netif_cfg.base;
    base_cfg.flags = static_cast<esp_netif_flags_t>(base_cfg.flags & ~ESP_NETIF_FLAG_GARP);
    netif_cfg.base = &base_cfg;
    eth_netif_ = esp_netif_new(&netif_cfg);
    if (!eth_netif_) {
        ESP_LOGE(kTag, "Failed to create netif");
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    // Create glue and attach
    glue_ = iot_eth_new_netif_glue(eth_handle_);
    if (!glue_) {
        ESP_LOGE(kTag, "Failed to create netif glue");
        esp_netif_destroy(eth_netif_);
        eth_netif_ = nullptr;
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    esp_netif_attach(eth_netif_, glue_);

    // Start iot_eth
    ret = iot_eth_start(eth_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to start iot_eth");
        iot_eth_del_netif_glue(glue_);
        glue_ = nullptr;
        esp_netif_destroy(eth_netif_);
        eth_netif_ = nullptr;
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
        return ret;
    }

    // Register IP event handler to detect when we get an IP address
    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                              &IpEventHandler, this,
                                              &ip_event_handler_instance_);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to register IP event handler");
        iot_eth_stop(eth_handle_);
        iot_eth_del_netif_glue(glue_);
        glue_ = nullptr;
        esp_netif_destroy(eth_netif_);
        eth_netif_ = nullptr;
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_ETH_LOST_IP,
                                              &IpEventHandler, this,
                                              &lost_ip_event_handler_instance_);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to register lost-IP event handler");
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                              ip_event_handler_instance_);
        ip_event_handler_instance_ = nullptr;
        iot_eth_stop(eth_handle_);
        iot_eth_del_netif_glue(glue_);
        glue_ = nullptr;
        esp_netif_destroy(eth_netif_);
        eth_netif_ = nullptr;
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
        return ret;
    }

    // Finish low-level setup, but keep the link down until registration and
    // ECNETDEVCTL/handshake are ready. This lets the AT control plane remain
    // alive while the modem searches or the application backs off CFUN retries.
    if (mediator_) {
        mediator_->on_stage_changed(mediator_, IOT_ETH_STAGE_LL_INIT, nullptr);
    }

    // Clear DNS cache
    tcpip_callback([](void* arg) -> void {
        dns_clear_cache();
    }, nullptr);

    return ESP_OK;
}

void UartEthModem::DeinitUart() {
    // UHCI mode: UART driver is not installed, just disconnect pins and reset GPIO
    // Disconnect UART pins (set to UART_PIN_NO_CHANGE disconnects the signal)
    uart_set_pin(config_.uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Reset GPIO pins to default state (input, no pull) to save power
    gpio_reset_pin(config_.tx_pin);
    gpio_reset_pin(config_.rx_pin);
}

void UartEthModem::DeinitGpio() {
    gpio_wakeup_disable(config_.srdy_pin);
    gpio_isr_handler_remove(config_.srdy_pin);
    // Re-enable sleep select (was disabled in InitGpio for MRDY)
    gpio_sleep_sel_en(config_.mrdy_pin);
    // Reset GPIO pins to default state (input, no pull) to save power
    gpio_reset_pin(config_.mrdy_pin);
    gpio_reset_pin(config_.srdy_pin);
}

void UartEthModem::DeinitIotEth() {
    // Unregister IP event handler first
    if (ip_event_handler_instance_) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                              ip_event_handler_instance_);
        ip_event_handler_instance_ = nullptr;
    }
    if (lost_ip_event_handler_instance_) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_ETH_LOST_IP,
                                              lost_ip_event_handler_instance_);
        lost_ip_event_handler_instance_ = nullptr;
    }
    if (eth_handle_) {
        iot_eth_stop(eth_handle_);
    }
    if (glue_) {
        iot_eth_del_netif_glue(glue_);
        glue_ = nullptr;
    }
    if (eth_netif_) {
        // Stop DHCP client first to prevent use-after-free in dhcp_fine_tmr
        // The DHCP timer runs periodically and accesses netif's DHCP data;
        // destroying netif without stopping DHCP causes crash.
        esp_netif_dhcpc_stop(eth_netif_);
        esp_netif_destroy(eth_netif_);
        eth_netif_ = nullptr;
    }
    if (eth_handle_) {
        iot_eth_uninstall(eth_handle_);
        eth_handle_ = nullptr;
    }
}

// Set MRDY level
void UartEthModem::SetMrdy(MrdyLevel level) {
    bool is_low = (level == MrdyLevel::Low);
    gpio_set_level(config_.mrdy_pin, is_low ? 0 : 1);
    mrdy_is_low_.store(is_low);
}

// Check if SRDY is low (slave is busy/has data)
bool UartEthModem::IsSrdyLow() {
    return gpio_get_level(config_.srdy_pin) == 0;
}

// Send ACK pulse: MRDY high for 50us
void UartEthModem::SendAckPulse() {
    // MRDY: low -> high (50us) -> low
    SetMrdy(MrdyLevel::High);  // High
    esp_rom_delay_us(kAckPulseUs);
    SetMrdy(MrdyLevel::Low);   // Low (back to busy)
}

// Configure SRDY interrupt type
// for_wakeup: true = LOW_LEVEL (for light sleep wakeup), false = edge detection
void UartEthModem::ConfigureSrdyInterrupt(bool for_wakeup) {
    if (for_wakeup) {
        // LOW_LEVEL trigger for light sleep wakeup
        gpio_set_intr_type(config_.srdy_pin, GPIO_INTR_LOW_LEVEL);
    } else {
        // Use POSEDGE to detect SRDY going high (slave ACK or entering sleep)
        gpio_set_intr_type(config_.srdy_pin, GPIO_INTR_ANYEDGE);
    }
    gpio_intr_enable(config_.srdy_pin);
}

// ISR handler for SRDY pin changes
// NOTE: Must use gpio_ll_* functions here for IRAM safety during Flash writes
void IRAM_ATTR UartEthModem::SrdyIsrHandler(void* arg) {
    auto* self = static_cast<UartEthModem*>(arg);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Disable interrupt to avoid repeated triggers (IRAM-safe)
    gpio_ll_intr_disable(&GPIO, self->config_.srdy_pin);

    // Determine event type based on current SRDY level (IRAM-safe)
    int level = gpio_ll_get_level(&GPIO, self->config_.srdy_pin);

    // Send event to queue for state machine processing
    Event event = {
        .type = (level == 0) ? EventType::SrdyLow : EventType::SrdyHigh,
        .rx_buffer = nullptr,
    };

    xQueueSendFromISR(self->event_queue_, &event, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void UartEthModem::SetNetworkEvent(UartEthModemEvent event, const std::string& detail) {
    // Always update the event value
    UartEthModemEvent old_event = network_event_.exchange(event);
    // Set event bit to notify waiting tasks
    if (event_group_) {
        xEventGroupSetBits(event_group_, kEventNetworkEventChanged);
    }
    
    if (old_event != event) {
        if (detail.empty()) {
            ESP_LOGI(kTag, "Network event: %s -> %s", GetNetworkEventName(old_event), GetNetworkEventName(event));
        } else {
            ESP_LOGI(kTag, "Network event: %s -> %s (%s)", GetNetworkEventName(old_event),
                     GetNetworkEventName(event), detail.c_str());
        }
        if (network_event_callback_) {
            network_event_callback_(event, detail);
        }
    }
}

void UartEthModem::IpEventHandler(void* arg, esp_event_base_t event_base,
                                  int32_t event_id, void* event_data) {
    auto* self = static_cast<UartEthModem*>(arg);
    
    if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP) {
        // Network is ready now
        self->ip_ready_ = true;
        self->SetNetworkEvent(UartEthModemEvent::Connected);
        if (self->event_group_) {
            xEventGroupSetBits(self->event_group_, kEventNetworkReady);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_LOST_IP) {
        self->ip_ready_ = false;
        self->SetDataLinkUp(false);
        self->SetNetworkEvent(UartEthModemEvent::RegistrationLost,
                              "cellular interface lost IP");
        if ((self->cell_info_.stat == 1 || self->cell_info_.stat == 5) && self->event_group_) {
            xEventGroupSetBits(self->event_group_, kEventRegistrationReady);
        }
    }
}

void UartEthModem::CleanupResources(bool cleanup_iot_eth) {
    // Cleanup iot_eth if requested
    if (cleanup_iot_eth) {
        DeinitIotEth();
    }

    // Stop the GPIO ISR before deleting any queue or event object it can
    // reference. Tasks have already joined when Stop() calls this routine.
    DeinitGpio();

    // Cleanup UHCI controller
    uart_uhci_.Deinit();

    // Cleanup reassembly buffer
    if (reassembly_buffer_) {
        free(reassembly_buffer_);
        reassembly_buffer_ = nullptr;
    }
    reassembly_size_ = 0;
    reassembly_expected_ = 0;

    // Cleanup TX queue (free any pending frames)
    if (tx_queue_) {
        TxFrame frame;
        while (xQueueReceive(tx_queue_, &frame, 0) == pdTRUE) {
            if (frame.data) {
                free(frame.data);
            }
        }
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
    }

    // Cleanup event queue
    if (event_queue_) {
        Event event;
        while (xQueueReceive(event_queue_, &event, 0) == pdTRUE) {
            // No dynamic data in events now
        }
        vQueueDelete(event_queue_);
        event_queue_ = nullptr;
    }

    // Cleanup UART
    DeinitUart();
}
