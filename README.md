# UART Ethernet Modem Component

ESP32 上的 UART Ethernet Modem 驱动组件。通过 UHCI DMA 实现高效的数据传输，并管理低功耗模式。

本组件适配以下 4G LTE Cat.1 Module：
- EC801E 串口网卡固件
- NT26 / NT21 串口网卡固件

3M 波特率测试下载速率 220KB/s，5M 波特率测试下载速率 360KB/s

## 功能特性

- **高效传输**: 使用 ESP32 的 UHCI DMA 硬件加速 UART 数据收发。
- **状态管理**: 完整的低功耗状态机，支持 MRDY/SRDY 唤醒机制。
- **协议支持**: 自动处理帧头部、校验以及 AT 命令。
- **事件驱动**: 异步事件处理架构。
- **APN / PDP 配置**: 支持上层注入 APN 和 PDP 类型，启动时自动写入模组并按需重启。
- **波特率自适应**: 启动阶段自动探测模组当前波特率（115200 / 2M / 3M），与目标值不一致时仅复位一次完成切换。

## 用法示例

```cpp
UartEthModem::Config cfg = {
    .uart_num = UART_NUM_1,
    .baud_rate = 3000000,
    .tx_pin = GPIO_NUM_17,
    .rx_pin = GPIO_NUM_18,
    .mrdy_pin = GPIO_NUM_5,
    .srdy_pin = GPIO_NUM_6,
};
auto modem = std::make_unique<UartEthModem>(cfg);

// Optional: customize APN / PDP type before Start().
// Skip this call to keep the modem's default PDP context.
modem->SetPdpContext("internet", "IP");

modem->SetNetworkEventCallback([](UartEthModem::UartEthModemEvent ev) {
    ESP_LOGI("app", "modem event: %s", UartEthModem::GetNetworkEventName(ev));
});

modem->Start();
```

## 变更日志 (Changelog)

### [0.4.0] - 2026-04-28
- 新增 `SetPdpContext(apn, pdp_type)` API，允许上层注入 APN 和 PDP 类型；为空时沿用模组默认配置。
- 新增 `GetImsi()` 接口（`AT+CIMI`）。
- 新增 `ConfiguringPdp` 事件，仅在实际改写 PDP context 时触发。
- 优化首次启动流程：波特率切换和 NAT 配置合并到同一次 `AT+ECRST`，缩短开机时长。
- `RunNormalModeInitSequence` 调整为先检查 NAT/baud，再统一进入 `CFUN=1`，去掉冗余的二次进入全功能态。

### [0.1.0] - 2026-01-19
- 初始版本：从项目 `main/hardware/network` 迁移为独立组件。

> 0.1.1 ~ 0.3.5 的中间版本以增量优化为主（飞行模式、ISR 安全性、DMA 缓冲、DNS 缓存管理等），详见 git log。

---

## 核心原理概述

### 1. 系统架构
驱动采用分层架构，通过 UHCI DMA 实现高效的数据传输，并通过状态机管理低功耗模式。

- **UartEthModem**: 主驱动类，管理状态机、帧协议、AT 命令。
- **UartUhci**: UHCI DMA 控制器，提供高效的 DMA 传输。

### 2. 状态机
状态机用于管理低功耗模式，控制 DMA 的启停和 MRDY/SRDY 信号。支持 `Idle`, `PendingActive`, `Active`, `PendingIdle` 四种状态。

### 3. UART DMA 机制
使用 ESP32 的 UHCI 外设和 GDMA 控制器，配合循环缓冲区池实现零拷贝风格的数据接收。

### 4. 帧协议格式
所有数据包均以 `0xAA 0x55` 开头，包含长度、类型、序列号和 CRC16 校验。
