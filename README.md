# UART Ethernet Modem Component

ESP32 上的 UART Ethernet Modem 驱动组件。通过 UHCI + GDMA 实现高效的接收，TX 端使用 UART FIFO 同步写入（节省 GDMA 通道），并通过状态机管理低功耗模式。

本组件适配以下 4G LTE Cat.1 Module：
- EC801E 串口网卡固件
- NT26 / NT21 串口网卡固件

3M 波特率测试下载速率 220KB/s，5M 波特率测试下载速率 360KB/s

## 功能特性

- **高效接收**: 使用 ESP32 的 UHCI + GDMA + idle EOF 实现零拷贝持续接收，缓冲区通过 GDMA owner 机制动态切换。
- **轻量发送**: TX 通过同步 UART FIFO 写入，不占用额外 GDMA 通道；由专用 `TxTask` 串行化所有发送。
- **状态管理**: 完整的低功耗状态机（`Idle` / `PendingActive` / `Active` / `PendingIdle`），支持 MRDY/SRDY 唤醒机制与按需 PM 锁。
- **协议支持**: 自动处理帧头部、校验以及 AT 命令；AT/Ethernet/握手帧统一走 4 字节自定义帧头。
- **事件驱动**: ISR、`TxTask`、应用层均通过 `event_queue_` 与 `event_group_` 与主任务异步通信。
- **APN / PDP 配置**: 支持上层注入 APN 和 PDP 类型，启动时自动写入模组并按需重启；并对外发出 `RequestingPdpContext` 事件，供同步回调场景下回填配置。
- **波特率自适应**: 启动阶段自动探测模组当前波特率（115200 / 2M / 3M），与目标值不一致时仅复位一次完成切换。

> 完整的工作原理（状态机、DMA 缓冲区池、帧协议、事件流、低功耗等）请参见 [`WORKING_PRINCIPLE.md`](./WORKING_PRINCIPLE.md)。

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

// Optional: customize APN / PDP type before Start(). Skip this call to keep
// the modem's default PDP context. Alternatively, if your event callback runs
// synchronously in the driver init task, you can call SetPdpContext() in
// response to the RequestingPdpContext event below.
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
- 新增 `RequestingPdpContext` 事件：在配置 PDP 之前发出，便于同步回调场景下回填 APN；异步派发的客户端仍需在 `Start()` 之前调用 `SetPdpContext`。
- 优化首次启动流程：波特率切换和 NAT 配置合并到同一次 `AT+ECRST`，缩短开机时长。
- `RunNormalModeInitSequence` 调整为先检查 NAT/baud，再统一进入 `CFUN=1`，去掉冗余的二次进入全功能态。

### [0.1.0] - 2026-01-19
- 初始版本：从项目 `main/hardware/network` 迁移为独立组件。

> 0.1.1 ~ 0.3.5 的中间版本以增量优化为主（飞行模式、ISR 安全性、DMA 缓冲、DNS 缓存管理等），详见 git log。

---

## 核心原理概述

> 详细内容请参考 [`WORKING_PRINCIPLE.md`](./WORKING_PRINCIPLE.md)。

### 1. 系统架构
驱动采用分层架构：

- **UartEthModem**: 主驱动类，管理状态机、帧协议、AT 命令、初始化序列。包含 `MainTask`（事件循环）、`InitTask`（启动序列）、`TxTask`（串行化发送）。
- **UartUhci**: 独立组件 [`components/uart-uhci`](../uart-uhci/README.md)，提供基于缓冲区池 + GDMA owner 机制的持续 RX，以及同步阻塞的 TX FIFO 写入和 PM 锁管理。

### 2. 状态机
控制 RX DMA 启停和 MRDY/SRDY 信号，共四个状态：`Idle`、`PendingActive`、`Active`、`PendingIdle`。空闲超时 500ms 后从 `Active` 切到 `PendingIdle`，双方都空闲时进入 `Idle` 并释放 PM 锁。

### 3. UART 收发机制
- **RX**：UHCI + GDMA 链表 + idle EOF 模式；所有缓冲区固定挂载，通过 GDMA owner 标志在 DMA/CPU 之间切换；上层处理完毕调用 `ReturnBuffer()` 归还。
- **TX**：直接调用 `uart_ll_write_txfifo` 同步写入（FIFO 满时短延时重试），不占用 GDMA 通道；进入/退出时持/放 PM 锁。

### 4. 帧协议格式
4 字节自定义帧头 + 0~1596 字节载荷，单帧最大 `kMaxFrameSize = 1600` 字节：

| 字段 | 位宽 | 说明 |
|------|------|------|
| `payload_length` | 12 bits | 载荷长度 |
| `seq_no` | 4 bits | 序列号（0-15 循环） |
| `type` | 2 bits | 0 = Ethernet，1 = AT 命令/响应 |
| `continue` / `flow_control` | 各 1 bit | 分片标志 / XOFF |
| `checksum` | 8 bits | `((sum>>8) ^ sum ^ 0x03) & 0xFF`，`sum = raw[0]+raw[1]+raw[2]` |

握手帧（`kHandshakeRequest` / `kHandshakeAck`）也以 Ethernet 类型承载。
