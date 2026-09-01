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
- **启动模式**: 支持普通联网模式、飞行模式和 RF 实验室测试模式；RF 测试模式只执行指定 AT 序列并保持 AT 传输层，不安装 `iot_eth` 网络接口。
- **APN / PDP 配置**: 支持上层注入 APN 和 PDP 类型，启动时自动写入模组并按需重启；并对外发出 `RequestingPdpContext` 事件，供同步回调场景下回填配置。
- **OOS 低功耗搜网**: 保留模组默认搜网行为；上层可在低频退避定时器到期时通过 `RestartRegistration()` 执行 `AT+CFUN=0` / `AT+CFUN=1`，不依赖可选的 `ECPLMNS` / `ECCFG` 指令。
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

// detail 携带事件的可读原因，错误事件（如 ErrorInitFailed）会给出具体失败
// 步骤，便于上层直接展示原因；正常状态变更时 detail 为空。
modem->SetNetworkEventCallback([](UartEthModem::UartEthModemEvent ev,
                                   const std::string& detail) {
    ESP_LOGI("app", "modem event: %s %s",
             UartEthModem::GetNetworkEventName(ev), detail.c_str());
});

modem->Start();
```

普通模式保留模组自带的 PLMN 搜网节奏。如果产品需要在长时间 OOS 后主动重启注册，
可在上层退避定时器到期且仍未联网时调用：

```cpp
esp_err_t err = modem->RestartRegistration();  // AT+CFUN=0, AT+CFUN=1
```

`Connecting` 对应 `CEREG=2` 正在搜网；`RegistrationLost` 对应
`CEREG=0/4` 未注册且未搜索 / 状态未知，也用于数据设备或 IP 从可用状态转为 OOS；
恢复调用会始终尝试发送 `CFUN=1`，即使 `CFUN=0` 响应超时，也不会把模组留在非全功能态。

不传参数时，`Start()` 默认使用普通联网模式。需要飞行模式或 RF 测试模式时，通过 `StartMode` 显式选择启动序列：

```cpp
modem->Start(UartEthModem::StartMode::kNormal);  // 普通联网模式，默认值
modem->Start(UartEthModem::StartMode::kFlight);  // 飞行模式，查询模组/SIM信息
modem->Start(UartEthModem::StartMode::kRfTest);  // RF实验室测试模式
```

RF 测试模式的初始化序列为：

```text
AtDetect
AT+ECSIMCFG="SimSimulator",1
AT+ECRST
等待 AT 恢复
AT+CFUN=1
RfTestReady
```

该模式不会执行 SIM/IMEI/注册状态查询，不会启动网卡握手，也不会安装 `iot_eth`。进入 `RfTestReady` 后，除非上层主动调用 `SendAt()`，组件不会继续发送普通应用 AT 指令。

退出 RF 测试模式时调用 `ExitRfTestMode()`：

```text
AT+ECSIMCFG="SimSimulator",0
AT+ECRST
等待 AT 恢复
AT+CFUN=0
```

该退出流程用于恢复正常 SIM 卡模式、重启模组使配置生效，并将模组切回 `AT+CFUN=0`，避免 RF 测试结束后继续保持全功能态。

## 变更日志 (Changelog)

### [0.6.5] - 2026-09-01
- 新增返回 `std::expected<CellInfo, esp_err_t>` 的 `QueryCellInfo()`；仅在本次 `AT+CEREG?` 返回完整 `stat` / `TAC` / `Cell ID` / `AcT` 时成功，不回退到旧缓存，并保留 AT 失败或响应无效的具体错误。
- 主动查询、注册状态事件和数据路径诊断共用同一套 CEREG 解析，避免不同调用路径对模组响应格式产生偏差。

### [0.6.4] - 2026-09-01
- 正常联网后收到 `ECRDY` 时发布 `ModemReset`，使上层可以拆除失效的数据面并重新执行完整初始化；修复同一驱动对象 `Stop()` 后再次 `Start()` 时残留停止事件导致新任务立即退出的问题。
- 新增 `DiagnoseDataPath()`，主动检查 AT 控制通道、`CEREG` 上报模式/注册状态和 `ECNETDEVCTL` 数据设备状态，供上层在可能丢失复位 URC 时决定是否完整重初始化；`ECNETDEVCTL` 按最后一个状态字段精确解析。
- OOS 恢复改为保留模组默认搜网，并新增 `RestartRegistration()` 供上层使用 `CFUN=0/1` 低频重启注册；普通初始化不再开启 `PlmnSearchPowerLevel=3`。
- 将单一实现文件按生命周期、平台、传输和模组控制职责拆分，保持公开接口与运行行为不变，便于按日志模块定位问题。
- `CEREG=2` 通过 `Connecting` 报告搜网，`CEREG=0/4` 即使从未成功注册也通过 `RegistrationLost` 报告不可用；0.6.2 的 `RequestPlmnSearch()` 与 `IsApplicationManagedPlmnSearchEnabled()` 仅保留源码兼容，固件不再发送 `ECPLMNS`。
- 注册恢复后保持 AT 控制任务存活，延后启动数据设备和 Ethernet link，取得 IP 后才发布 `Connected`。
- `Stop()` 等待全部任务退出，AT mutex 等待可响应停止标志，避免快速 Wi-Fi / 4G 切换时释放仍被任务使用的资源。

### [0.6.0] - 2026-06-28
- **接口变更**: `Start(bool flight_mode)` 替换为 `Start(StartMode mode = StartMode::kNormal)`，非默认模式需显式选择启动模式。
- 新增 `StartMode::kRfTest` 和 `RfTestReady` 事件，用于 RF 实验室测试模式。
- RF 测试模式执行 `AT+ECSIMCFG="SimSimulator",1`、`AT+ECRST`、`AT+CFUN=1` 后保持 AT 传输层，不安装 `iot_eth`，也不发送普通联网初始化指令。
- 新增 `ExitRfTestMode()`，退出 RF 测试模式时恢复正常 SIM 卡模式、重启模组并发送 `AT+CFUN=0`。

### [0.5.0] - 2026-06-07
- **接口变更**: `SetNetworkEventCallback` 的回调签名由 `void(UartEthModemEvent)` 调整为 `void(UartEthModemEvent, const std::string& detail)`。
- `ErrorInitFailed` 等错误事件现在携带具体失败原因（如 `Modem not detected`、`Network registration timeout`、`Handshake timeout` 等），上层无需查看串口日志即可定位失败步骤。

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

实现按故障域拆分为四个翻译单元，公开接口仍统一保留在
`include/uart_eth_modem.h`：

| 文件 | 职责 |
|------|------|
| `uart_eth_modem.cc` | 对象生命周期、公开 API、同步 AT 命令入口 |
| `uart_eth_modem_platform.cc` | UART/GPIO、`iot_eth`/`esp_netif`、中断与资源清理 |
| `uart_eth_modem_transport.cc` | TX/Main 任务、MRDY/SRDY 状态机、帧收发与重组 |
| `uart_eth_modem_control.cc` | Init 任务、AT 响应解析、SIM/PDP/PLMN 与启动序列 |

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
