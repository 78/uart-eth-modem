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

## 变更日志

版本变更与升级注意事项见 [CHANGELOG.md](CHANGELOG.md)。

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
| `uart_eth_tx_pool.{h,cc}` | 固定 TX 槽位、工作线程与等待者所有权；不依赖 RTOS 或 AT 协议 |
| `uart_eth_modem_control.cc` | Init 任务、AT 响应解析、SIM/PDP/PLMN 与启动序列 |

### 1. 系统架构
驱动采用分层架构：

- **UartEthModem**: 主驱动类，管理状态机、帧协议、AT 命令、初始化序列。包含 `MainTask`（事件循环）、`InitTask`（启动序列）、`TxTask`（串行化发送）。
- **UartUhci**: 独立组件 [`components/uart-uhci`](../uart-uhci/README.md)，提供基于缓冲区池 + GDMA owner 机制的持续 RX，以及同步阻塞的 TX FIFO 写入和 PM 锁管理。

### 2. 状态机
控制 RX DMA 启停和 MRDY/SRDY 信号，共四个状态：`Idle`、`PendingActive`、`Active`、`PendingIdle`。空闲超时 500ms 后从 `Active` 切到 `PendingIdle`，双方都空闲时进入 `Idle` 并释放 PM 锁。

### 3. UART 收发机制
- **RX**：UHCI + GDMA 链表 + idle EOF；DMA owner 与消费者持有状态共同管理缓冲。ISR 入队失败时标记延后归还，MainTask 定期回收，停止时排空所有 RX 持有者后才释放 DMA 池。
- **TX**：直接调用 `uart_ll_write_txfifo` 同步写入（FIFO 满时短延时重试），不占用 GDMA 通道；进入/退出时持/放 PM 锁。

### 发送内存与超时

发送池独立为 `UartEthTxPool`（`include/uart_eth_tx_pool.h`、`src/uart_eth_tx_pool.cc`），负责 `Acquire()`、`Complete()` 和 `ReleaseWaiter()`。它只保存固定槽位及工作线程/等待者各自的所有权；驱动负责整个池对象的内存分配、锁、队列、通知和协议封装。这个职责划分与 [MicroPixel PR #13](https://github.com/78/micropixel/pull/13) 的 `TxPool` 一致。

驱动使用 `std::unique_ptr` 持有池对象，无状态自定义删除器负责析构并调用 `heap_caps_free()`；初始化中途失败时自动回收。编译期断言保证智能指针与裸指针大小一致。队列中的槽位指针仅为借用，不单独释放；正常停止仍先等待工作线程和同步等待者退出，再 `reset()` 池对象。

实例配置示例（GPIO 等参数按板级配置补充）：

```cpp
UartEthModem::Config config;
config.tx_queue_depth = 32;
config.use_psram = true;
auto modem = std::make_unique<UartEthModem>(config);
```

- `Config::tx_queue_depth` 默认 32，表示待发送队列深度；池容量为 `tx_queue_depth + 2`，额外槽位容纳正在发送和已完成但同步调用者尚未释放的帧。只在启动时分配，运行中不扩容。零值或会导致容量/字节数溢出的值使 `Start()` 返回 `ESP_ERR_INVALID_ARG`。
- `Config::use_psram` 默认 `false`，不要求设备具有 PSRAM。设为 `true` 时 TX 池和 1600 字节重组工作区使用 `MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`，否则使用 `MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT`。所选堆分配失败返回 `ESP_ERR_NO_MEM`，不跨堆回退；不存在组件级 PSRAM Kconfig 开关。实例创建时复制配置，后续 Stop/Start 沿用该配置。
- 池元数据与槽位数组在同一次分配中，容量为 `sizeof(UartEthTxPool) + (tx_queue_depth + 2) × sizeof(Slot)`。每槽帧容量仍为 1600 字节。ESP32-S3 的默认池连同重组缓冲共需 56416 字节（未计分配器开销）；`use_psram=true` 时全部位于 PSRAM。队列本身只保存槽位指针，增大队列深度也会增加 FreeRTOS 队列的内部 SRAM 用量。
- 此配置不改变 RX DMA、任务栈、同步对象和 AT 字符串的分配方式。RX DMA 缓冲仍位于内部 SRAM。
- 每次发送不再创建数据缓冲或信号量。同步发送串行复用一个启动时创建的信号量；槽位状态由 `tx_mutex_` 保护，复用已有 `at_mutex_` 串行化 AT 和握手的同步等待者。
- 队列/池满时，异步发送返回 `ESP_ERR_NO_MEM`；同步发送最多重试 100 ms。发送载荷超过 1596 字节返回 `ESP_ERR_INVALID_SIZE`，不负责自动拆帧。
- 同步等待仍为 2 秒。超时只撤销等待者的持有权；发送任务仍持有槽位，完成后才允许复用。超时不表示该帧已取消发送。
- 停止时 TX 任务取消排队帧，唤醒同步调用者；MainTask 等待 TX/控制任务退出及停止通知发布完毕，再清理 netif、缓冲池、GPIO 和 UART。本次没有增加 AT-only 模式，也没有改造 AT 字符串接口。

主机回归：`python3 tests/run_host_tests.py`，使用生产方法与模拟 FreeRTOS 队列/信号量，并启用 ASan/UBSan。它不替代实机的吞吐、功耗与内部堆峰值验证。

### 停止与对象所有权

- `Stop()` 默认等待预算为 5000 ms，涵盖工作线程退出、AT/激活互斥锁和 netif 清理；耗时清理复用现有 MainTask，不新增任务或栈。
- 返回 `ESP_OK` 才可以销毁对象或重启。`ESP_ERR_TIMEOUT` 表示停止已请求、清理尚未完成，必须保留对象；可调用 `Stop(0)` 非阻塞轮询，或再次 `Stop(ms)` 有限等待。`IsStopping()` / `IsStopped()` 用于查询状态。
- 停止期间拒绝新发送和 `Start()`。Stop 位保持置位，直到下一次 Start 才清除，避免一个等待者消费取消通知后其他任务永久等待。不得从驱动自己的工作任务/直接事件回调中同步 Stop（返回 `ESP_ERR_INVALID_STATE`）；应调度给持有者处理。
- 调用方串行执行 Start/销毁与其它生命周期操作。析构函数不能报告超时：最后一次 Stop 失败会触发 `ESP_ERROR_CHECK`，绝不释放仍被工作任务访问的对象。因此正常业务应先检查 Stop 结果，超时后保留并重试，不能直接 `reset()`。
- `PrepareForShutdown(3000)` 在保持 AT 通道的同时阻止后续数据激活；互斥锁等待也有时限，失败返回 `ESP_ERR_TIMEOUT`，阻止激活的状态仍保留。它是板级优雅关闭前的独立步骤，CFUN/RF 退出命令时间不计入 Stop 的等待预算。
- FIFO TX 总等待预算为 1000 ms，覆盖填充 FIFO 和末字节发完；停止标志可提前取消。失败复位残留 TX FIFO 并释放 PM 锁。

停止回归：`python3 tests/run_stop_tests.py`；另可使用 `SANITIZERS=thread` 检查线程竞态。覆盖线程/清理阻塞、重复及并发 Stop、发布唤醒期间的资源保护、AT/激活锁超时、取消通知保持和安全重试。

### 缺卡时默认保留 AT

普通模式和飞行模式检测到 `+CME ERROR: 10`（缺卡），或重试后仍收到 SIM 未就绪的 `+CPIN` 回复时，默认保留已经检测成功的 AT 通道，无需配置开关或新启动模式。此时先设置状态，再报告 `ErrorNoSim`：`IsAtReady()` 为 true，`IsInitialized()` 为 false，`GetNetif()` 为 null。Init 任务退出，原有 RX/TX 任务继续处理查询和切卡命令，不配置 PDP、不创建网卡或启动数据激活。

AT 波特率检测失败、SIM 查询通信超时/错误仍触发初始化失败清理；不能把无响应当作缺卡。停止会立即撤销 AT 就绪状态。`ErrorNoSim` 的接入方应保留驱动并将查询/切卡工作安排到自己的任务；如果它主动调用 Stop，通道仍会按请求关闭。

切卡后需要成功 Stop 再 Start，或由板级策略重启整机，重新执行联网初始化。本次不自动检测插卡、不新增 `StartNetwork()`，也不改 AT 字符串的存储或长度限制。`iot_eth` 依赖保持 `^1.1.0`。

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
