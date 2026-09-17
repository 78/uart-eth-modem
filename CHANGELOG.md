# Changelog

## [0.7.0] - 2026-09-18
- 依赖 UHCI `^0.4.0`，保持 `iot_eth ^1.1.0` 和 ESP-IDF `>=6.0.1`。
- 新增独立 `UartEthTxPool`，固定槽位分别管理发送任务和等待者的所有权；同步超时不会释放仍在使用的内存。`Config::tx_queue_depth` 默认 32，`use_psram` 默认 false；需要 PSRAM 的产品显式设为 true，分配失败不跨堆回退。
- RX 队列满时延后归还缓冲，停止时排空 RX 持有者；同步发送复用完成信号量，发送时不再逐帧分配数据和通知对象。
- `Stop(timeout_ms = 5000)` 有限等待，超时由现有 MainTask 继续清理，拥有者必须保留对象并重试；新增 `IsStopping()` / `IsStopped()`，`PrepareForShutdown()` 也可返回超时。FIFO 发送支持期限和停止取消。
- 普通/飞行模式缺卡或 SIM 未就绪时默认保留 AT，新增 `IsAtReady()`；继续报告 `ErrorNoSim`，不创建网卡。AT 通信故障仍清理。切卡后通过成功 Stop/Start 或板级重启重新联网。
- AT 检测优先在配置波特率等待冷启动，再尝试备用速率。新增 RX、TX 池、FIFO、停止和 SIM 状态主机回归测试。

## [0.6.5] - 2026-09-01
- 新增返回 `std::expected<CellInfo, esp_err_t>` 的 `QueryCellInfo()`；仅在本次 `AT+CEREG?` 返回完整 `stat` / `TAC` / `Cell ID` / `AcT` 时成功，不回退到旧缓存，并保留 AT 失败或响应无效的具体错误。
- 主动查询、注册状态事件和数据路径诊断共用同一套 CEREG 解析，避免不同调用路径对模组响应格式产生偏差。

## [0.6.4] - 2026-09-01
- 正常联网后收到 `ECRDY` 时发布 `ModemReset`，使上层可以拆除失效的数据面并重新执行完整初始化；修复同一驱动对象 `Stop()` 后再次 `Start()` 时残留停止事件导致新任务立即退出的问题。
- 新增 `DiagnoseDataPath()`，主动检查 AT 控制通道、`CEREG` 上报模式/注册状态和 `ECNETDEVCTL` 数据设备状态，供上层在可能丢失复位 URC 时决定是否完整重初始化；`ECNETDEVCTL` 按最后一个状态字段精确解析。
- OOS 恢复改为保留模组默认搜网，并新增 `RestartRegistration()` 供上层使用 `CFUN=0/1` 低频重启注册；普通初始化不再开启 `PlmnSearchPowerLevel=3`。
- 将单一实现文件按生命周期、平台、传输和模组控制职责拆分，保持公开接口与运行行为不变，便于按日志模块定位问题。
- `CEREG=2` 通过 `Connecting` 报告搜网，`CEREG=0/4` 即使从未成功注册也通过 `RegistrationLost` 报告不可用；0.6.2 的 `RequestPlmnSearch()` 与 `IsApplicationManagedPlmnSearchEnabled()` 仅保留源码兼容，固件不再发送 `ECPLMNS`。
- 注册恢复后保持 AT 控制任务存活，延后启动数据设备和 Ethernet link，取得 IP 后才发布 `Connected`。
- `Stop()` 等待全部任务退出，AT mutex 等待可响应停止标志，避免快速 Wi-Fi / 4G 切换时释放仍被任务使用的资源。

## [0.6.0] - 2026-06-28
- **接口变更**: `Start(bool flight_mode)` 替换为 `Start(StartMode mode = StartMode::kNormal)`，非默认模式需显式选择启动模式。
- 新增 `StartMode::kRfTest` 和 `RfTestReady` 事件，用于 RF 实验室测试模式。
- RF 测试模式执行 `AT+ECSIMCFG="SimSimulator",1`、`AT+ECRST`、`AT+CFUN=1` 后保持 AT 传输层，不安装 `iot_eth`，也不发送普通联网初始化指令。
- 新增 `ExitRfTestMode()`，退出 RF 测试模式时恢复正常 SIM 卡模式、重启模组并发送 `AT+CFUN=0`。

## [0.5.0] - 2026-06-07
- **接口变更**: `SetNetworkEventCallback` 的回调签名由 `void(UartEthModemEvent)` 调整为 `void(UartEthModemEvent, const std::string& detail)`。
- `ErrorInitFailed` 等错误事件现在携带具体失败原因（如 `Modem not detected`、`Network registration timeout`、`Handshake timeout` 等），上层无需查看串口日志即可定位失败步骤。

## [0.4.0] - 2026-04-28
- 新增 `SetPdpContext(apn, pdp_type)` API，允许上层注入 APN 和 PDP 类型；为空时沿用模组默认配置。
- 新增 `GetImsi()` 接口（`AT+CIMI`）。
- 新增 `RequestingPdpContext` 事件：在配置 PDP 之前发出，便于同步回调场景下回填 APN；异步派发的客户端仍需在 `Start()` 之前调用 `SetPdpContext`。
- 优化首次启动流程：波特率切换和 NAT 配置合并到同一次 `AT+ECRST`，缩短开机时长。
- `RunNormalModeInitSequence` 调整为先检查 NAT/baud，再统一进入 `CFUN=1`，去掉冗余的二次进入全功能态。

## [0.1.0] - 2026-01-19
- 初始版本：从项目 `main/hardware/network` 迁移为独立组件。

> 0.1.1 ~ 0.3.5 的中间版本以增量优化为主（飞行模式、ISR 安全性、DMA 缓冲、DNS 缓存管理等），详见 git log。
