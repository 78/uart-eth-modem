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

## 变更日志 (Changelog)

### [0.1.0] - 2026-01-19
- 初始版本：从项目 `main/hardware/network` 迁移为独立组件。

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
