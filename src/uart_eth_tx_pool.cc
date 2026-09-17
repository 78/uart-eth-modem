// Copyright 2026 Terrence
// SPDX-License-Identifier: Apache-2.0

#include "uart_eth_tx_pool.h"

UartEthTxPool::Slot* UartEthTxPool::Acquire(size_t length, bool synchronous) {
    if (length == 0 || length > kFrameSize) return nullptr;
    for (auto& slot : slots_) {
        if (slot.worker || slot.waiter) continue;
        slot.length = length;
        slot.result = 0;
        slot.completed = false;
        slot.worker = true;
        slot.waiter = synchronous;
        return &slot;
    }
    return nullptr;
}

void UartEthTxPool::Complete(Slot& slot, int32_t result) {
    slot.result = result;
    slot.completed = true;
    slot.worker = false;
}

void UartEthTxPool::ReleaseWaiter(Slot& slot) {
    slot.waiter = false;
}
