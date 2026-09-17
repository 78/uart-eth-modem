// Copyright 2026 Terrence
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <span>
#include <cstddef>
#include <cstdint>

// Fixed storage and ownership only. The modem provides initialized slots whose
// lifetime covers the pool and serializes access. No queue, RTOS or AT dependency.
class UartEthTxPool {
public:
    static constexpr size_t kFrameSize = 1600;

    struct Slot {
        uint8_t data[kFrameSize];
        size_t length = 0;
        int32_t result = 0;
        bool worker = false;
        bool waiter = false;
        bool completed = false;
    };

    explicit UartEthTxPool(std::span<Slot> slots) : slots_(slots) {}
    UartEthTxPool(const UartEthTxPool&) = delete;
    UartEthTxPool& operator=(const UartEthTxPool&) = delete;

    // Acquire gives ownership to a producer/worker and optionally its waiter.
    // Returns nullptr for invalid length or exhaustion; never allocates.
    Slot* Acquire(size_t length, bool synchronous);
    // End the worker's ownership, including queue failure or stop cancellation.
    void Complete(Slot& slot, int32_t result);
    // Timeout releases only the waiter: the queued/in-flight worker remains.
    void ReleaseWaiter(Slot& slot);

private:
    std::span<Slot> slots_;
};
