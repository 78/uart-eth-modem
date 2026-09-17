// Copyright 2026 Terrence
// SPDX-License-Identifier: Apache-2.0

#include "uart_eth_tx_pool.h"

#include <vector>
#include <cassert>
#include <cstdio>
#include <limits>

int main() {
    for (size_t capacity : {1, 3, 34, 66}) {
    {
        std::vector<UartEthTxPool::Slot> storage(capacity);
        UartEthTxPool pool{storage};
        assert(!pool.Acquire(0, false));
        assert(!pool.Acquire(UartEthTxPool::kFrameSize + 1, false));
        assert(!pool.Acquire(std::numeric_limits<size_t>::max(), false));
        auto* slot = pool.Acquire(UartEthTxPool::kFrameSize, false);
        assert(slot && slot->length == UartEthTxPool::kFrameSize);
        pool.Complete(*slot, -7);
        assert(pool.Acquire(8, false) == slot);
        assert(slot->result == 0 && !slot->completed);
    }
    {
        std::vector<UartEthTxPool::Slot> storage(capacity);
        UartEthTxPool pool{storage};
        auto* slot = pool.Acquire(8, true);
        pool.Complete(*slot, -5);
        assert(!slot->worker && slot->waiter && slot->completed && slot->result == -5);
        assert(pool.Acquire(8, false) != slot);
        pool.ReleaseWaiter(*slot);
        assert(pool.Acquire(8, false) == slot);
    }
    {
        std::vector<UartEthTxPool::Slot> storage(capacity);
        UartEthTxPool pool{storage};
        auto* slot = pool.Acquire(8, true);
        slot->data[0] = 42;
        pool.ReleaseWaiter(*slot);  // Timeout while queued or in flight.
        assert(slot->worker && !slot->waiter && !slot->completed);
        assert(pool.Acquire(8, false) != slot);
        assert(slot->data[0] == 42);
        pool.Complete(*slot, 0);
        assert(pool.Acquire(8, false) == slot);
    }
    {
        std::vector<UartEthTxPool::Slot> storage(capacity);
        UartEthTxPool pool{storage};
        auto* slot = pool.Acquire(8, true);
        pool.Complete(*slot, -1);   // Failed enqueue cancels both owners.
        pool.ReleaseWaiter(*slot);
        assert(pool.Acquire(8, true) == slot);
        assert(slot->worker && slot->waiter && !slot->completed && slot->result == 0);
    }
    {
        std::vector<UartEthTxPool::Slot> storage(capacity);
        UartEthTxPool pool{storage};
        std::vector<UartEthTxPool::Slot*> slots(capacity);
        for (auto& slot : slots) { slot = pool.Acquire(8, true); assert(slot); }
        assert(!pool.Acquire(8, false));
        for (auto* slot : slots) pool.Complete(*slot, 0);
        assert(!pool.Acquire(8, false));  // Completed waiters still own every slot.
        for (auto* slot : slots) pool.ReleaseWaiter(*slot);
        for (auto* slot : slots) assert(pool.Acquire(8, false) == slot);
    }
    }
    puts("TX pool class: 5 ownership scenarios x 4 capacities passed (direct production class)");
}
