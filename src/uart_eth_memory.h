// Copyright 2026 Terrence
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <esp_heap_caps.h>

namespace uart_eth::memory {

// CPU-only TX/reassembly storage. Never fall back to another heap on failure.
inline void* AllocateBuffer(size_t size, bool use_psram) noexcept {
    return heap_caps_malloc(size, MALLOC_CAP_8BIT |
        (use_psram ? MALLOC_CAP_SPIRAM : MALLOC_CAP_INTERNAL));
}

}  // namespace uart_eth::memory
