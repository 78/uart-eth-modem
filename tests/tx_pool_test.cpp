#include "uart_eth_tx_pool.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <future>
#include <functional>
#include <memory>
#include <limits>
#include <mutex>
#include <new>
#include <thread>
#include <vector>
#include <utility>
using esp_err_t = int;
using UBaseType_t = uint32_t;
using uart_port_t = int;
using gpio_num_t = int;
constexpr int UART_NUM_1 = 1, GPIO_NUM_NC = -1;
using TickType_t = uint32_t;
using BaseType_t = int;
constexpr int ESP_OK = 0, ESP_ERR_NO_MEM = 1, ESP_ERR_INVALID_ARG = 2,
    ESP_ERR_INVALID_SIZE = 3, ESP_ERR_INVALID_STATE = 4, ESP_ERR_TIMEOUT = 5;
constexpr int MALLOC_CAP_SPIRAM = 1, MALLOC_CAP_8BIT = 2, MALLOC_CAP_INTERNAL = 4,
    pdTRUE = 1, pdFALSE = 0;
#define IRAM_ATTR
#define pdMS_TO_TICKS(ms) (ms)
std::atomic<int> allocations{0}, semaphore_creates{0}, semaphore_deletes{0};
std::atomic<int> live_pool_allocations{0};
bool fail_heap = false, fail_semaphore = false, fail_queue = false;
size_t test_depth = 32, last_allocation_size = 0;
bool test_psram = true, expected_psram = true;
std::function<void()> wait_hook;
void* heap_caps_malloc(size_t size, int caps) {
    assert(caps == ((expected_psram ? MALLOC_CAP_SPIRAM : MALLOC_CAP_INTERNAL) | MALLOC_CAP_8BIT));
    last_allocation_size = size;
    ++allocations;
    void* storage = fail_heap ? nullptr : malloc(size);
    if (storage) ++live_pool_allocations;
    return storage;
}
void heap_caps_free(void* storage) {
    assert(storage && live_pool_allocations > 0);
    --live_pool_allocations;
    free(storage);
}
#include "uart_eth_memory.h"
struct Queue { size_t capacity; std::mutex mutex; std::deque<std::vector<uint8_t>> items; size_t item_size; };
using QueueHandle_t = Queue*;
QueueHandle_t xQueueCreate(size_t count, size_t size) { return fail_queue ? nullptr : new Queue{count, {}, {}, size}; }
int xQueueSend(QueueHandle_t q, const void* item, TickType_t wait) {
    assert(wait == 0);
    std::lock_guard lock(q->mutex);
    if (q->items.size() == q->capacity) return pdFALSE;
    const auto* bytes = static_cast<const uint8_t*>(item);
    q->items.emplace_back(bytes, bytes + q->item_size);
    return pdTRUE;
}
int xQueueSendFromISR(QueueHandle_t q, const void* item, BaseType_t*) { return xQueueSend(q, item, 0); }
int xQueueReceive(QueueHandle_t q, void* item, TickType_t wait) {
    assert(wait == 0);
    std::lock_guard lock(q->mutex);
    if (q->items.empty()) return pdFALSE;
    memcpy(item, q->items.front().data(), q->item_size);
    q->items.pop_front();
    return pdTRUE;
}
void vQueueDelete(QueueHandle_t q) { delete q; }
struct Semaphore { std::mutex mutex; std::condition_variable condition; bool signaled = false; };
using SemaphoreHandle_t = Semaphore*;
SemaphoreHandle_t xSemaphoreCreateBinary() { ++semaphore_creates; return fail_semaphore ? nullptr : new Semaphore; }
int xSemaphoreTake(SemaphoreHandle_t s, TickType_t wait) {
    if (wait && wait_hook) { auto hook = std::exchange(wait_hook, {}); hook(); return pdFALSE; }
    std::unique_lock lock(s->mutex);
    if (!s->condition.wait_for(lock, std::chrono::milliseconds(wait), [&] { return s->signaled; })) return pdFALSE;
    s->signaled = false;
    return pdTRUE;
}
void xSemaphoreGive(SemaphoreHandle_t s) {
    std::lock_guard lock(s->mutex);
    s->signaled = true;
    s->condition.notify_one();
}
void vSemaphoreDelete(SemaphoreHandle_t s) { ++semaphore_deletes; delete s; }
TickType_t xTaskGetTickCount() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void vTaskDelay(TickType_t ticks) { std::this_thread::sleep_for(std::chrono::milliseconds(ticks)); }
struct UartUhci {
    struct RxBuffer { bool deferred = false; };
    struct RxEventData { RxBuffer* buffer; size_t recv_size; };
    void DeferReturnBuffer(RxBuffer* b) { b->deferred = true; }
};
class UartEthModem {
public:
    // These declarations and all method bodies below are read from production.
    // @TYPES@
    esp_err_t InitTxPool();
    void DeinitTxPool();
    esp_err_t QueueTxFrame(const uint8_t*, size_t, FrameType, bool, TxFrame**);
    void CompleteTxFrame(TxFrame*, esp_err_t);
    esp_err_t EnqueueTxFrame(const uint8_t*, size_t);
    esp_err_t SendFrame(const uint8_t*, size_t, FrameType);
    static bool UhciRxCallbackStatic(const UartUhci::RxEventData&, void*);
    enum class EventType { RxData };
    struct Event { EventType type; UartUhci::RxBuffer* rx_buffer; };
    Config config_;
    QueueHandle_t tx_queue_ = nullptr, event_queue_ = nullptr;
    TxPoolPtr tx_pool_;
    SemaphoreHandle_t tx_done_ = nullptr;
    std::mutex tx_mutex_;
    std::timed_mutex at_mutex_;
    std::atomic<bool> stop_flag_{false};
    std::atomic<uint8_t> seq_no_{0};
    UartUhci uart_uhci_;
};
// @METHODS@
struct Fixture : UartEthModem {
    Fixture(size_t depth = test_depth, bool use_psram = test_psram) {
        config_.tx_queue_depth = depth;
        config_.use_psram = use_psram;
        expected_psram = use_psram;
        assert(InitTxPool() == ESP_OK);
        assert(tx_queue_->capacity == depth);
        assert(last_allocation_size == sizeof(UartEthTxPool) + (depth + 2) * sizeof(TxFrame));
    }
    ~Fixture() { stop_flag_ = true; DeinitTxPool(); }
    // Both real callers (SendAt and handshake) hold the existing AT mutex.
    esp_err_t send(const uint8_t* data, size_t length, FrameType type) {
        std::lock_guard<std::timed_mutex> lock(at_mutex_);
        return SendFrame(data, length, type);
    }
    TxFrame* pop() {
        TxFrame* frame = nullptr;
        auto deadline = xTaskGetTickCount() + 1000;
        while (!xQueueReceive(tx_queue_, &frame, 0)) { assert(xTaskGetTickCount() < deadline); vTaskDelay(1); }
        assert(frame); return frame;
    }
};
const uint8_t payload[] = {1, 2, 3, 4};
int main(int argc, char** argv) {
    assert(argc == 3);
    assert(UartEthModem::Config{}.tx_queue_depth == 32);
    assert(!UartEthModem::Config{}.use_psram);
    test_depth = std::strtoul(argv[1], nullptr, 10);
    test_psram = expected_psram = std::strtoul(argv[2], nullptr, 10) != 0;
    {
        Fixture f;
        int start_alloc = allocations, start_sem = semaphore_creates;
        assert(f.EnqueueTxFrame(payload, 1597) == ESP_ERR_INVALID_SIZE);
        assert(f.EnqueueTxFrame(nullptr, 1) == ESP_ERR_INVALID_ARG);
        for (unsigned i = 0; i < f.config_.tx_queue_depth; ++i) assert(f.EnqueueTxFrame(payload, 4) == ESP_OK);
        assert(f.EnqueueTxFrame(payload, 4) == ESP_ERR_NO_MEM);
        auto* first = f.pop();
        assert(first->length == 8 && memcmp(first->data + 4, payload, 4) == 0);
        f.CompleteTxFrame(first, ESP_OK);
        for (unsigned i = 1; i < f.config_.tx_queue_depth; ++i) f.CompleteTxFrame(f.pop(), ESP_OK);
        for (int i = 0; i < 10000; ++i) {
            assert(f.EnqueueTxFrame(payload, 4) == ESP_OK);
            f.CompleteTxFrame(f.pop(), ESP_OK);
        }
        assert(allocations == start_alloc && semaphore_creates == start_sem);
    }
    {
        Fixture f;
        auto result = std::async(std::launch::async, [&] { return f.send(payload, 4, UartEthModem::FrameType::kAtCommand); });
        auto* slot = f.pop();
        f.CompleteTxFrame(slot, ESP_OK);
        assert(result.get() == ESP_OK);
        assert(!slot->worker && !slot->waiter);
    }
    {
        Fixture f;
        // Force wait expiry while the worker still owns the first slot.
        wait_hook = [] {};
        assert(f.send(payload, 4, UartEthModem::FrameType::kAtCommand) == ESP_ERR_TIMEOUT);
        auto* late = f.pop();
        assert(late->worker && !late->waiter);
        assert(f.EnqueueTxFrame(payload, 4) == ESP_OK);
        auto* other = f.pop();
        assert(other != late);
        f.CompleteTxFrame(late, ESP_OK);
        assert(!late->worker && !late->waiter && !f.tx_done_->signaled);
        f.CompleteTxFrame(other, ESP_OK);
        // Next synchronous request must not see the old completion.
        wait_hook = [] {};
        assert(f.send(payload, 4, UartEthModem::FrameType::kAtCommand) == ESP_ERR_TIMEOUT);
        f.CompleteTxFrame(f.pop(), ESP_OK);
    }
    {
        Fixture f;
        // Completion wins immediately after timeout; it must be read from slot.
        wait_hook = [&] { f.CompleteTxFrame(f.pop(), ESP_ERR_INVALID_STATE); };
        assert(f.send(payload, 4, UartEthModem::FrameType::kAtCommand) == ESP_ERR_INVALID_STATE);
        wait_hook = [] {};
        assert(f.send(payload, 4, UartEthModem::FrameType::kAtCommand) == ESP_ERR_TIMEOUT);
        f.CompleteTxFrame(f.pop(), ESP_OK);
    }
    {
        Fixture f;
        auto result = std::async(std::launch::async, [&] { return f.send(payload, 4, UartEthModem::FrameType::kAtCommand); });
        // Leave synchronous frame queued; cleanup must cancel it and join waiter.
        while (true) {
            std::lock_guard lock(f.tx_queue_->mutex);
            if (!f.tx_queue_->items.empty()) break;
        }
        f.stop_flag_ = true;
        f.DeinitTxPool();
        assert(result.get() == ESP_ERR_INVALID_STATE);
        assert(f.EnqueueTxFrame(payload, 4) == ESP_ERR_INVALID_STATE);
    }
    {
        Fixture f;
        std::atomic<unsigned> completed{0};
        std::thread worker([&] { for (int i = 0; i < 2; ++i) { f.CompleteTxFrame(f.pop(), ESP_OK); ++completed; } });
        auto a = std::async(std::launch::async, [&] { return f.send(payload, 4, UartEthModem::FrameType::kAtCommand); });
        auto b = std::async(std::launch::async, [&] { return f.send(payload, 4, UartEthModem::FrameType::kAtCommand); });
        assert(a.get() == ESP_OK && b.get() == ESP_OK);
        worker.join(); assert(completed == 2);
    }
    {
        UartEthModem f;
        f.config_.use_psram = test_psram;
        fail_heap = true;
        const int start_alloc = allocations;
        assert(f.InitTxPool() == ESP_ERR_NO_MEM && !f.tx_pool_);
        assert(allocations == start_alloc + 1);  // No fallback allocation.
        assert(live_pool_allocations == 0);
        fail_heap = false; fail_semaphore = true;
        assert(f.InitTxPool() == ESP_ERR_NO_MEM && !f.tx_pool_);
        assert(live_pool_allocations == 0);
        fail_semaphore = false;
    }
    {
        UartEthModem f;
        f.event_queue_ = xQueueCreate(1, sizeof(UartEthModem::Event));
        UartUhci::RxBuffer first, dropped;
        f.UhciRxCallbackStatic({&first, 16}, &f);
        f.UhciRxCallbackStatic({&dropped, 16}, &f);
        assert(!first.deferred && dropped.deferred);
        vQueueDelete(f.event_queue_);
    }
    {
        Fixture f;
        std::vector<UartEthModem::TxFrame*> held;
        // Drain the queue without completing work: pool exhaustion must be
        // independent of queue capacity and must not allocate more storage.
        const int start_alloc = allocations;
        for (size_t i = 0; i < (f.config_.tx_queue_depth + 2); ++i) {
            assert(f.EnqueueTxFrame(payload, 4) == ESP_OK);
            held.push_back(f.pop());
        }
        assert(f.EnqueueTxFrame(payload, 4) == ESP_ERR_NO_MEM);
        assert(allocations == start_alloc);
        for (auto* frame : held) f.CompleteTxFrame(frame, ESP_OK);
    }
    {
        UartEthModem f;
        const int start_alloc = allocations;
        for (size_t invalid : {size_t{0}, std::numeric_limits<size_t>::max(),
                               std::numeric_limits<size_t>::max() / sizeof(UartEthTxPool::Slot)}) {
            f.config_.tx_queue_depth = invalid;
            assert(f.InitTxPool() == ESP_ERR_INVALID_ARG);
            assert(!f.tx_pool_ && !f.tx_queue_ && !f.tx_done_);
        }
        assert(allocations == start_alloc);
    }
    {
        UartEthModem f;
        f.config_.use_psram = test_psram;
        fail_queue = true;
        assert(f.InitTxPool() == ESP_ERR_NO_MEM);
        assert(!f.tx_pool_ && !f.tx_queue_ && !f.tx_done_);
        assert(live_pool_allocations == 0);
        fail_queue = false;
        assert(f.InitTxPool() == ESP_OK);  // Retry after allocation failure.
        f.DeinitTxPool();
        assert(f.InitTxPool() == ESP_OK);  // Restart retains instance config.
        f.DeinitTxPool();
    }
    {
        Fixture small(1, true), large(64, false);
        assert(live_pool_allocations == 2);
        assert(small.EnqueueTxFrame(payload, 4) == ESP_OK);
        assert(small.EnqueueTxFrame(payload, 4) == ESP_ERR_NO_MEM);
        for (int i = 0; i < 64; ++i) assert(large.EnqueueTxFrame(payload, 4) == ESP_OK);
        small.CompleteTxFrame(small.pop(), ESP_OK);
        for (int i = 0; i < 64; ++i) large.CompleteTxFrame(large.pop(), ESP_OK);
    }
    // One failed creation above has no semaphore to delete.
    assert(semaphore_creates == semaphore_deletes + 1);
    assert(live_pool_allocations == 0);
    puts("TX/RX queue: 12 scenarios passed (production methods)");
}
