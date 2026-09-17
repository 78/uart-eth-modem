#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <future>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
using esp_err_t = int;
using EventBits_t = uint32_t;
using TickType_t = uint32_t;
using TaskHandle_t = int;
constexpr int ESP_OK = 0, ESP_ERR_TIMEOUT = 1, ESP_ERR_INVALID_STATE = 2, ESP_FAIL = 3,
    ESP_ERR_NOT_FOUND = 4, ESP_ERR_INVALID_RESPONSE = 5;
constexpr int pdFALSE = 0, pdTRUE = 1, configTICK_RATE_HZ = 1000;
constexpr TickType_t portMAX_DELAY = UINT32_MAX;
#define pdMS_TO_TICKS(ms) (ms)
#define ESP_LOGD(...)
#define ESP_LOGI(...)
#define ESP_LOGW(...)
#define ESP_LOGE(...)
thread_local TaskHandle_t current_task = 99;
TaskHandle_t xTaskGetCurrentTaskHandle() { return current_task; }
void vTaskDelay(TickType_t) {}  // SIM retry delays do not need real elapsed time.
int baud_result = ESP_OK;
esp_err_t uart_set_baudrate(int, int) { return baud_result; }
const char* esp_err_to_name(esp_err_t) { return "fake error"; }
int64_t esp_timer_get_time() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct EventGroup { std::mutex lock; std::condition_variable cv; EventBits_t bits = 0; };
EventBits_t xEventGroupGetBits(EventGroup* g) { std::lock_guard l(g->lock); return g->bits; }
EventBits_t xEventGroupSetBits(EventGroup* g, EventBits_t bits) {
    std::lock_guard l(g->lock); g->bits |= bits; g->cv.notify_all(); return g->bits;
}
void xEventGroupClearBits(EventGroup* g, EventBits_t bits) { std::lock_guard l(g->lock); g->bits &= ~bits; }
EventBits_t xEventGroupWaitBits(EventGroup* g, EventBits_t bits, int clear, int all, TickType_t wait) {
    std::unique_lock l(g->lock);
    const auto ready = [&] { return all ? (g->bits & bits) == bits : (g->bits & bits) != 0; };
    if (wait == portMAX_DELAY) g->cv.wait(l, ready);
    else g->cv.wait_for(l, std::chrono::milliseconds(wait), ready);
    auto result = g->bits;
    if (clear && ready()) g->bits &= ~bits;
    return result;
}
struct Queue {
    size_t item_size;
    std::mutex lock;
    std::condition_variable cv;
    std::deque<std::vector<uint8_t>> items;
    unsigned sends = 0;
    bool live = true;
    std::function<void()> before_send;
};
int xQueueSend(Queue* q, const void* item, TickType_t wait) {
    assert(wait == 0);
    if (q->before_send) q->before_send();
    std::lock_guard l(q->lock);
    assert(q->live);
    ++q->sends;
    const auto* b = static_cast<const uint8_t*>(item);
    q->items.emplace_back(b, b + q->item_size);
    q->cv.notify_all();
    return pdTRUE;
}
int xQueueReceive(Queue* q, void* item, TickType_t wait) {
    std::unique_lock l(q->lock);
    assert(q->live);
    const auto ready = [&] { return !q->items.empty(); };
    if (wait == portMAX_DELAY) q->cv.wait(l, ready);
    else q->cv.wait_for(l, std::chrono::milliseconds(wait), ready);
    if (q->items.empty()) return pdFALSE;
    memcpy(item, q->items.front().data(), q->item_size);
    q->items.pop_front(); return pdTRUE;
}
struct UartUhci {
    bool stopped = false;
    unsigned returned = 0;
    void ReclaimDeferredBuffers() {}
    bool IsReceiving() const { return !stopped; }
    void StopReceive() { stopped = true; }
    void ReturnBuffer(void*) { ++returned; }
};
struct UartEthModem {
    // @TYPES@
    enum class WorkingState { Idle, Active };
    enum class MrdyLevel { High };
    enum class EventType { Stop, RxData };
    enum class FrameType { kAtCommand };
    enum class StartMode { kNormal, kFlight, kRfTest };
    enum class UartEthModemEvent { ErrorNoSim, ErrorInitFailed, InFlightMode, Connecting };
    struct Config { int uart_num = 0, baud_rate = 3000000; } config_;
    int detect_baud_rate_ = 0;
    static constexpr uint32_t kModemSleepTimeoutS = 3;
    StartMode start_mode_ = StartMode::kNormal;
    struct { int stat = 0; } cell_info_;
    struct CellInfoResult {
        explicit operator bool() const { return true; }
        esp_err_t error() const { return ESP_OK; }
    };
    std::vector<UartEthModemEvent> notifications;
    std::vector<std::string> commands;
    std::function<esp_err_t(const std::string&, std::string&)> reply;
    std::function<void(UartEthModemEvent)> on_event;
    std::atomic<unsigned> netif_creates{0};
    unsigned activations = 0, info_queries = 0, pdp_configs = 0;
    struct Event { EventType type; void* rx_buffer; };
    struct TxFrame {};
    EventGroup group;
    EventGroup* event_group_ = &group;
    Queue events{sizeof(Event)}, tx{sizeof(TxFrame*)};
    Queue* event_queue_ = &events;
    Queue* tx_queue_ = &tx;
    TaskHandle_t main_task_ = 1, init_task_ = 2, tx_task_ = 3;
    std::atomic<bool> stop_flag_{false}, initialized_{true}, initializing_{false}, data_activation_blocked_{false};
    std::atomic<WorkingState> working_state_{WorkingState::Idle};
    std::timed_mutex data_activation_mutex_;
    std::timed_mutex at_mutex_;
    std::atomic<bool> handshake_done_{false}, debug_enabled_{false};
    bool waiting_for_at_response_ = false;
    std::string at_command_response_;
    std::function<void()> frame_sent;
    UartUhci uart_uhci_;
    std::promise<void> cleanup_entered, cleanup_release;
    std::atomic<bool> resources_live{true};
    UartEthModem() { xEventGroupSetBits(event_group_, kEventAtReady); }
    void RequestStop();
    esp_err_t Stop(uint32_t timeout_ms = 5000);
    esp_err_t PrepareForShutdown(uint32_t timeout_ms = 3000);
    esp_err_t SendAt(const std::string&, std::string&, uint32_t = 1000);
    esp_err_t SendFrame(const uint8_t* data, size_t size, FrameType) {
        if (frame_sent) frame_sent();
        if (reply) {
            std::string command(reinterpret_cast<const char*>(data), size);
            if (!command.empty() && command.back() == '\r') command.pop_back();
            commands.push_back(command);
            const auto result = reply(command, at_command_response_);
            if (result != ESP_OK) return result;  // Simulated transport failure.
            xEventGroupSetBits(event_group_, kEventAtResponse);
        }
        return ESP_OK;
    }
    void InitTaskRun();
    esp_err_t AtDetect();
    esp_err_t CheckSimCard();
    esp_err_t RunNormalModeInitSequence();
    esp_err_t RunFlightModeInitSequence();
    esp_err_t RunRfTestModeInitSequence() { return ESP_FAIL; }
    esp_err_t SendAtWithRetry(const std::string& cmd, std::string& response, uint32_t ms, int) {
        return SendAt(cmd, response, ms);
    }
    void SetNetworkEvent(UartEthModemEvent event, const std::string& = {}) {
        notifications.push_back(event);
        if (on_event) on_event(event);
    }
    void QueryModemInfo() { ++info_queries; }
    void ConfigurePdp() { ++pdp_configs; }
    CellInfoResult QueryCellInfo() { return {}; }
    esp_err_t InitIotEth() { ++netif_creates; return ESP_OK; }
    esp_err_t ActivateDataNetwork() { ++activations; return ESP_OK; }
    void MainTaskRun();
    void SetMrdy(MrdyLevel) {}
    TickType_t CalculateNextTimeout() { return portMAX_DELAY; }
    void HandleEvent(Event e) { if (e.type == EventType::RxData) uart_uhci_.ReturnBuffer(e.rx_buffer); }
    void HandleIdleTimeout() {}
    void EnterIdleState() { working_state_ = WorkingState::Idle; }
    void CleanupResources(bool all) {
        assert(all && uart_uhci_.stopped);
        const auto bits = xEventGroupGetBits(event_group_);
        assert((bits & (kEventTxTaskDone | kEventInitTaskDone | kEventStopPublished)) ==
               (kEventTxTaskDone | kEventInitTaskDone | kEventStopPublished));
        cleanup_entered.set_value();
        cleanup_release.get_future().wait();  // Simulate blocked netif/mutex cleanup.
        { std::lock_guard l(events.lock); events.live = false; }
        { std::lock_guard l(tx.lock); tx.live = false; }
        resources_live = false;
    }
};
// @METHODS@

void prepare_init(UartEthModem& m, std::string sim_reply) {
    m.initialized_ = false;
    m.initializing_ = true;
    xEventGroupClearBits(m.event_group_, m.kEventAtReady);
    xEventGroupSetBits(m.event_group_, m.kEventStart);
    m.reply = [sim_reply](const std::string& cmd, std::string& response) {
        response = cmd == "AT+CPIN?" ? sim_reply
            : cmd == "AT+ECNETCFG?" ? "+ECNETCFG: \"nat\",1\r\nOK\r\n"
            : "OK\r\n";
        return ESP_OK;
    };
}

int main() {
    {
        UartEthModem m;
        xEventGroupSetBits(m.event_group_, m.kEventShutdownComplete);
        assert(m.Stop(0) == ESP_OK && m.IsStopped() && !m.IsStopping());
        assert(m.events.sends == 0);
    }
    {
        UartEthModem m;
        for (int handle : {m.main_task_, m.init_task_, m.tx_task_}) {
            current_task = handle;
            assert(m.Stop(10) == ESP_ERR_INVALID_STATE);
            assert(!m.stop_flag_);
        }
        current_task = 99;
    }
    {
        UartEthModem m;
        // An unresponsive worker cannot force Stop to wait forever or free it.
        const auto start = std::chrono::steady_clock::now();
        assert(m.Stop(20) == ESP_ERR_TIMEOUT);
        assert(std::chrono::steady_clock::now() - start < 250ms);
        assert(m.IsStopping() && m.resources_live && !m.initialized_);
        assert(m.Stop(0) == ESP_ERR_TIMEOUT);
        assert(m.events.sends == 1 && m.tx.sends == 1);
    }
    {
        UartEthModem m;
        auto a = std::async(std::launch::async, [&] { return m.Stop(5); });
        auto b = std::async(std::launch::async, [&] { return m.Stop(5); });
        assert(a.get() == ESP_ERR_TIMEOUT && b.get() == ESP_ERR_TIMEOUT);
        assert(m.events.sends == 1 && m.tx.sends == 1);
    }
    {
        UartEthModem m;
        std::unique_lock active(m.data_activation_mutex_);
        auto wait = std::async(std::launch::async, [&] { return m.PrepareForShutdown(10); });
        assert(wait.get() == ESP_ERR_TIMEOUT && m.data_activation_blocked_);
        active.unlock();
        assert(m.PrepareForShutdown(0) == ESP_OK);
    }
    {
        UartEthModem m;
        // Main runs the real coordinator method; cleanup must wait for BOTH
        // workers and for the wakeup publisher, then survive multiple retries.
        UartEthModem::Event rx{UartEthModem::EventType::RxData, &m};
        xQueueSend(m.event_queue_, &rx, 0);
        auto entered = m.cleanup_entered.get_future();
        auto main = std::async(std::launch::async, [&] { m.MainTaskRun(); });
        assert(m.Stop(0) == ESP_ERR_TIMEOUT);
        assert(entered.wait_for(10ms) == std::future_status::timeout);
        xEventGroupSetBits(m.event_group_, m.kEventTxTaskDone);
        assert(entered.wait_for(10ms) == std::future_status::timeout);
        xEventGroupSetBits(m.event_group_, m.kEventInitTaskDone);
        assert(entered.wait_for(1s) == std::future_status::ready);
        const auto start = std::chrono::steady_clock::now();
        assert(m.Stop(20) == ESP_ERR_TIMEOUT && m.resources_live);
        assert(std::chrono::steady_clock::now() - start < 250ms);
        assert(xEventGroupGetBits(m.event_group_) & m.kEventStop);
        m.cleanup_release.set_value();
        main.get();
        assert(m.Stop(0) == ESP_OK && !m.resources_live && m.IsStopped());
        assert(m.uart_uhci_.returned == 1);
        // Retrying after cleanup must not touch queues already freed.
        assert(m.Stop(10) == ESP_OK);
    }
    {
        UartEthModem m;
        std::unique_lock busy(m.at_mutex_);
        auto waiter = std::async(std::launch::async, [&] {
            std::string response;
            return m.SendAt("AT", response, 10);
        });
        assert(waiter.get() == ESP_ERR_TIMEOUT);
    }
    {
        UartEthModem m;
        std::promise<void> sent;
        m.frame_sent = [&] { sent.set_value(); };
        auto waiter = std::async(std::launch::async, [&] {
            std::string response;
            return m.SendAt("AT", response, 10000);
        });
        sent.get_future().wait();
        assert(m.Stop(0) == ESP_ERR_TIMEOUT);
        assert(waiter.wait_for(250ms) == std::future_status::ready);
        assert(waiter.get() == ESP_ERR_INVALID_STATE);
        // The AT waiter must not consume Stop and strand TX/control waiters.
        assert(xEventGroupGetBits(m.event_group_) & m.kEventStop);
        assert(!m.waiting_for_at_response_);
        std::string response;
        assert(m.SendAt("AT", response, 100) == ESP_ERR_INVALID_STATE);
    }
    {
        UartEthModem m;
        std::promise<void> publishing, release_publisher;
        m.tx.before_send = [&] {
            publishing.set_value();
            release_publisher.get_future().wait();
        };
        auto entered = m.cleanup_entered.get_future();
        auto main = std::async(std::launch::async, [&] { m.MainTaskRun(); });
        auto stop = std::async(std::launch::async, [&] { return m.Stop(20); });
        publishing.get_future().wait();
        xEventGroupSetBits(m.event_group_, m.kEventInitTaskDone | m.kEventTxTaskDone);
        // Both workers are gone, but a preempted publisher still uses queues.
        assert(entered.wait_for(10ms) == std::future_status::timeout);
        assert(m.Stop(0) == ESP_ERR_TIMEOUT && m.resources_live);
        release_publisher.set_value();
        assert(entered.wait_for(1s) == std::future_status::ready);
        assert(stop.get() == ESP_ERR_TIMEOUT);
        m.cleanup_release.set_value();
        main.get();
        assert(m.Stop(0) == ESP_OK);
    }
    puts("9 stop/AT lifecycle scenarios passed");
    for (auto mode : {UartEthModem::StartMode::kNormal, UartEthModem::StartMode::kFlight}) {
        for (const auto* sim : {"+CME ERROR: 10\r\n", "+CPIN: SIM PIN\r\nOK\r\n"}) {
            UartEthModem m;
            prepare_init(m, sim);
            m.start_mode_ = mode;
            m.on_event = [&](auto event) {
                if (event == UartEthModem::UartEthModemEvent::ErrorNoSim) {
                    // Readiness is published before notifying the owner.
                    assert(m.IsAtReady() && !m.IsInitialized() && !m.initializing_);
                }
            };
            m.InitTaskRun();
            assert(m.IsAtReady() && !m.IsInitialized() && !m.IsStopping());
            assert(m.notifications == std::vector{UartEthModem::UartEthModemEvent::ErrorNoSim});
            assert(m.netif_creates == 0 && m.activations == 0 && m.pdp_configs == 0);
            assert(xEventGroupGetBits(m.event_group_) & m.kEventInitTaskDone);
            std::string response;
            assert(m.SendAt("AT+ECSIMCFG?", response, 100) == ESP_OK);
            assert(m.SendAt("AT+ECSIMCFG=SimSlot,1", response, 100) == ESP_OK);
            // Real cleanup coordinator must still stop this retained channel.
            auto main = std::async(std::launch::async, [&] { m.MainTaskRun(); });
            xEventGroupSetBits(m.event_group_, m.kEventTxTaskDone);
            m.cleanup_release.set_value();
            assert(m.Stop(1000) == ESP_OK);
            main.get();
            assert(!m.IsAtReady() && m.IsStopped());
            assert(m.SendAt("AT", response, 100) == ESP_ERR_INVALID_STATE);
        }
    }
    {
        UartEthModem m;
        prepare_init(m, "+CPIN: READY\r\nOK\r\n");
        auto init = std::async(std::launch::async, [&] { m.InitTaskRun(); });
        auto bits = xEventGroupWaitBits(m.event_group_, m.kEventInitDone, pdFALSE, pdTRUE, 1000);
        assert(bits & m.kEventInitDone);
        assert(m.IsInitialized() && m.IsAtReady() && m.netif_creates == 1);
        assert(m.Stop(0) == ESP_ERR_TIMEOUT);
        init.get();
        assert(m.pdp_configs == 1 && m.info_queries == 1);
    }
    {
        UartEthModem m;
        prepare_init(m, "+CPIN: READY\r\nOK\r\n");
        m.start_mode_ = UartEthModem::StartMode::kFlight;
        m.InitTaskRun();
        assert(m.IsAtReady() && m.IsInitialized() && m.netif_creates == 0);
        assert(m.notifications == std::vector{UartEthModem::UartEthModemEvent::InFlightMode});
    }
    {
        UartEthModem m;
        prepare_init(m, "+CME ERROR: 10\r\n");
        baud_result = ESP_FAIL;
        m.InitTaskRun();
        baud_result = ESP_OK;
        assert(m.IsStopping() && !m.IsAtReady() && m.netif_creates == 0);
        assert(m.notifications == std::vector{UartEthModem::UartEthModemEvent::ErrorInitFailed});
    }
    {
        UartEthModem m;
        prepare_init(m, "");
        auto reply = m.reply;
        m.reply = [reply](const std::string& cmd, std::string& response) {
            if (cmd == "AT+CPIN?") return ESP_ERR_TIMEOUT;
            return reply(cmd, response);
        };
        m.InitTaskRun();
        assert(m.IsStopping() && !m.IsAtReady() && m.netif_creates == 0);
        assert(m.notifications == std::vector{UartEthModem::UartEthModemEvent::ErrorInitFailed});
    }
    {
        UartEthModem m;
        prepare_init(m, "+CME ERROR: 100\r\n");
        m.InitTaskRun();
        assert(m.IsStopping() && !m.IsAtReady());
        assert(m.notifications == std::vector{UartEthModem::UartEthModemEvent::ErrorInitFailed});
    }
    {
        UartEthModem m;
        prepare_init(m, "");
        auto reply = m.reply;
        m.reply = [&](const std::string& cmd, std::string& response) {
            if (cmd == "AT+CPIN?") {
                m.RequestStop();
                response = "+CME ERROR: 10\r\n";
                return ESP_OK;
            }
            return reply(cmd, response);
        };
        m.InitTaskRun();
        assert(m.IsStopping() && !m.IsAtReady() && m.notifications.empty());
    }
    puts("10 SIM startup/recovery scenarios passed");
}
