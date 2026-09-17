# TX pool and RX enqueue regression

Run from the component root:

```sh
python3 tests/run_host_tests.py
```

Requires Python 3 and a C++20 compiler (`clang++` by default, override with `CXX`).
ASan/UBSan are enabled by default; set `SANITIZERS=thread` for a separate TSan run. The independent `UartEthTxPool` header/source are compiled
directly. Production protocol declarations and transport method bodies
are read directly from the sources; fake FreeRTOS queues/semaphores provide a
host scheduler. A test wrapper takes the AT mutex just like both real synchronous
callers, SendAt and ActivateDataNetwork's handshake.

The runner also compiles `tx_pool_ownership_test.cpp` directly against the pool
class, without ESP-IDF/FreeRTOS stubs or method extraction. Five pool scenarios
cover invalid lengths, asynchronous reuse, completion before waiter release,
timeout before worker completion, cancellation and full-pool waiter retention.
These scenarios run with 1, 3, 34 and 66 supplied slots.

Twelve transport scenarios run with the production runtime allocator in both PSRAM
and internal SRAM modes, at queue depths 1, 8, 32 and 64. Heap mocks verify the exact capabilities and
that allocation failure makes no fallback attempt. They cover:

- frame size validation, queue saturation, payload copy, and 10,000 sends with
  no further payload/semaphore allocations;
- normal synchronous completion;
- timeout followed by late completion, no premature slot reuse and no stale
  notification reaching a later request;
- completion racing the timeout boundary;
- cancellation of a queued synchronous frame and waiter exit before pool deletion;
- serialization of two synchronous callers;
- heap/semaphore allocation failure, including automatic pool cleanup by the
  production smart-pointer deleter when semaphore creation fails;
- RX queue-full callback recording deferred return;
- pool exhaustion independently of queue capacity, without fallback allocation;
- invalid zero/overflow capacities rejected before allocation;
- queue allocation failure with complete cleanup, retry and reinitialization;
- simultaneous instances with different capacities and heaps remaining independent.

Allocation accounting checks the requested pool size and verifies that cleanup
leaves no live pool allocations; the
production compile-time assertion also checks that the smart-pointer owner takes
no more space than a raw pointer.

These tests do not measure UART throughput, DMA behavior, allocator fragmentation,
or real internal SRAM high-water marks. Validate continuous traffic, queue/pool
saturation and Stop/Start on hardware before publishing. The selected heap is
strict: PSRAM mode fails initialization instead of falling back to internal SRAM.
This allocator option does not change AT strings or FIFO/Stop behavior.

## Stop and missing-SIM startup

`python3 tests/run_stop_tests.py` runs 9 stop/AT lifecycle scenarios and 10 SIM
startup/recovery scenarios, with ASan/UBSan (`SANITIZERS=thread` enables TSan).
It extracts the production AtDetect, CheckSimCard, normal/flight initialization,
InitTaskRun, SendAt and stop/coordinator methods. Fake AT replies cover absent
SIM, PIN-required SIM, ready SIM, transport timeout, non-SIM CME errors, and
cancellation during the SIM query. It checks that ErrorNoSim sees AT-ready state,
that slot queries/writes still work without a netif, and that explicit stop joins
the retained channel. These tests do not simulate automatic SIM hotplug recovery
or target UART/RTOS timing.
