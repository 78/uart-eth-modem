#!/usr/bin/env python3
"""Run the production TX pool/send methods with fake FreeRTOS queues/semaphores.
ASan/UBSan check timeout/late completion and cleanup; no UART hardware is simulated.
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def block(source, pattern):
    match = re.search(pattern, source, re.M)
    assert match, pattern
    depth = 0
    for token in re.finditer(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|[{}]', source[match.start():], re.S):
        if token[0] == '{': depth += 1
        if token[0] == '}':
            depth -= 1
            if depth == 0: return source[match.start():match.start() + token.end()]
    raise AssertionError(pattern)

source = (ROOT / 'src/uart_eth_modem_transport.cc').read_text()
header = (ROOT / 'include/uart_eth_modem.h').read_text()
names = ['InitTxPool', 'DeinitTxPool', 'QueueTxFrame', 'CompleteTxFrame', 'EnqueueTxFrame', 'SendFrame', 'UhciRxCallbackStatic']
methods = '\n\n'.join(block(source, r'^.*UartEthModem::' + name + r'\([^;]*?\)\s*\{') for name in names)
methods = block(source, r'^void UartEthModem::TxPoolDeleter::operator\(\)\([^;]*?\) const noexcept \{') + '\n\n' + methods
deleter = block(header, r'    struct TxPoolDeleter \{') + ';'
owner = re.search(r'    using TxPoolPtr = [^;]+;', header)[0]
owner_size = re.search(r'    static_assert\(sizeof\(TxPoolPtr\)[^;]+;', header)[0]
frame_type = block(header, r'    enum class FrameType[^\{]+\{') + ';'
frame_header = block(header, r'    struct FrameHeader \{') + ' __attribute__((packed));'
frame = 'using TxFrame = UartEthTxPool::Slot;'
config = block(header, r'    struct Config \{') + ';'
constants = '\n'.join(re.findall(r'^    static constexpr size_t kMaxFrameSize[^\n]+', header, re.M))
text = (ROOT / 'tests/tx_pool_test.cpp').read_text().replace('// @TYPES@', '\n'.join([frame_type, frame_header, constants, frame, deleter, owner, owner_size, config])).replace('// @METHODS@', methods)
with tempfile.TemporaryDirectory(prefix='uart-modem-test-') as tmp:
    cpp = Path(tmp) / 'test.cpp'
    cpp.write_text(text)
    # Compile the runtime allocator once, then exercise both heaps and capacities.
    (Path(tmp) / 'esp_heap_caps.h').write_text('// Heap functions supplied by harness.\n')
    binary = Path(tmp) / 'test'
    subprocess.run([os.environ.get('CXX', 'clang++'), '-std=c++20', '-g', '-O1', '-pthread',
                    '-fsanitize=' + os.environ.get('SANITIZERS', 'address,undefined'),
                    '-fno-omit-frame-pointer', '-I', tmp, '-I', str(ROOT / 'include'),
                    '-I', str(ROOT / 'src'),
                    str(ROOT / 'src/uart_eth_tx_pool.cc'), str(cpp), '-o', str(binary)], check=True)
    for use_psram in (True, False):
        for depth in (1, 8, 32, 64):
            print(f'Allocator: {"PSRAM" if use_psram else "internal SRAM"}, queue depth: {depth}', flush=True)
            subprocess.run([str(binary), str(depth), str(int(use_psram))], check=True)
    pool_binary = Path(tmp) / 'pool-test'
    subprocess.run([os.environ.get('CXX', 'clang++'), '-std=c++20', '-g', '-O1',
                    '-fsanitize=' + os.environ.get('SANITIZERS', 'address,undefined'),
                    '-fno-omit-frame-pointer',
                    '-I', str(ROOT / 'include'), str(ROOT / 'src/uart_eth_tx_pool.cc'),
                    str(ROOT / 'tests/tx_pool_ownership_test.cpp'), '-o', str(pool_binary)], check=True)
    subprocess.run([str(pool_binary)], check=True)
