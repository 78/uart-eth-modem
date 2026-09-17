#!/usr/bin/env python3
"""Exercise production stop/coordinator methods against threaded FreeRTOS fakes."""
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

header = (ROOT / 'include/uart_eth_modem.h').read_text()
source = (ROOT / 'src/uart_eth_modem.cc').read_text()
transport = (ROOT / 'src/uart_eth_modem_transport.cc').read_text()
control = (ROOT / 'src/uart_eth_modem_control.cc').read_text()
methods = '\n\n'.join(block(source, r'^.*UartEthModem::' + name + r'\([^;]*?\)\s*\{')
                       for name in ['RequestStop', 'Stop', 'PrepareForShutdown', 'SendAt'])
methods += '\n' + block(transport, r'^void UartEthModem::MainTaskRun\(\)\s*\{')
methods += '\n' + '\n'.join(block(control, r'^.*UartEthModem::' + name + r'\([^;]*?\)\s*\{')
                             for name in ['AtDetect', 'CheckSimCard', 'InitTaskRun',
                                          'RunNormalModeInitSequence', 'RunFlightModeInitSequence'])
constants = '\n'.join(re.findall(r'^    static constexpr uint32_t kEvent\w+ = \(1 << \d+\);[^\n]*', header, re.M))
accessors = '\n'.join(block(header, r'    bool ' + name + r'\(\) const \{')
                      for name in ['IsStopped', 'IsStopping', 'IsAtReady', 'IsInitialized'])
text = (ROOT / 'tests/stop_test.cpp').read_text().replace('// @TYPES@', constants + '\n' + accessors).replace('// @METHODS@', methods)
with tempfile.TemporaryDirectory(prefix='uart-stop-test-') as tmp:
    cpp = Path(tmp) / 'test.cpp'
    cpp.write_text(text)
    binary = Path(tmp) / 'test'
    subprocess.run([os.environ.get('CXX', 'clang++'), '-std=c++20', '-g', '-O1', '-pthread',
                    '-fsanitize=' + os.environ.get('SANITIZERS', 'address,undefined'),
                    '-fno-omit-frame-pointer', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=20)
