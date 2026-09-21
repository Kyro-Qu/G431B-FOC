# -*- coding: utf-8 -*-
"""
深入探查 m->state, m->mode, m->safety.fault_code 与 CLI 命令交互
"""
import time
import struct
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_HEAD = 0x2000008c
RX_TAIL = 0x2000008e
RX_BUF = 0x200034fc
RESP_BUF = 0x2000363c

session = ConnectHelper.session_with_chosen_probe(
    target_override='cortex_m',
    options={'halt_on_connect': False, 'resume_on_exit': True}
)
session.open()
t = session.target
if t.is_halted(): t.resume()

def exec_cmd(cmd_str):
    if not cmd_str.endswith('\r\n'):
        cmd_str = cmd_str.rstrip('\r\n') + '\r\n'
    raw = cmd_str.encode('ascii')
    head = t.read16(RX_HEAD)
    for b in raw:
        t.write8(RX_BUF + head, b)
        head = (head + 1) % 256
    t.write16(RX_HEAD, head)
    if t.is_halted(): t.resume()

    t0 = time.time()
    while time.time() - t0 < 0.6:
        tail = t.read16(RX_TAIL)
        if tail == head:
            break
        time.sleep(0.01)
    time.sleep(0.04)

    # 读回显
    resp_raw = bytes(t.read_memory_block8(RESP_BUF, 128))
    resp = resp_raw.split(b'\x00')[0].decode('ascii', 'replace').strip()
    return resp

print("=== 0. 清除 fault ===")
r = exec_cmd("fault clear")
print("fault clear 回显:", repr(r))

print("=== 1. 执行 disable ===")
r = exec_cmd("disable")
print("disable 回显:", repr(r))

print("=== 2. 查看 status ===")
r = exec_cmd("status")
print("status 回显:\n", r)

print("=== 3. 执行 mode pos ===")
r = exec_cmd("mode pos")
print("mode pos 回显:", repr(r))

print("=== 4. 再次查看 status ===")
r = exec_cmd("status")
print("status 回显:\n", r)

if t.is_halted(): t.resume()
session.close()
