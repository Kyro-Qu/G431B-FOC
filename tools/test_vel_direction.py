# -*- coding: utf-8 -*-
"""
诊断有感模式下速度环的响应方向与稳定性
测试目标: target 50 RPM，使能 0.5s，观察转速是收敛到 +50 还是发散飞车
"""
import time
import struct
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_HEAD = 0x2000008c
RX_TAIL = 0x2000008e
RX_BUF = 0x200034fc

session = ConnectHelper.session_with_chosen_probe(
    target_override='cortex_m',
    options={'halt_on_connect': False, 'resume_on_exit': True}
)
session.open()
t = session.target
if t.is_halted(): t.resume()

def run_cmd(cmd_str):
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
    while time.time() - t0 < 0.8:
        if t.read16(RX_TAIL) == head:
            break
        time.sleep(0.01)
    time.sleep(0.02)

def read_telemetry():
    raw156 = bytes(t.read_memory_block8(MOTOR_BASE + 156, 8))
    state = raw156[2]
    mode = raw156[3]
    raw_calib = bytes(t.read_memory_block8(MOTOR_BASE + 208, 8))
    calib_valid = raw_calib[0]
    direction = struct.unpack('<b', raw_calib[2:3])[0]
    raw_pos = bytes(t.read_memory_block8(MOTOR_BASE + 288, 16))
    pos = struct.unpack_from('<f', raw_pos, 0)[0]
    vel = struct.unpack_from('<f', raw_pos, 8)[0]
    raw_iq = bytes(t.read_memory_block8(MOTOR_BASE + 252, 4))
    iq = struct.unpack_from('<f', raw_iq, 0)[0]
    return state, mode, calib_valid, direction, pos, vel, iq

try:
    print("1. 清障与状态检查...")
    run_cmd("fault clear")
    run_cmd("disable")
    st, mode, calib_valid, direction, pos0, vel0, iq0 = read_telemetry()
    print(f"初始状态: state={st}, mode={mode}, calib={calib_valid}, dir={direction}")

    print("\n2. 进入速度模式 (mode vel)，target 50.0 RPM...")
    run_cmd("mode vel")
    run_cmd("target 50.0")
    run_cmd("enable")
    t0 = time.time()
    while time.time() - t0 < 0.6:
        st, mode, calib_valid, direction, pos, vel, iq = read_telemetry()
        print(f"  [速度环] state={st}, vel={vel:+.1f} RPM, iq={iq:+.3f}A, pos={pos:+.3f}")
        time.sleep(0.05)

    print("\n3. 安全停机...")
    run_cmd("disable")
    time.sleep(0.1)

finally:
    if t.is_halted(): t.resume()
    session.close()
