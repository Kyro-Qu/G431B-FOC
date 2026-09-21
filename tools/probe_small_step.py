# -*- coding: utf-8 -*-
"""
高频探查位置环阶跃微观时序
"""
import time
import struct
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_HEAD = 0x2000008c
RX_TAIL = 0x2000008e
RX_BUF = 0x200034fc

session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
session.open()
t = session.target
if t.is_halted(): t.resume()

def run_cmd(cmd_str):
    if not cmd_str.endswith('\r\n'): cmd_str += '\r\n'
    raw = cmd_str.encode('ascii')
    head = t.read16(RX_HEAD)
    for b in raw:
        t.write8(RX_BUF + head, b)
        head = (head + 1) % 256
    t.write16(RX_HEAD, head)
    if t.is_halted(): t.resume()
    t0 = time.time()
    while time.time() - t0 < 0.8:
        if t.read16(RX_TAIL) == head: break
        time.sleep(0.01)
    time.sleep(0.02)

def read_sample():
    # 读 target(216), vel_ref(228), iq_ref(252), pos(288), vel_obs(296)
    # pos_origin_rad 在 796
    raw = bytes(t.read_memory_block8(MOTOR_BASE + 216, 88))
    tgt = struct.unpack_from('<f', raw, 0)[0]
    vel_ref = struct.unpack_from('<f', raw, 228 - 216)[0]
    iq_ref = struct.unpack_from('<f', raw, 252 - 216)[0]
    pos = struct.unpack_from('<f', raw, 288 - 216)[0]
    vel_obs = struct.unpack_from('<f', raw, 296 - 216)[0]
    origin = struct.unpack_from('<f', bytes(t.read_memory_block8(MOTOR_BASE + 796, 4)), 0)[0]
    return tgt, vel_ref, iq_ref, pos, vel_obs, origin

try:
    print("1. 复位与配置...")
    run_cmd("fault clear")
    run_cmd("disable")
    run_cmd("mode pos")
    run_cmd("pos kp 3.00")
    run_cmd("pos ki 0.00") # 先关掉积分，排除积分饱和干扰
    run_cmd("pos vkp 0.020")
    run_cmd("pos vmax 80")
    run_cmd("pos accel 80")
    run_cmd("target 0.000")
    run_cmd("enable")
    time.sleep(0.3)

    tgt, vel_ref, iq_ref, pos0, vel_obs, origin = read_sample()
    print(f"使能稳态: origin={origin:.4f}, pos={pos0:.4f}, err={origin+tgt-pos0:.4f}, iq={iq_ref:.3f}A, vel={vel_obs:.2f}")

    print("\n2. 下发微小阶跃 target 0.050 rad (约 2.8°)...")
    run_cmd("target 0.050")
    t0 = time.time()
    samples = []
    for _ in range(40):
        tgt, vel_ref, iq_ref, pos, vel_obs, origin = read_sample()
        samples.append((time.time() - t0, tgt, vel_ref, iq_ref, pos, vel_obs, origin))
        time.sleep(0.03)

    print(f"{'t(s)':>6} | {'pos':>9} | {'d_pos':>8} | {'vel_obs':>9} | {'iq_ref':>8} | {'err_to_goal':>11}")
    print("-" * 62)
    for ts, tgt, vel_ref, iq_ref, pos, vel_obs, origin in samples:
        goal = origin + tgt
        err_g = goal - pos
        d_p = pos - pos0
        print(f"{ts:6.3f} | {pos:9.4f} | {d_p:+8.4f} | {vel_obs:9.2f} | {iq_ref:+8.3f} | {err_g:+11.4f}")

    print("\n3. 停机...")
    run_cmd("disable")
    time.sleep(0.1)

finally:
    if t.is_halted(): t.resume()
    session.close()
