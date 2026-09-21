# -*- coding: utf-8 -*-
"""
精细测试 90° 阶跃 (1.5708 rad) 在不同阻尼下的过冲、上升时间与收敛性
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

def send_cmd(cmd_str):
    if not cmd_str.endswith('\r\n'): cmd_str = cmd_str.rstrip('\r\n') + '\r\n'
    raw = list(cmd_str.encode('ascii'))
    head = t.read16(RX_HEAD)
    n = len(raw)
    first_chunk = min(n, 256 - head)
    t.write_memory_block8(RX_BUF + head, raw[:first_chunk])
    if n > first_chunk:
        t.write_memory_block8(RX_BUF, raw[first_chunk:])
    new_head = (head + n) % 256
    t.write16(RX_HEAD, new_head)
    t0 = time.time()
    while time.time() - t0 < 0.5:
        if t.read16(RX_TAIL) == new_head: break
        time.sleep(0.005)
    time.sleep(0.01)

def read_sample():
    raw = bytes(t.read_memory_block8(MOTOR_BASE + 216, 88))
    tgt = struct.unpack_from('<f', raw, 0)[0]
    iq_ref = struct.unpack_from('<f', raw, 248 - 216)[0]
    pos = struct.unpack_from('<f', raw, 288 - 216)[0]
    vel_obs = struct.unpack_from('<f', raw, 296 - 216)[0]
    origin = struct.unpack_from('<f', bytes(t.read_memory_block8(MOTOR_BASE + 796, 4)), 0)[0]
    return tgt, iq_ref, pos, vel_obs, origin

def test_step_90deg(vkp_val, kp_val=2.50, vmax=100.0, accel=120.0):
    print(f"\n================ 90° 阶跃测试: vkp={vkp_val:.4f}, kp={kp_val:.2f} ================")
    send_cmd("fault clear")
    send_cmd("disable")
    send_cmd("mode pos")
    send_cmd(f"pos kp {kp_val:.2f}")
    send_cmd("pos ki 0.00")
    send_cmd(f"pos vkp {vkp_val:.4f}")
    send_cmd(f"pos vmax {vmax:.1f}")
    send_cmd(f"pos accel {accel:.1f}")
    send_cmd("target 0.000")
    send_cmd("enable")
    time.sleep(0.2)

    tgt, iq, pos0, vel, origin0 = read_sample()
    print(f"使能稳态: pos={pos0:.4f}, iq={iq:.3f}A, vel={vel:.1f} RPM")

    step_goal = 1.5708
    print(f"下发 90° 阶跃 (target {step_goal:.4f} rad)...")
    send_cmd(f"target {step_goal:.4f}")
    t0 = time.time()
    max_d_pos = -999.0
    min_d_pos = 999.0
    max_abs_vel = 0.0
    settled = False

    history = []
    for _ in range(40): # 采样 1.2 秒
        t_cur = time.time() - t0
        tgt, iq, pos, vel, origin = read_sample()
        d_p = pos - pos0
        err = (origin + tgt) - pos
        if d_p > max_d_pos: max_d_pos = d_p
        if d_p < min_d_pos: min_d_pos = d_p
        if abs(vel) > max_abs_vel: max_abs_vel = abs(vel)
        history.append((t_cur, pos, d_p, vel, iq, err))
        time.sleep(0.03)

    # 打印前中后关键帧
    for i, h in enumerate(history):
        if i % 4 == 0 or i == len(history) - 1:
            print(f"  t={h[0]:5.3f}s | pos={h[1]:8.3f} | d_pos={h[2]:+6.3f} | vel={h[3]:+7.1f} RPM | iq={h[4]:+6.3f}A | err={h[5]:+6.3f}")

    final_h = history[-1]
    final_err = final_h[5]
    final_vel = final_h[3]
    overshoot = (max_d_pos - step_goal) if max_d_pos > step_goal else 0.0
    overshoot_pct = (overshoot / step_goal) * 100.0

    print(f"--- 评估结果 ---")
    print(f"目标位移: {step_goal:.4f} rad | 最大位移: {max_d_pos:.4f} rad | 过冲: {overshoot:+.4f} rad ({overshoot_pct:.1f}%)")
    print(f"终态误差: {final_err:+.4f} rad ({final_err*57.3:+.2f}°) | 终态转速: {final_vel:.1f} RPM | 最大转速: {max_abs_vel:.1f} RPM")

    send_cmd("disable")
    time.sleep(0.1)
    return overshoot_pct, abs(final_err), max_abs_vel

try:
    results = {}
    for v in [0.0003, 0.0006, 0.0010, 0.0015]:
        results[v] = test_step_90deg(v)
finally:
    send_cmd("disable")
    if t.is_halted(): t.resume()
    session.close()
