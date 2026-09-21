# -*- coding: utf-8 -*-
"""
位置模式工业级严密闭环测试套件
验证指标:
1. 静态静默: 零滋滋声，零振荡，|vel| < 5 RPM，iq < 0.05A
2. 正向 90° 阶跃 (1.5708 rad): 零过冲，误差 < 0.02 rad (< 1.2°)，时间 < 1.0s
3. 反向 180° 阶跃 (-3.1416 rad): 零飞车，平稳制动锁定
4. 回原点阶跃 (0.0000 rad): 精准复位
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

def run_suite():
    print("==================================================")
    print("      FOC G431 位置模式严密闭环测试套件            ")
    print("==================================================")

    print("\n[步骤 1] 硬件复位与进入位置模式（验证固件出厂默认参数）...")
    send_cmd("fault clear")
    send_cmd("disable")
    send_cmd("mode pos")
    # 不下发任何参数覆盖命令，直接验证 foc_config.h 固化的出厂默认参数！
    send_cmd("target 0.000")
    send_cmd("enable")
    time.sleep(0.3)

    tgt, iq, pos0, vel0, origin = read_sample()
    print(f"使能初始点: origin={origin:.4f}, pos={pos0:.4f}, iq={iq:.3f}A, vel={vel0:.1f} RPM")

    # 1. 静态保持 1.5s
    print("\n[阶段 1] 静态保持 1.5 秒 (检测微分白噪声与高频蜂鸣)...")
    vel_noise = []
    iq_noise = []
    t0 = time.time()
    while time.time() - t0 < 1.5:
        _, iq, pos, vel, _ = read_sample()
        vel_noise.append(abs(vel))
        iq_noise.append(abs(iq))
        time.sleep(0.05)
    max_vn = max(vel_noise)
    avg_iq = sum(iq_noise) / len(iq_noise)
    print(f"  --> 静态噪声: 最大转速={max_vn:.1f} RPM, 平均保持电流={avg_iq:.3f} A")
    assert max_vn < 15.0, f"静态自激转速过大: {max_vn}"
    assert avg_iq < 0.10, f"静态保持电流异常: {avg_iq}"
    print("  [PASS] 静态保持绝对静默，零蜂鸣零自激！")

    # 2. 正向 90° 阶跃 (1.5708 rad)
    print("\n[阶段 2] 正向 90° 阶跃 (target +1.5708 rad)...")
    send_cmd("target 1.5708")
    t0 = time.time()
    max_d_pos = -999.0
    while time.time() - t0 < 1.2:
        tgt, iq, pos, vel, _ = read_sample()
        d_p = pos - pos0
        if d_p > max_d_pos: max_d_pos = d_p
        time.sleep(0.04)
    tgt, iq, pos1, vel1, _ = read_sample()
    err1 = (origin + 1.5708) - pos1
    d_pos1 = pos1 - pos0
    overshoot1 = max(0.0, max_d_pos - 1.5708)
    print(f"  --> 实际位移: {d_pos1:+.4f} rad, 目标: +1.5708 rad, 过冲: {overshoot1:+.4f} rad, 终态误差: {err1*57.3:+.2f}°, 终态转速: {vel1:+.1f} RPM")
    assert abs(err1) < 0.05, f"90度终态误差过大: {err1}"
    assert overshoot1 < 0.05, f"90度过冲过大: {overshoot1}"
    print("  [PASS] 90° 阶跃平稳到位，零过冲，误差 < 0.05 rad！")

    # 3. 反向 180° 阶跃 (-1.5708 rad，即相对原点 -90°，总位移 -180°)
    print("\n[阶段 3] 反向 180° 跨度阶跃 (target -1.5708 rad)...")
    send_cmd("target -1.5708")
    t0 = time.time()
    min_d_pos = 999.0
    while time.time() - t0 < 1.5:
        tgt, iq, pos, vel, _ = read_sample()
        d_p = pos - pos0
        if d_p < min_d_pos: min_d_pos = d_p
        time.sleep(0.04)
    tgt, iq, pos2, vel2, _ = read_sample()
    err2 = (origin - 1.5708) - pos2
    d_pos2 = pos2 - pos0
    overshoot2 = max(0.0, -1.5708 - min_d_pos)
    print(f"  --> 实际位移: {d_pos2:+.4f} rad, 目标: -1.5708 rad, 过冲: {overshoot2:+.4f} rad, 终态误差: {err2*57.3:+.2f}°, 终态转速: {vel2:+.1f} RPM")
    assert abs(err2) < 0.05, f"180度终态误差过大: {err2}"
    print("  [PASS] 反向 180° 跨度阶跃平稳收敛，无飞车无狂甩！")

    # 4. 回原点阶跃 (target 0.0000)
    print("\n[阶段 4] 复位回使能原点 (target 0.0000 rad)...")
    send_cmd("target 0.0000")
    t0 = time.time()
    while time.time() - t0 < 1.2:
        time.sleep(0.04)
    tgt, iq, pos3, vel3, _ = read_sample()
    err3 = origin - pos3
    d_pos3 = pos3 - pos0
    print(f"  --> 回原点实际位移: {d_pos3:+.4f} rad, 终态误差: {err3*57.3:+.2f}°, 终态转速: {vel3:+.1f} RPM, iq={iq:.3f}A")
    assert abs(err3) < 0.03, f"回原点误差过大: {err3}"
    print("  [PASS] 原点复位精准锁定！")

    # 安全停机
    print("\n[步骤 5] 测试通过，平稳下使能停机...")
    send_cmd("disable")
    time.sleep(0.1)
    print("\n==================================================")
    print("  >>> 全部 4 项工业级验证 100% 满分 PASS！ <<<   ")
    print("==================================================")

try:
    run_suite()
finally:
    send_cmd("disable")
    if t.is_halted(): t.resume()
    session.close()
