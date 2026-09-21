# -*- coding: utf-8 -*-
"""
探查不同 pos_vkp 阻尼系数下的 90° 阶跃稳定性
测试梯度: vkp = 0.001, 0.002, 0.005, 0.010
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
    iq_ref = struct.unpack_from('<f', raw, 248 - 216)[0] # 正确偏移 248
    pos = struct.unpack_from('<f', raw, 288 - 216)[0]
    vel_obs = struct.unpack_from('<f', raw, 296 - 216)[0]
    origin = struct.unpack_from('<f', bytes(t.read_memory_block8(MOTOR_BASE + 796, 4)), 0)[0]
    return tgt, iq_ref, pos, vel_obs, origin

def test_damping(vkp_val):
    print(f"\n================ 测试 pos vkp = {vkp_val:.4f} ================")
    send_cmd("fault clear")
    send_cmd("disable")
    send_cmd("mode pos")
    send_cmd("pos kp 1.50")
    send_cmd("pos ki 0.00")
    send_cmd(f"pos vkp {vkp_val:.4f}")
    send_cmd("pos vmax 60")
    send_cmd("pos accel 40")
    send_cmd("target 0.000")
    send_cmd("enable")
    time.sleep(0.2)

    tgt, iq, pos0, vel, origin0 = read_sample()
    print(f"使能初始: pos={pos0:.4f}, iq={iq:.3f}A, vel={vel:.1f} RPM")

    # 给 0.5 rad (约 28.6 度) 阶跃
    print(f"下发 target 0.500 rad 阶跃...")
    send_cmd("target 0.500")
    t0 = time.time()
    max_vel = 0.0
    settled = False
    for i in range(25): # 采样 0.75s
        t_cur = time.time() - t0
        tgt, iq, pos, vel, origin = read_sample()
        if abs(vel) > max_vel: max_vel = abs(vel)
        err = (origin + tgt) - pos
        d_pos = pos - pos0
        if i % 3 == 0 or abs(vel) > 300:
            print(f"  t={t_cur:5.3f}s: pos={pos:8.3f}, d_pos={d_pos:+6.3f}, vel={vel:+7.1f} RPM, iq={iq:+6.3f}A, err={err:+6.3f}")
        if abs(err) < 0.03 and abs(vel) < 10.0 and t_cur > 0.2:
            settled = True
        time.sleep(0.03)

    send_cmd("disable")
    print(f"测试结果: max_vel={max_vel:.1f} RPM, 收敛达成={settled}")
    time.sleep(0.1)
    return max_vel, settled

try:
    for v in [0.0005, 0.0010, 0.0020, 0.0050]:
        max_v, ok = test_damping(v)
        if max_v > 1000:
            print(f"警告: vkp={v} 触发高速振荡 (max_vel={max_v:.1f})，停止更高增益测试！")
            break
finally:
    send_cmd("disable")
    if t.is_halted(): t.resume()
    session.close()
