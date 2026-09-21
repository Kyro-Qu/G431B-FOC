# -*- coding: utf-8 -*-
"""
严谨硬件自动化验证脚本: 验证 pos abs / pos step / pos zero / pos rel
带有全时段看门狗监控：转速上限、电流上限、极限环振荡检测
"""
import sys
import time
import struct
import math
from pyocd.core.helpers import ConnectHelper

session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
session.open()
t = session.target
if t.is_halted(): t.resume()

MOTOR_ADDR = 0x20002e1c
RX_HEAD_ADDR = 0x2000008c
RX_TAIL_ADDR = 0x2000008e
RX_BUF_ADDR  = 0x20003508

def send_cmd(cmd_str):
    if not cmd_str.endswith('\r\n'): cmd_str += '\r\n'
    raw = cmd_str.encode('ascii')
    head = t.read16(RX_HEAD_ADDR)
    for b in raw:
        t.write8(RX_BUF_ADDR + head, b)
        head = (head + 1) % 256
    t.write16(RX_HEAD_ADDR, head)
    if t.is_halted(): t.resume()
    t0 = time.time()
    while time.time() - t0 < 0.5:
        if t.read16(RX_TAIL_ADDR) == head: break
        time.sleep(0.01)

def read_telemetry():
    data = bytes(t.read_memory_block8(MOTOR_ADDR, 836))
    state = data[0x9e]
    mode = data[0x9f]
    target, = struct.unpack('<f', data[0x0d8:0x0dc])
    pos_rad, = struct.unpack('<f', data[0x11c:0x120])
    vel_obs, = struct.unpack('<f', data[0x124:0x128])
    iq, = struct.unpack('<f', data[0x13c:0x140])
    origin, = struct.unpack('<f', data[0x31c:0x320])
    latch, = struct.unpack('<f', data[0x318:0x31c])
    return {
        'state': state,
        'mode': mode,
        'target': target,
        'origin': origin,
        'pos': pos_rad,
        'vel': vel_obs,
        'iq': iq,
        'latch': latch
    }

def safe_stop():
    send_cmd("disable")
    time.sleep(0.05)

def wait_and_monitor(duration_s, expected_goal_rad, tol_rad=0.08):
    t0 = time.time()
    samples = []
    max_rpm = 0.0
    max_iq = 0.0
    while time.time() - t0 < duration_s:
        telem = read_telemetry()
        samples.append(telem)
        rpm = abs(telem['vel'])
        iq = abs(telem['iq'])
        if rpm > max_rpm: max_rpm = rpm
        if iq > max_iq: max_iq = iq

        # 安全看门狗保护
        if rpm > 150.0:
            safe_stop()
            raise RuntimeError(f"看门狗跳闸: 速度超标 {rpm:.1f} RPM > 150.0 RPM!")
        if iq > 1.2:
            safe_stop()
            raise RuntimeError(f"看门狗跳闸: 电流超标 {iq:.2f} A > 1.2 A!")
        time.sleep(0.02)

    final = samples[-1]
    err = abs(final['pos'] - expected_goal_rad)
    tail_rpms = [s['vel'] for s in samples[-15:]]
    steady_speed = max(abs(v) for v in tail_rpms)
    steady_iq = final['iq']

    # 振荡检测
    sign_changes = 0
    for i in range(len(tail_rpms)-1):
        if (tail_rpms[i] * tail_rpms[i+1]) < -1.0:
            sign_changes += 1

    is_pass = (err <= tol_rad) and (steady_speed < 2.0) and (sign_changes <= 2)
    return {
        'pass': is_pass,
        'pos': final['pos'],
        'goal': expected_goal_rad,
        'err_deg': math.degrees(err),
        'max_rpm': max_rpm,
        'steady_speed': steady_speed,
        'steady_iq': steady_iq,
        'sign_changes': sign_changes
    }

print("=== 开始严格硬件全功能位置模式测试 ===")

# 1. 切换到 POS 模式并使能
send_cmd("mode pos")
time.sleep(0.05)
send_cmd("enable")
time.sleep(0.2)

init_telem = read_telemetry()
print(f"使能初始状态: state={init_telem['state']} (1=RUN), mode={init_telem['mode']} (3=POS), origin={init_telem['origin']:.4f}rad, pos={init_telem['pos']:.4f}rad")
assert init_telem['state'] == 1, "使能失败！"

test_results = []

# 测试 1: pos step 30deg (相对当前实际物理位置步进 +30 度)
print("\n--- 测试 1: pos step 30deg ---")
cur_p = read_telemetry()['pos']
step_val = math.radians(30.0)
goal_1 = cur_p + step_val
send_cmd("pos step 30deg")
res_1 = wait_and_monitor(2.0, goal_1)
print(f"结果: PASS={res_1['pass']} 目标={math.degrees(goal_1):.1f}° 实际={math.degrees(res_1['pos']):.1f}° 误差={res_1['err_deg']:.2f}° 最大转速={res_1['max_rpm']:.1f}RPM 稳态转速={res_1['steady_speed']:.2f}RPM 稳态Iq={res_1['steady_iq']*1000:.1f}mA")
test_results.append(("pos step 30deg", res_1))

# 测试 2: 再次 pos step 60deg (连续步进 +60 度)
print("\n--- 测试 2: pos step 60deg ---")
cur_p = read_telemetry()['pos']
step_val = math.radians(60.0)
goal_2 = cur_p + step_val
send_cmd("pos step 60deg")
res_2 = wait_and_monitor(2.5, goal_2)
print(f"结果: PASS={res_2['pass']} 目标={math.degrees(goal_2):.1f}° 实际={math.degrees(res_2['pos']):.1f}° 误差={res_2['err_deg']:.2f}° 最大转速={res_2['max_rpm']:.1f}RPM 稳态转速={res_2['steady_speed']:.2f}RPM 稳态Iq={res_2['steady_iq']*1000:.1f}mA")
test_results.append(("pos step 60deg", res_2))

# 测试 3: pos zero (将当前位置设为新用户原点)
print("\n--- 测试 3: pos zero (重设原点) ---")
cur_pos_before_zero = read_telemetry()['pos']
send_cmd("pos zero")
time.sleep(0.15)
telem_after_zero = read_telemetry()
origin_err = abs(telem_after_zero['origin'] - cur_pos_before_zero)
rel_pos = telem_after_zero['pos'] - telem_after_zero['origin']
print(f"重设原点: 原点={telem_after_zero['origin']:.4f}rad, 当前实际={telem_after_zero['pos']:.4f}rad, 相对位置={rel_pos:.4f}rad, target={telem_after_zero['target']:.4f}rad")
pass_zero = (origin_err < 0.05) and (abs(rel_pos) < 0.05) and (abs(telem_after_zero['target']) < 0.001)
test_results.append(("pos zero", {'pass': pass_zero, 'origin_err': origin_err, 'rel_pos': rel_pos}))

# 测试 4: pos rel 0.25turn (相对于新原点转 1/4 圈 = 90deg)
print("\n--- 测试 4: pos rel 0.25turn (相对新原点) ---")
goal_4 = telem_after_zero['origin'] + 0.5 * math.pi
send_cmd("pos rel 0.25turn")
res_4 = wait_and_monitor(2.5, goal_4)
print(f"结果: PASS={res_4['pass']} 目标={math.degrees(goal_4):.1f}° 实际={math.degrees(res_4['pos']):.1f}° 误差={res_4['err_deg']:.2f}° 最大转速={res_4['max_rpm']:.1f}RPM 稳态转速={res_4['steady_speed']:.2f}RPM 稳态Iq={res_4['steady_iq']*1000:.1f}mA")
test_results.append(("pos rel 0.25turn", res_4))

# 测试 5: pos abs 0.000rad (绝对物理回原点 0 rad)
print("\n--- 测试 5: pos abs 0.000rad (绝对物理坐标锁定 0 rad) ---")
goal_5 = 0.0
send_cmd("pos abs 0.000rad")
res_5 = wait_and_monitor(3.5, goal_5)
print(f"结果: PASS={res_5['pass']} 目标={math.degrees(goal_5):.1f}° 实际={math.degrees(res_5['pos']):.1f}° 误差={res_5['err_deg']:.2f}° 最大转速={res_5['max_rpm']:.1f}RPM 稳态转速={res_5['steady_speed']:.2f}RPM 稳态Iq={res_5['steady_iq']*1000:.1f}mA")
test_results.append(("pos abs 0.000rad", res_5))

# 测试 6: pos abs 180deg (绝对物理坐标锁定 180 度 = pi rad)
print("\n--- 测试 6: pos abs 180deg (绝对物理坐标锁定 180 度) ---")
goal_6 = math.pi
send_cmd("pos abs 180deg")
res_6 = wait_and_monitor(3.5, goal_6)
print(f"结果: PASS={res_6['pass']} 目标={math.degrees(goal_6):.1f}° 实际={math.degrees(res_6['pos']):.1f}° 误差={res_6['err_deg']:.2f}° 最大转速={res_6['max_rpm']:.1f}RPM 稳态转速={res_6['steady_speed']:.2f}RPM 稳态Iq={res_6['steady_iq']*1000:.1f}mA")
test_results.append(("pos abs 180deg", res_6))

# 结束测试，安全停机
safe_stop()
time.sleep(0.1)
print("\n=== 测试汇总 ===")
all_pass = True
for name, res in test_results:
    p = res['pass']
    if not p: all_pass = False
    print(f"  [{'PASS' if p else 'FAIL'}] {name}")

print(f"\n全部通过: {all_pass}")
session.close()
sys.exit(0 if all_pass else 1)
