# -*- coding: utf-8 -*-
"""
大角度位置阶跃全梯度严密测试套件 (0.50 rad -> 12.57 rad -> 0.00 rad)
梯度:
  1. +0.50 rad  (~28.6°)
  2. +1.00 rad  (~57.3°)
  3. +1.57 rad  (90.0°, 1/4圈)
  4. +3.14 rad  (180.0°, 半圈)
  5. +6.28 rad  (360.0°, 1整圈)
  6. +12.57 rad (720.0°, 2整圈)
  7. 0.00 rad   (反转 720.0° 回零)
"""
import time
import struct
import math
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_QUEUE_HEAD_ADDR = 0x2000008c
RX_QUEUE_TAIL_ADDR = 0x2000008e
RX_QUEUE_BUF_ADDR = 0x200034fc
CMD_RX_QUEUE_SIZE = 256

OFF_STATE = 0x9e
OFF_MODE = 0x9f
OFF_CALIB_VALID = 0xd0
OFF_TARGET = 0xd8
OFF_IQ_REF = 0xf8
OFF_POS = 0x120
OFF_VEL_OBS = 0x128
OFF_VEL_FILT = 0x12c

def inject_cmd(target, cmd_str):
    if not cmd_str.endswith('\n'):
        cmd_str += '\n'
    cmd_bytes = cmd_str.encode('ascii')

    head_tail = target.read32(RX_QUEUE_HEAD_ADDR)
    head = head_tail & 0xFFFF

    for b in cmd_bytes:
        target.write8(RX_QUEUE_BUF_ADDR + head, b)
        head = (head + 1) % CMD_RX_QUEUE_SIZE

    target.write16(RX_QUEUE_HEAD_ADDR, head)
    t0 = time.time()
    while time.time() - t0 < 1.0:
        cur_tail = target.read16(RX_QUEUE_TAIL_ADDR)
        if cur_tail == head:
            break
        time.sleep(0.01)

def read_motor(target):
    data = bytes(target.read_memory_block8(MOTOR_BASE, 0x140))
    state = data[OFF_STATE]
    mode = data[OFF_MODE]
    calib_valid = data[OFF_CALIB_VALID]
    target_val = struct.unpack_from('<f', data, OFF_TARGET)[0]
    iq_ref = struct.unpack_from('<f', data, OFF_IQ_REF)[0]
    pos = struct.unpack_from('<f', data, OFF_POS)[0]
    vel_obs = struct.unpack_from('<f', data, OFF_VEL_OBS)[0]
    vel_filt = struct.unpack_from('<f', data, OFF_VEL_FILT)[0]

    return {
        'state': state,
        'mode': mode,
        'calib_valid': calib_valid,
        'target': target_val,
        'iq_ref': iq_ref,
        'pos': pos,
        'vel_obs': vel_obs,
        'vel_filt': vel_filt
    }

def test_single_step(target, target_rad, max_wait_sec):
    print(f"\n=======================================================")
    print(f"  阶跃测试: target = {target_rad:+.4f} rad ({target_rad*180.0/math.pi:+.1f}°)")
    print(f"=======================================================")

    # 读当前状态
    st_init = read_motor(target)
    init_pos = st_init['pos']

    inject_cmd(target, f"target {target_rad:.4f}")
    t0 = time.time()
    samples = []

    # 高频采样 50Hz
    while time.time() - t0 < max_wait_sec:
        s = read_motor(target)
        s['t'] = time.time() - t0
        samples.append(s)
        time.sleep(0.02)

    v_filts = [s['vel_filt'] for s in samples]
    v_obss = [s['vel_obs'] for s in samples]
    iqs = [s['iq_ref'] for s in samples]
    poses = [s['pos'] for s in samples]

    # 分析指标
    max_v = max(abs(v) for v in v_filts)
    max_iq = max(abs(i) for i in iqs)
    final_pos = poses[-1]

    # 实际走过的弧度增量
    actual_delta = final_pos - init_pos
    expected_delta = target_rad - st_init['target']

    # 超调分析
    if expected_delta > 0:
        max_overshoot = max(0.0, max(poses) - (init_pos + expected_delta))
    else:
        max_overshoot = max(0.0, (init_pos + expected_delta) - min(poses))

    # 稳态残余速度 (最后 10 拍平均)
    tail_v = sum(v_filts[-10:]) / 10.0
    tail_iq = sum(iqs[-10:]) / 10.0

    print(f"实测响应指标 ({len(samples)} 采样点, 耗时 {max_wait_sec:.1f}s):")
    print(f"  最大滤波转速 : {max_v:.1f} RPM")
    print(f"  最大电流给定 : {max_iq:.3f} A (安全硬限幅: 0.50A)")
    print(f"  起始位置     : {init_pos:.4f} rad")
    print(f"  最终到达位置 : {final_pos:.4f} rad")
    print(f"  位置变化增量 : {actual_delta:+.4f} rad (目标增量: {expected_delta:+.4f} rad)")
    print(f"  最大超调量   : {max_overshoot:.4f} rad ({max_overshoot*180.0/math.pi:.2f}°)")
    print(f"  稳态残余速度 : {tail_v:.2f} RPM")
    print(f"  稳态保持电流 : {tail_iq:.3f} A")

    # 判定准则
    is_safe = max_iq <= 0.501
    is_stable = abs(tail_v) < 1.0
    print(f"  安全硬限幅合规: {'PASS' if is_safe else 'FAIL'}")
    print(f"  稳态锁定平稳度: {'PASS' if is_stable else 'FAIL'}")

    return {
        'target_rad': target_rad,
        'max_v': max_v,
        'max_iq': max_iq,
        'actual_delta': actual_delta,
        'expected_delta': expected_delta,
        'overshoot': max_overshoot,
        'tail_v': tail_v,
        'tail_iq': tail_iq,
        'is_safe': is_safe,
        'is_stable': is_stable
    }

def main():
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    target = session.target
    if target.is_halted(): target.resume()

    print("#######################################################")
    print("#   FOC_G431 真实硬件大角度全梯度严密实测套件启动      #")
    print("#######################################################")

    # 1. 确保在 POS 模式并使能
    inject_cmd(target, "disarm")
    time.sleep(0.1)
    inject_cmd(target, "mode pos")
    inject_cmd(target, "target 0.000")
    inject_cmd(target, "enable")
    time.sleep(0.3)

    # 梯度阶跃列表: (目标角度 rad, 观测时长 秒)
    steps = [
        (0.50, 2.0),    # ~28.6°
        (1.00, 2.5),    # ~57.3°
        (1.5708, 3.0),  # 90°
        (3.1416, 4.0),  # 180°
        (6.2832, 5.5),  # 360° (1圈)
        (12.5664, 7.0), # 720° (2圈)
        (0.0000, 7.0)   # 反转 720° 回零
    ]

    results = []
    for tgt_rad, wait_sec in steps:
        res = test_single_step(target, tgt_rad, wait_sec)
        results.append(res)
        time.sleep(0.3)

    print("\n#######################################################")
    print("#                    全梯度测试总结汇报                #")
    print("#######################################################")
    print(f"{'目标(rad)':<10} {'期望增量':<10} {'实测增量':<10} {'最大RPM':<10} {'最大Iq(A)':<10} {'超调(deg)':<10} {'稳态判定':<8}")
    print("-" * 75)
    for r in results:
        print(f"{r['target_rad']:<10.4f} {r['expected_delta']:<10.4f} {r['actual_delta']:<10.4f} {r['max_v']:<10.1f} {r['max_iq']:<10.3f} {r['overshoot']*180/math.pi:<10.2f} {'PASS' if r['is_stable'] and r['is_safe'] else 'FAIL'}")

    print("\n安全停机 disarm...")
    inject_cmd(target, "disarm")
    session.close()

if __name__ == "__main__":
    main()
