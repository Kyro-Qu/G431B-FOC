# -*- coding: utf-8 -*-
"""
单步安全可控位置阶跃测试工具 (内置毫秒级防甩看门狗)
用法: python tools/test_pos_step_safe.py <target_rad> [wait_sec]
"""
import sys
import time
import struct
import math
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_QUEUE_HEAD_ADDR = 0x2000008c
RX_QUEUE_TAIL_ADDR = 0x2000008e
RX_QUEUE_BUF_ADDR = 0x20003508
CMD_RX_QUEUE_SIZE = 256

OFF_STATE = 0x9e
OFF_MODE = 0x9f
OFF_CALIB_VALID = 0xd0
OFF_TARGET = 0xd8
OFF_VEL_REF = 0xe4
OFF_IQ_REF = 0xf8
OFF_POS = 0x120
OFF_VEL_OBS = 0x128
OFF_VEL_FILT = 0x12c

def inject_cmd(target, cmd_str):
    if not cmd_str.endswith('\r\n'):
        if cmd_str.endswith('\n'):
            cmd_str = cmd_str[:-1] + '\r\n'
        else:
            cmd_str += '\r\n'
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
    vel_ref = struct.unpack_from('<f', data, OFF_VEL_REF)[0]
    iq_ref = struct.unpack_from('<f', data, OFF_IQ_REF)[0]
    pos = struct.unpack_from('<f', data, OFF_POS)[0]
    vel_obs = struct.unpack_from('<f', data, OFF_VEL_OBS)[0]
    vel_filt = struct.unpack_from('<f', data, OFF_VEL_FILT)[0]

    return {
        'state': state,
        'mode': mode,
        'calib_valid': calib_valid,
        'target': target_val,
        'vel_ref': vel_ref,
        'iq_ref': iq_ref,
        'pos': pos,
        'vel_obs': vel_obs,
        'vel_filt': vel_filt
    }

def safe_abort(target, reason, sample=None):
    print(f"\n[!!! 触发安全看门狗紧急熔断 !!!] 原因: {reason}")
    if sample:
        print(f"熔断瞬间遥测: pos={sample['pos']:.4f}, vel_obs={sample['vel_obs']:.1f}, vel_filt={sample['vel_filt']:.1f}, iq={sample['iq_ref']:.3f}, vel_ref={sample['vel_ref']:.1f}")
    inject_cmd(target, "disable")
    time.sleep(0.05)
    st = read_motor(target)
    print(f"紧急停机完成: state={st['state']} (0=IDLE), pos={st['pos']:.4f}")
    sys.exit(1)

def main():
    if len(sys.argv) < 2:
        print("用法: python tools/test_pos_step_safe.py <target_rad> [wait_sec]")
        sys.exit(1)

    target_rad = float(sys.argv[1])
    wait_sec = float(sys.argv[2]) if len(sys.argv) >= 3 else 3.0

    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    target = session.target
    if target.is_halted(): target.resume()

    # 1. 检查校准与状态
    st0 = read_motor(target)
    if st0['calib_valid'] == 0:
        print("物理零点未校准，开始执行校准...")
        inject_cmd(target, "calib")
        t0 = time.time()
        while time.time() - t0 < 15.0:
            st = read_motor(target)
            if st['calib_valid'] != 0 and st['state'] == 0:
                print("校准成功完成！")
                break
            time.sleep(0.3)
        else:
            safe_abort(target, "校准超时")

    # 2. 若未使能，则进入 pos 模式并使能
    st = read_motor(target)
    if st['state'] != 1 or st['mode'] != 3:
        print("电机未在 POS 运行态，正在初始化使能...")
        inject_cmd(target, "mode pos")
        inject_cmd(target, "target 0.000")
        inject_cmd(target, "enable")
        time.sleep(0.3)

    st_before = read_motor(target)
    init_pos = st_before['pos']
    init_tgt = st_before['target']
    expected_delta = target_rad - init_tgt

    print(f"=======================================================")
    print(f"  单步安全阶跃测试: target -> {target_rad:+.4f} rad ({target_rad*180/math.pi:+.1f}°)")
    print(f"  当前位置: {init_pos:.4f} rad, 当前target: {init_tgt:.4f} rad")
    print(f"  预期位移增量: {expected_delta:+.4f} rad ({expected_delta*180/math.pi:+.1f}°)")
    print(f"  安全看门狗阈值: 转速 < 120 RPM, 电流 <= 0.50A")
    print(f"=======================================================")

    # 注入阶跃目标
    inject_cmd(target, f"target {target_rad:.4f}")
    t0 = time.time()
    samples = []
    sign_flips = 0
    last_v_sign = 0

    while time.time() - t0 < wait_sec:
        s = read_motor(target)
        s['t'] = time.time() - t0
        samples.append(s)

        # 毫秒级看门狗 1: 绝对转速上限保护 (防甩保护)
        if abs(s['vel_filt']) > 120.0:
            safe_abort(target, f"真实滤波转速超限! vel_filt={s['vel_filt']:.1f} RPM", s)
        if abs(s['vel_obs']) > 160.0:
            safe_abort(target, f"观测器瞬态转速超限! vel_obs={s['vel_obs']:.1f} RPM", s)

        # 毫秒级看门狗 2: 绝对电流限幅检查
        if abs(s['iq_ref']) > 0.505:
            safe_abort(target, f"电流给定超出绝对物理硬限幅! iq_ref={s['iq_ref']:.3f} A", s)

        # 毫秒级看门狗 3: 连续大幅度振荡检测
        cur_sign = 1 if s['vel_filt'] > 30.0 else (-1 if s['vel_filt'] < -30.0 else 0)
        if cur_sign != 0:
            if last_v_sign != 0 and cur_sign != last_v_sign:
                sign_flips += 1
                if sign_flips >= 4:
                    safe_abort(target, f"检测到连续剧烈振荡翻转! 已翻转 {sign_flips} 次", s)
            last_v_sign = cur_sign

        time.sleep(0.02) # 50Hz 采样

    # 分析实测结果
    poses = [s['pos'] for s in samples]
    v_filts = [s['vel_filt'] for s in samples]
    iqs = [s['iq_ref'] for s in samples]
    final_pos = poses[-1]
    actual_delta = final_pos - init_pos
    max_v = max(abs(v) for v in v_filts)
    max_iq = max(abs(i) for i in iqs)
    tail_v = sum(v_filts[-10:]) / 10.0
    tail_iq = sum(iqs[-10:]) / 10.0

    if expected_delta > 0:
        overshoot = max(0.0, max(poses) - (init_pos + expected_delta))
    else:
        overshoot = max(0.0, (init_pos + expected_delta) - min(poses))

    print(f"\n实测响应指标 ({len(samples)} 采样点, 耗时 {wait_sec:.1f}s):")
    print(f"  起始位置     : {init_pos:.4f} rad")
    print(f"  最终到达位置 : {final_pos:.4f} rad")
    print(f"  实际位移增量 : {actual_delta:+.4f} rad (预期: {expected_delta:+.4f} rad)")
    print(f"  最大瞬时转速 : {max_v:.1f} RPM")
    print(f"  最大电流给定 : {max_iq:.3f} A")
    print(f"  超调量       : {overshoot:.4f} rad ({overshoot*180/math.pi:.2f}°)")
    print(f"  稳态残余速度 : {tail_v:.2f} RPM")
    print(f"  稳态保持电流 : {tail_iq:.3f} A")

    is_safe = max_iq <= 0.505 and max_v <= 120.0
    is_stable = abs(tail_v) < 1.0
    print(f"  安全看门狗状态: {'PASS (未触发任何异常熔断)' if is_safe else 'FAIL'}")
    print(f"  稳态锁定状态  : {'PASS (平稳锁定)' if is_stable else 'FAIL'}")

    session.close()

if __name__ == "__main__":
    main()
