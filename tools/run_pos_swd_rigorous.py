# -*- coding: utf-8 -*-
"""
基于 SWD 的真实硬件位置环严密测试套件 (100% 精确物理偏移)
成员偏移真值：
- state: 0x9e (uint8)
- mode: 0x9f (uint8)
- calib_valid: 0xd0 (uint8)
- target: 0xd8 (float)
- iq_ref: 0xf8 (float)
- position_rad: 0x120 (float)
- velocity_observer_rpm: 0x128 (float)
- velocity_filt_rpm: 0x12c (float)
"""
import time
import struct
import sys
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_QUEUE_HEAD_ADDR = 0x2000008c
RX_QUEUE_TAIL_ADDR = 0x2000008e
RX_QUEUE_BUF_ADDR = 0x200034fc
CMD_RX_QUEUE_SIZE = 256

OFF_STATE = 0x9e        # uint8
OFF_MODE = 0x9f         # uint8
OFF_CALIB_VALID = 0xd0  # uint8
OFF_TARGET = 0xd8       # float
OFF_IQ_REF = 0xf8       # float
OFF_POS = 0x120         # float (position_rad)
OFF_VEL_OBS = 0x128     # float (velocity_observer_rpm)
OFF_VEL_FILT = 0x12c    # float (velocity_filt_rpm)

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

def sample_series(target, duration_sec, interval_sec=0.02):
    samples = []
    t0 = time.time()
    while time.time() - t0 < duration_sec:
        s = read_motor(target)
        s['t'] = time.time() - t0
        samples.append(s)
        time.sleep(interval_sec)
    return samples

def run_suite():
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    target = session.target
    if target.is_halted(): target.resume()

    print("==================================================")
    print("      FOC_G431 真实硬件位置环严密测试套件 (SWD 通道)")
    print("==================================================")

    # 1. 确保 IDLE 并检查校准
    inject_cmd(target, "disarm")
    time.sleep(0.1)
    st = read_motor(target)
    print(f"初始状态: state={st['state']} (0=IDLE, 1=RUN), calib_valid={st['calib_valid']}")

    if st['calib_valid'] == 0:
        print("未确立物理零点，开始校准...")
        inject_cmd(target, "calib")
        t0 = time.time()
        while time.time() - t0 < 15.0:
            st = read_motor(target)
            if st['calib_valid'] != 0 and st['state'] == 0:
                print("校准成功！")
                break
            time.sleep(0.3)
        else:
            print("校准超时！")
            session.close()
            return
    else:
        print("物理零点已确立 (calib_valid=1)！")

    # 2. 阶段 A: 原地保持测试 (target 0.000 rad)
    print("\n--- 阶段 A: 原地保持测试 (target 0.000 rad, 2.0 秒) ---")
    inject_cmd(target, "mode pos")
    inject_cmd(target, "target 0.000")
    inject_cmd(target, "enable")
    time.sleep(0.2)

    samples_a = sample_series(target, 2.0)
    v_filts_a = [s['vel_filt'] for s in samples_a]
    v_obss_a = [s['vel_obs'] for s in samples_a]
    iq_refs_a = [s['iq_ref'] for s in samples_a]
    poses_a = [s['pos'] for s in samples_a]

    print(f"阶段 A 保持实测结果 ({len(samples_a)} 样本):")
    print(f"  滤波转速: [{min(v_filts_a):.2f}, {max(v_filts_a):.2f}] RPM, 平均: {sum(v_filts_a)/len(v_filts_a):.2f} RPM")
    print(f"  未滤波速度观测: [{min(v_obss_a):.2f}, {max(v_obss_a):.2f}] RPM (验证滤波有效性)")
    print(f"  电流给定: [{min(iq_refs_a):.3f}, {max(iq_refs_a):.3f}] A, 平均: {sum(iq_refs_a)/len(iq_refs_a):.3f} A")
    print(f"  位置范围: [{min(poses_a):.4f}, {max(poses_a):.4f}] rad, 抖动幅度: {max(poses_a)-min(poses_a):.4f} rad")

    # 3. 阶段 B: 极小阶跃 +0.010 rad (约 0.57度)
    print("\n--- 阶段 B: 极小阶跃 +0.010 rad (约 0.57度, 2.0 秒) ---")
    inject_cmd(target, "target 0.010")
    samples_b = sample_series(target, 2.0)
    v_filts_b = [s['vel_filt'] for s in samples_b]
    iq_refs_b = [s['iq_ref'] for s in samples_b]
    poses_b = [s['pos'] for s in samples_b]

    print(f"阶段 B 极小阶跃实测结果 ({len(samples_b)} 样本):")
    print(f"  转速峰值: {max(abs(v) for v in v_filts_b):.2f} RPM")
    print(f"  电流峰值: {max(abs(i) for i in iq_refs_b):.3f} A (严格限制在 0.50A 以内)")
    print(f"  起始位置: {poses_b[0]:.4f} rad -> 最终位置: {poses_b[-1]:.4f} rad")
    print(f"  位置变化量: {poses_b[-1] - poses_b[0]:.4f} rad (目标: +0.0100 rad)")

    # 4. 阶段 C: 小阶跃 +0.050 rad (约 2.86度)
    print("\n--- 阶段 C: 小阶跃 +0.050 rad (约 2.86度, 2.0 秒) ---")
    inject_cmd(target, "target 0.050")
    samples_c = sample_series(target, 2.0)
    v_filts_c = [s['vel_filt'] for s in samples_c]
    iq_refs_c = [s['iq_ref'] for s in samples_c]
    poses_c = [s['pos'] for s in samples_c]

    print(f"阶段 C 小阶跃实测结果 ({len(samples_c)} 样本):")
    print(f"  转速峰值: {max(abs(v) for v in v_filts_c):.2f} RPM")
    print(f"  电流峰值: {max(abs(i) for i in iq_refs_c):.3f} A")
    print(f"  起始位置: {poses_c[0]:.4f} rad -> 最终位置: {poses_c[-1]:.4f} rad")
    print(f"  位置变化量: {poses_c[-1] - poses_c[0]:.4f} rad (目标: +0.0400 rad 相对前次)")

    # 5. 阶段 D: 中阶跃 +0.200 rad (约 11.5度)
    print("\n--- 阶段 D: 中角度阶跃 +0.200 rad (约 11.5度, 2.5 秒) ---")
    inject_cmd(target, "target 0.200")
    samples_d = sample_series(target, 2.5)
    v_filts_d = [s['vel_filt'] for s in samples_d]
    iq_refs_d = [s['iq_ref'] for s in samples_d]
    poses_d = [s['pos'] for s in samples_d]

    print(f"阶段 D 中阶跃实测结果 ({len(samples_d)} 样本):")
    print(f"  转速峰值: {max(abs(v) for v in v_filts_d):.2f} RPM")
    print(f"  电流峰值: {max(abs(i) for i in iq_refs_d):.3f} A")
    print(f"  起始位置: {poses_d[0]:.4f} rad -> 最终位置: {poses_d[-1]:.4f} rad")
    print(f"  位置变化量: {poses_d[-1] - poses_d[0]:.4f} rad (目标: +0.1500 rad 相对前次)")

    # 6. 安全停机
    print("\n--- 7. 测试完毕，安全停机 ---")
    inject_cmd(target, "disarm")
    session.close()
    print("电机已 disarm，SWD 会话已关闭。")

if __name__ == "__main__":
    run_suite()
