# -*- coding: utf-8 -*-
"""
位置环双向往返动态测试 (SWD 通道)
路径: 0.000 -> +0.100 -> +0.300 -> +0.100 -> 0.000
检验正反双向阶跃跟踪、加减速平滑性、超调抑制与稳态锁定。
"""
import time
import struct
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

def run_step(target, target_rad, hold_sec):
    print(f"\n>>> 设定目标 target = {target_rad:+.3f} rad, 观测 {hold_sec:.1f} 秒...")
    inject_cmd(target, f"target {target_rad:.4f}")
    t0 = time.time()
    samples = []
    while time.time() - t0 < hold_sec:
        s = read_motor(target)
        s['t'] = time.time() - t0
        samples.append(s)
        time.sleep(0.03)

    v_filts = [s['vel_filt'] for s in samples]
    iqs = [s['iq_ref'] for s in samples]
    poses = [s['pos'] for s in samples]

    print(f"    轨迹表现 ({len(samples)} 样本):")
    print(f"    转速范围: [{min(v_filts):.1f}, {max(v_filts):.1f}] RPM")
    print(f"    电流范围: [{min(iqs):.3f}, {max(iqs):.3f}] A (安全限幅 0.50A)")
    print(f"    位置轨迹: 起始={poses[0]:.4f} -> 峰值={max(poses):.4f}/谷值={min(poses):.4f} -> 终点={poses[-1]:.4f} rad")
    return samples

def main():
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    target = session.target
    if target.is_halted(): target.resume()

    print("=== 开始双向往返动态阶跃测试 ===")
    inject_cmd(target, "mode pos")
    inject_cmd(target, "target 0.000")
    inject_cmd(target, "enable")
    time.sleep(0.2)

    # 步骤 1: 原点保持
    run_step(target, 0.000, 1.5)

    # 步骤 2: 正向阶跃 +0.100 rad (5.73度)
    run_step(target, 0.100, 1.8)

    # 步骤 3: 进一步正向阶跃 +0.300 rad (17.19度)
    run_step(target, 0.300, 2.0)

    # 步骤 4: 反向返回阶跃 +0.100 rad
    run_step(target, 0.100, 1.8)

    # 步骤 5: 反向返回原点 0.000 rad
    run_step(target, 0.000, 1.8)

    print("\n=== 测试完成，安全停机 disarm ===")
    inject_cmd(target, "disarm")
    session.close()

if __name__ == "__main__":
    main()
