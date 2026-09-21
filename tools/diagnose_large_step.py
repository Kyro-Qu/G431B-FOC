# -*- coding: utf-8 -*-
"""
诊断 4.7124 rad (270度) 阶跃的详细时域采样，找出 vel_obs 冲高的原因
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
    print(f"\n[安全看门狗熔断] 原因: {reason}")
    if sample:
        print(f"熔断点: t={sample['t']:.3f}s, pos={sample['pos']:.4f}, vel_obs={sample['vel_obs']:.1f}, vel_filt={sample['vel_filt']:.1f}, iq={sample['iq_ref']:.3f}, vel_ref={sample['vel_ref']:.1f}")
    inject_cmd(target, "disable")
    time.sleep(0.05)

def main():
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    target = session.target
    if target.is_halted(): target.resume()

    inject_cmd(target, "mode pos")
    inject_cmd(target, "target 0.000")
    inject_cmd(target, "enable")
    time.sleep(0.3)

    st0 = read_motor(target)
    print(f"起始态: pos={st0['pos']:.4f}, vel_obs={st0['vel_obs']:.2f}, vel_filt={st0['vel_filt']:.2f}, iq={st0['iq_ref']:.3f}")

    target_rad = 4.7124
    print(f"\n注入阶跃 target -> {target_rad:.4f} rad (270度)...")
    inject_cmd(target, f"target {target_rad:.4f}")

    samples = []
    t0 = time.time()
    aborted = False

    while time.time() - t0 < 3.0:
        s = read_motor(target)
        s['t'] = time.time() - t0
        samples.append(s)

        if abs(s['vel_filt']) > 120.0:
            safe_abort(target, f"vel_filt超限 ({s['vel_filt']:.1f} RPM)", s)
            aborted = True
            break
        if abs(s['vel_obs']) > 220.0:
            safe_abort(target, f"vel_obs超限 ({s['vel_obs']:.1f} RPM)", s)
            aborted = True
            break
        if abs(s['iq_ref']) > 0.505:
            safe_abort(target, f"iq_ref超限 ({s['iq_ref']:.3f} A)", s)
            aborted = True
            break

        time.sleep(0.01) # 100Hz 采样

    if not aborted:
        inject_cmd(target, "disable")

    print(f"\n捕获到 {len(samples)} 个时域样本:")
    print(f"{'t(s)':<7} {'pos(rad)':<11} {'vel_ref':<10} {'vel_obs':<10} {'vel_filt':<10} {'iq_ref':<8}")
    print("-" * 65)
    for s in samples:
        print(f"{s['t']:<7.3f} {s['pos']:<11.4f} {s['vel_ref']:<10.1f} {s['vel_obs']:<10.1f} {s['vel_filt']:<10.1f} {s['iq_ref']:<8.3f}")

    session.close()

if __name__ == "__main__":
    main()
