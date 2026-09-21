# -*- coding: utf-8 -*-
"""
大角度阶跃时域高频追踪 (100Hz 详细时域打点)
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
OFF_VEL_REF = 0xdc
OFF_IQ_REF = 0xf8
OFF_POS = 0x120
OFF_VEL_OBS = 0x128
OFF_VEL_FILT = 0x12c
OFF_TRAJ_ACTIVE = 0x238 # 需要确认 traj 在 motor 里的偏移

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
    data = bytes(target.read_memory_block8(MOTOR_BASE, 0x160))
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

def main():
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    target = session.target
    if target.is_halted(): target.resume()

    print("=== 开始大角度阶跃高频时域追踪 ===")
    inject_cmd(target, "mode pos")
    inject_cmd(target, "target 0.000")
    inject_cmd(target, "enable")
    time.sleep(0.5)

    st0 = read_motor(target)
    print(f"起始点: pos={st0['pos']:.4f} rad, vel_filt={st0['vel_filt']:.2f} RPM, iq={st0['iq_ref']:.3f} A")

    # 给大角度阶跃 +3.1416 rad (180度)
    print("\n--- 注入指令: target 3.1416 ---")
    inject_cmd(target, "target 3.1416")

    t0 = time.time()
    records = []
    while time.time() - t0 < 3.5:
        st = read_motor(target)
        st['t'] = time.time() - t0
        records.append(st)
        time.sleep(0.02) # 50Hz

    print("\n时间(s)   目标(rad)  实际pos(rad)  vel_ref(RPM)  vel_filt(RPM)  iq_ref(A)")
    print("-" * 75)
    for i, r in enumerate(records):
        if i % 3 == 0 or abs(r['vel_filt']) > 50 or i == len(records)-1:
            print(f"{r['t']:6.3f}   {r['target']:9.4f}  {r['pos']:12.4f}  {r['vel_ref']:12.2f}  {r['vel_filt']:13.2f}  {r['iq_ref']:9.3f}")

    print("\n安全停机 disarm...")
    inject_cmd(target, "disarm")
    session.close()

if __name__ == "__main__":
    main()
