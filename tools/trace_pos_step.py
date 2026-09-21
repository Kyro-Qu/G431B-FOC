# -*- coding: utf-8 -*-
"""
超微观起步 300ms 高频采样 (200Hz 详细时域切片)
"""
import time
import struct
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_QUEUE_HEAD_ADDR = 0x2000008c
RX_QUEUE_TAIL_ADDR = 0x2000008e
RX_QUEUE_BUF_ADDR = 0x200034fc

OFF_TARGET = 0xd8
OFF_VEL_REF = 0xe4
OFF_IQ_REF = 0xf8
OFF_POS = 0x120
OFF_VEL_OBS = 0x128
OFF_VEL_FILT = 0x12c

def inject_cmd(target, cmd_str):
    if not cmd_str.endswith('\r\n'):
        cmd_str += '\r\n'
    cmd_bytes = cmd_str.encode('ascii')
    head = target.read16(RX_QUEUE_HEAD_ADDR)
    for b in cmd_bytes:
        target.write8(RX_QUEUE_BUF_ADDR + head, b)
        head = (head + 1) % 256
    target.write16(RX_QUEUE_HEAD_ADDR, head)
    t0 = time.time()
    while time.time() - t0 < 0.5:
        if target.read16(RX_QUEUE_TAIL_ADDR) == head:
            break
        time.sleep(0.005)

def read_point(target):
    data = bytes(target.read_memory_block8(MOTOR_BASE, 0x140))
    return {
        'vel_ref': struct.unpack_from('<f', data, OFF_VEL_REF)[0],
        'iq_ref': struct.unpack_from('<f', data, OFF_IQ_REF)[0],
        'pos': struct.unpack_from('<f', data, OFF_POS)[0],
        'vel_obs': struct.unpack_from('<f', data, OFF_VEL_OBS)[0],
        'vel_filt': struct.unpack_from('<f', data, OFF_VEL_FILT)[0]
    }

def main():
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    target = session.target
    if target.is_halted(): target.resume()

    inject_cmd(target, "mode pos")
    inject_cmd(target, "target 0.000")
    inject_cmd(target, "enable")
    time.sleep(0.3)

    p0 = read_point(target)
    print(f"使能初始点: pos={p0['pos']:.4f}, vel_obs={p0['vel_obs']:.2f}, iq={p0['iq_ref']:.3f}")

    # 给微小阶跃 0.200 rad，观测其高频起步
    print("\n--- 注入 target 0.200 ---")
    inject_cmd(target, "target 0.200")

    records = []
    t0 = time.time()
    while time.time() - t0 < 0.4:
        r = read_point(target)
        r['t'] = time.time() - t0
        records.append(r)
        time.sleep(0.005) # 200Hz

    print(f"采样完成 ({len(records)} 点):")
    print(f"{'t(s)':<8} {'pos(rad)':<12} {'vel_ref(RPM)':<14} {'vel_obs(RPM)':<14} {'vel_filt(RPM)':<14} {'iq(A)':<8}")
    print("-" * 75)
    for r in records[::2]: # 打印前 100ms~400ms
        print(f"{r['t']:<8.3f} {r['pos']:<12.4f} {r['vel_ref']:<14.2f} {r['vel_obs']:<14.2f} {r['vel_filt']:<14.2f} {r['iq_ref']:<8.3f}")

    inject_cmd(target, "disable")
    session.close()

if __name__ == "__main__":
    main()
