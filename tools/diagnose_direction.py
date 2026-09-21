# -*- coding: utf-8 -*-
"""
极低电流 (0.2A) 物理旋转方向与编码器增量极性诊断脚本
1. 保证绝对物理安全 (0.2A 只有正常空载电流的 10%)
2. 测定 iq_ref > 0 时 position_rad 是增加还是减小
"""
import time
import struct
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_HEAD = 0x2000008c
RX_TAIL = 0x2000008e
RX_BUF = 0x200034fc

session = ConnectHelper.session_with_chosen_probe(
    target_override='cortex_m',
    options={'halt_on_connect': False, 'resume_on_exit': True}
)
session.open()
t = session.target
if t.is_halted(): t.resume()

def run_cmd(cmd_str):
    if not cmd_str.endswith('\r\n'):
        cmd_str = cmd_str.rstrip('\r\n') + '\r\n'
    raw = cmd_str.encode('ascii')
    head = t.read16(RX_HEAD)
    for b in raw:
        t.write8(RX_BUF + head, b)
        head = (head + 1) % 256
    t.write16(RX_HEAD, head)
    if t.is_halted(): t.resume()
    t0 = time.time()
    while time.time() - t0 < 0.8:
        if t.read16(RX_TAIL) == head:
            break
        time.sleep(0.01)
    time.sleep(0.02)

def read_pos_and_calib():
    raw_calib = bytes(t.read_memory_block8(MOTOR_BASE + 208, 8))
    calib_valid = raw_calib[0]
    direction = struct.unpack('<b', raw_calib[2:3])[0]
    offset = struct.unpack('<f', raw_calib[4:8])[0]
    raw_pos = bytes(t.read_memory_block8(MOTOR_BASE + 288, 16))
    pos = struct.unpack_from('<f', raw_pos, 0)[0]
    vel = struct.unpack_from('<f', raw_pos, 8)[0]
    return calib_valid, direction, offset, pos, vel

try:
    print("1. 清障与状态检查...")
    run_cmd("fault clear")
    run_cmd("disable")
    valid, direction, offset, pos0, vel0 = read_pos_and_calib()
    print(f"校准参数: valid={valid}, direction={direction}, offset={offset:.4f}")
    print(f"初始位置: pos={pos0:.4f} rad")

    if valid == 0:
        print("执行 calib...")
        run_cmd("calib")
        for _ in range(30):
            time.sleep(0.4)
            valid, direction, offset, pos0, vel0 = read_pos_and_calib()
            if valid != 0:
                print("校准完成！")
                break

    print("\n2. 进入力矩模式 (mode iq)，施加微弱正向力矩 target +0.20A (持续 0.4s)...")
    run_cmd("mode iq")
    run_cmd("target 0.20")
    run_cmd("enable")
    time.sleep(0.4)
    valid, direction, offset, pos1, vel1 = read_pos_and_calib()
    print(f"施加 target +0.20A 后: pos={pos1:.4f} rad, delta={pos1-pos0:+.4f} rad, vel={vel1:+.2f} RPM")

    print("\n3. 停止力矩 (target 0.00A)...")
    run_cmd("target 0.00")
    time.sleep(0.2)

    print("\n4. 施加微弱反向力矩 target -0.20A (持续 0.4s)...")
    run_cmd("target -0.20")
    time.sleep(0.4)
    valid, direction, offset, pos2, vel2 = read_pos_and_calib()
    print(f"施加 target -0.20A 后: pos={pos2:.4f} rad, delta={pos2-pos1:+.4f} rad, vel={vel2:+.2f} RPM")

    print("\n5. 安全停机...")
    run_cmd("disable")
    time.sleep(0.1)

finally:
    if t.is_halted(): t.resume()
    session.close()
