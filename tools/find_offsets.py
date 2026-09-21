# -*- coding: utf-8 -*-
"""
严格按照 foc_motor.h 计算各字段 offset 并验证
"""
from pyocd.core.helpers import ConnectHelper
import struct

# 读 500 字节
with ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True}) as session:
    t = session.target
    if t.is_halted(): t.resume()
    raw = bytes(t.read_memory_block8(0x20002e1c, 450))

# 打印所有可能为 state 的位置 (等于 0, 1, 2, 3)
for off in range(0, 350, 4):
    val = struct.unpack_from('<I', raw, off)[0]
    f = struct.unpack_from('<f', raw, off)[0]
    # 我们知道 params.pole_pairs = 7.0 在 offset 12
    # params.max_rpm = 12000.0 在 offset 32
    if off == 12:
        print(f"[{off}] pole_pairs = {f}")
    if abs(f - 12000.0) < 1.0:
        print(f"[{off}] max_rpm = {f}")
    if abs(f - 8000.0) < 1.0:
        print(f"[{off}] traj_accel_rpm_s = {f}")
    if abs(f - 1000.0) < 1.0:
        print(f"[{off}] runtime.fieldweak_enter_rpm = {f}")

print("\n打印 100~200 字节的解析:")
for off in range(100, 220, 4):
    val = struct.unpack_from('<I', raw, off)[0]
    f = struct.unpack_from('<f', raw, off)[0]
    print(f"+{off:03d} (0x{off:02x}): uint32=0x{val:08x} ({val:4d}), float={f:10.4f}")
