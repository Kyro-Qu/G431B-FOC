# -*- coding: utf-8 -*-
"""
验证 FOC_G431 电机参数存储与手写 CLI 闭环
"""
import struct
import time
from pyocd.core.helpers import ConnectHelper

with ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True}) as session:
    t = session.target
    if t.is_halted():
        t.resume()

    # 读 g_foc_motors[0] 前 48 字节 (12 字节指针 + 28 字节 foc_motor_params_t)
    raw = bytes(t.read_memory_block8(0x20002e1c, 48))
    drv, cur, sensor = struct.unpack_from('<III', raw, 0)
    pp, rs, ls, ke, max_i, hard_i, max_rpm = struct.unpack_from('<7f', raw, 12)

    print("=== M0 Current RAM Params ===")
    print(f"pp:          {pp:.0f}")
    print(f"rs:          {rs:.4f} Ohm")
    print(f"ls:          {ls*1e6:.2f} uH")
    print(f"ke:          {ke:.4f} V/krpm")
    print(f"max_i:       {max_i:.2f} A")
    print(f"hard_i:      {hard_i:.2f} A")
    print(f"max_rpm:     {max_rpm:.0f} RPM")

    # 验证 max_rpm 是我们固化成功的 7500 RPM
    assert abs(max_rpm - 7500.0) < 1.0, f"Expected max_rpm=7500, got {max_rpm}"
    assert abs(pp - 6.0) < 0.1, f"Expected pp=6, got {pp}"
    assert abs(hard_i - 12.0) < 0.1, f"Expected hard_i=12.0, got {hard_i}"

    print("\n>>> ALL CHECKS PASSED: max_rpm 7500 persisted & loaded correctly! <<<")
