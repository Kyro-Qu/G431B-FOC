# -*- coding: utf-8 -*-
import struct
from pyocd.core.helpers import ConnectHelper

with ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True}) as session:
    t = session.target
    if t.is_halted(): t.resume()
    # traj 在 foc_motor_t 中
    # 让我们通过结构体偏移找 pos_origin_rad
    # foc_motor.h:
    # traj 是 foc_traj_t
    # float traj_target_latch;
    # float pos_origin_rad;
    # 让我们读 250~412
    raw = bytes(t.read_memory_block8(0x20002e1c, 412))

print("=== 搜索可能为 pos_origin_rad (约 115 或 507) 的字段 ===")
for off in range(200, 400, 4):
    f = struct.unpack_from('<f', raw, off)[0]
    if abs(f - 115.0) < 5.0 or abs(f - 507.0) < 5.0:
        print(f"[{off}] float={f:.4f}")
