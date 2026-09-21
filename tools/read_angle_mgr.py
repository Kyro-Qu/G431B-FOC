# -*- coding: utf-8 -*-
"""
读取 MCU 当前电机结构体里的 cfg 参数
"""
import struct
from pyocd.core.helpers import ConnectHelper

session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
session.open()
t = session.target
raw = bytes(t.read_memory_block8(0x20002e1c, 824))

# 检查 m0_cfg 在内存中的位置
# foc_motor_t 开头是:
# params (7*4 = 28)
# cfg:
# current_bw_rads (4)
# vel_kp, vel_ki, vel_ramp_rpm_s, vel_lpf_tf, vel_friction_a, vel_start_a, vel_start_rpm, vel_track_kp, vel_track_limit_rad, vel_track_rpm (10*4 = 40)
# pos_kp, pos_ki, pos_vel_kp, pos_vel_limit_rpm, traj_enable, traj_accel_rpm_s (6*4 = 24)
# ...
print("扫描 cfg 区域...")
for off in range(28, 140, 4):
    f = struct.unpack_from('<f', raw, off)[0]
    u = struct.unpack_from('<I', raw, off)[0]
    print(f"[{off:03d}] uint=0x{u:08x}, float={f:12.4f}")

session.close()
