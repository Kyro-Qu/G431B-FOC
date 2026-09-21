# -*- coding: utf-8 -*-
"""
完整解析 g_foc_motors[0] (0x20002e1c) 的内存布局
"""
import struct
from pyocd.core.helpers import ConnectHelper

with ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True}) as session:
    t = session.target
    if t.is_halted(): t.resume()
    raw = bytes(t.read_memory_block8(0x20002e1c, 320))

print("=== 内存结构解析 (0x20002e1c) ===")
# 0~12: 3 pointers (drv, cur, sensor)
drv, cur, sensor = struct.unpack_from('<III', raw, 0)
print(f"drv=0x{drv:08x}, cur=0x{cur:08x}, sensor=0x{sensor:08x}")

# 12~40: foc_motor_params_t (7 floats)
pp, rs, ls, kv, max_i, max_rpm, hard_i = struct.unpack_from('<7f', raw, 12)
print(f"params: pp={pp:.1f}, rs={rs:.3f}, ls={ls*1e6:.1f}uH, kv={kv:.0f}, max_i={max_i:.2f}A, max_rpm={max_rpm:.0f}, hard_i={hard_i:.2f}A")

# 40~108: foc_ctrl_cfg_t
# 40: current_bw_rads
# 44: vel_kp, 48: vel_ki, 52: vel_ramp, 56: vel_lpf_tf, 60: friction, 64: vel_start_a, 68: vel_start_rpm, 72: vel_track_kp, 76: track_limit, 80: track_rpm
# 84: pos_kp, 88: pos_ki, 92: pos_vel_kp, 96: pos_vel_limit_rpm, 100: traj_enable (uint8 + 3 pad), 104: traj_accel_rpm_s
pos_kp, pos_ki, pos_vel_kp, pos_vmax = struct.unpack_from('<4f', raw, 84)
traj_en = raw[100]
traj_accel = struct.unpack_from('<f', raw, 104)[0]
print(f"ctrl_cfg: pos_kp={pos_kp:.3f}, pos_ki={pos_ki:.3f}, pos_vkp={pos_vel_kp:.4f}, pos_vmax={pos_vmax:.1f}, traj_en={traj_en}, traj_accel={traj_accel:.1f}")

# 108~132: decouple_enable(1B)+pad(3B), deadtime_comp_v(4B), stall_enable(1B)+pad(3B), stall_rpm(4B), stall_timeout_ms(2B)+pad(2B)
# 128: runtime.angle_delay_cycles(4B), fieldweak_enable(1B)+pad(3B), fieldweak_enter_rpm(4B)...
# 让我们看看 132~160
# dt_fast, slow_div 在哪里？
# 让我们看 140~180
print("\n=== 140 ~ 180 详细字节 ===")
for off in range(140, 180, 4):
    u = struct.unpack_from('<I', raw, off)[0]
    f = struct.unpack_from('<f', raw, off)[0]
    b0, b1, b2, b3 = raw[off], raw[off+1], raw[off+2], raw[off+3]
    print(f"+{off:03d} (0x{off:02x}): 0x{u:08x} | bytes: {b0:3d}, {b1:3d}, {b2:3d}, {b3:3d} | float: {f:10.4f}")

# 200~230:
print("\n=== 200 ~ 240 详细字节 ===")
for off in range(200, 240, 4):
    u = struct.unpack_from('<I', raw, off)[0]
    f = struct.unpack_from('<f', raw, off)[0]
    b0, b1, b2, b3 = raw[off], raw[off+1], raw[off+2], raw[off+3]
    print(f"+{off:03d} (0x{off:02x}): 0x{u:08x} | bytes: {b0:3d}, {b1:3d}, {b2:3d}, {b3:3d} | float: {f:10.4f}")
