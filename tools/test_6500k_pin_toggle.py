# -*- coding: utf-8 -*-
import time
import serial
from pyocd.core.helpers import ConnectHelper

session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
session.open()
t = session.target

print("=== 测试 6.5Mbps 下往 COM44 发送 1000 个 0x00 时 PB4 的电平 ===")
ser = serial.Serial("COM44", 6500000, timeout=0.1)
time.sleep(0.05)

ser.write(b"\x00" * 1000)
ser.flush()

active_levels = []
t0 = time.time()
while time.time() - t0 < 0.2:
    idr = t.read32(0x48000400 + 0x10)
    active_levels.append((idr >> 4) & 1)

print(f"6.5Mbps 发送期间 PB4 采样 ({len(active_levels)} 次): 1={active_levels.count(1)}, 0={active_levels.count(0)}")

ser.close()
session.close()
