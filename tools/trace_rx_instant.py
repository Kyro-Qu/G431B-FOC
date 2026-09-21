# -*- coding: utf-8 -*-
"""
实时抓取 MCU USART2 ISR 在 PC 发送瞬间的变化
"""
import time
import serial
from pyocd.core.helpers import ConnectHelper

session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
session.open()
t = session.target

ser = serial.Serial("COM44", 6500000, timeout=0.1)
time.sleep(0.05)

print("正在监听 USART2 ISR 状态...")
ser.write(b"version\r\n")

# 高频抓取 50 次 ISR
isr_samples = []
for _ in range(50):
    isr = t.read32(0x40004400 + 0x1c)
    cndtr = t.read32(0x40020020)
    isr_samples.append((isr, cndtr))

ser.close()
session.close()

# 统计分析
unique_isrs = set(s[0] for s in isr_samples)
print("捕获到的 ISR 集合:", [f"0x{x:08x}" for x in unique_isrs])
print("捕获到的 CNDTR 集合:", set(s[1] for s in isr_samples))
