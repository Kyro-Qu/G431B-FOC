# -*- coding: utf-8 -*-
"""
在 6.5Mbps 下执行往返端到端测试，并监控 DMA 与内存
"""
import time
import serial
from pyocd.core.helpers import ConnectHelper

session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
session.open()
t = session.target
if t.is_halted(): t.resume()

ser = serial.Serial("COM44", 6500000, timeout=0.2)
time.sleep(0.1)

cndtr_0 = t.read32(0x40020020)
head_0 = t.read16(0x2000008c)
print(f"发送前: DMA1_CH2 CNDTR={cndtr_0}, rx_head={head_0}")

ser.reset_input_buffer()
cmd = b"status\r\n"
print(f"发送命令: {cmd}")
ser.write(cmd)
ser.flush()
time.sleep(0.05)

cndtr_1 = t.read32(0x40020020)
head_1 = t.read16(0x2000008c)
print(f"发送后: DMA1_CH2 CNDTR={cndtr_1}, rx_head={head_1}")

rx_data = bytearray()
t0 = time.time()
while time.time() - t0 < 0.5:
    chunk = ser.read(ser.in_waiting or 1)
    if chunk: rx_data.extend(chunk)

print(f"PC 接收结果: len={len(rx_data)} 字节")
if len(rx_data) > 0:
    print(f"内容预览: {repr(bytes(rx_data[:150]))}")

ser.close()
session.close()
