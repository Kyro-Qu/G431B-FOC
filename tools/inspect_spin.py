# -*- coding: utf-8 -*-
"""测试开环旋转时的实际 dq 电流和反电势 (COM44)"""
import sys
import time
import serial

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

ser = serial.Serial("COM44", 6500000, timeout=0.2)
time.sleep(0.2)
ser.reset_input_buffer()

def send(cmd):
    ser.write((cmd + "\n").encode("ascii"))
    time.sleep(0.08)
    if ser.in_waiting:
        return ser.read(ser.in_waiting).decode("ascii", errors="replace")
    return ""

send("log 0")
send("fault clear")
send("mode vf")
send("vq 0.8")
send("rpm 300")
send("enable")
time.sleep(1.0)

print("--- 稳态运行 300 RPM 采样 ---")
for _ in range(5):
    st = send("status")
    for line in st.splitlines():
        if any(k in line for k in ("vel", "id=", "iu=", "vd=", "vbus")):
            print(" ", line)
    time.sleep(0.2)

send("disable")
send("fault clear")
ser.close()
print("测试完毕。")
