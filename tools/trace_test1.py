# -*- coding: utf-8 -*-
import serial
import time
import re

s = serial.Serial('COM44', 6500000, timeout=0.5)

def send(cmd, delay=0.04):
    s.write((cmd + '\r\n').encode('ascii'))
    time.sleep(delay)
    buf = ''
    if s.in_waiting:
        buf = s.read(s.in_waiting).decode('ascii', errors='ignore')
    return buf

print("send fault clear:", repr(send('fault clear')))
print("send enc fault clear:", repr(send('enc fault clear')))
print("send feedback auto:", repr(send('feedback auto')))
print("send feedback speed 420 320:", repr(send('feedback speed 420 320')))
print("send mode vel:", repr(send('mode vel')))
print("send vel ramp 800:", repr(send('vel ramp 800')))
print("send target 800:", repr(send('target 800')))
print("send enable:", repr(send('enable')))

for i in range(10):
    time.sleep(0.2)
    st = send('status', 0.02)
    fb = send('feedback', 0.02)
    sl = send('sensorless status', 0.02)
    rpm = 0.0
    for line in st.splitlines():
        if 'vel=' in line or 'M0' in line or 'fault' in line or 'calib' in line:
            print(f"  [tick {i}] {line}")

send('disable')
s.close()
