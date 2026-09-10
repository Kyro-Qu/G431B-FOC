# -*- coding: utf-8 -*-
import serial
import time

s = serial.Serial('COM44', 6500000, timeout=0.5)

def send(cmd, delay=0.05):
    s.reset_input_buffer()
    s.write((cmd + '\r\n').encode('ascii'))
    time.sleep(delay)
    buf = ''
    if s.in_waiting:
        buf = s.read(s.in_waiting).decode('ascii', errors='ignore')
    return buf

print("1. Check status:")
print(send('status'))

print("2. Calib:")
print(send('fault clear'))
print(send('calib'))
for i in range(20):
    time.sleep(0.3)
    st = send('status')
    if 'calib=1' in st and 'M0 IDLE' in st:
        print(f"Calib finished at iteration {i}!")
        break

print("3. Try spin:")
print("mode vel ->", send('mode vel'))
print("vel ramp 800 ->", send('vel ramp 800'))
print("target 800 ->", send('target 800'))
print("enable ->", repr(send('enable')))

for i in range(10):
    time.sleep(0.3)
    st = send('status')
    vel = "none"
    state = "none"
    for line in st.splitlines():
        if 'M0 ' in line: state = line
        if 'vel=' in line: vel = line
    print(f"[{i}] {state} | {vel}")

print("disable ->", send('disable'))
s.close()
