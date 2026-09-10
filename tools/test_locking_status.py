# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.1)
s.reset_input_buffer()

def cmd(c):
    s.write((c + '\n').encode())
    time.sleep(0.01)
    res = s.read_all().decode(errors='ignore')
    return res.strip()

cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

time.sleep(1.8)
for _ in range(5):
    fb = cmd('feedback')
    ss = cmd('sensorless status')
    st = cmd('status')
    print("FB:", [l for l in fb.splitlines() if 'feedback:' in l])
    print("SS:", [l for l in ss.splitlines() if 'sensorless:' in l])
    print("ST:", [l for l in st.splitlines() if 'vel=' in l or 'id=' in l or 'vd=' in l])
    time.sleep(0.08)

cmd('disable')
s.close()
