# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.2)
s.reset_input_buffer()

def cmd(c):
    s.write((c + '\n').encode())
    time.sleep(0.01)
    return s.read_all().decode(errors='ignore').strip()

cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

# Wait until in obs_locking (e.g. 2.1s)
time.sleep(2.0)
for _ in range(5):
    fb = cmd('feedback')
    ss = cmd('sensorless status')
    st = cmd('status')
    print("--- SNAPSHOT ---")
    for l in fb.splitlines():
        if 'feedback:' in l: print("FB:", l)
    for l in ss.splitlines():
        if 'sensorless:' in l: print("SS:", l)
    for l in st.splitlines():
        if any(k in l for k in ['vel=', 'vel_obs=', 'id=', 'vd=']):
            print("ST:", l)
    time.sleep(0.05)

cmd('disable')
s.close()
