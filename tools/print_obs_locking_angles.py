# -*- coding: utf-8 -*-
import serial, time, math

ser = serial.Serial('COM44', 6500000, timeout=0.1)
ser.reset_input_buffer()

def cmd(s):
    ser.write((s + '\n').encode('utf-8'))
    time.sleep(0.008)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

time.sleep(2.0) # in obs_locking
for _ in range(10):
    bench = cmd('sensorless bench')
    st = cmd('status')
    fb = cmd('feedback')
    print("---")
    for l in fb.splitlines():
        if 'feedback:' in l: print(l)
    for l in bench.splitlines():
        if 'Obs 2 (VESC)' in l or 'theta_e' in l: print(l)
    for l in st.splitlines():
        if 'vel=' in l or 'pos=' in l: print(l)
    time.sleep(0.05)

cmd('disable')
ser.close()
