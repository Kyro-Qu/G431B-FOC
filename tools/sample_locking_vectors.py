# -*- coding: utf-8 -*-
import serial, time

ser = serial.Serial('COM44', 6500000, timeout=0.2)
ser.reset_input_buffer()

def cmd(s):
    ser.write((s + '\n').encode('utf-8'))
    time.sleep(0.01)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

# wait until it is in obs_locking (e.g. 1.8s)
time.sleep(1.8)
for i in range(5):
    fb = cmd('feedback')
    bench = cmd('sensorless bench')
    st = cmd('status')
    print(f"--- SAMPLE {i} ---")
    for l in fb.splitlines():
        if 'feedback:' in l: print("FB:   ", l)
    for l in bench.splitlines():
        if 'Obs 2' in l or 'theta_e' in l or 'speed_rpm' in l or 'flux_mag' in l:
            print("BENCH:", l)
    for l in st.splitlines():
        if 'vel=' in l or 'vel_obs=' in l or 'id=' in l or 'vd=' in l:
            print("ST:   ", l)
    time.sleep(0.05)

cmd('disable')
ser.close()
