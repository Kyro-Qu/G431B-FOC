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

time.sleep(1.8) # in obs_locking
for _ in range(5):
    fb = cmd('feedback')
    bench = cmd('sensorless bench')
    st = cmd('status')
    print("=== SNAPSHOT ===")
    print("feedback:", [l for l in fb.splitlines() if 'feedback:' in l])
    print("status:", [l for l in st.splitlines() if 'vel=' in l or 'id=' in l or 'vd=' in l])
    print("bench:")
    for l in bench.splitlines():
        if any(k in l for k in ['Obs 2', 'theta_e', 'speed_rpm', 'flux_mag', 'cycles', 'diag']):
            print("  ", l)
    time.sleep(0.1)

cmd('disable')
ser.close()
