# -*- coding: utf-8 -*-
import serial, time

ser = serial.Serial('COM44', 6500000, timeout=0.5)
ser.reset_input_buffer()

def cmd(s):
    ser.write((s + '\n').encode('utf-8'))
    time.sleep(0.015)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

cmd('disable')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

time.sleep(2.0) # wait until obs_locking
for _ in range(10):
    fb = cmd('feedback')
    st = cmd('status')
    bench = cmd('sensorless bench')
    print("---")
    for line in fb.splitlines():
        if 'feedback:' in line: print(line)
    for line in bench.splitlines():
        if 'Obs 2 (VESC)' in line or 'theta_e' in line: print(line)
    time.sleep(0.05)

cmd('disable')
ser.close()
