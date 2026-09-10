# -*- coding: utf-8 -*-
import serial, time

ser = serial.Serial('COM44', 6500000, timeout=0.5)
ser.reset_input_buffer()

def cmd(s):
    ser.write((s + '\n').encode('utf-8'))
    time.sleep(0.015)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

print("cmd feedback:", cmd('feedback'))
cmd('disable')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')
for i in range(15):
    time.sleep(0.2)
    print(f"[{i*0.2:.1f}s]", cmd('feedback'))

cmd('disable')
ser.close()
