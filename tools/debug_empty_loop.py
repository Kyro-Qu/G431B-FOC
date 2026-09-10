# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.1)
s.reset_input_buffer()

def send_cmd(c):
    s.write((c + '\n').encode())
    time.sleep(0.01)
    res = s.read_all().decode(errors='ignore')
    return res

print("Send enable...")
send_cmd('disable')
send_cmd('fault clear')
send_cmd('feedback sensorless')
send_cmd('feedback if 0.60 500')
send_cmd('mode vel')
send_cmd('target 500')
r_en = send_cmd('enable')
print("enable response:", repr(r_en))

for i in range(5):
    time.sleep(0.5)
    r_fb = send_cmd('feedback')
    r_st = send_cmd('status')
    print(f"\nStep {i+1}:")
    print("fb:", repr(r_fb))
    print("st:", repr(r_st)[:100])

send_cmd('disable')
s.close()
