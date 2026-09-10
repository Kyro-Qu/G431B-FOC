# -*- coding: utf-8 -*-
import serial
import time

s = serial.Serial('COM44', 6500000, timeout=0.5)

def send(cmd, delay=0.04):
    s.write((cmd + '\r\n').encode('ascii'))
    time.sleep(delay)
    buf = ''
    if s.in_waiting:
        buf = s.read(s.in_waiting).decode('ascii', errors='ignore')
    return buf

print("fault clear ->", repr(send('fault clear')))
print("enc fault clear ->", repr(send('enc fault clear')))
print("sensorless algo vesc ->", repr(send('sensorless algo vesc')))
print("deadtime obs 1 ->", repr(send('deadtime obs 1')))
print("deadtime volt 0.17 ->", repr(send('deadtime volt 0.17')))

st = send('status')
print("status ->", repr(st))
print("'calib=1' in st?", ('calib=1' in st))

if 'calib=1' not in st:
    print("执行自动校准与基准对齐...")
    c_out = send('calib')
    print("calib out ->", repr(c_out))
    for i in range(30):
        time.sleep(0.3)
        st_c = send('status')
        print(f"wait {i}: 'calib=1' in st_c? {('calib=1' in st_c)}, 'M0 IDLE'? {('M0 IDLE' in st_c)}, len={len(st_c)}")
        if 'calib=1' in st_c and 'M0 IDLE' in st_c:
            print("Calib complete!")
            break

s.close()
