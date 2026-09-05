# -*- coding: utf-8 -*-
"""校准过程逐帧观察：500ms 一帧，全程盯 calib_state/vel/vd/iq"""
import serial, time

ser = serial.Serial('COM44', 6500000, timeout=0.5)

def cmd(c, wait=0.4):
    ser.write((c + '\r\n').encode())
    time.sleep(wait)
    return ser.read(65536).decode('gbk', errors='replace')[:200]

def status_one():
    ser.write(b'status\r\n')
    time.sleep(0.45)
    txt = ser.read(65536).decode('gbk', errors='replace')
    g = lambda p: next((l for l in txt.split('\r\n') if l.startswith(p)), '?')
    return f"{g('M0 ')} | {g('vel=')} | {g('vd=')} | {g('iq=')} | {g('calib_state=')} | {g('fault=')}"

print(cmd('disable')); time.sleep(0.3)
print(cmd('fault clear')); time.sleep(0.3)
print(cmd('mode vf')); time.sleep(0.3)
print(cmd('angle enc'))
print(cmd('calib', wait=0.5))
for i in range(24):  # 12s
    print(f'[{(i+1)*0.5:.1f}s]', status_one())
    time.sleep(0.1)
    # 提前结束
    if 'calib=1' in status_one() or 'FAULT' in status_one():
        break
ser.close()
print('DONE')
