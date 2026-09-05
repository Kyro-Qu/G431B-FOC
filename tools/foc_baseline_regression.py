# -*- coding: utf-8 -*-
"""基线回归：16k 基线固件 2000rpm 30s"""
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
    return f"{g('M0 ')} | {g('vel=')} | {g('iq=')} | {g('cs_fault=')}"

print(cmd('disable')); time.sleep(0.3)
print(cmd('fault clear')); time.sleep(0.3)
print(cmd('mode vf')); time.sleep(0.3)
print(cmd('angle enc'))
print(cmd('calib', wait=1.0))
ok = False
for i in range(20):
    txt = cmd('status', wait=0.5)
    if 'calib=1' in txt:
        print(f'calib OK [{i}]')
        ok = True
        break
    if 'FAULT' in txt.split('\r\n')[0]:
        print(f'CALIB FAULT [{i}]')
        ser.close(); exit(1)
    time.sleep(0.5)
if not ok:
    print('calib timeout')
    ser.close(); exit(1)

print(cmd('mode vel'))
print(cmd('target 2000'))
print('enable:', cmd('enable', wait=1.0).strip()[:60])

prev = None
frozen = 0
dead = False
for i in range(60):  # 30s
    s = status_one()
    print(f'[{(i+1)*0.5:.1f}s]', s)
    if 'FAULT' in s:
        dead = True
        break
    v = s.split('vel=')[1].split('rpm')[0] if 'vel=' in s else '?'
    if v not in ('?', '0.0') and v == prev:
        frozen += 1
    else:
        frozen = 0
    prev = v
    if frozen >= 4:
        dead = True
        print('  -> 快环冻结')
        break
    time.sleep(0.1)

print('\n结果:', 'FAULT/冻结' if dead else '30s 稳定')
ser.close()
