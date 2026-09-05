# -*- coding: utf-8 -*-
"""无感切换实验：angle enc -> calib -> obs 1 -> vel 2000 -> enable -> obs 2 -0.28"""
import serial, time, sys

ser = serial.Serial('COM44', 6500000, timeout=0.5)

def cmd(c, wait=0.4):
    ser.write((c + '\r\n').encode())
    time.sleep(wait)
    r = ser.read(65536)
    txt = r.decode('gbk', errors='replace')
    lines = [l for l in txt.split('\r\n')
             if l and not l.startswith(('th=', 'obs_diff', 'iq_raw'))]
    return ' | '.join(lines)[:400]

def status_tail(n=3, wait=1.0):
    ser.write(b'status\r\n')
    time.sleep(wait)
    txt = ser.read(65536).decode('gbk', errors='replace')
    lines = [l for l in txt.split('\r\n')
             if any(k in l for k in ('vel=', 'iq=', 'fault', 'M0', 'id='))]
    return ' || '.join(lines[-n*5:])[-300:]

print('== step1: angle enc ==')
print(cmd('angle enc'))
print('== step2: calib ==')
print(cmd('calib', wait=5.0))
print('== step3: obs 1 ==')
print(cmd('obs 1'))
print('== step4: mode vel + target 2000 ==')
print(cmd('mode vel'))
print(cmd('target 2000'))
print('== step5: enable ==')
print(cmd('enable', wait=3.0))
print('-- 稳定观察 2s --')
for i in range(2):
    print(status_tail())
    time.sleep(1.0)
print('== step6: obs 2 -0.28 实时切换 ==')
print(cmd('obs 2 -0.28', wait=1.0))
print('-- 切换后观察 3s --')
for i in range(3):
    print(status_tail())
    time.sleep(1.0)
ser.close()
print('DONE')
