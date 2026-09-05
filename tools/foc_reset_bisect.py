# -*- coding: utf-8 -*-
"""最小复现：区分复位原因——A组 不开obs enable；B组 开obs enable"""
import serial, time

ser = serial.Serial('COM44', 6500000, timeout=0.5)

def cmd(c, wait=0.4):
    ser.write((c + '\r\n').encode())
    time.sleep(wait)
    r = ser.read(65536)
    txt = r.decode('gbk', errors='replace')
    lines = [l for l in txt.split('\r\n')
             if l and not l.startswith(('th=', 'obs_diff', 'iq_raw'))]
    return ' | '.join(lines)[:300]

def status_tail():
    ser.write(b'status\r\n')
    time.sleep(0.6)
    txt = ser.read(65536).decode('gbk', errors='replace')
    lines = [l for l in txt.split('\r\n')
             if any(k in l for k in ('vel=', 'iq=', 'fault', 'M0'))]
    return ' || '.join(lines[-5:])[-260:]

def run_test(tag, obs_on):
    print(f'---- {tag} (obs={obs_on}) ----')
    cmd('disable'); time.sleep(0.3)
    cmd('angle enc')
    cmd('calib', wait=5.0)
    print('calib+angle ok')
    if obs_on:
        print(cmd('obs 1'))
    else:
        print(cmd('obs 0'))
    cmd('mode vel')
    cmd('target 2000')
    print('enable:', cmd('enable', wait=2.5))
    for i in range(3):
        print(status_tail())
        time.sleep(0.8)

run_test('A组: 不开obs', 0)
run_test('B组: 开obs', 1)
ser.close()
print('DONE')
