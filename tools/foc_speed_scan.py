# -*- coding: utf-8 -*-
"""速度扫描：500 -> 1000 -> 1500 -> 2000，每档 enable 3s 看是否快环死"""
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
    st = next((l for l in txt.split('\r\n') if l.startswith('M0 ')), '?')
    vel = next((l for l in txt.split('\r\n') if l.startswith('vel=')), '?')
    cs = next((l for l in txt.split('\r\n') if l.startswith('cs_fault=')), '?')
    cpu = next((l for l in txt.split('\r\n') if l.startswith('cpu=')), '?')
    return f'{st} | {vel} | {cs} | {cpu}'

# 校准
print(cmd('disable')); time.sleep(0.3)
print(cmd('fault clear')); time.sleep(0.3)
print(cmd('mode vf')); time.sleep(0.3)
print(cmd('angle enc'))
print(cmd('calib', wait=1.0))
for i in range(16):
    txt = cmd('status', wait=0.5)
    if 'calib=1' in txt:
        print(f'calib OK [{i}]')
        break
    if 'FAULT' in txt.split('\r\n')[0]:
        print(f'CALIB FAULT [{i}]')
        ser.close(); exit(1)
    time.sleep(0.5)
else:
    print('calib timeout')
    ser.close(); exit(1)

print(cmd('obs 0'))
print(cmd('mode vel'))

# 速度扫描
for target in [500, 1000, 1500, 2000]:
    print(f'\n=== target={target} ===')
    print(cmd(f'target {target}'))
    print(cmd('enable', wait=0.5))
    ok = True
    for j in range(6):  # 3s
        s = status_one()
        print(f'  [{j}]', s)
        if 'FAULT' in s:
            ok = False
            break
        time.sleep(0.2)
    if ok:
        print(f'  {target} OK')
    else:
        print(f'  {target} FAULT')
        break
    # 回 IDLE 准备下一档
    print(cmd('disable')); time.sleep(0.5)

ser.close()
print('DONE')
