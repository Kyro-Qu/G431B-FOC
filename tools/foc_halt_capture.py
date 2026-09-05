# -*- coding: utf-8 -*-
"""halt capture v3: fault clear -> calib(poll) -> enable -> freeze detect -> pyocd halt dump"""
import serial, time, subprocess, sys

ser = serial.Serial('COM44', 6500000, timeout=0.5)

def cmd(c, wait=0.4):
    ser.write((c + '\r\n').encode())
    time.sleep(wait)
    return ser.read(65536).decode('gbk', errors='replace')[:200]

def get_status():
    ser.write(b'status\r\n')
    time.sleep(0.5)
    return ser.read(65536).decode('gbk', errors='replace')

def field(txt, prefix):
    l = next((l for l in txt.split('\r\n') if l.startswith(prefix)), '')
    return l

print(cmd('disable')); time.sleep(0.3)
print(cmd('fault clear')); time.sleep(0.3)
print(cmd('mode vf')); time.sleep(0.3)   # 清 vel 残留，calib 必须在 IDLE/vf
print(cmd('angle enc'))
print(cmd('calib', wait=1.0))

# poll calib done (calib=1 in status)
ok = False
for i in range(20):
    txt = get_status()
    cb = field(txt, 'calib=')
    st = field(txt, 'M0 ')
    print(f'calib poll[{i}]: {cb} | {st}')
    if cb.startswith('calib=1'):
        ok = True
        break
    if 'FAULT' in st:
        print('CALIB FAULT')
        break
    time.sleep(0.5)
if not ok:
    ser.close()
    sys.exit(1)

print(cmd('obs 0'))
print(cmd('mode vel'))
print(cmd('target 2000'))
print('enable:', cmd('enable', wait=2.0).strip()[:80])

prev_vel, frozen, dead = None, 0, False
for i in range(30):
    txt = get_status()
    st = field(txt, 'M0 ')
    vel = field(txt, 'vel=')
    cs = field(txt, 'cs_fault=')
    print(f'[{i}] {st} | {vel} | {cs}')
    v = vel.split('=')[1] if '=' in vel else '?'
    if v not in ('?', '0.0rpm') and v == prev_vel:
        frozen += 1
    else:
        frozen = 0
    prev_vel = v
    if 'FAULT' in st or frozen >= 3:
        dead = True
        break
ser.close()

if not dead:
    print('no death observed')
    sys.exit(0)

print('=== fast loop dead, halt capture ===')
time.sleep(0.5)
r = subprocess.run([sys.executable, '_halt_capture.py'],
                   capture_output=True, text=True, timeout=60)
print(r.stdout)
if r.returncode != 0:
    print('STDERR:', r.stderr[:600])
print('DONE')
