# -*- coding: utf-8 -*-
import serial, time

ser = serial.Serial('COM44', 6500000, timeout=0.5)
ser.reset_input_buffer()

def cmd(s):
    ser.write((s + '\n').encode('utf-8'))
    time.sleep(0.015)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

print("--- Resetting ---")
cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

t0 = time.time()
print("Tracing state transitions...")
while time.time() - t0 < 3.5:
    fb = cmd('feedback')
    # look for mode=sensorless
    for line in fb.splitlines():
        if 'feedback: mode=sensorless' in line:
            parts = dict(kv.split('=') for kv in line.split()[1:] if '=' in kv)
            state = parts.get('state', '')
            streak = parts.get('streak', '0')
            blend = parts.get('blend', '0')
            lost = parts.get('lost', '0')
            spd_obs = parts.get('spd_obs', '0')
            spd_open = parts.get('spd_open', '0')
            st_val = int(streak) if streak.isdigit() else 0
            if st_val > 7000 or state in ('blend', 'run', 'lost', 'safe_stop'):
                print(f"[{time.time()-t0:.3f}] {state:12s} blend={blend} streak={streak} lost={lost} spd_obs={spd_obs} spd_open={spd_open}")
            if state in ('lost', 'safe_stop'):
                break
    if 'lost' in fb or 'safe_stop' in fb:
        break
    time.sleep(0.01)

st = cmd('status')
print("Status at end:\n", st)
cmd('disable')
ser.close()
