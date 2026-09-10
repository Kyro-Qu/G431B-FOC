# -*- coding: utf-8 -*-
import serial, time

ser = serial.Serial('COM44', 6500000, timeout=0.5)
ser.reset_input_buffer()

def cmd(s):
    ser.write((s + '\n').encode('utf-8'))
    time.sleep(0.02)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

print("Time, State, SpdOpen, SpdObs, SpdEnc, Lock, Conf, Streak, Iq, Id, Lost")
t0 = time.time()
for _ in range(30):
    time.sleep(0.1)
    fb = cmd('feedback')
    st = cmd('status')

    # Parse fb: feedback: mode=sensorless state=if_accel blend=0.00 spd_open=46.7 spd_obs=27.5 lock=0 conf=0.00 streak=0 lost=0 if_curr=0.40 if_rpm=500
    state, spd_open, spd_obs, lock, conf, streak, lost = "unknown", 0.0, 0.0, 0, 0.0, 0, 0
    for line in fb.splitlines():
        if "feedback: mode=sensorless" in line:
            parts = dict(kv.split('=') for kv in line.split()[1:] if '=' in kv)
            state = parts.get('state', '')
            spd_open = float(parts.get('spd_open', '0'))
            spd_obs = float(parts.get('spd_obs', '0'))
            lock = int(parts.get('lock', '0'))
            conf = float(parts.get('conf', '0'))
            streak = int(parts.get('streak', '0'))
            lost = int(parts.get('lost', '0'))

    spd_enc = 0.0
    id_val, iq_val = 0.0, 0.0
    for line in st.splitlines():
        if line.startswith('vel='):
            # vel=0.0rpm
            spd_enc = float(line.replace('vel=', '').replace('rpm', ''))
        if 'id=' in line and 'iq=' in line:
            parts = line.split()
            id_val = float(parts[0].replace('id=', '').replace('A', ''))
            iq_val = float(parts[2].replace('iq=', '').replace('A', ''))

    print(f"{time.time()-t0:.2f}, {state}, {spd_open:.1f}, {spd_obs:.1f}, {spd_enc:.1f}, {lock}, {conf:.2f}, {streak}, {iq_val:.3f}, {id_val:.3f}, {lost}")

cmd('disable')
ser.close()
