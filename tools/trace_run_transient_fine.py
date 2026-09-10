# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.05)
s.reset_input_buffer()

def cmd(c):
    s.write((c + '\n').encode())
    time.sleep(0.005)
    return s.read_all().decode(errors='ignore').strip()

cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

t0 = time.time()
print("High frequency logging from 1.5s to 3.5s...")
records = []
while time.time() - t0 < 3.8:
    t = time.time() - t0
    if t >= 1.5:
        fb = cmd('feedback')
        st = cmd('status')
        records.append((t, fb, st))
        time.sleep(0.02)
    else:
        time.sleep(0.05)

cmd('disable')
s.close()

for t, fb, st in records:
    fb_l = [l for l in fb.splitlines() if 'feedback:' in l]
    st_l = [l for l in st.splitlines() if 'iq=' in l and 'id=' in l]
    st_v = [l for l in st.splitlines() if 'vel_obs=' in l]
    enc_v = [l for l in st.splitlines() if 'vel=' in l and not 'vel_obs' in l and not 'vel_filt' in l]

    fb_str = fb_l[0] if fb_l else ""
    st_str = st_l[0] if st_l else ""
    v_str = st_v[0] if st_v else ""
    ev_str = enc_v[0] if enc_v else ""

    # Extract state, spd_obs, etc.
    if fb_str:
        tokens = dict(kv.split('=') for kv in fb_str.split()[1:] if '=' in kv)
        st_state = tokens.get('state', '')
        spd_obs = tokens.get('spd_obs', '')
        delta = tokens.get('delta', '')
        blend = tokens.get('blend', '')
        print(f"[{t:5.2f}s] {st_state:12s} bl={blend:4s} d={delta:7s} spd_obs={spd_obs:7s} | {ev_str:14s} | {st_str}")
