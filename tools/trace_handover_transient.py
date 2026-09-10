# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.1)
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
print("Starting high-res capture...")
records = []
while time.time() - t0 < 3.2:
    t = time.time() - t0
    fb = cmd('feedback')
    st = cmd('status')
    records.append((t, fb, st))
    time.sleep(0.01)

cmd('disable')
s.close()

for t, fb, st in records:
    fb_lines = [l for l in fb.splitlines() if 'feedback: mode=sensorless' in l]
    st_vel = [l for l in st.splitlines() if 'vel_obs=' in l]
    st_id = [l for l in st.splitlines() if 'id=' in l and 'iq=' in l]
    st_m0 = [l for l in st.splitlines() if 'M0 ' in l]
    if fb_lines:
        line = fb_lines[0]
        # extract state
        tokens = dict(kv.split('=') for kv in line.split()[1:] if '=' in kv)
        st_val = tokens.get('state', '')
        if st_val in ('obs_locking', 'blend', 'run', 'lost', 'safe_stop'):
            print(f"[{t:6.3f}s] {st_val:12s} bl={tokens.get('blend',''):5s} delta={tokens.get('delta',''):8s} lost={tokens.get('lost',''):2s} strk={tokens.get('streak',''):5s}")
            if st_vel: print(f"         {st_vel[0]}")
            if st_id:  print(f"         {st_id[0]}")
            if st_m0 and 'FAULT' in st_m0[0]: print(f"         {st_m0[0]}")
