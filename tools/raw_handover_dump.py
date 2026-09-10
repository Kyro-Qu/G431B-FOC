# -*- coding: utf-8 -*-
import serial, time

ser = serial.Serial('COM44', 6500000, timeout=0.1)
ser.reset_input_buffer()

def cmd(s):
    ser.write((s + '\n').encode('utf-8'))
    time.sleep(0.008)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

t0 = time.time()
history = []
while time.time() - t0 < 3.2:
    t = time.time() - t0
    fb = cmd('feedback')
    st = cmd('status')
    history.append((t, fb, st))
    time.sleep(0.005)

cmd('disable')
ser.close()

print(f"Total samples collected: {len(history)}")
for t, fb, st in history:
    # check if blend or run or lost
    fb_line = [l for l in fb.splitlines() if 'feedback: mode=sensorless' in l]
    st_vel = [l for l in st.splitlines() if 'vel_obs=' in l]
    st_id = [l for l in st.splitlines() if 'id=' in l and 'iq=' in l]
    if fb_line:
        s = fb_line[0]
        if any(x in s for x in ['obs_locking', 'blend', 'run', 'lost']):
            print(f"[{t:.3f}] {s}")
            if st_vel: print(f"        {st_vel[0]}")
            if st_id:  print(f"        {st_id[0]}")
