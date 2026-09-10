# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.05)
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

time.sleep(2.5) # right around blend/run transition
t0 = time.time()
while time.time() - t0 < 1.0:
    st = cmd('status')
    fb = cmd('feedback')
    for l in fb.splitlines():
        if 'feedback:' in l:
            print(f"[{time.time()-t0:.3f}s] {l}")
    for l in st.splitlines():
        if any(k in l for k in ['vel_obs=', 'iq=', 'tgt=']):
            print(f"         {l}")
    time.sleep(0.04)

cmd('disable')
s.close()
