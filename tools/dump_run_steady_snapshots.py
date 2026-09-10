# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.05)
s.reset_input_buffer()

def cmd(c):
    s.write((c + '\n').encode())
    time.sleep(0.015)
    return s.read_all().decode(errors='ignore').strip()

print("=== 启动纯无感并在稳态 RUN 期间抓取 10 次详细快照 ===")
cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

# 启动并加速至 500 RPM，经过 locking(0.5s) 和 blend(0.15s)，在 t=3.2s 时进入纯无感稳态 RUN
time.sleep(3.2)

for i in range(10):
    fb = cmd('feedback')
    st = cmd('status')
    ss = cmd('sensorless status')
    print(f"\n--- [RUN 稳态采样 #{i+1}] ---")
    for l in fb.splitlines():
        if 'feedback:' in l: print("  FB:", l)
    for l in st.splitlines():
        if any(k in l for k in ['vel=', 'vel_obs=', 'id=', 'vd=', 'vbus=']):
            print("  ST:", l)
    for l in ss.splitlines():
        if 'sensorless:' in l: print("  SS:", l)
    time.sleep(0.2)

cmd('disable')
s.close()
