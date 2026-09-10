# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.1)
s.reset_input_buffer()

def cmd(c):
    s.write((c + '\n').encode())
    time.sleep(0.01)
    return s.read_all().decode(errors='ignore').strip()

print("=== 启动纯无感并等待 5 秒进入 500 RPM 真正稳态后连续抓取 10 次快照 ===")
cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

# 等待 5.0 秒，让目标速度斜坡完全落定在 500 RPM 且转速完全稳定
time.sleep(5.0)

for i in range(10):
    fb = cmd('feedback')
    st = cmd('status')
    ss = cmd('sensorless status')
    print(f"\n--- [RUN 真正稳态采样 #{i+1}] ---")
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
