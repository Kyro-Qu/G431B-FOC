# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.2)

def flush():
    time.sleep(0.02)
    s.read_all()

def cmd(c):
    flush()
    s.write((c + '\r\n').encode())
    time.sleep(0.05)
    return s.read_all().decode(errors='ignore')

print("=== 纯无感 Sensorless Primary 运行中反向设定触发失锁保护 ===")
cmd('disable')
cmd('fault clear')
time.sleep(0.6)

cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

print("1. 等待电机进入纯无感 RUN 稳态...")
entered_run = False
t0 = time.time()
while time.time() - t0 < 5.0:
    fb = cmd('feedback')
    if 'state=run' in fb:
        print("   [*] 电机已进入 RUN 稳态运行！")
        entered_run = True
        break
    time.sleep(0.1)

if not entered_run:
    print("   [!] 未能进入 RUN 态，测试终止")
    cmd('disable')
    s.close()
    exit(1)

print("2. 注入反向激变命令 target -500 (触发纯无感速度方向严重不匹配保护 lost_reason=3)...")
cmd('target -500')

# 等待 80ms 观察保护拦截
time.sleep(0.08)

fb_after = cmd('feedback')
st_after = cmd('status')

print("\n3. 保护拦截状态快照:")
fb_l = [l for l in fb_after.splitlines() if 'feedback:' in l]
st_l = [l for l in st_after.splitlines() if 'M0 ' in l or 'fault=' in l]
id_l = [l for l in st_after.splitlines() if 'id=' in l and 'iq=' in l]

print("   Feedback:", fb_l[0] if fb_l else fb_after)
print("   Status Fault:", st_l)
print("   Currents:", id_l[0] if id_l else "")

cmd('disable')
s.close()
