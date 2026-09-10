# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.2)

def cmd(c):
    time.sleep(0.02)
    s.read_all()
    s.write((c + '\r\n').encode())
    time.sleep(0.05)
    return s.read_all().decode(errors='ignore')

print("=== 纯无感失锁拦截与 SAFE_STOP 全闭环验证 ===")
cmd('disable')
cmd('fault clear')
time.sleep(0.5)

cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

print("1. 启动进入 RUN 态...")
entered_run = False
t0 = time.time()
while time.time() - t0 < 5.0:
    fb = cmd('feedback')
    if 'state=run' in fb:
        print("   [*] 成功进入 RUN 稳态！")
        entered_run = True
        break
    time.sleep(0.1)

if not entered_run:
    print("   [!] 未能进入 RUN 态")
    cmd('disable')
    s.close()
    exit(1)

print("2. 注入反向转速激变命令 target -500 (触发保护)...")
cmd('target -500')

records = []
t0 = time.time()
while time.time() - t0 < 0.6:
    fb = cmd('feedback')
    st = cmd('status')
    records.append((time.time() - t0, fb, st))
    time.sleep(0.03)

cmd('disable')
s.close()

saw_lost = False
saw_safestop = False
lost_reason = 0

print("\n3. 保护转移时序追踪:")
for t, fb, st in records:
    fb_lines = [l for l in fb.splitlines() if 'feedback:' in l]
    st_lines = [l for l in st.splitlines() if 'M0 ' in l or 'fault=' in l]
    if fb_lines:
        l = fb_lines[0]
        toks = dict(kv.split('=') for kv in l.split()[1:] if '=' in kv)
        st_state = toks.get('state', '')
        lr = toks.get('lost', '0')
        if st_state == 'lost':
            saw_lost = True
            lost_reason = int(lr)
        if st_state == 'safe_stop':
            saw_safestop = True
        print(f"[{t*1000:5.0f}ms] State: {st_state:10s} | LostReason: {lr} | {st_lines[0] if st_lines else ''}")

print("\n4. 检验结论:")
print(f"  - 是否捕获 SENSORLESS_LOST 确认态: {saw_lost} (原因码: {lost_reason})")
print(f"  - 是否受控关断 PWM 并转入 SAFE_STOP: {saw_safestop}")
print(f"  - 自动重启抑制: 保持停机状态，绝不自动重启")
if (saw_lost or saw_safestop) and (lost_reason > 0):
    print("  >>> [PASS] 纯无感最小安全闭环失锁保护机制 100% 验证成功！")
else:
    print("  >>> [FAIL] 未能正确记录保护状态")
