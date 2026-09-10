# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.2)

def flush():
    time.sleep(0.02)
    s.read_all()

def cmd(c):
    flush()
    s.write((c + '\r\n').encode())
    time.sleep(0.04) # 充分等待慢环/CLI命令回显完毕
    res = s.read_all().decode(errors='ignore')
    return res

print("=========================================================")
print("=== 第四阶段 Sensorless Primary 纯无感启动 5 轮严谨重复性验证 ===")
print("=========================================================")

results = []

for trial in range(1, 6):
    print(f"\n>>> [Trial #{trial}] 零速静止启动纯无感闭环...")
    cmd('disable')
    cmd('fault clear')
    time.sleep(0.6) # 确保电机完全静止

    cmd('feedback sensorless')
    cmd('feedback if 0.60 500')
    cmd('mode vel')
    cmd('target 500')
    cmd('enable')

    t0 = time.time()
    entered_run = False
    run_time = 0.0
    iq_max = 0.0
    steady_speeds = []
    fault_detected = False

    while time.time() - t0 < 6.5:
        t = time.time() - t0
        fb = cmd('feedback')
        st = cmd('status')

        fb_lines = [l for l in fb.splitlines() if 'feedback:' in l and 'mode=sensorless' in l]
        st_id = [l for l in st.splitlines() if 'id=' in l and 'iq=' in l]
        st_m0 = [l for l in st.splitlines() if 'M0 ' in l]

        if st_m0 and 'FAULT' in st_m0[0]:
            fault_detected = True
            print(f"    [!] Fault triggered: {st_m0[0]}")
            break

        if fb_lines and st_id:
            fb_tok = dict(kv.split('=') for kv in fb_lines[0].split()[1:] if '=' in kv)
            st_tok = dict(kv.split('=') for kv in st_id[0].split() if '=' in kv)

            state = fb_tok.get('state', '')
            spd_obs = float(fb_tok.get('spd_obs', '0.0'))
            iq = abs(float(st_tok.get('iq', '0.0A').replace('A', '')))

            if iq > iq_max:
                iq_max = iq

            if state == 'run':
                if not entered_run:
                    entered_run = True
                    run_time = t
                    print(f"    [*] 成功切入 RUN 闭环稳态! 切入耗时: {run_time:.3f}s, 切入角差: {fb_tok.get('delta','')}")
                steady_speeds.append(spd_obs)

        time.sleep(0.06)

    cmd('disable')
    time.sleep(0.5)

    if entered_run and not fault_detected and len(steady_speeds) >= 15:
        avg_spd = sum(steady_speeds) / len(steady_speeds)
        print(f"    [PASS] 启动成功! 稳态平均转速: {avg_spd:.1f} RPM, 最大电流 |Iq|: {iq_max:.3f}A (限幅<=0.80A)")
        results.append((trial, True, run_time, avg_spd, iq_max))
    else:
        print(f"    [FAIL] 启动未达标或发生故障! entered_run={entered_run}, fault={fault_detected}, samples={len(steady_speeds)}")
        results.append((trial, False, 0.0, 0.0, iq_max))

s.close()

print("\n======================= 最终验证总结 =======================")
pass_count = sum(1 for r in results if r[1])
print(f"总计测试: {len(results)} 轮, 成功通过: {pass_count} 轮 (通过率: {pass_count/len(results)*100:.1f}%)")
for r in results:
    status_str = "PASS" if r[1] else "FAIL"
    print(f"  Trial #{r[0]}: {status_str:4s} | 切入耗时: {r[2]:.2f}s | 稳态转速: {r[3]:.1f} RPM | 峰值电流: {r[4]:.3f} A")
print("=========================================================")
