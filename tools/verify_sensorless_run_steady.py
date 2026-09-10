# -*- coding: utf-8 -*-
import serial, time

s = serial.Serial('COM44', 6500000, timeout=0.1)
s.reset_input_buffer()

def cmd(c):
    s.write((c + '\n').encode())
    time.sleep(0.005)
    return s.read_all().decode(errors='ignore').strip()

print("=== 纯无感 Sensorless Primary 500 RPM 稳态长跑测试 (持续 6.0s) ===")
cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

t0 = time.time()
records = []
while time.time() - t0 < 6.5:
    t = time.time() - t0
    fb = cmd('feedback')
    st = cmd('status')
    records.append((t, fb, st))
    time.sleep(0.03)

cmd('disable')
s.close()

run_count = 0
iq_max = 0.0
vel_samples = []

for t, fb, st in records:
    fb_lines = [l for l in fb.splitlines() if 'feedback: mode=sensorless' in l]
    st_vel = [l for l in st.splitlines() if 'vel_obs=' in l]
    st_id = [l for l in st.splitlines() if 'id=' in l and 'iq=' in l]
    st_m0 = [l for l in st.splitlines() if 'M0 ' in l]
    if fb_lines:
        line = fb_lines[0]
        tokens = dict(kv.split('=') for kv in line.split()[1:] if '=' in kv)
        st_val = tokens.get('state', '')
        # print key milestones
        if st_val in ('if_start', 'if_accel', 'obs_locking', 'blend', 'run', 'lost', 'safe_stop'):
            vel_str = st_vel[0] if st_vel else ""
            id_str = st_id[0] if st_id else ""
            if st_val == 'run':
                run_count += 1
                # parse iq
                for item in id_str.split():
                    if item.startswith('iq='):
                        val = abs(float(item.split('=')[1].replace('A','')))
                        if val > iq_max: iq_max = val
                    if item.startswith('vel_obs='):
                        v_val = float(item.split('=')[1].replace('rpm',''))
                        vel_samples.append(v_val)
            print(f"[{t:6.3f}s] state={st_val:12s} bl={tokens.get('blend',''):5s} delta={tokens.get('delta',''):8s} lock={tokens.get('lock',''):1s} | {vel_str} | {id_str}")

print("\n=== 统计报告 ===")
print(f"稳态 RUN 采样点数: {run_count} 点 (~ {run_count * 0.03:.2f} 秒稳态闭环运行)")
print(f"RUN 状态最大电流 |Iq|: {iq_max:.3f} A (限幅门限 <= 0.80 A)")
if vel_samples:
    avg_vel = sum(vel_samples) / len(vel_samples)
    print(f"估计转速平均值: {avg_vel:.1f} RPM (目标 500 RPM, 稳态误差 {abs(avg_vel - 500.0):.1f} RPM)")
