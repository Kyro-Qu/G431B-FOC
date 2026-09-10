# -*- coding: utf-8 -*-
"""
diagnose_steady_oscillation.py
高频抓取稳态运行中的速度、Iq、Vq、PI项等，分析振荡周期与频率
"""
import serial
import time

s = serial.Serial('COM44', 6500000, timeout=0.1)
s.reset_input_buffer()

def cmd(c):
    time.sleep(0.005)
    s.read_all()
    s.write((c + '\r\n').encode())
    time.sleep(0.02)
    return s.read_all().decode(errors='ignore')

print("=== 启动纯无感并在稳态下抓取 100 组详细遥测快照 ===")
cmd('disable')
cmd('fault clear')
time.sleep(0.5)

cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

# 等待进入 RUN 稳态 + 2.5 秒
time.sleep(5.5)

records = []
t0 = time.time()
print("开始密集采样 3.0 秒...")
while time.time() - t0 < 3.0:
    t = time.time() - t0
    fb = cmd('feedback')
    st = cmd('status')
    records.append((t, fb, st))
    time.sleep(0.015)

cmd('disable')
s.close()

parsed = []
for t, fb, st in records:
    fb_lines = [l for l in fb.splitlines() if 'feedback:' in l and 'mode=sensorless' in l]
    st_vel = [l for l in st.splitlines() if 'vel_obs=' in l]
    st_id = [l for l in st.splitlines() if 'id=' in l and 'iq=' in l]
    st_v = [l for l in st.splitlines() if 'vd=' in l]

    if fb_lines and st_vel and st_id and st_v:
        fb_tok = dict(kv.split('=') for kv in fb_lines[0].split()[1:] if '=' in kv)
        st_vel_parts = st_vel[0].split()
        st_id_tok = dict(kv.split('=') for kv in st_id[0].split() if '=' in kv)
        st_v_tok = dict(kv.split('=') for kv in st_v[0].split() if '=' in kv)

        # parse fields
        spd_obs = float(fb_tok.get('spd_obs', '0.0'))
        vel_filt = 0.0
        ref_rpm = 0.0
        for part in st_vel_parts:
            if part.startswith('vel_filt='):
                vel_filt = float(part.split('=')[1].replace('rpm', ''))
            elif part.startswith('ref='):
                ref_rpm = float(part.split('=')[1].replace('rpm', ''))

        iq = float(st_id_tok.get('iq', '0.0A').replace('A', ''))
        iq_ref = float(st_id_tok.get('iq_ref', '0.0A').replace('A', ''))
        id_val = float(st_id_tok.get('id', '0.0A').replace('A', ''))
        vd = float(st_v_tok.get('vd', '0.0V').replace('V', ''))
        vq = float(st_v_tok.get('vq', '0.0V').replace('V', ''))

        parsed.append((t, spd_obs, vel_filt, ref_rpm, iq, iq_ref, id_val, vd, vq))

print(f"\n成功解析 {len(parsed)} 组时间序列快照:")
print(f"{'Time(s)':7s} | {'spd_obs':8s} | {'vel_filt':8s} | {'err':7s} | {'iq_ref':7s} | {'iq':7s} | {'Vd':6s} | {'Vq':6s}")
print("-" * 75)
for p in parsed[:40]:
    err = p[3] - p[2]
    print(f"{p[0]:7.3f} | {p[1]:8.1f} | {p[2]:8.1f} | {err:7.1f} | {p[5]:7.3f} | {p[4]:7.3f} | {p[7]:6.3f} | {p[8]:6.3f}")

# 分析极值与零交叉周期 (估算波动主频)
if len(parsed) > 10:
    speeds = [p[1] for p in parsed]
    mean_s = sum(speeds) / len(speeds)
    crossings = []
    for i in range(1, len(speeds)):
        if (speeds[i-1] - mean_s) * (speeds[i] - mean_s) < 0:
            crossings.append(parsed[i][0])
    if len(crossings) >= 2:
        half_periods = [crossings[j] - crossings[j-1] for j in range(1, len(crossings))]
        avg_half_period = sum(half_periods) / len(half_periods)
        est_freq = 1.0 / (2.0 * avg_half_period)
        print(f"\n[波形动力学特征] 平均速度均值: {mean_s:.1f} RPM")
        print(f"过零点个数: {len(crossings)}, 平均半周期: {avg_half_period*1000:.1f} ms -> 振荡主频约为: {est_freq:.2f} Hz")
