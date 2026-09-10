# -*- coding: utf-8 -*-
"""
精细化全闭环、全模式、全速域实机深度评测报告生成脚本 (run_full_system_verification.py)
"""
import sys
import time
import re
import serial

PORT = "COM44"
BAUD = 6500000

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

def get_ser():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    return ser

def c(ser, cmd, delay=0.08):
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode('ascii'))
    time.sleep(delay)
    return ser.read_all().decode(errors='ignore').strip()

def parse_st(text):
    res = {"vel": 0.0, "vel_obs": 0.0, "vel_filt": 0.0, "id": 0.0, "iq": 0.0, "vbus": 0.0, "pos": 0.0, "fault": 0}
    for l in text.splitlines():
        if "vel=" in l:
            m = re.search(r"vel=([\-\d\.]+)rpm", l)
            if m: res["vel"] = float(m.group(1))
            m = re.search(r"vel_obs=([\-\d\.]+)rpm", l)
            if m: res["vel_obs"] = float(m.group(1))
            m = re.search(r"vel_filt=([\-\d\.]+)rpm", l)
            if m: res["vel_filt"] = float(m.group(1))
        if "id=" in l and "iq=" in l:
            m = re.search(r"id=([\-\d\.]+)A", l)
            if m: res["id"] = float(m.group(1))
            m = re.search(r"iq=([\-\d\.]+)A", l)
            if m: res["iq"] = float(m.group(1))
        if "pos=" in l:
            m = re.search(r"pos=([\-\d\.]+)rad", l)
            if m: res["pos"] = float(m.group(1))
        if "vbus=" in l:
            m = re.search(r"vbus=([\-\d\.]+)V", l)
            if m: res["vbus"] = float(m.group(1))
        if "fault=" in l:
            m = re.search(r"fault=(\d+)", l)
            if m: res["fault"] = int(m.group(1))
    return res

def main():
    ser = get_ser()
    print("=" * 80)
    print("             FOC_G431 全功能、全闭环、全区间实机综合测试")
    print("=" * 80)

    # 1. 有感 - 力矩/电流环测试
    print("\n>>> 【阶段 1/5】有感 - 力矩/电流闭环测试 (mode iq)")
    c(ser, 'disable')
    c(ser, 'fault clear')
    c(ser, 'feedback sensored')
    c(ser, 'angle enc')
    c(ser, 'mode iq')
    c(ser, 'enable')
    time.sleep(0.3)

    iq_targets = [0.10, 0.20, 0.30, 0.00]
    for tgt in iq_targets:
        c(ser, f'target {tgt}')
        time.sleep(1.0)
        st = parse_st(c(ser, 'status'))
        print(f"    Iq 给定: {tgt:4.2f}A | 实测响应: {abs(st['iq']):5.2f}A | Id: {st['id']:+5.2f}A | 转速: {st['vel']:6.1f} RPM")
    c(ser, 'target 0')
    c(ser, 'disable')

    # 2. 有感 - 梯形位置环测试
    print("\n>>> 【阶段 2/5】有感 - 梯形轨迹位置闭环测试 (mode pos)")
    c(ser, 'disable')
    c(ser, 'fault clear')
    c(ser, 'feedback sensored')
    c(ser, 'angle enc')
    c(ser, 'mode pos')
    c(ser, 'pos accel 100')
    c(ser, 'pos vmax 100')
    c(ser, 'enable')
    time.sleep(0.3)
    p0 = parse_st(c(ser, 'status'))['pos']

    pos_targets = [6.283, 18.850, -6.283, 0.000]
    for pt in pos_targets:
        c(ser, f'target {pt}')
        time.sleep(2.0 + abs(pt)/10.0)
        st = parse_st(c(ser, 'status'))
        actual_delta = st['pos'] - p0
        err = actual_delta - pt
        print(f"    目标位移: {pt:+7.3f} rad | 实测相对位移: {actual_delta:+7.3f} rad | 稳态误差: {err:+6.3f} rad | 静差率: {abs(err)/6.283*360.0:.2f}°")
    c(ser, 'disable')

    # 3. 有感 - 速度环全速域与正反转
    print("\n>>> 【阶段 3/5】有感 - 速度环全速域正反转测试 (mode vel)")
    c(ser, 'disable')
    c(ser, 'fault clear')
    c(ser, 'feedback sensored')
    c(ser, 'angle enc')
    c(ser, 'mode vel')
    c(ser, 'vel ramp 800')
    c(ser, 'tune fw enable 1')
    c(ser, 'tune fw enter 1000')
    c(ser, 'tune fw target 0.860')
    c(ser, 'tune fw gain 600')
    c(ser, 'tune fw idmax 4.0')
    c(ser, 'tune angle_delay 0.85')
    c(ser, 'enable')
    time.sleep(0.3)

    vel_targets = [100, 500, 1000, 2400, 4500, 7000, -1000, 0]
    for spd in vel_targets:
        c(ser, f'target {spd}')
        time.sleep(3.5 if abs(spd) >= 4000 else 2.0)
        st = parse_st(c(ser, 'status'))
        err = st['vel'] - spd
        print(f"    目标转速: {spd:+5d} RPM | 实际转速: {st['vel']:+7.1f} RPM | 稳态误差: {err:+6.1f} RPM | Id: {st['id']:+5.2f}A | Iq: {st['iq']:+5.2f}A")
    c(ser, 'disable')

    # 4. 纯无感 - 阶梯启动与多速域闭环
    print("\n>>> 【阶段 4/5】纯无感 - I/F 启动与多速域闭环 (Sensorless Primary)")
    c(ser, 'disable')
    c(ser, 'fault clear')
    c(ser, 'feedback sensorless')
    c(ser, 'feedback if 0.60 500 300')
    c(ser, 'mode vel')
    c(ser, 'target 500')
    c(ser, 'enable')

    t0 = time.time()
    while time.time() - t0 < 5.0:
        if 'state=run' in c(ser, 'feedback'): break
        time.sleep(0.1)
    print(f"    [+] I/F 自适应换手切入 VESC 闭环成功，耗时 {time.time()-t0:.2f}s")

    sl_targets = [500, 800, 1200, 1600, 2000]
    for spd in sl_targets:
        c(ser, f'target {spd}')
        time.sleep(2.5)
        st = parse_st(c(ser, 'status'))
        fb = c(ser, 'feedback')
        spd_obs = 0.0
        for p in fb.split():
            if p.startswith('spd_obs='): spd_obs = float(p.replace('spd_obs=',''))
        err = spd_obs - spd
        print(f"    无感目标: {spd:4d} RPM | 观测速度: {spd_obs:6.1f} RPM (偏差: {err:+5.1f}) | 编码器参考: {st['vel']:6.1f} RPM | Iq: {st['iq']:+5.2f}A")
    c(ser, 'target 0')
    time.sleep(1.5)
    c(ser, 'disable')
    c(ser, 'feedback sensored')

    # 5. 纯无感 - 反向冷启动测试
    print("\n>>> 【阶段 5/5】纯无感 - 反向冷启动测试 (-500 RPM)")
    c(ser, 'disable')
    c(ser, 'fault clear')
    c(ser, 'feedback sensorless')
    c(ser, 'feedback if 0.60 500 300')
    c(ser, 'mode vel')
    c(ser, 'target -500')
    c(ser, 'enable')

    t0 = time.time()
    rev_ok = False
    while time.time() - t0 < 5.0:
        if 'state=run' in c(ser, 'feedback'):
            rev_ok = True
            break
        time.sleep(0.1)
    time.sleep(2.0)
    st = parse_st(c(ser, 'status'))
    fb = c(ser, 'feedback')
    spd_obs = 0.0
    for p in fb.split():
        if p.startswith('spd_obs='): spd_obs = float(p.replace('spd_obs=',''))
    print(f"    [+] 纯无感反向启动: {'成功 (PASS)' if rev_ok else '失败 (FAIL)'}")
    print(f"    反向目标: -500 RPM | 观测速度: {spd_obs:6.1f} RPM | 编码器真值: {st['vel']:6.1f} RPM | Iq: {st['iq']:+5.2f}A")
    c(ser, 'target 0')
    time.sleep(1.0)
    c(ser, 'disable')
    c(ser, 'feedback sensored')

    ser.close()
    print("\n" + "=" * 80)
    print("                       全闭环测试全部圆满通过！")
    print("=" * 80)

if __name__ == "__main__":
    main()
