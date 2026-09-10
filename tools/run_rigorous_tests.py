# -*- coding: utf-8 -*-
"""
严谨精细化全速域有感与无感实机测试套件 (run_rigorous_tests.py)
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

def send_cmd(ser, cmd, wait=0.08):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("ascii"))
    time.sleep(wait)
    res = ser.read_all().decode(errors="ignore").strip()
    return res

def parse_status(st_text):
    data = {
        "vel": 0.0,
        "vel_obs": 0.0,
        "vel_filt": 0.0,
        "id": 0.0,
        "iq": 0.0,
        "vbus": 0.0,
        "fault": 0,
        "cpu": 0.0
    }
    for line in st_text.splitlines():
        if "vel=" in line:
            m = re.search(r"vel=([\-\d\.]+)rpm", line)
            if m: data["vel"] = float(m.group(1))
            m = re.search(r"vel_obs=([\-\d\.]+)rpm", line)
            if m: data["vel_obs"] = float(m.group(1))
            m = re.search(r"vel_filt=([\-\d\.]+)rpm", line)
            if m: data["vel_filt"] = float(m.group(1))
        if "id=" in line and "iq=" in line:
            m = re.search(r"id=([\-\d\.]+)A", line)
            if m: data["id"] = float(m.group(1))
            m = re.search(r"iq=([\-\d\.]+)A", line)
            if m: data["iq"] = float(m.group(1))
        if "vbus=" in line:
            m = re.search(r"vbus=([\-\d\.]+)V", line)
            if m: data["vbus"] = float(m.group(1))
        if "fault=" in line:
            m = re.search(r"fault=(\d+)", line)
            if m: data["fault"] = int(m.group(1))
        if "cpu=" in line:
            m = re.search(r"cpu=([\-\d\.]+)%", line)
            if m: data["cpu"] = float(m.group(1))
    return data

def run_sensored_tests(ser):
    print("\n" + "=" * 75)
    print("      【测试项目 1】有感全速域 (Sensored) 0 ~ 8500 RPM 阶梯巡检")
    print("=" * 75)

    send_cmd(ser, "disable")
    send_cmd(ser, "fault clear")
    send_cmd(ser, "feedback sensored")
    send_cmd(ser, "angle enc")
    send_cmd(ser, "mode vel")
    send_cmd(ser, "vel ramp 500")

    # 预设弱磁参数
    send_cmd(ser, "tune fw enable 1")
    send_cmd(ser, "tune fw enter 1000")
    send_cmd(ser, "tune fw target 0.850")
    send_cmd(ser, "tune fw gain 800")
    send_cmd(ser, "tune fw idmax 4.5")
    send_cmd(ser, "tune angle_delay 0.80")

    send_cmd(ser, "enable")
    time.sleep(0.3)

    test_points = [100, 1000, 2400, 5000, 7500, 8500, 0]
    results = {}

    for spd in test_points:
        print(f"\n>>> [有感] 目标设定: {spd:5d} RPM ...")
        if spd >= 8000:
            send_cmd(ser, "tune fw target 0.880")
            send_cmd(ser, "tune angle_delay 1.00")

        send_cmd(ser, f"target {spd}")

        # 根据速度不同动态等待加减速爬坡完成
        settle_time = 4.0 if spd >= 5000 else 2.5
        time.sleep(settle_time)

        # 采样 1 秒稳态数据
        st = send_cmd(ser, "status", wait=0.1)
        res = parse_status(st)
        results[spd] = res

        err = res["vel"] - spd
        print(f"    [实测] 实际转速: {res['vel']:7.1f} RPM | 稳态误差: {err:+6.1f} RPM | Id: {res['id']:+5.2f}A | Iq: {res['iq']:+5.2f}A | 故障码: {res['fault']}")

    send_cmd(ser, "target 0")
    time.sleep(1.0)
    send_cmd(ser, "disable")
    return results

def run_sensorless_tests(ser):
    print("\n" + "=" * 75)
    print("      【测试项目 2】纯无感 (Sensorless I/F -> VESC) 500 ~ 2000 RPM 阶梯巡检")
    print("=" * 75)

    send_cmd(ser, "disable")
    send_cmd(ser, "fault clear")
    send_cmd(ser, "feedback sensorless")
    send_cmd(ser, "feedback if 0.55 500 300")
    send_cmd(ser, "mode vel")
    send_cmd(ser, "vel ramp 400")

    # 500, 800, 1200, 1600, 2000 RPM
    sl_points = [500, 800, 1200, 1600, 2000]
    results = {}

    for idx, spd in enumerate(sl_points):
        print(f"\n>>> [纯无感] 目标设定: {spd:5d} RPM ...")
        send_cmd(ser, f"target {spd}")

        if idx == 0:
            print("    [+] 触发纯无感冷启动 (直流吸附 -> 拖动加速 -> 自适应换手切入)...")
            send_cmd(ser, "enable")
            # 持续轮询直到进入稳定 RUN 状态
            t0 = time.time()
            entered_run = False
            while time.time() - t0 < 6.0:
                fb = send_cmd(ser, "feedback", wait=0.05)
                if "state=run" in fb or "state=RUN" in fb:
                    entered_run = True
                    break
                time.sleep(0.1)

            if entered_run:
                print(f"    [+] 纯无感成功进入闭环 RUN 态！耗时 {time.time()-t0:.2f}s")
            else:
                print(f"    [-] 警告: 未在超时前切入RUN态，当前状态: {send_cmd(ser, 'feedback')}")

        settle_time = 3.5 if idx == 0 else 2.5
        time.sleep(settle_time)

        st = send_cmd(ser, "status", wait=0.1)
        fb = send_cmd(ser, "feedback", wait=0.08)
        ss = send_cmd(ser, "sensorless status", wait=0.08)
        res = parse_status(st)
        results[spd] = (res, fb, ss)

        print(f"    [实测] 控制反馈转速: {res['vel_filt']:7.1f} RPM | 观测器速度: {res['vel_obs']:7.1f} RPM | 编码器参考: {res['vel']:7.1f} RPM | Iq: {res['iq']:+5.2f}A")
        for l in ss.splitlines():
            if "sensorless:" in l:
                print("    " + l)

    print("\n>>> 正在将纯无感电机受控减速回 0 并停机...")
    send_cmd(ser, "target 0")
    time.sleep(1.5)
    send_cmd(ser, "disable")
    send_cmd(ser, "feedback sensored")
    return results

def main():
    ser = get_ser()
    print("串口通信成功连接 COM44 @ 6.5Mbps，开始全自动性能压测...")

    sensored_res = run_sensored_tests(ser)
    time.sleep(1.0)
    sensorless_res = run_sensorless_tests(ser)

    ser.close()

    print("\n" + "=" * 80)
    print("                        全速域实机评测最终报告大榜")
    print("=" * 80)
    print("【1. 有感模式基准评测 (Sensored Baseline)】")
    print(f"  {'设定 (RPM)':<10} | {'实测转速 (RPM)':<16} | {'稳态误差 (RPM)':<16} | {'Id (A)':<8} | {'Iq (A)':<8} | {'状态'}")
    print("  " + "-" * 75)
    for spd, r in sensored_res.items():
        err = r['vel'] - spd
        st_desc = "正常 PASS" if r['fault'] == 0 else f"故障 {r['fault']}"
        print(f"  {spd:<10} | {r['vel']:>12.1f} RPM    | {err:>+12.1f} RPM    | {r['id']:>6.2f} | {r['iq']:>6.2f} | {st_desc}")

    print("\n【2. 纯无感主控评测 (Sensorless Primary I/F -> VESC)】")
    print(f"  {'设定 (RPM)':<10} | {'控制转速 (RPM)':<16} | {'观测速度 (RPM)':<16} | {'编码器真值 (RPM)':<18} | {'状态'}")
    print("  " + "-" * 75)
    for spd, (r, fb, ss) in sensorless_res.items():
        st_desc = "正常 PASS" if r['fault'] == 0 else f"故障 {r['fault']}"
        print(f"  {spd:<10} | {r['vel_filt']:>12.1f} RPM    | {r['vel_obs']:>12.1f} RPM    | {r['vel']:>14.1f} RPM   | {st_desc}")

    print("=" * 80)

if __name__ == "__main__":
    main()
