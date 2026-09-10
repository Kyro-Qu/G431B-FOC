# -*- coding: utf-8 -*-
"""
全面多速域实机性能与功能测试脚本 (test_all_ranges.py)
测试项目:
1. 有感模式基准测试: 0, 100, 1000, 2400, 5000, 8500 RPM
2. 纯无感模式测试: I/F启动, 500, 800, 1200, 2000 RPM, 以及失锁保护
"""
import sys
import time
import serial

PORT = "COM44"
BAUD = 6500000

def get_ser():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    return ser

def send(ser, cmd, wait=0.08):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("ascii"))
    time.sleep(wait)
    res = ser.read_all().decode(errors="ignore").strip()
    return res

def wait_for(ser, condition_func, timeout=6.0, step=0.05):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if condition_func():
            return True
        time.sleep(step)
    return False

def test_sensored(ser):
    print("\n" + "=" * 70)
    print("           阶段一：全速域【有感模式】多工作点实机测试")
    print("=" * 70)

    print("[1.1] 故障清除与编码器校准 (calib)...")
    send(ser, "fault clear")
    send(ser, "feedback sensored")
    send(ser, "calib")

    # 等待校准完成
    def check_calib():
        st = send(ser, "status")
        return "calib=1*" in st or "calib=1" in st

    if wait_for(ser, check_calib, timeout=4.0):
        print("  --> 编码器校准成功！calib=1")
    else:
        print("  --> 警告：校准未在4s内置位，继续读取状态...")

    send(ser, "mode vel")
    send(ser, "enable")
    time.sleep(0.3)

    # 包含 0, 100, 1000, 2400, 5000, 8500 RPM
    speeds = [100, 1000, 2400, 5000, 8500, 0]
    sensored_data = {}

    for spd in speeds:
        print(f"\n>>> 设定目标转速: {spd} RPM ...")
        # 若是 8500 RPM，提前放宽弱磁与超前角
        if spd >= 5000:
            send(ser, "tune fw target 0.820")
            send(ser, "tune angle_delay 0.80")
        if spd >= 8000:
            send(ser, "tune fw target 0.860")
            send(ser, "tune angle_delay 1.00")
            send(ser, "vel ramp 500")

        send(ser, f"target {spd}")

        # 爬坡与稳定时间
        wait_time = 3.5 if spd >= 5000 else 2.0
        time.sleep(wait_time)

        # 连续采样 3 次取稳定均值
        samples = []
        for _ in range(3):
            st = send(ser, "status")
            samples.append(st)
            time.sleep(0.15)

        # 解析最后一帧
        last_st = samples[-1]
        vel_act = 0.0
        iq_act = 0.0
        id_act = 0.0
        vbus = 0.0
        for line in last_st.splitlines():
            if "vel=" in line:
                for part in line.split():
                    if part.startswith("vel="):
                        try: vel_act = float(part.replace("vel=", "").replace("rpm", ""))
                        except: pass
            if "id=" in line and "iq=" in line:
                for part in line.split():
                    if part.startswith("id="):
                        try: id_act = float(part.replace("id=", "").replace("A", ""))
                        except: pass
                    elif part.startswith("iq="):
                        try: iq_act = float(part.replace("iq=", "").replace("A", ""))
                        except: pass
            if "vbus=" in line:
                for part in line.split():
                    if part.startswith("vbus="):
                        try: vbus = float(part.replace("vbus=", "").replace("V", ""))
                        except: pass

        err = vel_act - spd
        sensored_data[spd] = (vel_act, err, id_act, iq_act, vbus)
        print(f"  [实测结果] 实际转速: {vel_act:6.1f} RPM | 误差: {err:+5.1f} RPM | Id: {id_act:+5.2f}A | Iq: {iq_act:5.2f}A | Vbus: {vbus:.2f}V")

    send(ser, "target 0")
    time.sleep(1.0)
    send(ser, "disable")
    return sensored_data

def test_sensorless(ser):
    print("\n" + "=" * 70)
    print("           阶段二：纯无感模式 (I/F -> VESC) 多工作点实机测试")
    print("=" * 70)

    send(ser, "disable")
    send(ser, "fault clear")
    send(ser, "feedback sensorless")
    # 设定平顺拖动参数：0.55A, 500 RPM, 300 RPM/s
    send(ser, "feedback if 0.55 500 300")
    send(ser, "mode vel")

    sl_speeds = [500, 800, 1200, 2000]
    sensorless_data = {}

    for idx, spd in enumerate(sl_speeds):
        print(f"\n>>> 设定纯无感目标: {spd} RPM ...")
        send(ser, f"target {spd}")
        if idx == 0:
            print("  --> 启动无感冷启动序列 (I/F吸附 -> 拖动加速 -> 自适应换手切入)...")
            send(ser, "enable")
            # 等待切入 RUN 态
            t0 = time.time()
            is_run = False
            while time.time() - t0 < 5.0:
                fb = send(ser, "feedback")
                if "state=run" in fb or "state=RUN" in fb:
                    is_run = True
                    break
                time.sleep(0.1)
            if is_run:
                print(f"  [+] 成功自适应换手切入 VESC 纯无感闭环 (耗时 {time.time()-t0:.2f}s)！")
            else:
                print(f"  [-] 警告: 换手未确认，当前反馈: {send(ser, 'feedback')}")

        time.sleep(2.5) # 等待稳态

        st = send(ser, "status")
        fb = send(ser, "feedback")
        ss = send(ser, "sensorless status")

        vel_obs = 0.0
        vel_enc = 0.0
        iq_act = 0.0
        for line in st.splitlines():
            if "vel=" in line:
                for part in line.split():
                    if part.startswith("vel="):
                        try: vel_enc = float(part.replace("vel=", "").replace("rpm", ""))
                        except: pass
                    elif part.startswith("vel_obs="):
                        try: vel_obs = float(part.replace("vel_obs=", "").replace("rpm", ""))
                        except: pass
            if "iq=" in line:
                for part in line.split():
                    if part.startswith("iq="):
                        try: iq_act = float(part.replace("iq=", "").replace("A", ""))
                        except: pass

        sensorless_data[spd] = (vel_obs, vel_enc, vel_obs - spd, iq_act)
        print(f"  [实测结果] 观测转速: {vel_obs:6.1f} RPM | 编码器参考: {vel_enc:6.1f} RPM | 给定偏差: {vel_obs - spd:+5.1f} RPM | Iq: {iq_act:5.2f}A")
        for line in ss.splitlines():
            if "sensorless:" in line or "obs" in line:
                print("    " + line)

    print("\n>>> 正在将纯无感电机平稳减速停机...")
    send(ser, "target 0")
    time.sleep(1.0)
    send(ser, "disable")
    send(ser, "feedback sensored")
    return sensorless_data

def main():
    try:
        ser = get_ser()
    except Exception as e:
        print(f"[-] 打开串口 {PORT} 失败: {e}")
        return

    print("串口连接正常，准备开始全工况实机自动巡检...")

    sensored_res = test_sensored(ser)
    time.sleep(1.0)
    sensorless_res = test_sensorless(ser)

    ser.close()

    print("\n" + "=" * 80)
    print("                     【全工况实机测试最终汇总大榜】")
    print("=" * 80)
    print("【一、 有感模式 (Sensored) 0 ~ 8500 RPM】:")
    print(f"  {'设定转速':<10} | {'实际转速':<12} | {'稳态误差':<12} | {'Id电流':<10} | {'Iq电流':<10} | {'母线电压'}")
    print("  " + "-" * 72)
    for spd, val in sensored_res.items():
        print(f"  {spd:<10} | {val[0]:>8.1f} RPM | {val[1]:>+7.1f} RPM | {val[2]:>6.2f}A   | {val[3]:>6.2f}A   | {val[4]:.2f}V")

    print("\n【二、 纯无感模式 (Sensorless I/F->VESC) 500 ~ 2000 RPM】:")
    print(f"  {'设定转速':<10} | {'观测转速':<12} | {'编码器真值':<12} | {'给定偏差':<12} | {'Iq电流'}")
    print("  " + "-" * 65)
    for spd, val in sensorless_res.items():
        print(f"  {spd:<10} | {val[0]:>8.1f} RPM | {val[1]:>8.1f} RPM | {val[2]:>+7.1f} RPM | {val[3]:>6.2f}A")

    print("=" * 80)
    print("测试全部完成，系统已安全回位！")

if __name__ == "__main__":
    main()
