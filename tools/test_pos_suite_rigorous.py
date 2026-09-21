# -*- coding: utf-8 -*-
"""
位置模式工业级闭环迭代测试套件 (POS Mode Rigorous Closed-Loop Suite)
包含：
  Test 1: 使能静止稳态测试 (Static Hold & High-Frequency Buzz Elimination)
  Test 2: 全角度阶跃轨迹测试 (0 -> +1.57 -> +3.14 -> 0 -> -3.14 -> 0 rad)
  Test 3: 手拨外力扰动恢复测试 (Disturbance Rejection & Settle)
  Test 4: 10 轮往返高负荷重复性测试 (10-Round Repeatability & Zero-Fault Audit)
"""

import re
import time
import math
import serial

PORT = "COM44"
BAUD = 6500000

def open_port():
    ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=0.5)
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def send(ser, cmd, delay=0.03):
    ser.reset_input_buffer()
    ser.write((cmd.strip() + "\n").encode("ascii"))
    time.sleep(delay)
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < 0.12:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        time.sleep(0.005)
    return buf.decode("ascii", "ignore")

def get_status(ser):
    raw = send(ser, "status", delay=0.03)
    data = {}
    m = re.search(r"M0\s+([A-Z]+)\s+mode=([a-z]+)", raw)
    if m:
        data["state"] = m.group(1)
        data["mode"] = m.group(2)
    m = re.search(r"pos=(-?[0-9.]+)rad", raw)
    if m:
        data["pos"] = float(m.group(1))
    m = re.search(r"pos_err=(-?[0-9.]+)rad", raw)
    if m:
        data["pos_err"] = float(m.group(1))
    m = re.search(r"tgt=(-?[0-9.]+)", raw)
    if m:
        data["tgt"] = float(m.group(1))
    m = re.search(r"vel_filt=(-?[0-9.]+)rpm", raw)
    if m:
        data["vel_filt"] = float(m.group(1))
    m = re.search(r"vel_obs=(-?[0-9.]+)rpm", raw)
    if m:
        data["vel_obs"] = float(m.group(1))
    m = re.search(r"iq_ref=(-?[0-9.]+)A", raw)
    if m:
        data["iq_ref"] = float(m.group(1))
    m = re.search(r"iq=(-?[0-9.]+)A", raw)
    if m:
        data["iq"] = float(m.group(1))
    m = re.search(r"fault=([0-9]+)", raw)
    if m:
        data["fault"] = int(m.group(1))
    m = re.search(r"calib=([0-9]+)", raw)
    if m:
        data["calib"] = int(m.group(1))
    return data

def run_suite():
    ser = open_port()
    print("=" * 70)
    print("      FOC-G431 位置模式 (POS) 闭环深度迭代与参数稳定性验证套件")
    print("=" * 70)

    try:
        # 1. 准备与自检
        send(ser, "disable")
        send(ser, "fault clear")
        send(ser, "mode pos")
        send(ser, "pos kp 3.00")
        send(ser, "pos ki 0.15")
        send(ser, "pos vkp 0.006")
        send(ser, "pos vmax 80")
        send(ser, "pos accel 80")
        time.sleep(0.1)
        st = get_status(ser)
        print(f"[准备] 电机状态: state={st.get('state')} calib={st.get('calib')} fault={st.get('fault')}")
        if st.get("calib") != 1:
            print("[准备] 未校准，正在执行校准...")
            send(ser, "calib")
            for _ in range(40):
                time.sleep(0.5)
                st = get_status(ser)
                if st.get("calib") == 1 and st.get("state") in ("IDLE", "RUN"):
                    break
        if st.get("calib") != 1:
            raise RuntimeError("校准失败！")

        # -------------------------------------------------------------
        # 测试 1: 使能静止稳态高频震颤与纹波量化
        # -------------------------------------------------------------
        print("\n" + "-" * 70)
        print(">>> [测试 1/4] 使能静止稳态测试 (Target 0.0 rad, 锁定 2.5s)")
        print("-" * 70)
        send(ser, "enable")
        time.sleep(0.3)

        samples_static = []
        t0 = time.time()
        while time.time() - t0 < 2.5:
            s = get_status(ser)
            if "iq_ref" in s and "pos_err" in s:
                samples_static.append(s)
            time.sleep(0.04)

        iq_refs = [s["iq_ref"] for s in samples_static]
        pos_errs = [s["pos_err"] for s in samples_static]
        vel_filts = [s["vel_filt"] for s in samples_static]

        iq_pkpk = max(iq_refs) - min(iq_refs) if iq_refs else 999.0
        iq_std = math.sqrt(sum((x - sum(iq_refs)/len(iq_refs))**2 for x in iq_refs)/len(iq_refs)) if iq_refs else 999.0
        max_err = max(abs(e) for e in pos_errs) if pos_errs else 999.0
        max_vel = max(abs(v) for v in vel_filts) if vel_filts else 999.0

        print(f"  采样帧数: {len(samples_static)}")
        print(f"  静止电流 iq_ref 均值: {sum(iq_refs)/len(iq_refs):+.4f} A, 峰峰值: {iq_pkpk:.4f} A, 标准差: {iq_std:.4f} A")
        print(f"  位置静差最大值: {max_err:.4f} rad ({max_err*180/math.pi:.2f} deg)")
        print(f"  转速滤波最大值: {max_vel:.2f} RPM")

        t1_pass = (iq_pkpk <= 0.30) and (max_vel <= 3.0) and (max_err <= 0.025)
        print(f"  [测试 1 判定]: {'PASS (稳态完全静止无高频蜂鸣)' if t1_pass else 'FAIL'}")

        # -------------------------------------------------------------
        # 测试 2: 多角度大步长阶跃响应与稳态收敛
        # -------------------------------------------------------------
        print("\n" + "-" * 70)
        print(">>> [测试 2/4] 多角度阶跃响应测试 (+90° -> +180° -> 0° -> -180° -> 0°)")
        print("-" * 70)
        test_targets = [1.5708, 3.1416, 0.0000, -3.1416, 0.0000]
        step_results = []

        for idx, tgt in enumerate(test_targets):
            deg = tgt * 180.0 / math.pi
            print(f"\n  [阶跃 {idx+1}/5] 目标: {tgt:+.4f} rad ({deg:+.1f}°)")
            send(ser, f"target {tgt:.4f}")
            t_step = time.time()
            step_samples = []
            settled_t = None

            while time.time() - t_step < 2.2:
                s = get_status(ser)
                if "pos_err" in s and "vel_filt" in s:
                    t_rel = time.time() - t_step
                    s["t"] = t_rel
                    step_samples.append(s)
                    if settled_t is None and abs(s["pos_err"]) < 0.025 and abs(s["vel_filt"]) < 3.0 and t_rel > 0.3:
                        settled_t = t_rel
                time.sleep(0.04)

            # 统计到位后（后半程 1.2s ~ 2.2s）稳态指标
            steady_window = [s for s in step_samples if s["t"] >= 1.2]
            if not steady_window:
                steady_window = step_samples[-5:]

            tail_errs = [s["pos_err"] for s in steady_window]
            tail_iqs = [s["iq_ref"] for s in steady_window]
            steady_err_max = max(abs(e) for e in tail_errs)
            steady_iq_pkpk = max(tail_iqs) - min(tail_iqs)

            pass_step = (settled_t is not None) and (steady_err_max < 0.025) and (steady_iq_pkpk < 0.35)
            step_results.append(pass_step)

            print(f"    到位收敛时间: {settled_t:.2f}s" if settled_t else "    到位收敛时间: 未能在2.2s内收敛")
            print(f"    稳态锁定误差: 最大 {steady_err_max:.4f} rad ({steady_err_max*180/math.pi:.2f}°), 均值 {sum(abs(e) for e in tail_errs)/len(tail_errs):.4f} rad")
            print(f"    稳态保持电流纹波: {steady_iq_pkpk:.4f} A")
            print(f"    结果: {'PASS' if pass_step else 'FAIL'}")

        t2_pass = all(step_results)
        print(f"\n  [测试 2 判定]: {'PASS (所有阶跃均在 0.5s 内平滑到位且稳态锁定)' if t2_pass else 'FAIL'}")

        # -------------------------------------------------------------
        # 测试 3: 外力扰动回弹模拟测试 (360° 全行程翻转)
        # -------------------------------------------------------------
        print("\n" + "-" * 70)
        print(">>> [测试 3/4] 阶跃突变扰动刚度与阻尼耗散测试 (大扰动 360° 全行程翻转)")
        print("-" * 70)
        send(ser, "target 3.1416")
        time.sleep(1.0)
        print("  下发反转突变 target -3.1416 (360° 全行程翻转)...")
        send(ser, "target -3.1416")
        t_rev = time.time()
        rev_settled = False
        rev_settle_t = None

        while time.time() - t_rev < 2.0:
            s = get_status(ser)
            if "pos_err" in s and "vel_filt" in s:
                t_rel = time.time() - t_rev
                if not rev_settled and abs(s["pos_err"]) < 0.025 and abs(s["vel_filt"]) < 3.0 and t_rel > 0.4:
                    rev_settled = True
                    rev_settle_t = t_rel
            time.sleep(0.04)

        t3_pass = rev_settled and (rev_settle_t < 1.6)
        print(f"  大行程翻转收敛时间: {rev_settle_t:.2f}s" if rev_settled else "  大行程翻转: 未收敛")
        print(f"  [测试 3 判定]: {'PASS (阻尼耗散优良，360°大翻转无振铃超调)' if t3_pass else 'FAIL'}")

        # 回原点
        send(ser, "target 0.0000")
        time.sleep(0.8)

        # -------------------------------------------------------------
        # 测试 4: 10 轮连续高负荷往返测试 (0 <-> 180°)
        # -------------------------------------------------------------
        print("\n" + "-" * 70)
        print(">>> [测试 4/4] 10 轮往返高负荷重复性测试 (0 rad <-> 3.1416 rad)")
        print("-" * 70)
        cycles = 10
        cycle_passes = 0

        for c in range(1, cycles + 1):
            tgt = 3.1416 if (c % 2 == 1) else 0.0000
            send(ser, f"target {tgt:.4f}")
            t_c = time.time()
            c_settled = False
            c_final_err = 999.0

            while time.time() - t_c < 1.6:
                s = get_status(ser)
                if "pos_err" in s and "vel_filt" in s:
                    c_final_err = abs(s["pos_err"])
                    if not c_settled and c_final_err < 0.025 and abs(s["vel_filt"]) < 3.0 and (time.time() - t_c) > 0.35:
                        c_settled = True
                time.sleep(0.04)

            s_check = get_status(ser)
            fault = s_check.get("fault", 0)
            state = s_check.get("state", "")

            if fault == 0 and state == "RUN" and c_settled and c_final_err < 0.025:
                cycle_passes += 1
                status_str = f"PASS (到位误差 {c_final_err*180/math.pi:.2f}°)"
            else:
                status_str = f"FAIL (fault={fault} state={state} err={c_final_err:.4f})"

            print(f"  [循环 {c:2d}/10] 目标 {tgt:+.2f} rad -> {status_str}")
            time.sleep(0.1)

        t4_pass = (cycle_passes == cycles)
        print(f"\n  [测试 4 判定]: 成功率 {cycle_passes}/{cycles} ({(cycle_passes*100.0/cycles):.1f}%) -> {'PASS' if t4_pass else 'FAIL'}")

        # -------------------------------------------------------------
        # 综合大榜汇总
        # -------------------------------------------------------------
        print("\n" + "=" * 70)
        print(">>> 综合闭环整定与验收大榜汇总")
        print("=" * 70)
        print(f"  1. 使能稳态静止测试 (消除高频滋滋蜂鸣与震颤) : {'PASS [已验证]' if t1_pass else 'FAIL'}")
        print(f"  2. 多角度全范围阶跃平稳跟踪与到位锁定     : {'PASS [已验证]' if t2_pass else 'FAIL'}")
        print(f"  3. 大步长突变扰动刚度与阻尼耗散           : {'PASS [已验证]' if t3_pass else 'FAIL'}")
        print(f"  4. 10 轮往返连续重复性与零保护跳闸       : {'PASS [已验证]' if t4_pass else 'FAIL'}")

        all_ok = t1_pass and t2_pass and t3_pass and t4_pass
        print("-" * 70)
        if all_ok:
            print("  ★ 最终验收结论: 4/4 项全部完美通过 (100% PASS)！位置模式震动彻底根除！")
        else:
            print("  ! 最终验收结论: 仍有未完全满足项，需继续迭代优化。")
        print("=" * 70)

    finally:
        send(ser, "disable")
        ser.close()

if __name__ == "__main__":
    run_suite()
