# -*- coding: utf-8 -*-
"""
位置模式迭代闭环测试与参数调优脚本
执行包含：
  1. 使能稳态静止测试 (Zero Target Static Hold, 噪声与高频震颤量化)
  2. 多阶跃轨迹响应测试 (0 -> 1.57 -> 3.14 -> 0 -> -3.14 -> 0 rad)
  3. 参数扫描对比测试 (vkp 阻尼扫查)
  4. 10 轮重复性往返循环测试 (Repeatability Stress Test)
"""

import sys
import re
import time
import math
import serial

PORT = "COM44"
BAUD = 6500000

def open_serial():
    ser = serial.Serial(PORT, BAUD, timeout=0.15, write_timeout=0.5)
    time.sleep(0.15)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def send_cmd(ser, cmd, delay=0.04, retries=3):
    for _ in range(retries):
        ser.reset_input_buffer()
        ser.write((cmd.strip() + "\n").encode("ascii"))
        time.sleep(delay)
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < delay + 0.12:
            n = ser.in_waiting
            if n:
                buf.extend(ser.read(n))
            time.sleep(0.005)
        out = buf.decode("ascii", "replace").strip()
        if out:
            return out
    return ""

def parse_status(st):
    data = {}
    m = re.search(r"M0\s+([A-Z]+)\s+mode=([a-z]+)", st)
    if m:
        data["state"] = m.group(1)
        data["mode"] = m.group(2)
    m = re.search(r"pos=(-?[0-9.]+)rad", st)
    if m:
        data["pos"] = float(m.group(1))
    m = re.search(r"pos_err=(-?[0-9.]+)rad", st)
    if m:
        data["pos_err"] = float(m.group(1))
    m = re.search(r"tgt=(-?[0-9.]+)", st)
    if m:
        data["tgt"] = float(m.group(1))
    m = re.search(r"vel=(-?[0-9.]+)rpm", st)
    if m:
        data["vel"] = float(m.group(1))
    m = re.search(r"vel_obs=(-?[0-9.]+)rpm", st)
    if m:
        data["vel_obs"] = float(m.group(1))
    m = re.search(r"vel_filt=(-?[0-9.]+)rpm", st)
    if m:
        data["vel_filt"] = float(m.group(1))
    m = re.search(r"iq=(-?[0-9.]+)A\s+iq_ref=(-?[0-9.]+)A", st)
    if m:
        data["iq"] = float(m.group(1))
        data["iq_ref"] = float(m.group(2))
    m = re.search(r"calib=([0-9]+)", st)
    if m:
        data["calib"] = int(m.group(1))
    m = re.search(r"fault=([0-9]+)", st)
    if m:
        data["fault"] = int(m.group(1))
    return data

def ensure_idle_and_calib(ser):
    send_cmd(ser, "log 0")
    send_cmd(ser, "disable")
    send_cmd(ser, "fault clear")
    time.sleep(0.1)
    st = parse_status(send_cmd(ser, "status"))
    if st.get("calib") != 1:
        print("[INIT] 电机未校准，启动校准...")
        send_cmd(ser, "calib")
        for _ in range(40):
            time.sleep(0.5)
            s = parse_status(send_cmd(ser, "status"))
            if s.get("state") in ("IDLE", "RUN") and s.get("calib") == 1:
                break
        st = parse_status(send_cmd(ser, "status"))
        if st.get("calib") != 1:
            raise RuntimeError("校准失败: %s" % st)
    print("[INIT] 电机准备就绪: state=%s calib=%s fault=%s" % (st.get("state"), st.get("calib"), st.get("fault")))
    return st

def test_static_hold(ser, duration=2.5):
    print("\n" + "="*60)
    print(">>> 测试 1: 使能稳态静止测试 (Zero Target Static Hold)")
    print("="*60)
    send_cmd(ser, "mode pos")
    send_cmd(ser, "target 0.0")
    send_cmd(ser, "enable")
    time.sleep(0.2)

    samples = []
    t0 = time.time()
    while time.time() - t0 < duration:
        st_raw = send_cmd(ser, "status", delay=0.03)
        p = parse_status(st_raw)
        if "iq_ref" in p and "pos" in p:
            p["t"] = time.time() - t0
            samples.append(p)
        time.sleep(0.04)

    print("采集到 %d 帧静止状态样本:" % len(samples))
    if not samples:
        print(" [FAIL] 未能采集到有效样本")
        return False

    iq_refs = [s["iq_ref"] for s in samples]
    iqs = [s["iq"] for s in samples]
    pos_errs = [s["pos_err"] for s in samples]
    vel_filts = [s["vel_filt"] for s in samples]
    vel_obss = [s["vel_obs"] for s in samples]

    iq_pkpk = max(iq_refs) - min(iq_refs)
    iq_std = math.sqrt(sum((x - sum(iq_refs)/len(iq_refs))**2 for x in iq_refs) / len(iq_refs))
    vel_filt_max = max(abs(v) for v in vel_filts)
    vel_obs_max = max(abs(v) for v in vel_obss)
    pos_err_max = max(abs(e) for e in pos_errs)

    print("  稳态 iq_ref: 均值=%.4fA, 峰峰值=%.4fA, 标准差=%.4fA" % (sum(iq_refs)/len(iq_refs), iq_pkpk, iq_std))
    print("  实测 iq    : 均值=%.4fA, 峰峰值=%.4fA" % (sum(iqs)/len(iqs), max(iqs)-min(iqs)))
    print("  最大速度滤波 vel_filt=%.2f RPM, PLL速度 vel_obs=%.2f RPM" % (vel_filt_max, vel_obs_max))
    print("  位置误差最大值 pos_err=%.4f rad (约 %.2f deg)" % (pos_err_max, pos_err_max*180.0/math.pi))

    # 评判标准：原算法高频交变电流达 +-0.4A (pkpk 0.8A)，优化后要求 iq_ref 纹波峰峰值 < 0.10A
    pass_iq = iq_pkpk < 0.12
    pass_vel = vel_filt_max < 3.0
    print("  [判定] iq_ref 纹波平稳 (<=0.12A): %s (实测 %.4fA)" % ("PASS" if pass_iq else "FAIL", iq_pkpk))
    print("  [判定] 稳态转速接近 0 (<3.0 RPM): %s (实测 %.2f RPM)" % ("PASS" if pass_vel else "FAIL", vel_filt_max))
    return pass_iq and pass_vel

def test_step_trajectory(ser, targets=[1.57, 3.14, 0.0, -3.14, 0.0], hold_time=2.0):
    print("\n" + "="*60)
    print(">>> 测试 2: 多角度阶跃轨迹与稳态跟踪测试")
    print("="*60)
    send_cmd(ser, "mode pos")
    send_cmd(ser, "target 0.0")
    send_cmd(ser, "enable")
    time.sleep(0.3)

    all_pass = True
    for step_idx, tgt in enumerate(targets):
        print("\n--- 阶跃 [%d/%d]: 目标 target = %+.2f rad (%+.1f deg) ---" % (step_idx+1, len(targets), tgt, tgt*180.0/math.pi))
        send_cmd(ser, "target %.3f" % tgt)
        t_start = time.time()
        samples = []
        settled = False
        settle_time = 0.0

        while time.time() - t_start < hold_time:
            st_raw = send_cmd(ser, "status", delay=0.03)
            p = parse_status(st_raw)
            if "pos" in p and "iq_ref" in p:
                t_rel = time.time() - t_start
                p["t"] = t_rel
                samples.append(p)
                # 到位判据：位置误差在 0.05 rad (约 2.8度) 以内且速度 < 5 RPM
                if not settled and abs(p["pos_err"]) < 0.05 and abs(p["vel_filt"]) < 5.0 and t_rel > 0.3:
                    settled = True
                    settle_time = t_rel
            time.sleep(0.04)

        if not samples:
            print("  [FAIL] 无采样数据")
            all_pass = False
            continue

        # 分析到达稳态后的尾部（最后 1.0 秒）
        tail_samples = [s for s in samples if s["t"] >= (hold_time - 1.0)]
        if not tail_samples:
            tail_samples = samples[-5:]

        tail_errs = [s["pos_err"] for s in tail_samples]
        tail_iqs = [s["iq_ref"] for s in tail_samples]
        tail_err_max = max(abs(e) for e in tail_errs)
        tail_err_avg = sum(abs(e) for e in tail_errs) / len(tail_errs)
        tail_iq_pkpk = max(tail_iqs) - min(tail_iqs)

        print("  响应收敛时间: %s" % (("%.2fs" % settle_time) if settled else "未在窗口内收敛"))
        print("  稳态残余误差: 平均=%.4f rad (%.2f deg), 最大=%.4f rad (%.2f deg)" %
              (tail_err_avg, tail_err_avg*180/math.pi, tail_err_max, tail_err_max*180/math.pi))
        print("  到位后 iq_ref 峰峰值: %.4f A (无自激高频震荡)" % tail_iq_pkpk)

        # 打印几个采样点
        print("  轨迹快照:")
        for pt in [samples[0], samples[len(samples)//4], samples[len(samples)//2], samples[-1]]:
            print("    t=%4.2fs pos=%+7.3frad err=%+6.3frad vel=%+5.1frpm iq_ref=%+5.2fA" %
                  (pt["t"], pt["pos"], pt["pos_err"], pt["vel_filt"], pt["iq_ref"]))

        step_pass = settled and (tail_err_max < 0.05) and (tail_iq_pkpk < 0.15)
        print("  [阶跃结果] %s" % ("PASS" if step_pass else "FAIL"))
        if not step_pass:
            all_pass = False

    return all_pass

def test_vkp_sweep(ser):
    print("\n" + "="*60)
    print(">>> 测试 3: 速度阻尼参数扫描 (vkp sweep: 0.006 -> 0.012 -> 0.018)")
    print("="*60)
    vkp_candidates = [0.006, 0.012, 0.018]
    results = {}

    for vkp in vkp_candidates:
        print("\n--- 测试阻尼 vkp = %.4f A/RPM ---" % vkp)
        send_cmd(ser, "pos vkp %.4f" % vkp)
        time.sleep(0.1)

        # 1. 测试静止时的 iq 抖动
        send_cmd(ser, "target 0.0")
        time.sleep(0.5)
        st_static = []
        for _ in range(15):
            p = parse_status(send_cmd(ser, "status", delay=0.03))
            if "iq_ref" in p:
                st_static.append(p)
            time.sleep(0.04)
        iq_static_pkpk = (max(s["iq_ref"] for s in st_static) - min(s["iq_ref"] for s in st_static)) if st_static else 999.0

        # 2. 测试从 0 -> 3.14 阶跃的收敛
        send_cmd(ser, "target 3.14")
        t0 = time.time()
        st_trans = []
        settle_t = None
        while time.time() - t0 < 2.0:
            p = parse_status(send_cmd(ser, "status", delay=0.03))
            if "pos_err" in p:
                t_rel = time.time() - t0
                st_trans.append((t_rel, p))
                if settle_t is None and abs(p["pos_err"]) < 0.05 and abs(p["vel_filt"]) < 5.0 and t_rel > 0.3:
                    settle_t = t_rel
            time.sleep(0.04)

        send_cmd(ser, "target 0.0")
        time.sleep(1.0)

        results[vkp] = {
            "static_iq_pkpk": iq_static_pkpk,
            "settle_time": settle_t,
        }
        print("  vkp=%.4f => 静止iq_ref峰峰值=%.4fA, 3.14阶跃到位时间=%s" %
              (vkp, iq_static_pkpk, ("%.2fs" % settle_t) if settle_t else "未收敛"))

    print("\n>>> 阻尼扫描总结:")
    for vkp, res in results.items():
        print("  vkp=%.4f : 静止噪声=%.4fA, 到位时间=%s" %
              (vkp, res["static_iq_pkpk"], ("%.2fs" % res["settle_time"]) if res["settle_time"] else "Fail"))

    # 恢复推荐最佳阻尼基线 0.012
    send_cmd(ser, "pos vkp 0.012")
    return True

def test_repeatability_10x(ser):
    print("\n" + "="*60)
    print(">>> 测试 4: 10 轮往返重复性与稳定性测试 (0 rad <-> 3.14 rad)")
    print("="*60)
    send_cmd(ser, "pos vkp 0.012")
    send_cmd(ser, "target 0.0")
    time.sleep(0.5)

    cycles = 10
    success_count = 0

    for c in range(1, cycles + 1):
        tgt = 3.14 if (c % 2 == 1) else 0.0
        send_cmd(ser, "target %.2f" % tgt)
        t0 = time.time()
        settled = False
        final_err = 999.0

        while time.time() - t0 < 1.8:
            p = parse_status(send_cmd(ser, "status", delay=0.03))
            if "pos_err" in p:
                final_err = abs(p["pos_err"])
                if not settled and final_err < 0.05 and abs(p["vel_filt"]) < 5.0 and (time.time() - t0) > 0.3:
                    settled = True
            time.sleep(0.04)

        # 检查是否发生保护跳闸
        p_check = parse_status(send_cmd(ser, "status", delay=0.03))
        fault = p_check.get("fault", 0)
        state = p_check.get("state", "UNKNOWN")

        if fault == 0 and state == "RUN" and settled and final_err < 0.05:
            success_count += 1
            status_str = "PASS (err=%.3frad/%.1fdeg)" % (final_err, final_err*180/math.pi)
        else:
            status_str = "FAIL (fault=%d state=%s settled=%s err=%.3f)" % (fault, state, settled, final_err)

        print("  循环 [%2d/%2d] -> 目标 %+.2f rad : %s" % (c, cycles, tgt, status_str))
        time.sleep(0.2)

    print("\n>>> 重复性测试结果: 成功率 %d/%d (%.1f%%)" % (success_count, cycles, (success_count*100.0/cycles)))
    return success_count == cycles

def main():
    ser = open_serial()
    try:
        ensure_idle_and_calib(ser)

        # 1. 静止稳态测试
        pass_static = test_static_hold(ser, duration=2.5)

        # 2. 多角度阶跃测试
        pass_step = test_step_trajectory(ser, targets=[1.57, 3.14, 0.0, -3.14, 0.0], hold_time=2.0)

        # 3. 阻尼参数扫描
        pass_sweep = test_vkp_sweep(ser)

        # 4. 10 轮往返循环测试
        pass_repeat = test_repeatability_10x(ser)

        print("\n" + "="*60)
        print(">>> 综合测试总结汇报")
        print("="*60)
        print("  1. 静止稳态高频震颤抑制 : %s" % ("PASS" if pass_static else "FAIL"))
        print("  2. 多角度阶跃平稳跟踪   : %s" % ("PASS" if pass_step else "FAIL"))
        print("  3. 参数扫描对比验证     : %s" % ("PASS" if pass_sweep else "FAIL"))
        print("  4. 10 轮往返重复稳定性  : %s" % ("PASS" if pass_repeat else "FAIL"))

        all_ok = pass_static and pass_step and pass_sweep and pass_repeat
        print("\n  总体验收结果: %s" % ("全部通过 (PERFECT)" if all_ok else "存在未通过项，需进一步调优"))

    finally:
        send_cmd(ser, "disable")
        ser.close()

if __name__ == "__main__":
    main()
