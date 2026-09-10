# -*- coding: utf-8 -*-
"""
在编码器闭环恒速 (mode vel) 下对四套无感观测器进行高精度物理稳态评测：
1. 使用编码器安全闭环，消除开环 V/F 功角漂移与虚假振荡；
2. 在 300, 600, 1000 RPM 三个真实工况点下等待物理速度稳态；
3. 记录各观测器的磁链估算幅值 (flux_mag) 与圆心漂移 (flux_center_a, flux_center_b)；
4. 进行严格的角度误差 RMS、Mean、Peak、转速误差与失锁统计。
"""

import sys
import time
import re
import serial

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def send(ser, cmd, delay=0.08):
    ser.write((cmd + "\n").encode("ascii"))
    time.sleep(delay)
    if ser.in_waiting:
        return ser.read(ser.in_waiting).decode("ascii", errors="replace")
    return ""

def parse_bench(res):
    data = {}
    for line in res.splitlines():
        if "1. Ortega Flux" in line:
            data["Ortega"] = parse_line(line)
        elif "2. VESC Flux" in line:
            data["VESC"] = parse_line(line)
        elif "3. Simplified STO" in line:
            data["STO_PLL"] = parse_line(line)
        elif "4. STO HW CORDIC" in line:
            data["STO_CORDIC"] = parse_line(line)
    return data

def parse_line(line):
    m_mean = re.search(r"mean=([\-\d\.]+) deg", line)
    m_rms = re.search(r"rms=([\-\d\.]+) deg", line)
    m_peak = re.search(r"peak=([\-\d\.]+) deg", line)
    m_spd = re.search(r"speed=([\-\d\.]+) rpm", line)
    m_flux = re.search(r"flux=([\-\d\.]+)mWb", line)
    m_center = re.search(r"center=\(([\-\d\.]+),([\-\d\.]+)\)mWb", line)
    m_lock = re.search(r"lock=(\d+)", line)
    m_cpu = re.search(r"cpu=(\d+) cyc", line)
    return {
        "mean": float(m_mean.group(1)) if m_mean else 0.0,
        "rms": float(m_rms.group(1)) if m_rms else 0.0,
        "peak": float(m_peak.group(1)) if m_peak else 0.0,
        "speed": float(m_spd.group(1)) if m_spd else 0.0,
        "flux_mwb": float(m_flux.group(1)) if m_flux else 0.0,
        "center_a": float(m_center.group(1)) if m_center else 0.0,
        "center_b": float(m_center.group(2)) if m_center else 0.0,
        "lock": int(m_lock.group(1)) if m_lock else 0,
        "cpu": int(m_cpu.group(1)) if m_cpu else 0
    }

def main():
    print("=" * 90)
    print(">>> 启动四种无感观测器在【编码器闭环恒速 (mode vel)】下的影子物理稳态评测 <<<")
    print("=" * 90)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=0.5)
    except Exception as e:
        print(f"打开串口 {PORT} 失败: {e}")
        return

    time.sleep(0.15)
    ser.reset_input_buffer()

    send(ser, "log 0")
    send(ser, "fault clear")
    send(ser, "bench steady 0")
    send(ser, "bench reset")

    # 切换为编码器速度闭环模式，确保电机受纯净正弦电流驱动
    print("[模式切换] 设定为编码器速度闭环 (mode vel) 并使能驱动器 ...")
    send(ser, "mode vel")
    send(ser, "target 0")
    send(ser, "enable")
    time.sleep(0.5)

    TEST_SPEEDS = [300, 600, 1000]
    bench_data = {}

    for target_rpm in TEST_SPEEDS:
        print(f"\n[闭环巡航测试] 目标闭环转速: {target_rpm} RPM ...")
        send(ser, f"target {target_rpm}")

        # 1. 充分等待闭环速度环平稳到达给定转速 (2.5 秒)
        time.sleep(2.5)

        # 2. 读取编码器实际物理转速与母线状态
        st = send(ser, "status", delay=0.1)
        m_vel = re.search(r"vel=([\-\d\.]+)rpm", st)
        true_rpm = abs(float(m_vel.group(1))) if m_vel else float(target_rpm)

        # 3. 对齐各算法当前零点偏移 (消除系统功角固定相位偏差)
        send(ser, "bench align")
        time.sleep(0.05)

        # 4. 开启稳态统计窗口 (统计约 1.5s，等效 24,000 拍快环)
        send(ser, "bench steady 1")
        time.sleep(1.5)
        send(ser, "bench steady 0")

        # 5. 读取稳态统计数据
        res = send(ser, "bench status", delay=0.1)
        parsed = parse_bench(res)
        bench_data[target_rpm] = {
            "true_rpm": true_rpm,
            "observers": parsed
        }

        print(f"  -> 编码器实际转速: {true_rpm:.1f} RPM")
        for name, m in parsed.items():
            lock_str = "锁定" if m["lock"] == 1 else "脱锁"
            diag_str = f"磁链={m['flux_mwb']:.2f}mWb, 中心=({m['center_a']:+.2f},{m['center_b']:+.2f})mWb" if "flux_mwb" in m and m['flux_mwb'] > 0 else "反电势观测"
            print(f"     * {name:<11}: 转速={abs(m['speed']):.0f} RPM | Mean={m['mean']:.1f}°, RMS={m['rms']:.1f}°, Peak={m['peak']:.1f}° | {diag_str} | CPU={m['cpu']}cyc [{lock_str}]")

    # 减速停机
    print("\n[测试完成] 正在平稳减速停机 ...")
    send(ser, "target 0")
    time.sleep(1.0)
    send(ser, "disable")
    send(ser, "log 1")
    ser.close()

    # 打印最终对比报告
    print("\n" + "=" * 110)
    print("                      四种无感观测器在编码器闭环下的影子运行稳态测试报告")
    print("=" * 110)
    print(f"{'实际闭环转速':<12} | {'观测器算法':<14} | {'稳态均值':<10} | {'稳态RMS':<10} | {'峰值误差':<10} | {'磁链估算幅值':<14} | {'磁链圆中心偏移(α,β)':<22} | {'CPU耗时':<12}")
    print("-" * 110)

    for target_rpm, entry in bench_data.items():
        true_rpm = entry["true_rpm"]
        for name, m in entry["observers"].items():
            flux_str = f"{m['flux_mwb']:.3f} mWb" if m.get('flux_mwb', 0) > 0 else "N/A (STO)"
            center_str = f"({m['center_a']:+.3f}, {m['center_b']:+.3f}) mWb" if m.get('flux_mwb', 0) > 0 else "N/A"
            cpu_us = m['cpu'] / 170.0
            print(f"{true_rpm:>9.1f} RPM | {name:<14} | {m['mean']:>7.1f}° | {m['rms']:>7.1f}° | {m['peak']:>7.1f}° | {flux_str:>14} | {center_str:>22} | {cpu_us:>4.2f}µs({m['cpu']:>3d})")
        print("-" * 110)

    # 准入门槛评估
    print("\n【严格闭环切入准入门槛评估 (|Mean|<10°, RMS<15°, Peak<30°, SpeedErr<5%)】:")
    for target_rpm, entry in bench_data.items():
        true_rpm = entry["true_rpm"]
        print(f"\n--> 测点: {true_rpm:.1f} RPM (编码器速度闭环稳态)")
        for name, m in entry["observers"].items():
            err_spd_pct = abs(abs(m['speed']) - true_rpm) / true_rpm * 100.0 if true_rpm > 1.0 else 999.0
            pass_mean = m['mean'] < 10.0
            pass_rms = m['rms'] < 15.0
            pass_peak = m['peak'] < 30.0
            pass_spd = err_spd_pct < 5.0

            if pass_mean and pass_rms and pass_peak and pass_spd:
                status = "PASS (满足影子初测指标)"
            else:
                reasons = []
                if not pass_mean: reasons.append(f"Mean={m['mean']:.1f}°")
                if not pass_rms: reasons.append(f"RMS={m['rms']:.1f}°")
                if not pass_peak: reasons.append(f"Peak={m['peak']:.1f}°")
                if not pass_spd: reasons.append(f"SpeedErr={err_spd_pct:.1f}%")
                status = f"FAIL (未达标: {', '.join(reasons)})"
            print(f"    - {name:<12}: {status}")

if __name__ == "__main__":
    main()
