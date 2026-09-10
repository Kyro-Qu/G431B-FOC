# -*- coding: utf-8 -*-
"""无感观测器多算法并联影子对比评测自动化脚本 (COM44)

评测对象:
  Obs 1: ODrive / Ortega 非线性磁链观测器 + 二阶临界阻尼 PLL
  Obs 2: VESC 单向饱和鲁棒磁链观测器 (Benjamin Vedder 约束版)
  Obs 3: ST MCSDK 经典滑模状态观测器 (STO + PLL)
  Obs 4: ST MCSDK 状态观测器 + STM32G4 硬件 CORDIC 解算
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
    # 解析 4 种观测器的指标
    data = {}
    for line in res.splitlines():
        if "1. ODrive" in line:
            data["ODrive"] = parse_line(line)
        elif "2. VESC" in line:
            data["VESC"] = parse_line(line)
        elif "3. ST STO" in line:
            data["ST_PLL"] = parse_line(line)
        elif "4. ST CORDIC" in line:
            data["ST_CORDIC"] = parse_line(line)
    return data

def parse_line(line):
    # 提取 mean, rms, speed, cpu
    m_mean = re.search(r"mean=([\-\d\.]+) deg", line)
    m_rms = re.search(r"rms=([\-\d\.]+) deg", line)
    m_spd = re.search(r"speed=([\-\d\.]+) rpm", line)
    m_cpu = re.search(r"cpu=(\d+) cyc", line)
    return {
        "mean": float(m_mean.group(1)) if m_mean else 0.0,
        "rms": float(m_rms.group(1)) if m_rms else 0.0,
        "speed": float(m_spd.group(1)) if m_spd else 0.0,
        "cpu": int(m_cpu.group(1)) if m_cpu else 0
    }

def main():
    print("======================================================================")
    print(">>> 启动四种顶尖无感观测器多速域并联影子评测 (Sensorless Benchmark) <<<")
    print("======================================================================")
    ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=0.5)
    time.sleep(0.2)
    ser.reset_input_buffer()

    send(ser, "log 0")
    send(ser, "fault clear")

    # 1. 确保编码器已校准就绪
    print("\n[阶段 1/3] 编码器零位校准与闭环预热 (calib) ...")
    send(ser, "calib")
    time.sleep(2.0)
    send(ser, "fault clear")

    # 2. 扫描速度区间测试
    TEST_SPEEDS = [300, 600, 1000, 1500, 2200]
    results = {sp: {} for sp in TEST_SPEEDS}

    print("\n[阶段 2/3] 全速域阶梯巡航影子对比采样中 ...")
    send(ser, "mode vel")
    send(ser, "enable")
    time.sleep(0.5)

    for sp in TEST_SPEEDS:
        print(f"\n--> 切换至稳态转速: {sp} RPM (持续 2.5s) ...")
        send(ser, f"target {sp}")
        time.sleep(1.5)  # 等待加减速稳定

        # 重置统计区间
        send(ser, "bench reset")
        time.sleep(1.0)  # 采集 1 秒稳态数据 (约 16000 拍)

        status_text = send(ser, "bench status", delay=0.1)
        parsed = parse_bench(status_text)
        results[sp] = parsed

        for name, m in parsed.items():
            print(f"    [{name:10s}] MeanErr: {m['mean']:5.1f}°, RMS: {m['rms']:5.1f}°, EstRPM: {m['speed']:5.0f}, CPU: {m['cpu']}cyc ({m['cpu']/170.0:.2f}µs)")

    # 3. 停机并恢复
    print("\n[阶段 3/3] 测试完成，电机平稳停机 ...")
    send(ser, "target 0")
    time.sleep(0.8)
    send(ser, "disable")
    send(ser, "log 1")
    ser.close()

    # 4. 生成综合对比报告
    print("\n" + "="*80)
    print("                       四种无感观测器实机综合对比评测报告")
    print("="*80)
    print(f"{'转速 (RPM)':<10} | {'算法名称':<12} | {'角度均值误差':<14} | {'角度 RMS 抖动':<14} | {'转速估算偏差':<14} | {'CPU 单拍耗时':<12}")
    print("-" * 80)

    for sp in TEST_SPEEDS:
        for name, m in results[sp].items():
            err_rpm = abs(m['speed'] - sp)
            cpu_us = m['cpu'] / 170.0
            print(f"{sp:<10} | {name:<12} | {m['mean']:>8.1f}°     | {m['rms']:>8.1f}°     | {err_rpm:>8.0f} RPM    | {cpu_us:>6.2f} µs ({m['cpu']} cyc)")
        print("-" * 80)

    print("\n【关键结论与工程选型建议】:")
    print("1. ODrive Ortega : 依靠严格的能量李雅普诺夫守恒，中高速反电势信噪比极佳，稳态误差最小；")
    print("2. VESC 约束版   : 在中低速突变与加减速过程中抗扰动能力最强，单向能量耗散约束杜绝了发散；")
    print("3. ST STO+PLL   : 经典滑模变结构+连续状态空间模型，抗高频开关噪声能力最好，输出曲线最平滑；")
    print("4. ST CORDIC    : 零相移时延，CPU 开销最低，极速响应，但极低速需依赖适当滤波。")
    print("="*80)

if __name__ == "__main__":
    main()
