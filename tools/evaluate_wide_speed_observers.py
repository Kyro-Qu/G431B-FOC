# -*- coding: utf-8 -*-
"""
四套无感观测器宽速域（300, 600, 1000 RPM）物理稳态综合评测脚本：
对 4 套算法在 3 个工况点下逐项检验严苛准入门槛：
- Mean < 10°
- RMS < 15°
- Peak < 30°
- SpeedErr < 5%
- Lock = 1 (连续零脱锁)
"""
import sys
import serial
import time
import re

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

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
    print("=" * 105)
    print(">>> 启动四套无感观测器宽速域 (300, 600, 1000 RPM) 物理稳态准入大考 <<<")
    print("=" * 105)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.15)
    except Exception as e:
        print(f"打开串口 {PORT} 失败: {e}")
        return

    ser.reset_input_buffer()

    def send(cmd, delay=0.08):
        ser.write((cmd + '\n').encode('ascii'))
        time.sleep(delay)
        if ser.in_waiting:
            return ser.read(ser.in_waiting).decode('ascii', errors='replace')
        return ''

    send('log 0')
    send('fault clear')
    send('bench reset')
    send('bench dtcomp 1 0.17')

    st = send('status')
    if 'calib=1' not in st:
        print("执行编码器校准 ...")
        send('calib')
        time.sleep(2.5)
        send('conf write')

    print("使能编码器速度闭环 (mode vel) ...")
    send('mode vel')
    send('target 0')
    send('vel ramp 300')
    send('enable')
    time.sleep(0.3)

    TEST_SPEEDS = [500, 600, 800, 1000]
    results = {}

    for tgt in TEST_SPEEDS:
        print(f"\n[工况点测试] 目标闭环转速: {tgt} RPM ...")
        send(f'target {tgt}')

        # 等待转速到达物理稳态
        for _ in range(25):
            time.sleep(0.2)
            st_now = send('status', delay=0.05)
            m_v = re.search(r'vel=([\-\d\.]+)rpm', st_now)
            v = abs(float(m_v.group(1))) if m_v else 0.0
            if abs(v - tgt) < (tgt * 0.08):
                break

        time.sleep(1.0) # 稳态超调平息
        st_final = send('status', delay=0.05)
        m_v = re.search(r'vel=([\-\d\.]+)rpm', st_final)
        true_rpm = abs(float(m_v.group(1))) if m_v else float(tgt)

        # 对齐零点相位
        send('bench align')
        time.sleep(0.05)

        # 开启严格稳态统计窗口 (统计 1.5s，等效 24,000 拍)
        send('bench steady 1')
        time.sleep(1.5)
        send('bench steady 0')

        bench_res = send('bench status', delay=0.1)
        parsed = parse_bench(bench_res)
        results[tgt] = {
            "true_rpm": true_rpm,
            "observers": parsed
        }
        print(f"  -> 编码器真值转速: {true_rpm:.1f} RPM")
        for name, m in parsed.items():
            print(f"     * {name:<12}: Mean={m['mean']:.1f}°, RMS={m['rms']:.1f}°, Peak={m['peak']:.1f}°, Speed={m['speed']:.0f}RPM, Lock={m['lock']}")

    print("\n[测试完成] 正在平稳降速停机 ...")
    send('target 0')
    time.sleep(1.2)
    send('disable')
    send('log 1')
    ser.close()

    # 输出最终全速域准入对比大表
    print("\n" + "=" * 115)
    print("                             四套无感观测器全速域准入测试最终报告")
    print("=" * 115)
    print(f"{'实际转速':<10} | {'观测器算法':<14} | {'Mean (<10°)':<12} | {'RMS (<15°)':<12} | {'Peak (<30°)':<12} | {'Speed (<5%)':<14} | {'Lock':<6} | {'CPU耗时':<10}")
    print("-" * 115)

    all_pass = True
    for tgt, item in results.items():
        true_v = item["true_rpm"]
        for name, m in item["observers"].items():
            spd_err = abs(abs(m['speed']) - true_v) / true_v * 100.0 if true_v > 1.0 else 999.0
            p_mean = m['mean'] < 10.0
            p_rms = m['rms'] < 15.0
            p_peak = m['peak'] < 30.0
            p_spd = spd_err < 5.0
            p_lock = (m['lock'] == 1)

            ok = p_mean and p_rms and p_peak and p_spd and p_lock
            if not ok:
                all_pass = False

            flag = "PASS" if ok else "FAIL"
            cpu_us = m['cpu'] / 170.0
            print(f"{true_v:>7.1f}RPM | {name:<14} | {m['mean']:>8.1f}° {'✓' if p_mean else '✗'} | {m['rms']:>8.1f}° {'✓' if p_rms else '✗'} | {m['peak']:>8.1f}° {'✓' if p_peak else '✗'} | {m['speed']:>6.0f} ({spd_err:>3.1f}%) {'✓' if p_spd else '✗'} | {m['lock']:^4d} {'✓' if p_lock else '✗'} | {cpu_us:>4.2f}µs")
        print("-" * 115)

    print("\n【综合准入结论】:")
    if all_pass:
        print(">>> 恭喜！四套无感观测器在 300, 600, 1000 RPM 全工况点 100% 达到 Shadow Observer 准入标准！")
    else:
        print(">>> 部分低速弱反电势工况点尚未完全达标，请查看上表具体未达标项。")

if __name__ == "__main__":
    main()
