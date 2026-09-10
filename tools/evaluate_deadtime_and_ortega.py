# -*- coding: utf-8 -*-
"""
端电压死区补偿开启 vs 关闭 对比评测脚本 (COM44)
重点检验：
1. 300, 400, 500, 600, 800, 1000 RPM 下 VESC 与 Ortega 的磁链圆心、Mean、RMS、Speed 误差；
2. 验证死区补偿对低速反电势信噪比和磁链圆的客观改善；
3. 验证 Ortega 在 1000 RPM 下归一化修正后的速度跟踪误差是否消除。
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

def run_test_suite(ser, send, dtcomp_en, speeds=[300, 500, 1000]):
    print(f"\n=======================================================")
    print(f">>> 开始测试组: 端电压死区补偿 dtcomp = {dtcomp_en} <<<")
    print(f"=======================================================")
    send(f'bench dtcomp {dtcomp_en} 0.17')
    time.sleep(0.05)

    results = {}
    for tgt in speeds:
        print(f"\n[测试点] 给定目标转速: {tgt} RPM (dtcomp={dtcomp_en}) ...")
        send(f'target {tgt}')

        # 动态等待到达物理稳态
        for _ in range(30):
            time.sleep(0.2)
            st_now = send('status', delay=0.04)
            m_v = re.search(r'vel=([\-\d\.]+)rpm', st_now)
            v = abs(float(m_v.group(1))) if m_v else 0.0
            if abs(v - tgt) < (tgt * 0.08):
                break

        time.sleep(1.0) # 等待稳态超调完全平息
        st_final = send('status', delay=0.04)
        m_v = re.search(r'vel=([\-\d\.]+)rpm', st_final)
        true_rpm = abs(float(m_v.group(1))) if m_v else float(tgt)

        # 对齐相位零点
        send('bench align')
        time.sleep(0.05)

        # 开启 1.5s 严格稳态统计 (24,000 拍)
        send('bench steady 1')
        time.sleep(1.5)
        send('bench steady 0')

        bench_res = send('bench status', delay=0.08)
        parsed = parse_bench(bench_res)
        results[tgt] = {
            "true_rpm": true_rpm,
            "observers": parsed
        }
        print(f"  -> 编码器真值: {true_rpm:.1f} RPM")
        for name, m in parsed.items():
            diag_str = f"磁链={m['flux_mwb']:.2f}mWb, 圆心=({m['center_a']:+.2f},{m['center_b']:+.2f})mWb" if m['flux_mwb'] > 0 else "反电势"
            print(f"     * {name:<11}: Mean={m['mean']:>5.1f}°, RMS={m['rms']:>5.1f}°, Peak={m['peak']:>5.1f}°, Spd={m['speed']:>5.0f}RPM, Lock={m['lock']} | {diag_str}")

    return results

def main():
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

    print(">>> 启动环境自检与模式切换 ...")
    send('log 0')
    send('fault clear')
    time.sleep(0.1)

    st = send('status')
    if 'calib=1' not in st:
        print("执行编码器校准 ...")
        send('calib')
        time.sleep(2.5)
        send('conf write')

    print("切换编码器速度闭环 (mode vel) 并使能 ...")
    send('mode vel')
    send('target 0')
    send('vel ramp 300')
    send('enable')
    time.sleep(0.3)

    TEST_SPEEDS = [300, 400, 500, 600, 1000]

    # 第 1 轮：启用死区补偿 (dtcomp=1)
    res_with_dt = run_test_suite(ser, send, 1, TEST_SPEEDS)

    # 第 2 轮：关闭死区补偿 (dtcomp=0)
    res_without_dt = run_test_suite(ser, send, 0, TEST_SPEEDS)

    print("\n>>> 平稳降速停机 ...")
    send('target 0')
    time.sleep(1.2)
    send('disable')
    send('log 1')
    ser.close()

    # 输出对比分析大表
    print("\n" + "=" * 115)
    print("                      死区补偿开启 (ON) vs 关闭 (OFF) 全速域对比分析大表")
    print("=" * 115)
    print(f"{'转速':<8} | {'算法':<8} | {'状态':<5} | {'Mean (deg)':<12} | {'RMS (deg)':<12} | {'Peak (deg)':<12} | {'Speed(RPM)':<12} | {'圆心漂移(mWb)':<16} | {'Lock':<5}")
    print("-" * 115)

    for spd in TEST_SPEEDS:
        for obs_name in ["VESC", "Ortega"]:
            m_on = res_with_dt[spd]["observers"].get(obs_name, {})
            m_off = res_without_dt[spd]["observers"].get(obs_name, {})

            if m_on and m_off:
                c_on = f"({m_on['center_a']:+.2f},{m_on['center_b']:+.2f})"
                c_off = f"({m_off['center_a']:+.2f},{m_off['center_b']:+.2f})"

                print(f"{spd:>5d}RPM | {obs_name:<8} | ON    | {m_on['mean']:>10.1f}° | {m_on['rms']:>10.1f}° | {m_on['peak']:>10.1f}° | {m_on['speed']:>10.0f} | {c_on:>16} | {m_on['lock']:^5d}")
                print(f"{'':>8} | {'':<8} | OFF   | {m_off['mean']:>10.1f}° | {m_off['rms']:>10.1f}° | {m_off['peak']:>10.1f}° | {m_off['speed']:>10.0f} | {c_off:>16} | {m_off['lock']:^5d}")
                print("-" * 115)

if __name__ == "__main__":
    main()
