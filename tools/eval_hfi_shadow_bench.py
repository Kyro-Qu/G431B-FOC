# -*- coding: utf-8 -*-
"""
HFI 影子模块多工况评估脚本 (eval_hfi_shadow_bench.py)
工况覆盖: 0 (静止), 50, 100, 200, 300 RPM
控制模式: 编码器闭环主控 (Sensored Primary)，HFI 仅作为影子运行 (obs5_hfi)
严禁将 HFI 接入实际换相控制角！
"""

import serial
import time
import math
import sys

PORT = 'COM44'
BAUD = 6500000

TEST_SPEEDS = [0, 50, 100, 200, 300]
INJ_VOLTS = [1.0, 1.5] # 探测电压梯次: 1.0V 标准, 1.5V 加强

def run_cmd(ser, cmd_str, wait_sec=0.03):
    ser.reset_input_buffer()
    ser.write((cmd_str + '\n').encode('ascii'))
    time.sleep(wait_sec)
    res = ser.read_all().decode(errors='ignore').strip()
    return res

def parse_bench_status(status_str):
    """
    解析 bench status 输出中的 HFI 指标:
    5. HFI Square-Wave : en=1, Vinj=1.00V, mean=12.3 deg, rms=15.4 deg, peak=28.1 deg, speed=51 rpm, conf=0.85, rip_iq=0.045A, lock=1, cpu=1045 cyc
    """
    metrics = {
        'en': 0, 'vinj': 0.0, 'mean': 0.0, 'rms': 0.0, 'peak': 0.0,
        'speed_est': 0.0, 'conf': 0.0, 'rip_iq': 0.0, 'lock': 0, 'cpu': 0,
        'unlock_cnt': 0
    }
    for line in status_str.splitlines():
        if '5. HFI' in line:
            parts = line.split(',')
            for p in parts:
                p = p.strip()
                if 'en=' in p:
                    metrics['en'] = int(p.split('en=')[1].split()[0])
                if 'Vinj=' in p:
                    metrics['vinj'] = float(p.split('Vinj=')[1].replace('V','').split()[0])
                if 'mean=' in p:
                    metrics['mean'] = float(p.split('mean=')[1].replace('deg','').split()[0])
                if 'rms=' in p:
                    metrics['rms'] = float(p.split('rms=')[1].replace('deg','').split()[0])
                if 'peak=' in p:
                    metrics['peak'] = float(p.split('peak=')[1].replace('deg','').split()[0])
                if 'speed=' in p:
                    metrics['speed_est'] = float(p.split('speed=')[1].replace('rpm','').split()[0])
                if 'conf=' in p:
                    metrics['conf'] = float(p.split('conf=')[1].split()[0])
                if 'rip_iq=' in p:
                    metrics['rip_iq'] = float(p.split('rip_iq=')[1].replace('A','').split()[0])
                if 'lock=' in p:
                    metrics['lock'] = int(p.split('lock=')[1].split()[0])
                if 'cpu=' in p:
                    metrics['cpu'] = int(p.split('cpu=')[1].replace('cyc','').split()[0])
    return metrics

def main():
    print("=" * 70)
    print("      DJI 2312S 连续 FOC HFI 影子模块 5 工况科学评估与基准大榜")
    print("=" * 70)

    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    ser.reset_input_buffer()

    # 1. 停机清障与准备
    run_cmd(ser, 'disable')
    run_cmd(ser, 'fault clear')
    run_cmd(ser, 'bench hfi 0')
    run_cmd(ser, 'feedback sensored')
    run_cmd(ser, 'mode vel')

    # 检查是否已校准
    st = run_cmd(ser, 'status')
    if 'calib=1' not in st:
        print("[*] 正在执行编码器零位校准 (calib full)...")
        run_cmd(ser, 'calib full')
        for _ in range(40):
            time.sleep(0.2)
            st = run_cmd(ser, 'status')
            if 'calib=1' in st and 'IDLE' in st:
                print("[+] 校准完成！")
                break
    else:
        print("[+] 编码器已标定有效！")

    results = []

    for spd in TEST_SPEEDS:
        for vinj in INJ_VOLTS:
            print(f"\n>>> 正在测试工况: 目标转速 = {spd} RPM | 注入电压 = {vinj} V")
            run_cmd(ser, 'target 0' if spd == 0 else f'target {spd}')
            run_cmd(ser, 'enable')

            # 等待速度环稳定
            time.sleep(1.5 if spd > 0 else 0.5)

            # 先记录无注入基线 Iq 纹波
            st_no_inj = run_cmd(ser, 'status')

            # 开启 HFI 影子注入
            run_cmd(ser, f'bench hfi 1 {vinj}')
            # 自动对齐初始相位 offset
            run_cmd(ser, 'bench align')
            time.sleep(0.3)

            # 记录锁定时间 (Lock-in latency)
            t_start = time.time()
            locked = False
            lock_time_ms = 9999
            for _ in range(50):
                time.sleep(0.05)
                b_st = run_cmd(ser, 'bench status', 0.05)
                m = parse_bench_status(b_st)
                if m['lock'] == 1:
                    lock_time_ms = int((time.time() - t_start) * 1000)
                    locked = True
                    break

            print(f"    - 锁定状态: {'LOCKED (' + str(lock_time_ms) + 'ms)' if locked else 'UNLOCKED/SEARCHING'}")

            # 进入稳态统计采样窗口 (统计 3.0 秒)
            run_cmd(ser, 'bench reset')
            run_cmd(ser, 'bench steady 1')
            time.sleep(3.0)
            run_cmd(ser, 'bench steady 0')

            # 抓取稳态统计指标
            b_st_final = run_cmd(ser, 'bench status', 0.08)
            res_m = parse_bench_status(b_st_final)
            res_m['target_rpm'] = spd
            res_m['lock_time_ms'] = lock_time_ms if locked else -1

            # 抓取电机状态
            motor_st = run_cmd(ser, 'status')

            # 关闭 HFI 注入
            run_cmd(ser, 'bench hfi 0')
            time.sleep(0.2)

            print(f"    - 角度误差: Mean={res_m['mean']:.1f}°, RMS={res_m['rms']:.1f}°, Peak={res_m['peak']:.1f}°")
            print(f"    - 估算转速: {res_m['speed_est']:.1f} RPM | 置信度: {res_m['conf']:.2f}")
            print(f"    - 载波电流纹波: {res_m['rip_iq']:.3f} A | 单拍耗时: {res_m['cpu']} cycles")

            results.append(res_m)

    run_cmd(ser, 'target 0')
    time.sleep(0.5)
    run_cmd(ser, 'disable')
    ser.close()

    # 打印最终评测总表
    print("\n" + "=" * 95)
    print("                     DJI 2312S HFI 影子评估最终完整大榜")
    print("=" * 95)
    print(f"{'Speed':<8}{'Vinj':<8}{'MeanErr':<10}{'RMSErr':<10}{'PeakErr':<10}{'LockTime':<12}{'Conf':<8}{'IqRip':<10}{'CPU':<10}{'Verdict'}")
    print("-" * 95)
    for r in results:
        # 收敛及可用性严格判据: RMS < 20°, Peak < 45°, Lock == 1, Conf >= 0.70
        is_pass = (r['lock'] == 1) and (r['rms'] < 20.0) and (r['peak'] < 45.0) and (r['conf'] >= 0.60)
        verdict = "PASS" if is_pass else "FAIL/POOR"
        lock_str = f"{r['lock_time_ms']}ms" if r['lock_time_ms'] > 0 else "TIMEOUT"
        print(f"{r['target_rpm']:<8}{r['vinj']:<8.1f}{r['mean']:<10.1f}{r['rms']:<10.1f}{r['peak']:<10.1f}{lock_str:<12}{r['conf']:<8.2f}{r['rip_iq']:<10.3f}{r['cpu']:<10}{verdict}")
    print("=" * 95)

if __name__ == '__main__':
    main()
