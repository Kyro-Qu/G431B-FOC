# -*- coding: utf-8 -*-
"""
eval_sensorless_steady_30s.py
严格遵循用户规范的 30 秒纯无感稳态评估工具:
1. 丢弃启动和 blend 阶段;
2. 进入 SENSORLESS_RUN 状态后额外等待 2.5 秒确保完全进入电气稳态;
3. 连续采样 30.0 秒;
4. 输出:
   - 采样点数与有效时长
   - 平均速度 (Mean Speed) 与相对目标速度误差 (Target Error)
   - 速度波动标准差 / RMS 波动 (Speed Std / Ripple RMS)
   - 速度峰峰值 (Peak-to-Peak) 与最大绝对偏差 (Max Abs Error)
   - Iq RMS 与峰值 |Iq_peak|
   - 观测器置信度最低值 (conf_min) 与平均值 (conf_avg)
   - lock 丢失次数 (lock_lost_cnt)
   - 故障与保护状态检查 (fault / SAFE_STOP)
"""
import serial
import time
import math

PORT = 'COM44'
BAUD = 6500000

def open_serial():
    s = serial.Serial(PORT, BAUD, timeout=0.15)
    time.sleep(0.05)
    s.reset_input_buffer()
    return s

def send_cmd(s, cmd_str, delay=0.03):
    time.sleep(0.01)
    s.read_all()
    s.write((cmd_str + '\r\n').encode())
    time.sleep(delay)
    return s.read_all().decode(errors='ignore')

def main():
    target_rpm = 500.0
    print(f"==================================================================")
    print(f"=== VESC Sensorless Primary 30秒长稳态性能基准评估 (Target: {target_rpm:.0f} RPM) ===")
    print(f"==================================================================")

    s = open_serial()

    # 1. 停机与清除故障
    send_cmd(s, 'disable')
    send_cmd(s, 'fault clear')
    time.sleep(0.6)

    # 2. 配置纯无感
    send_cmd(s, 'feedback sensorless')
    send_cmd(s, 'feedback if 0.60 500')
    send_cmd(s, 'mode vel')
    send_cmd(s, f'target {target_rpm:.0f}')

    print(">>> 1. 启动并等待纯无感切入 RUN 稳态...")
    send_cmd(s, 'enable')

    t_start = time.time()
    entered_run = False
    run_enter_time = 0.0

    while time.time() - t_start < 6.0:
        fb = send_cmd(s, 'feedback', delay=0.04)
        if 'state=run' in fb:
            entered_run = True
            run_enter_time = time.time() - t_start
            print(f"    [*] 成功切入 SENSORLESS_RUN! 耗时: {run_enter_time:.2f}s")
            break
        time.sleep(0.05)

    if not entered_run:
        print("    [FAIL] 未能在 6.0s 内切入 RUN 状态，终止评估！")
        send_cmd(s, 'disable')
        s.close()
        return

    # 3. 严格遵循规范：进入 RUN 后额外等待至少 2 秒，彻底丢弃过渡残差
    discard_wait = 2.5
    print(f">>> 2. 进入 RUN 态，额外沉淀 {discard_wait:.1f}s 以丢弃切入残余暂态...")
    time.sleep(discard_wait)

    # 4. 连续采样 30 秒
    sample_duration = 30.0
    print(f">>> 3. 开始连续 30.0 秒稳态高精度采样 (周期约 40ms)...")

    speeds = []
    iqs = []
    confs = []
    lock_lost_cnt = 0
    fault_detected = False
    fault_info = ""

    t_eval_start = time.time()
    while time.time() - t_eval_start < sample_duration:
        t_now = time.time() - t_eval_start
        fb = send_cmd(s, 'feedback', delay=0.02)
        st = send_cmd(s, 'status', delay=0.02)

        fb_lines = [l for l in fb.splitlines() if 'feedback:' in l and 'mode=sensorless' in l]
        st_id = [l for l in st.splitlines() if 'id=' in l and 'iq=' in l]
        st_m0 = [l for l in st.splitlines() if 'M0 ' in l]

        if st_m0 and 'FAULT' in st_m0[0]:
            fault_detected = True
            fault_info = st_m0[0]
            print(f"    [!] 运行中断故障: {fault_info}")
            break

        if fb_lines:
            toks = dict(kv.split('=') for kv in fb_lines[0].split()[1:] if '=' in kv)
            state = toks.get('state', '')
            if state != 'run':
                print(f"    [!] 偏离 RUN 状态: state={state}")
                lock_lost_cnt += 1

            lock_val = int(toks.get('lock', '1'))
            if lock_val == 0:
                lock_lost_cnt += 1

            spd = float(toks.get('spd_obs', '0.0'))
            conf = float(toks.get('conf', '0.0'))
            speeds.append(spd)
            confs.append(conf)

        if st_id:
            st_tok = dict(kv.split('=') for kv in st_id[0].split() if '=' in kv)
            iq_val = float(st_tok.get('iq', '0.0A').replace('A', ''))
            iqs.append(iq_val)

        time.sleep(0.02)

    actual_duration = time.time() - t_eval_start
    print(f">>> 4. 采样完成 (实际有效时长: {actual_duration:.2f}s, 样本数: {len(speeds)} 点)")

    # 5. 安全关机
    send_cmd(s, 'disable')
    s.close()

    # 6. 详细统计分析
    if len(speeds) < 50 or fault_detected:
        print(f"[FAIL] 样本数过少 ({len(speeds)}) 或发生故障，无法生成基准报告！")
        return

    n = len(speeds)
    mean_speed = sum(speeds) / n
    target_error = mean_speed - target_rpm
    target_error_pct = (abs(target_error) / target_rpm) * 100.0

    # 波动标准差 (Ripple RMS)
    variance = sum((v - mean_speed) ** 2 for v in speeds) / (n - 1)
    std_speed = math.sqrt(variance)

    # 相对目标的跟踪 RMS 误差
    target_rms_err = math.sqrt(sum((v - target_rpm) ** 2 for v in speeds) / n)

    max_spd = max(speeds)
    min_spd = min(speeds)
    peak_to_peak = max_spd - min_spd
    max_dev_from_target = max(abs(v - target_rpm) for v in speeds)

    iq_rms = math.sqrt(sum(q ** 2 for q in iqs) / len(iqs)) if iqs else 0.0
    iq_peak = max(abs(q) for q in iqs) if iqs else 0.0

    conf_min = min(confs) if confs else 0.0
    conf_avg = sum(confs) / len(confs) if confs else 0.0

    print("\n" + "=" * 66)
    print("                30 秒长跑稳态性能基准报告 (Baseline)")
    print("=" * 66)
    print(f" 目标设定转速 (Target)      : {target_rpm:.1f} RPM")
    print(f" 平均转速 (Mean Speed)       : {mean_speed:.2f} RPM")
    print(f" 相对目标平均误差 (Error)    : {target_error:+.2f} RPM ({target_error_pct:.2f}%)")
    print(f" 速度波动标准差 (Ripple RMS) : {std_speed:.2f} RPM")
    print(f" 相对目标跟踪 RMS 误差       : {target_rms_err:.2f} RPM")
    print(f" 转速极值区间 [Min ~ Max]    : [{min_spd:.1f} ~ {max_spd:.1f}] RPM")
    print(f" 速度峰峰值 (Peak-to-Peak)   : {peak_to_peak:.1f} RPM")
    print(f" 最大绝对偏差 (Max Abs Dev)  : {max_dev_from_target:.1f} RPM")
    print("-" * 66)
    print(f" Iq 有效值 (Iq RMS)          : {iq_rms:.3f} A")
    print(f" Iq 绝对峰值 (|Iq_peak|)     : {iq_peak:.3f} A (限幅门限 <= 0.80 A)")
    print(f" 观测器最小置信度 (conf_min) : {conf_min:.2f} (准入门限 >= 0.70)")
    print(f" 观测器平均置信度 (conf_avg) : {conf_avg:.2f}")
    print(f" 锁定丢失次数 (lock_lost_cnt): {lock_lost_cnt} 次")
    print(f" 硬件与系统故障 (Fault/BOR)  : 0 (正常运行无中断)")
    print("=" * 66)

if __name__ == '__main__':
    main()
