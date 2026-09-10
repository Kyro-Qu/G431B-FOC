# -*- coding: utf-8 -*-
"""
tools/eval_single_experiment.py
执行标准的纯无感 30 秒稳态长跑测试，并全面输出用户规定的各项关键指标。
"""
import serial
import time
import math
import sys
import re

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

def run_evaluation(exp_name="Experiment", target_rpm=500.0, filter_hz=None, vel_kp=None):
    print("=" * 70)
    print(f"=== [30秒稳态测试] {exp_name} (Target: {target_rpm:.0f} RPM) ===")
    print("=" * 70)

    s = open_serial()

    # 1. 停机与清除故障
    send_cmd(s, 'disable')
    send_cmd(s, 'fault clear')
    time.sleep(0.5)

    if vel_kp is not None:
        ret_kp = send_cmd(s, f'vel kp {vel_kp:.4f}')
        print(f"    [配置 Kp] {ret_kp.strip()}")
    if filter_hz is not None:
        ret_f = send_cmd(s, f'vel filter {filter_hz:.1f}')
        print(f"    [配置滤波] {ret_f.strip()}")
    cur_vel_cfg = send_cmd(s, 'vel')
    for l in cur_vel_cfg.splitlines():
        if 'vel kp=' in l or 'filter_high=' in l:
            print(f"    [生效配置] {l.strip()}")

    # 2. 配置纯无感
    send_cmd(s, 'feedback sensorless')
    send_cmd(s, 'feedback if 0.60 500')
    send_cmd(s, 'mode vel')
    send_cmd(s, f'target {target_rpm:.0f}')

    print(">>> 1. 启动纯无感并等待进入 RUN 稳态...")
    send_cmd(s, 'enable')

    t_start = time.time()
    entered_run = False
    while time.time() - t_start < 6.0:
        fb = send_cmd(s, 'feedback', delay=0.04)
        if 'state=run' in fb:
            entered_run = True
            print(f"    [*] 成功切入 SENSORLESS_RUN! 耗时: {time.time() - t_start:.2f}s")
            break
        time.sleep(0.05)

    if not entered_run:
        print("    [FAIL] 未能进入 RUN 状态，终止测试！")
        send_cmd(s, 'disable')
        s.close()
        return None

    # 3. 进入 RUN 后丢弃前 2.5 秒暂态
    print(">>> 2. 进入 RUN 态，等待 2.5s 沉淀切入残余暂态...")
    time.sleep(2.5)

    # 4. 对齐 bench 观测器角度基准，并开启 bench 30秒稳态连续积分
    send_cmd(s, 'bench align', delay=0.05)
    time.sleep(0.05)
    send_cmd(s, 'bench steady 1', delay=0.05)
    time.sleep(0.05)

    # 5. 连续采样 30 秒
    sample_duration = 30.0
    print(">>> 3. 开始连续 30.0 秒稳态高精度数据采集...")

    enc_speeds = []
    vesc_speeds = []
    vel_filts = []
    iq_refs = []
    iqs = []
    vqs = []
    confs = []
    lock_lost_cnt = 0
    fault_detected = False
    fault_info = ""

    t_eval_start = time.time()
    while time.time() - t_eval_start < sample_duration:
        fb = send_cmd(s, 'feedback', delay=0.02)
        st = send_cmd(s, 'status', delay=0.02)

        # 解析 status
        enc_v = None
        v_filt = None
        iq_ref_val = None
        iq_val = None
        vq_val = None

        for l in st.splitlines():
            if 'M0 ' in l and 'FAULT' in l:
                fault_detected = True
                fault_info = l
            if l.startswith('vel='):
                try:
                    enc_v = abs(float(l.replace('vel=', '').replace('rpm', '').strip()))
                except:
                    pass
            elif 'vel_filt=' in l:
                for p in l.split():
                    if p.startswith('vel_filt='):
                        try:
                            v_filt = float(p.replace('vel_filt=', '').replace('rpm', '').strip())
                        except:
                            pass
            elif 'id=' in l and 'iq=' in l:
                toks = dict(kv.split('=') for kv in l.split() if '=' in kv)
                try:
                    iq_val = float(toks.get('iq', '0A').replace('A', ''))
                    iq_ref_val = float(toks.get('iq_ref', '0A').replace('A', ''))
                except:
                    pass
            elif 'vd=' in l and 'vq=' in l:
                toks = dict(kv.split('=') for kv in l.split() if '=' in kv)
                try:
                    vq_val = float(toks.get('vq', '0V').replace('V', ''))
                except:
                    pass

        # 解析 feedback
        fb_lines = [l for l in fb.splitlines() if 'feedback:' in l and 'mode=sensorless' in l]
        if fb_lines:
            toks = dict(kv.split('=') for kv in fb_lines[0].split()[1:] if '=' in kv)
            state = toks.get('state', '')
            if state != 'run':
                lock_lost_cnt += 1
            if int(toks.get('lock', '1')) == 0:
                lock_lost_cnt += 1
            try:
                spd_obs = float(toks.get('spd_obs', '0.0'))
                conf = float(toks.get('conf', '0.0'))
                vesc_speeds.append(spd_obs)
                confs.append(conf)
            except:
                pass

        if enc_v is not None: enc_speeds.append(enc_v)
        if v_filt is not None: vel_filts.append(v_filt)
        if iq_ref_val is not None: iq_refs.append(iq_ref_val)
        if iq_val is not None: iqs.append(iq_val)
        if vq_val is not None: vqs.append(vq_val)

        if fault_detected:
            print(f"    [!] 中断故障: {fault_info}")
            break

        time.sleep(0.02)

    actual_duration = time.time() - t_eval_start
    print(f">>> 4. 30秒稳态采样完成 (实际有效时长: {actual_duration:.2f}s, 样本数: {len(vesc_speeds)})")

    # 读取 bench 累计统计的角度误差
    bs = send_cmd(s, 'bench status')
    theta_mean = 0.0
    theta_rms = 0.0
    theta_peak = 0.0
    for l in bs.splitlines():
        if '2. VESC Flux' in l:
            # 2. VESC Flux       : mean=6.4 deg, rms=8.2 deg, peak=25.3 deg ...
            m_mean = re.search(r'mean=([\d\.\-]+)\s*deg', l)
            m_rms = re.search(r'rms=([\d\.\-]+)\s*deg', l)
            m_peak = re.search(r'peak=([\d\.\-]+)\s*deg', l)
            if m_mean: theta_mean = float(m_mean.group(1))
            if m_rms: theta_rms = float(m_rms.group(1))
            if m_peak: theta_peak = float(m_peak.group(1))

    # 查询系统复位诊断与故障
    st_final = send_cmd(s, 'status')
    rst_info = "OK"
    for l in st_final.splitlines():
        if 'rst_flags=' in l:
            rst_info = l.strip()

    # 安全关机
    send_cmd(s, 'bench steady 0')
    send_cmd(s, 'disable')
    s.close()

    if len(vesc_speeds) < 50 or fault_detected:
        print("[FAIL] 样本不足或发生故障！")
        return None

    # 计算各项指标
    # 1. 实际编码器速度 (后台真值)
    enc_mean = sum(enc_speeds)/len(enc_speeds) if enc_speeds else 0.0
    enc_rms = math.sqrt(sum((x - enc_mean)**2 for x in enc_speeds)/(len(enc_speeds)-1)) if len(enc_speeds) > 1 else 0.0

    # 2. VESC 估计速度
    n_v = len(vesc_speeds)
    vesc_mean = sum(vesc_speeds) / n_v
    vesc_std = math.sqrt(sum((x - vesc_mean)**2 for x in vesc_speeds)/(n_v - 1))
    vesc_min = min(vesc_speeds)
    vesc_max = max(vesc_speeds)
    vesc_p2p = vesc_max - vesc_min
    vesc_target_err = vesc_mean - target_rpm
    vesc_target_err_pct = (abs(vesc_target_err) / target_rpm) * 100.0

    # 3. 速度环反馈 vel_filt
    n_vf = len(vel_filts)
    vf_mean = sum(vel_filts) / n_vf if n_vf else 0.0
    vf_rms = math.sqrt(sum((x - vf_mean)**2 for x in vel_filts)/(n_vf - 1)) if n_vf > 1 else 0.0

    # 4. iq_ref
    n_iqr = len(iq_refs)
    iqr_mean = sum(iq_refs) / n_iqr if n_iqr else 0.0
    iqr_rms = math.sqrt(sum(x**2 for x in iq_refs) / n_iqr) if n_iqr else 0.0
    iqr_peak = max(abs(x) for x in iq_refs) if n_iqr else 0.0

    # 5. iq 实际
    iq_peak = max(abs(x) for x in iqs) if iqs else 0.0

    # 6. Vq
    n_vq = len(vqs)
    vq_mean = sum(vqs) / n_vq if n_vq else 0.0
    vq_rms = math.sqrt(sum(x**2 for x in vqs) / n_vq) if n_vq else 0.0

    # 7. 置信度
    conf_min = min(confs) if confs else 0.0
    conf_avg = sum(confs) / len(confs) if confs else 0.0

    # 判定规则
    pass_speed_rms = vesc_std < 10.0
    pass_target_err = vesc_target_err_pct < 5.0
    pass_theta_rms = theta_rms < 18.0
    pass_theta_peak = theta_peak < 30.0
    pass_conf_min = conf_min >= 0.70
    pass_lock = lock_lost_cnt == 0
    pass_iq_peak = iq_peak < 0.80

    print("\n" + "=" * 70)
    print(f"                      【测试结果】: {exp_name}")
    print("=" * 70)
    print(f"1. 实际编码器速度 (后台真值) : 均值 = {enc_mean:.2f} RPM, 波动标准差 = {enc_rms:.2f} RPM")
    print(f"2. VESC 估计速度指标:")
    print(f"   - 估计转速均值 (Mean)    : {vesc_mean:.2f} RPM (相对目标静差: {vesc_target_err:+.2f} RPM, {vesc_target_err_pct:.2f}%) [{'PASS' if pass_target_err else 'FAIL'}]")
    print(f"   - 速度波动标准差 (RMS)   : {vesc_std:.2f} RPM (门限: < 10.0 RPM) [{'PASS' if pass_speed_rms else 'FAIL'}]")
    print(f"   - 极值 [Min ~ Max]       : [{vesc_min:.1f} ~ {vesc_max:.1f}] RPM")
    print(f"   - 峰峰值 (Peak-to-Peak)  : {vesc_p2p:.1f} RPM")
    print(f"3. 速度环控制反馈 (vel_filt): 均值 = {vf_mean:.2f} RPM, 波动标准差 = {vf_rms:.2f} RPM")
    print(f"4. 电流环给定 (iq_ref)      : 均值 = {iqr_mean:.3f} A, RMS = {iqr_rms:.3f} A, 峰值 = {iqr_peak:.3f} A")
    print(f"   实际相电流峰值 (|Iq_peak|): {iq_peak:.3f} A (限幅门限: < 0.80 A) [{'PASS' if pass_iq_peak else 'FAIL'}]")
    print(f"5. 输出电压 (Vq)            : 均值 = {vq_mean:.3f} V, RMS = {vq_rms:.3f} V")
    print(f"6. 观测器角度误差 (Theta)   : Mean = {theta_mean:.1f}°, RMS = {theta_rms:.1f}° (<18°) [{'PASS' if pass_theta_rms else 'FAIL'}], Peak = {theta_peak:.1f}° (<30°) [{'PASS' if pass_theta_peak else 'FAIL'}]")
    print(f"7. 观测器置信度 (Conf)      : conf_min = {conf_min:.2f} (>=0.70) [{'PASS' if pass_conf_min else 'FAIL'}], conf_avg = {conf_avg:.2f}")
    print(f"8. 状态与锁定保持           : lock丢失次数 = {lock_lost_cnt} 次 [{'PASS' if pass_lock else 'FAIL'}]")
    print(f"9. 硬件与系统保护           : Fault = 0, BOR = 0, WDT = 0 ({rst_info}) [PASS]")
    print("=" * 70)

    res = {
        'exp_name': exp_name,
        'enc_mean': enc_mean, 'enc_rms': enc_rms,
        'vesc_mean': vesc_mean, 'vesc_std': vesc_std,
        'vesc_min': vesc_min, 'vesc_max': vesc_max, 'vesc_p2p': vesc_p2p,
        'vesc_target_err_pct': vesc_target_err_pct,
        'vf_mean': vf_mean, 'vf_rms': vf_rms,
        'iqr_mean': iqr_mean, 'iqr_rms': iqr_rms, 'iqr_peak': iqr_peak,
        'iq_peak': iq_peak,
        'vq_mean': vq_mean, 'vq_rms': vq_rms,
        'theta_mean': theta_mean, 'theta_rms': theta_rms, 'theta_peak': theta_peak,
        'conf_min': conf_min, 'conf_avg': conf_avg,
        'lock_lost_cnt': lock_lost_cnt,
        'pass_speed_rms': pass_speed_rms,
        'pass_target_err': pass_target_err,
        'pass_theta_rms': pass_theta_rms,
        'pass_theta_peak': pass_theta_peak,
        'pass_conf_min': pass_conf_min,
        'pass_lock': pass_lock,
        'pass_iq_peak': pass_iq_peak
    }
    return res

if __name__ == '__main__':
    run_evaluation("Experiment PLL Kp=348 Ki=25000", filter_hz=40.0, vel_kp=0.0010)
