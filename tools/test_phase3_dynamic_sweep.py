# -*- coding: utf-8 -*-
"""
工业级动态加减速故障注入 (±800 RPM/s & 正反转) 与平滑接管全场景自动化验证套件
严格遵循独立 500 ms 稳态准入评估体系与全动态门禁诊断输出：
  一、稳态准入指标 (独立 500 ms 窗口满程采样评估):
      1. RMS(°) < 18.0° (rms_gate_pass)
      2. Peak(°) < 30.0° (peak_gate_pass)
      3. SpeedErr(%) < 5.0% (speed_gate_pass)
      4. Lock == 1 (lock_gate_pass)
      5. ConfWin >= 0.70 (conf_gate_pass, 窗口内 min/max/mean 严审)
      6. Qualified_Streak >= 8000 (500ms 连续无中断硬与门达标)
  二、动态接管指标:
      1. 接管响应时间 (ms) < 300 ms
      2. Iq_peak < 0.80 A (电流冲击峰值门禁)
      3. 动态平稳过渡，无失步、无飞车、无停机报警 (fault==0)
      4. 接管后到达目标转速并稳态运行，SpeedErr(%) < 5.0%
      5. 故障消除后 100% 成功平滑回退编码器主控 (Blend < 0.05)
"""
import sys
import serial
import time
import re
import math

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def main():
    print("=" * 155)
    print(">>> 启动动态加减速过程 (±800 RPM/s & 正反转) 故障注入与平滑接管工业级全场景测试 <<<")
    print("=" * 155)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"[FAIL] 无法打开串口 {PORT}: {e}")
        return

    time.sleep(0.1)

    def send(cmd, delay=0.04):
        ser.reset_input_buffer()
        ser.write((cmd + '\r\n').encode('ascii'))
        time.sleep(delay)
        buf = ''
        while ser.in_waiting:
            buf += ser.read(ser.in_waiting).decode('ascii', errors='replace')
            time.sleep(0.005)
        return buf.strip()

    def query():
        st = send('status', 0.04)
        fb = send('feedback', 0.03)
        sl = send('sensorless status', 0.03)

        rpm = 0.0
        iq = 0.0
        flt = 0
        fb_state = "unknown"
        fb_blend = 0.0
        enc_health = "OK"
        lock = 0
        conf = 0.0
        conf_inst = 0.0
        conf_win = 0.0
        over_cnt = 0
        err_deg = 0.0
        spd_err = 0.0
        qual_ticks = 0
        streak_ticks = 0
        win_rms = 0.0
        win_peak = 0.0
        win_min = 1.0

        for line in (st + '\n' + fb + '\n' + sl).splitlines():
            line = line.strip()
            if line.startswith('vel='):
                m = re.search(r'vel=([-\d\.]+)rpm', line)
                if m: rpm = float(m.group(1))
            elif line.startswith('id='):
                m = re.search(r'iq=([-\d\.]+)A', line)
                if m: iq = float(m.group(1))
            elif line.startswith('fault='):
                m = re.search(r'fault=(\d+)', line)
                if m: flt = int(m.group(1))
            elif line.startswith('feedback:'):
                m_st = re.search(r'state=(\w+)', line)
                if m_st: fb_state = m_st.group(1)
                m_bl = re.search(r'blend=([-\d\.]+)', line)
                if m_bl: fb_blend = float(m_bl.group(1))
                m_h = re.search(r'enc_health=(\w+)', line)
                if m_h: enc_health = m_h.group(1)
            elif line.startswith('sensorless:'):
                m_l = re.search(r'lock=(\d+)', line)
                if m_l: lock = int(m_l.group(1))
                m_strk = re.search(r'streak=(\d+)', line)
                if m_strk: streak_ticks = int(m_strk.group(1))
                m_ci = re.search(r'conf_inst=([-\d\.]+)', line)
                if m_ci: conf_inst = float(m_ci.group(1))
                m_cw = re.search(r'conf_win=([-\d\.]+)', line)
                if m_cw: conf_win = float(m_cw.group(1))
                m_c = re.search(r'conf=([-\d\.]+)', line)
                if m_c: conf = float(m_c.group(1))
                m_ov = re.search(r'over_cnt=(\d+)', line)
                if m_ov: over_cnt = int(m_ov.group(1))
                m_e = re.search(r'err=([-\d\.]+)deg', line)
                if m_e: err_deg = float(m_e.group(1))
                m_se = re.search(r'spd_err=([-\d\.]+)', line)
                if m_se: spd_err = float(m_se.group(1))
                m_q = re.search(r'qual=(\d+)', line)
                if m_q: qual_ticks = int(m_q.group(1))
                m_rms = re.search(r'win_rms=([-\d\.]+)', line)
                if m_rms: win_rms = float(m_rms.group(1))
                m_pk = re.search(r'win_peak=([-\d\.]+)', line)
                if m_pk: win_peak = float(m_pk.group(1))
                m_wm = re.search(r'win_min=([-\d\.]+)', line)
                if m_wm: win_min = float(m_wm.group(1))

        if conf_win == 0.0 and conf > 0.0:
            conf_win = conf

        return {
            'rpm': rpm, 'iq': iq, 'flt': flt,
            'state': fb_state, 'blend': fb_blend, 'health': enc_health,
            'lock': lock, 'conf': conf, 'conf_inst': conf_inst, 'conf_win': conf_win,
            'over_cnt': over_cnt, 'err_deg': err_deg, 'spd_err': spd_err,
            'qual_ticks': qual_ticks, 'streak_ticks': streak_ticks,
            'win_rms': win_rms, 'win_peak': win_peak, 'win_min': win_min
        }

    # 初始化配置
    send('fault clear', 0.1)
    send('enc fault clear', 0.05)
    send('sensorless algo vesc', 0.05)
    send('deadtime obs 1', 0.05)
    send('deadtime volt 0.17', 0.05)

    st = send('status', 0.1)
    if ('calib=1' not in st) or ('fault=' in st and 'fault=0' not in st):
        print("执行自动校准与基准对齐...")
        send('fault clear', 0.1)
        res_calib = send('calib', 0.2)
        print(f"  calib 指令响应: {res_calib.strip()}")
        for k in range(50):
            time.sleep(0.3)
            st_c = send('status', 0.1)
            if 'calib=1' in st_c and 'M0 IDLE' in st_c:
                print("  基准对齐完成: calib=1, M0 IDLE")
                break
        else:
            print(f"  [WARN] 自动校准可能未就绪: {st_c.strip()}")
    else:
        print("  编码器基准已处于就绪状态 (calib=1)")

    # 定义 6 项核心动态测试用例
    test_cases = [
        {
            'name': '加速爬坡 (+800 RPM/s) 注入 FREEZE',
            'start_rpm': 800,
            'target_rpm': 1800,
            'ramp': 800,
            'inject_at_rpm': 1200,
            'fault_cmd': 'enc fault freeze',
            'wait_rpm': 1800
        },
        {
            'name': '加速爬坡 (+800 RPM/s) 注入 STEP (+60°)',
            'start_rpm': 800,
            'target_rpm': 1800,
            'ramp': 800,
            'inject_at_rpm': 1200,
            'fault_cmd': 'enc fault step 60',
            'wait_rpm': 1800
        },
        {
            'name': '减速爬坡 (-800 RPM/s) 注入 FREEZE',
            'start_rpm': 1800,
            'target_rpm': 800,
            'ramp': 800,
            'inject_at_rpm': 1300,
            'fault_cmd': 'enc fault freeze',
            'wait_rpm': 800
        },
        {
            'name': '减速爬坡 (-800 RPM/s) 注入 STEP (+60°)',
            'start_rpm': 1800,
            'target_rpm': 800,
            'ramp': 800,
            'inject_at_rpm': 1300,
            'fault_cmd': 'enc fault step 60',
            'wait_rpm': 800
        },
        {
            'name': '反向加速 (-800 RPM/s) 注入 FREEZE',
            'start_rpm': -800,
            'target_rpm': -1800,
            'ramp': 800,
            'inject_at_rpm': -1200,
            'fault_cmd': 'enc fault freeze',
            'wait_rpm': -1800
        },
        {
            'name': '反向减速 (+800 RPM/s) 注入 STEP (+60°)',
            'start_rpm': -1800,
            'target_rpm': -800,
            'ramp': 800,
            'inject_at_rpm': -1300,
            'fault_cmd': 'enc fault step 60',
            'wait_rpm': -800
        }
    ]

    # 支持命令行参数过滤用例: python test_phase3_dynamic_sweep.py [case_idx...]
    selected_indices = []
    for arg in sys.argv[1:]:
        if arg.isdigit():
            idx = int(arg) - 1
            if 0 <= idx < len(test_cases):
                selected_indices.append(idx)
    if selected_indices:
        test_cases = [test_cases[i] for i in selected_indices]
        print(f"  [指定运行用例] 选中 {len(test_cases)} 项用例执行")

    results = []

    for tc in test_cases:
        print("\n" + "=" * 115)
        print(f">>> 执行动态工况: {tc['name']} <<<")
        print("=" * 115)

        print("  [DEBUG CMD] 发送初始化与使能命令:")
        for c in ['fault clear', 'enc fault clear', 'feedback auto', 'feedback speed 420 320', 'mode vel', f"vel ramp {tc['ramp']}", f"target {tc['start_rpm']}", 'enable']:
            res = send(c, 0.08)
            print(f"    {c} -> {res.strip()}")
        st_chk = send('status', 0.08)
        for l in st_chk.splitlines():
            if 'fault=' in l or 'state=' in l or 'calib=' in l:
                print(f"    chk: {l.strip()}")

        # 流程 1: 先等待电机达到目标动态工况起始转速并让长窗充分建立 (允许最多 8.0s)
        t_reach_start = time.time()
        speed_reached = False
        while time.time() - t_reach_start < 8.0:
            time.sleep(0.1)
            d = query()
            # 不仅要求转速到达，而且要求 streak 已经稳步进入稳态建立 (streak >= 8000)
            if abs(d['rpm'] - tc['start_rpm']) < 40.0 and d['streak_ticks'] >= 8000:
                speed_reached = True
                break

        if not speed_reached:
            print(f"  [WARN] 未能在 8.0s 内达到目标起始转速或 streak 未满 8000 (转速: {d['rpm']:.1f}, streak: {d['streak_ticks']})")

        # 流程 2 & 3: 进入独立的 500 ms 稳态准入窗口，整个窗口内持续满程采样并检查所有门禁
        print(f"  --> 进入独立 500 ms 稳态准入采样窗口...")
        qualification_start_timestamp = time.time()
        window_samples = []

        # 持续采样 500 ms，即使 qualified_streak 先达到 8000 也不得提前结束，必须采满整个窗口
        t_win_end = qualification_start_timestamp + 0.50
        while time.time() < t_win_end:
            s = query()
            s['t'] = time.time()
            window_samples.append(s)
            time.sleep(0.08)
        qualification_end_timestamp = time.time()

        # 流程 4: 窗口结束后综合统计与判定
        conf_win_list = [s['conf_win'] for s in window_samples]
        conf_min = min(conf_win_list)
        conf_max = max(conf_win_list)
        conf_mean = sum(conf_win_list) / float(len(conf_win_list))

        streak_start = window_samples[0]['streak_ticks']
        streak_end = window_samples[-1]['streak_ticks']
        qual_start = window_samples[0]['qual_ticks']
        qual_end = window_samples[-1]['qual_ticks']

        # 分析 qualified_streak 连续状态
        if streak_end >= 8000:
            streak_reason = f"ContinuousMet({streak_end}ticks, >=500ms uninterrupted)"
        elif streak_end > streak_start:
            streak_reason = f"Accumulating({streak_start}->{streak_end}ticks, <8000ticks)"
        else:
            streak_reason = f"InterruptedOrZero(end={streak_end}ticks, reset on gate failure)"

        # 动态纹波保护是否激活
        dynamic_hold_active = any(s.get('over_cnt', 0) > 0 for s in window_samples)

        # 门禁布尔判定
        conf_gate_pass = (conf_min >= 0.70)
        rms_max = max(s['win_rms'] for s in window_samples)
        rms_gate_pass = (rms_max < 18.0)
        peak_max = max(s['win_peak'] for s in window_samples)
        peak_gate_pass = (peak_max < 30.0)

        # 速度误差门禁 (SpeedErr < 5.0%)
        spd_err_pcts = [(s['spd_err'] / max(100.0, abs(s['rpm']))) * 100.0 for s in window_samples]
        spderr_max = max(spd_err_pcts)
        speed_gate_pass = (spderr_max < 5.0)

        # 锁定门禁
        lock_all = [s['lock'] for s in window_samples]
        lock_gate_pass = all(l == 1 for l in lock_all)

        # 正式连续准入门禁: 使用 qualified_streak >= 8000
        streak_gate_pass = (streak_end >= 8000)
        # 容忍评分门禁: qualified_cycles >= 8000
        qual_gate_pass = (qual_end >= 8000)

        # 打印要求的全部详细诊断字段
        print(f"  [准入诊断-时间] Start={qualification_start_timestamp:.3f}s, End={qualification_end_timestamp:.3f}s (耗时={(qualification_end_timestamp-qualification_start_timestamp)*1000:.1f}ms, 样本数={len(window_samples)})")
        print(f"  [准入诊断-置信] ConfWin: min={conf_min:.2f}, max={conf_max:.2f}, mean={conf_mean:.2f}")
        print(f"  [准入诊断-连续] Qualified_Streak: start={streak_start}, end={streak_end} ({streak_end/16:.1f}ms) | 连续状态: {streak_reason}")
        print(f"  [准入诊断-容忍] Qualified_Cycles: start={qual_start}, end={qual_end} ({qual_end/16:.1f}ms) | DynamicHoldActive={dynamic_hold_active}")
        print(f"  [准入诊断-门禁] ConfGate={conf_gate_pass} | LockGate={lock_gate_pass} | RMSGate={rms_gate_pass}({rms_max:.1f}°) | PeakGate={peak_gate_pass}({peak_max:.1f}°) | SpdGate={speed_gate_pass}({spderr_max:.1f}%) | StreakGate={streak_gate_pass}")

        pre_steady_pass = conf_gate_pass and lock_gate_pass and rms_gate_pass and peak_gate_pass and speed_gate_pass and streak_gate_pass
        d_pre = window_samples[-1]
        print(f"  [准入综合判定] -> {'PASS' if pre_steady_pass else 'FAIL/COND'}")

        # 2. 启动加减速爬坡并注入故障
        send(f"target {tc['target_rpm']}")
        t_ramp_start = time.time()
        injected = False
        t_inject = 0.0
        d_inject_pre = d_pre

        target_dir = 1 if tc['target_rpm'] > tc['start_rpm'] else -1
        while time.time() - t_ramp_start < 4.0:
            d = query()
            curr_spd = d['rpm']
            trigger = False
            if target_dir > 0 and curr_spd >= tc['inject_at_rpm']:
                trigger = True
            elif target_dir < 0 and curr_spd <= tc['inject_at_rpm']:
                trigger = True

            if trigger and not injected:
                d_inject_pre = d
                t_inject = time.time()
                send(tc['fault_cmd'])
                injected = True
                print(f"  [>> 故障注入时刻 <<] 转速={curr_spd:.1f} RPM (目标注入点={tc['inject_at_rpm']}), 命令: {tc['fault_cmd']}")
                break
            time.sleep(0.02)

        # 3. 监控接管瞬态过程 (800ms)
        iq_peak = 0.0
        handover_ok = False
        handover_time_ms = 0.0
        t_track_end = time.time() + 0.8

        while time.time() < t_track_end:
            d = query()
            if abs(d['iq']) > abs(iq_peak):
                iq_peak = d['iq']
            if (d['state'] in ['sensorless', 'to_sensorless']) or (d['blend'] > 0.8):
                if not handover_ok:
                    handover_ok = True
                    handover_time_ms = (time.time() - t_inject) * 1000.0
            time.sleep(0.03)

        # 4. 等待到达最终设定目标转速并稳态保持 (3.5s)
        t_wait_target = time.time()
        target_reached = False
        d_post = None
        while time.time() - t_wait_target < 3.5:
            d = query()
            if abs(d['rpm'] - tc['wait_rpm']) < 50.0 and d['state'] == 'sensorless':
                target_reached = True
                d_post = d
                break
            time.sleep(0.1)

        if not d_post:
            d_post = query()

        final_spderr_rpm = abs(d_post['rpm'] - tc['wait_rpm'])
        final_spderr_pct = (final_spderr_rpm / abs(tc['wait_rpm'])) * 100.0

        # 动态接管各项指标门禁判定
        iq_ok = (abs(iq_peak) < 0.80)
        ho_time_ok = (handover_ok and handover_time_ms < 300.0)
        post_spd_ok = (target_reached and final_spderr_pct < 5.0)

        print(f"  [动态接管结果] 耗时={handover_time_ms:.1f}ms, Iq峰值={iq_peak:.2f}A, 最终转速={d_post['rpm']:.1f} RPM (误差={final_spderr_rpm:.1f}RPM / {final_spderr_pct:.2f}%), 状态={d_post['state']}")

        # 5. 清除故障，验证平滑回退
        time.sleep(0.4)
        send('enc fault clear')
        t_fb_start = time.time()
        fallback_ok = False
        fallback_time_ms = 0.0
        while time.time() - t_fb_start < 2.5:
            d = query()
            if (d['state'] in ['candidate', 'sensored'] or d['blend'] < 0.05) and d['flt'] == 0:
                fallback_ok = True
                fallback_time_ms = (time.time() - t_fb_start) * 1000.0
                break
            time.sleep(0.05)

        print(f"  [回退恢复结果] 恢复编码器主控={'SUCCESS' if fallback_ok else 'FAILED'}, 耗时={fallback_time_ms:.1f}ms, 状态={d['state']}, Blend={d['blend']:.2f}")

        # 减速停机复位
        send('target 0')
        time.sleep(0.8)
        send('disable')
        time.sleep(0.3)

        # 综合判定: 严格遵循门禁，任一前置准入不达标绝不标 PASS
        fail_reasons = []
        if not peak_gate_pass: fail_reasons.append(f"PrePeak({peak_max:.1f}°)>30°")
        if not rms_gate_pass: fail_reasons.append(f"PreRMS({rms_max:.1f}°)>18°")
        if not conf_gate_pass: fail_reasons.append(f"PreConfWinMin({conf_min:.2f})<0.70")
        if not streak_gate_pass: fail_reasons.append(f"PreStreak({streak_end})<8000")
        if not speed_gate_pass: fail_reasons.append(f"PreSpdErr({spderr_max:.1f}%)>5%")
        if not lock_gate_pass: fail_reasons.append("PreUnlock")
        if not iq_ok: fail_reasons.append(f"IqPk({iq_peak:.2f}A)>0.80A")
        if not ho_time_ok: fail_reasons.append("HandoverTimeout")
        if not post_spd_ok: fail_reasons.append(f"PostSpdErr({final_spderr_pct:.1f}%)>5%")
        if not fallback_ok: fail_reasons.append("FallbackFailed")
        if d['flt'] != 0: fail_reasons.append(f"Fault({d['flt']})")

        verdict = "PASS"
        if len(fail_reasons) > 0:
            if handover_ok and fallback_ok and d['flt'] == 0:
                verdict = "CONDITIONAL"
            else:
                verdict = "FAIL"

        results.append({
            'name': tc['name'],
            'verdict': verdict,
            'reasons': ", ".join(fail_reasons) if fail_reasons else "All criteria met",
            'pre_rms': rms_max,
            'pre_peak': peak_max,
            'pre_lock': 1 if lock_gate_pass else 0,
            'pre_conf_inst': d_pre['conf_inst'],
            'pre_conf_win': conf_mean,
            'pre_conf_min': conf_min,
            'pre_over_cnt': d_pre['over_cnt'],
            'pre_streak_ms': streak_end / 16.0,
            'pre_qual_ms': qual_end / 16.0,
            'inject_spd': d_inject_pre['rpm'],
            't_ho': handover_time_ms,
            'iq_pk': iq_peak,
            'post_spd': d_post['rpm'],
            'post_spderr_pct': final_spderr_pct,
            't_fb': fallback_time_ms,
            'fb_ok': fallback_ok
        })

    ser.close()

    # 输出两套指标大榜
    print("\n" + "=" * 165)
    print(">>> 工业级动态工况测试报告 (严格区分 Streak 连续准入与 Qual 容忍评分，绝不掩盖超限项) <<<")
    print("=" * 165)
    header = f"{'测试工况':<32}|{'判定':^7}|{'RMS(°)':^7}|{'Peak(°)':^8}|{'Lock':^5}|{'C_Win':^6}|{'C_Min':^6}|{'Streak(ms)':^11}|{'Qual(ms)':^9}|{'IqPk(A)':^8}|{'接管(ms)':^9}|{'稳态Err%':^8}|{'回退':^6}|{'判定原因'}"
    print(header)
    print("-" * 165)
    pass_cnt = 0
    cond_cnt = 0
    fail_cnt = 0
    for r in results:
        if r['verdict'] == 'PASS': pass_cnt += 1
        elif r['verdict'] == 'CONDITIONAL': cond_cnt += 1
        else: fail_cnt += 1
        fb_str = "OK" if r['fb_ok'] else "FAIL"
        print(f"{r['name']:<32}|{r['verdict']:^7}|{r['pre_rms']:^7.1f}|{r['pre_peak']:^8.1f}|{r['pre_lock']:^5}|{r['pre_conf_win']:^6.2f}|{r['pre_conf_min']:^6.2f}|{r['pre_streak_ms']:^11.0f}|{r['pre_qual_ms']:^9.0f}|{r['iq_pk']:^8.2f}|{r['t_ho']:^9.1f}|{r['post_spderr_pct']:^7.2f}%|{fb_str:^6}| {r['reasons']}")
    print("=" * 165)
    print(f"测试总结: 总用例 = {len(results)} | PASS = {pass_cnt} | CONDITIONAL = {cond_cnt} | FAIL = {fail_cnt}")
    print("=" * 165)

if __name__ == '__main__':
    main()
