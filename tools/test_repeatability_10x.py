# -*- coding: utf-8 -*-
"""
无感闭环自动接管与回退 10/10 次连续重复性稳定性测试套件
测试典型工况:
  1200 RPM 稳态运行中注入 FREEZE 故障 -> 无感接管 -> 保持运行 -> 清除故障 -> 平滑回退
连续循环 10 次，统计:
  - 10 次接管成功率 (目标 100%)
  - 10 次回退成功率 (目标 100%)
  - 故障接管前 6 项准入品质 (RMS, Peak, SpdErr, Lock, Conf, Qual)
  - 瞬态响应耗时分布与离散度
  - 电流冲击峰值分布与离散度
  - 最终稳态转速跟踪误差
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
    print("=" * 125)
    print(">>> 启动无感自动接管与回退 10/10 次连续重复性稳定性测试 <<<")
    print("=" * 125)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"[FAIL] 无法打开串口 {PORT}: {e}")
        return

    time.sleep(0.1)

    def send(cmd, delay=0.04):
        ser.write((cmd + '\r\n').encode('ascii'))
        time.sleep(delay)
        buf = ''
        if ser.in_waiting:
            buf = ser.read(ser.in_waiting).decode('ascii', errors='ignore')
        return buf

    def query():
        st = send('status', 0.03)
        fb = send('feedback', 0.03)
        sl = send('sensorless status', 0.03)

        rpm = 0.0
        iq = 0.0
        flt = 0
        fb_state = "unknown"
        fb_blend = 0.0
        lock = 0
        conf = 0.0
        err_deg = 0.0
        spd_err = 0.0
        qual_ticks = 0
        win_rms = 0.0
        win_peak = 0.0

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
            elif line.startswith('sensorless:'):
                m_l = re.search(r'lock=(\d+)', line)
                if m_l: lock = int(m_l.group(1))
                m_c = re.search(r'conf=([-\d\.]+)', line)
                if m_c: conf = float(m_c.group(1))
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

        return {
            'rpm': rpm, 'iq': iq, 'flt': flt,
            'state': fb_state, 'blend': fb_blend,
            'lock': lock, 'conf': conf, 'err_deg': err_deg, 'spd_err': spd_err,
            'qual_ticks': qual_ticks, 'win_rms': win_rms, 'win_peak': win_peak
        }

    # 初始化运行环境
    send('fault clear')
    send('enc fault clear')
    send('sensorless algo vesc')
    send('deadtime obs 1')
    send('deadtime volt 0.17')
    send('feedback auto')
    send('feedback speed 450 350')
    send('mode vel')
    send('vel ramp 800')
    send('target 1200')
    send('enable')

    print("加速至 1200 RPM 并建立长窗口稳态基准 (2.5s)...")
    time.sleep(2.5)

    results = []
    TOTAL_ROUNDS = 10

    for r in range(1, TOTAL_ROUNDS + 1):
        print(f"\n--- [Round {r:02d}/{TOTAL_ROUNDS}] ---")
        # 1. 检验注入前的稳态品质
        d_pre = query()
        print(f"  [前置品质] Lock={d_pre['lock']}, Conf={d_pre['conf']:.2f}, RMS={d_pre['win_rms']:.1f}°, Peak={d_pre['win_peak']:.1f}°, Qual={d_pre['qual_ticks']/16:.0f}ms, 转速={d_pre['rpm']:.1f} RPM")

        # 2. 注入 FREEZE 故障
        t_inject = time.time()
        send('enc fault freeze')
        print("  [⚡ 故障注入] enc fault freeze")

        # 3. 监控接管瞬态 (600ms)
        iq_peak = 0.0
        handover_ok = False
        handover_time_ms = 0.0
        t_end = time.time() + 0.6

        while time.time() < t_end:
            d = query()
            if abs(d['iq']) > abs(iq_peak):
                iq_peak = d['iq']
            if (d['state'] in ['sensorless', 'to_sensorless']) or (d['blend'] > 0.8):
                if not handover_ok:
                    handover_ok = True
                    handover_time_ms = (time.time() - t_inject) * 1000.0
            time.sleep(0.03)

        # 4. 稳态运行检验 (0.8s)
        time.sleep(0.8)
        d_mid = query()
        sensorless_hold_ok = (d_mid['state'] == 'sensorless') and (abs(d_mid['rpm'] - 1200.0) < 50.0) and (d_mid['flt'] == 0)
        print(f"  [无感接管] 耗时={handover_time_ms:.1f}ms, Iq峰值={iq_peak:.2f}A, 运行转速={d_mid['rpm']:.1f} RPM, 状态={d_mid['state']}, Hold={'OK' if sensorless_hold_ok else 'FAIL'}")

        # 5. 清除故障，平滑回退
        t_fallback = time.time()
        send('enc fault clear')
        print("  [恢复编码器] enc fault clear")
        fallback_ok = False
        fallback_time_ms = 0.0
        t_fb_end = time.time() + 1.5

        while time.time() < t_fb_end:
            d = query()
            if (d['state'] in ['candidate', 'sensored']) and (d['blend'] < 0.05) and (d['flt'] == 0):
                fallback_ok = True
                fallback_time_ms = (time.time() - t_fallback) * 1000.0
                break
            time.sleep(0.04)

        # 6. 等待下一个回合前的稳态重建 (1.2s)
        time.sleep(1.2)
        d_post = query()
        print(f"  [回退恢复] 耗时={fallback_time_ms:.1f}ms, 状态={d_post['state']}, Blend={d_post['blend']:.2f}, 当前转速={d_post['rpm']:.1f} RPM")

        round_pass = handover_ok and sensorless_hold_ok and fallback_ok and (d_post['flt'] == 0)
        results.append({
            'round': r,
            'pass': round_pass,
            'qual_ms': d_pre['qual_ticks'] / 16.0,
            'rms': d_pre['win_rms'],
            'peak': d_pre['win_peak'],
            'spd_err': d_pre['spd_err'],
            'conf': d_pre['conf'],
            'iq_pre': d_pre['iq'],
            'iq_pk': iq_peak,
            't_handover': handover_time_ms,
            't_fallback': fallback_time_ms,
            'rpm_mid': d_mid['rpm']
        })

    # 测试结束，平稳停机
    send('target 0')
    time.sleep(1.0)
    send('disable')
    ser.close()

    print("\n" + "=" * 135)
    print(">>> 10/10 次连续无差错重复性稳定性测试大榜 (1200 RPM 稳态 FREEZE 接管与平滑回退) <<<")
    print("=" * 135)
    header = f"{'轮次':^6}|{'Pass?':^7}|{'Qual(ms)':^9}|{'RMS(°)':^8}|{'Peak(°)':^9}|{'Conf':^6}|{'IqPre(A)':^9}|{'IqPk(A)':^9}|{'接管耗时(ms)':^14}|{'回退耗时(ms)':^14}|{'稳态转速(RPM)':^14}"
    print(header)
    print("-" * 135)
    pass_cnt = 0
    t_ho_list = []
    iq_pk_list = []
    for res in results:
        status_str = "PASS" if res['pass'] else "FAIL"
        if res['pass']:
            pass_cnt += 1
            t_ho_list.append(res['t_handover'])
            iq_pk_list.append(abs(res['iq_pk']))
        print(f"{res['round']:^6}|{status_str:^7}|{res['qual_ms']:^9.0f}|{res['rms']:^8.1f}|{res['peak']:^9.1f}|{res['conf']:^6.2f}|{res['iq_pre']:^9.2f}|{res['iq_pk']:^9.2f}|{res['t_handover']:^14.1f}|{res['t_fallback']:^14.1f}|{res['rpm_mid']:^14.1f}")
    print("=" * 135)

    avg_ho = sum(t_ho_list) / len(t_ho_list) if t_ho_list else 0
    max_iq = max(iq_pk_list) if iq_pk_list else 0
    print(f"统计汇总: 成功率 = {pass_cnt}/{TOTAL_ROUNDS} ({pass_cnt/TOTAL_ROUNDS*100:.1f}%) | 平均接管耗时 = {avg_ho:.1f}ms | 最大电流冲击 = {max_iq:.2f}A")
    if pass_cnt == TOTAL_ROUNDS:
        print(">>> 10/10 次连续无差错重复性稳定性验证 100% 满分通过！<<<")
    else:
        print(">>> 存在失败轮次，详见上方数据 <<<")

if __name__ == '__main__':
    main()
