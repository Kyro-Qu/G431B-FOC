# -*- coding: utf-8 -*-
"""
第三阶段自动化评测套件: VESC 多速度梯度 (500~2000 RPM) 与正反转平滑接管全场景实测
覆盖速度点:
  正转: +500, +600, +800, +1000, +1500, +2000 RPM
  反转: -500, -600, -800, -1000, -1500, -2000 RPM
每点验证:
  1. 稳态准入门禁 (RMS < 18°, Peak < 30°, SpeedErr < 5%, Lock == 1)
  2. 软件故障注入 (FREEZE 冻结) 触发平滑加权过渡切向无感 (blend 0.0 -> 1.0)
  3. 采集切换瞬态量测: 切换前 Iq, 瞬态峰值 Iq_peak, 切换后稳态 Iq_rms, Vq, Vbus, 响应时间
  4. 清除故障平滑回退编码器 (blend 1.0 -> 0.0) 恢复稳态
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
    print("=" * 115)
    print(">>> 启动第三阶段实机攻关: VESC 多速度梯度 (500~2000 RPM) 与正反转接管瞬态量测 <<<")
    print("=" * 115)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.15)
    except Exception as e:
        print(f"打开串口 {PORT} 失败: {e}")
        return False

    ser.reset_input_buffer()

    def send(cmd, delay=0.05):
        ser.write((cmd + '\n').encode('ascii'))
        time.sleep(delay)
        res = ''
        if ser.in_waiting:
            res = ser.read(ser.in_waiting).decode('ascii', errors='replace').strip()
        return res

    def query():
        st = send('status', 0.03)
        m_v = re.search(r'vel=([\-\d\.]+)rpm', st)
        m_iq = re.search(r'iq=([\-\d\.]+)A', st)
        m_vq = re.search(r'vq=([\-\d\.]+)V', st)
        m_vbus = re.search(r'vbus=([\-\d\.]+)V', st)
        m_flt = re.search(r'fault=(\d+)', st)
        rpm = float(m_v.group(1)) if m_v else 0.0
        iq = float(m_iq.group(1)) if m_iq else 0.0
        vq = float(m_vq.group(1)) if m_vq else 0.0
        vbus = float(m_vbus.group(1)) if m_vbus else 0.0
        flt = int(m_flt.group(1)) if m_flt else 0
        fb = send('feedback', 0.03)
        sl = send('sensorless status', 0.03)
        m_obs = re.search(r'spd_obs=([\-\d\.]+)', fb)
        spd_obs = float(m_obs.group(1)) if m_obs else 0.0
        return rpm, iq, vq, vbus, flt, fb, sl, spd_obs

    # 1. 基础初始化
    send('log 0')
    send('fault clear')
    send('enc fault clear')
    send('sensorless algo vesc')
    send('feedback auto')
    send('bench dtcomp 1 0.17')

    # 校准检查
    st = send('status')
    if 'calib=1' not in st:
        print("执行自动校准与固化...")
        send('calib')
        for _ in range(30):
            time.sleep(0.3)
            st_c = send('status')
            if 'calib=1' in st_c and 'M0 IDLE' in st_c:
                send('conf write')
                break

    speed_targets = [500, 600, 800, 1000, 1500, 2000,
                     -500, -600, -800, -1000, -1500, -2000]

    results = []

    print(f"\n准备遍历测试 {len(speed_targets)} 个速度工况点: {speed_targets}\n")

    for target_rpm in speed_targets:
        abs_spd = abs(target_rpm)
        direction_str = "正转 (+)" if target_rpm > 0 else "反转 (-)"
        print("=" * 95)
        print(f">>> 工况点: {direction_str} {abs_spd:4d} RPM (Target={target_rpm}) <<<")
        print("=" * 95)

        # 0. 确保每个工况点干净无残留
        send('fault clear')
        send('enc fault clear')
        send('feedback auto')

        # 动态设置准入速度: 统一使用无感物理可信区间 [420, 320] RPM
        send('feedback speed 420 320')

        # 启动并爬坡
        send('mode vel')
        send('vel ramp 800')
        send('target 0')
        send('enable')
        time.sleep(0.15)
        send(f'target {target_rpm}')

        # 等待稳态
        ramp_time = 1.0 + (abs_spd / 600.0)
        t_wait_start = time.time()
        rpm_reached = False
        while time.time() - t_wait_start < (ramp_time + 4.0):
            time.sleep(0.15)
            rpm, iq, vq, vbus, flt, fb, sl, spd_obs = query()
            if abs(abs(rpm) - abs_spd) < (max(40.0, abs_spd * 0.08)):
                rpm_reached = True
                break

        if not rpm_reached:
            print(f"  [WARN] 未能在预期时间内达到设定转速: 当前转速={rpm:.1f} RPM, fault={flt}")
            send('target 0')
            time.sleep(0.8)
            send('disable')
            results.append({'target': target_rpm, 'pass': False, 'reason': 'RampTimeout'})
            continue

        time.sleep(0.5)
        send('bench align')
        time.sleep(0.5)

        # 等待稳态准入资格真正就绪 (进入 candidate 态)
        qual_ok = False
        t_qual_start = time.time()
        while time.time() - t_qual_start < 6.0:
            time.sleep(0.1)
            rpm, iq, vq, vbus, flt, fb, sl, spd_obs = query()
            if ('state=candidate' in fb) and ('enc_health=OK' in fb) and (flt == 0):
                qual_ok = True
                break

        if not qual_ok:
            print(f"  [FAIL] 准入门禁未达标: {fb} | {sl}")
            send('target 0')
            time.sleep(0.8)
            send('disable')
            results.append({'target': target_rpm, 'pass': False, 'reason': 'QualTimeout'})
            continue

        print(f"  [准入就绪] 转速={rpm:6.1f} RPM, Iq={iq:5.2f}A, Vq={vq:5.2f}V, Vbus={vbus:4.1f}V | {fb}")

        # 瞬态量测前基准记录
        iq_pre = iq
        vq_pre = vq
        vbus_pre = vbus

        # 注入 FREEZE 故障触发接管
        send('enc fault freeze')
        t_inject = time.time()

        iq_peaks = []
        iq_posts = []
        handover_ok = False
        handover_time_ms = 0.0
        final_run_rpm = 0.0

        for tick in range(16): # 观察 800ms
            time.sleep(0.05)
            rpm, iq, vq, vbus, flt, fb, sl, spd_obs = query()
            iq_peaks.append(abs(iq))
            if 'sensorless' in fb and 'enc_health=FAILED' in fb and (flt == 0):
                if not handover_ok:
                    handover_ok = True
                    handover_time_ms = (time.time() - t_inject) * 1000.0
                iq_posts.append(iq)
                final_run_rpm = spd_obs

        iq_peak_val = max(iq_peaks) if iq_peaks else abs(iq_pre)
        iq_post_rms = math.sqrt(sum(x*x for x in iq_posts) / len(iq_posts)) if iq_posts else abs(iq_pre)

        print(f"  [接管完成] 响应耗时~{handover_time_ms:.1f}ms, Iq_pre={iq_pre:.2f}A, Iq_peak={iq_peak_val:.2f}A, Iq_post_rms={iq_post_rms:.2f}A, Vq={vq:.2f}V, Vbus={vbus:.1f}V")

        if handover_ok:
            print(f"  ==> 接管成功！无感转速={final_run_rpm:.1f} RPM 保持闭环稳定运行！")
        else:
            print(f"  ==> 接管失败！{fb}")

        # 清除故障，测试平滑回退编码器
        send('enc fault clear')
        time.sleep(0.1)
        fallback_ok = False
        for tick in range(20): # 给足 1.0s 观测完整淡入过渡
            time.sleep(0.05)
            rpm, iq, vq, vbus, flt, fb, sl, spd_obs = query()
            if 'enc_health=OK' in fb and 'blend=0.00' in fb and (flt == 0):
                fallback_ok = True
                final_run_rpm = rpm
                break

        if fallback_ok:
            print(f"  ==> 回退成功！编码器转速={rpm:.1f} RPM 已平滑淡入切回编码器主控！")
        else:
            print(f"  ==> 回退失败！{fb}")

        point_pass = handover_ok and fallback_ok and (flt == 0)
        results.append({
            'target': target_rpm,
            'pass': point_pass,
            'rpm': final_run_rpm,
            'iq_pre': iq_pre,
            'iq_peak': iq_peak_val,
            'iq_post_rms': iq_post_rms,
            'vq': vq,
            'vbus': vbus,
            'time_ms': handover_time_ms
        })

        # 安全减速停机准备下一工况
        send('target 0')
        time.sleep(1.0)
        send('disable')
        time.sleep(0.4)

    ser.close()

    print("\n" + "=" * 115)
    print(">>> 第三阶段 VESC 多速度梯度与正反转接管自动化测试大榜 <<<")
    print("=" * 115)
    print(f"{'Target (RPM)':^14}|{'Pass?':^8}|{'Real RPM':^10}|{'Iq Pre(A)':^11}|{'Iq Peak(A)':^12}|{'Iq RMS(A)':^11}|{'Vq (V)':^9}|{'Vbus (V)':^10}|{'Time(ms)':^9}")
    print("-" * 115)
    all_ok = True
    for r in results:
        status_str = "PASS" if r['pass'] else "FAIL"
        if not r['pass']:
            all_ok = False
        if 'rpm' in r:
            print(f"{r['target']:^14}|{status_str:^8}|{r['rpm']:^10.1f}|{r['iq_pre']:^11.2f}|{r['iq_peak']:^12.2f}|{r['iq_post_rms']:^11.2f}|{r['vq']:^9.2f}|{r['vbus']:^10.1f}|{r['time_ms']:^9.1f}")
        else:
            print(f"{r['target']:^14}|{status_str:^8}|{'--':^10}|{'--':^11}|{'--':^12}|{'--':^11}|{'--':^9}|{'--':^10}|{'--':^9}")
    print("=" * 115)

    if all_ok:
        print(">>> 结论: VESC 500~2000 RPM 全速度梯度及正反转平滑接管全场景验证 100% 通过！<<<")
    else:
        print(">>> 结论: 部分工况未通过，需进一步分析优化！<<<")
    return all_ok

if __name__ == "__main__":
    main()
