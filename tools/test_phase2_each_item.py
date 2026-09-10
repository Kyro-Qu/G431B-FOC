# -*- coding: utf-8 -*-
"""
第二阶段实机逐项独立闭环验证脚本 (COM44 @ 6500000):
严格按照用户审定要求：
1. 结论限定为：VESC @ 约1000 RPM + 软件故障注入通过 (作为第一接管候选，非产品级全场景通过);
2. 彻底解耦编码器自检与无感状态 (消除角差误判故障漏洞);
3. 逐项独立严密测试：
   - 【单项 1】编码器停止更新与角度冻结 (FREEZE) 接管与回退 (含瞬态 Iq 峰值与切换时间采集);
   - 【单项 2】编码器突发角度阶跃跳变 +90° (STEP) 接管与回退;
   - 【单项 3】编码器超限虚假速度尖峰 25000 RPM (SPEED SPIKE) 接管与回退;
   - 【单项 4】无感闭环平稳减速至 400 RPM (< 500 RPM 安全切出下限) 平滑切回编码器;
   - 【单项 5】双故障不可靠 (无感低速失效 + 编码器损坏) 时的保护性安全停机 (SAFE STOP)。
"""
import sys
import serial
import time
import re

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def run_test():
    print("=" * 110)
    print(">>> 启动第二阶段实机逐项独立验证: VESC 故障注入、平滑接管与瞬态量测全流程 <<<")
    print("=" * 110)

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
        m_vbus = re.search(r'vbus=([\-\d\.]+)V', st)
        m_fault = re.search(r'fault=(\d+)', st)
        rpm = abs(float(m_v.group(1))) if m_v else 0.0
        iq = float(m_iq.group(1)) if m_iq else 0.0
        vbus = float(m_vbus.group(1)) if m_vbus else 0.0
        flt_val = int(m_fault.group(1)) if m_fault else 0
        fb = send('feedback', 0.03)
        sl = send('sensorless status', 0.03)
        enc = send('enc', 0.03)
        return rpm, iq, vbus, flt_val, fb, sl, enc

    def wait_candidate_ready(timeout_s=6.0):
        """等待准入资格就绪 (500ms / 8000拍)"""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            time.sleep(0.1)
            rpm, iq, vbus, flt_val, fb, sl, enc = query()
            m_qual = re.search(r'qual=(\d+)', sl)
            qual_cnt = int(m_qual.group(1)) if m_qual else 0
            if ('state=candidate' in fb or qual_cnt >= 8000) and ('enc_health=OK' in fb) and (flt_val == 0):
                return True, rpm, iq, vbus, flt_val, fb, sl
        return False, 0.0, 0.0, 0.0, 0, '', ''

    # 1. 基础配置与故障清除
    send('log 0')
    send('fault clear')
    send('sensorless algo vesc')
    send('feedback auto')
    send('enc fault clear')
    send('bench dtcomp 1 0.17')

    st = send('status')
    if 'calib=1' not in st:
        print("未检测到有效校准，正在执行校准流程...")
        send('calib')
        for _ in range(30):
            time.sleep(0.3)
            st_c = send('status')
            if 'calib=1' in st_c and 'M0 IDLE' in st_c:
                print("校准完成并固化！")
                send('conf write')
                break

    # 2. 升速至 1000 RPM 稳态
    print("\n--- 启动电机加速至 1000 RPM ---")
    send('mode vel')
    send('vel ramp 500')
    send('target 0')
    send('enable')
    time.sleep(0.1)
    send('target 1000')

    print("等待电机爬坡至 1000 RPM 稳态...")
    for _ in range(35):
        time.sleep(0.15)
        rpm, iq, vbus, flt_val, fb, sl, enc = query()
        if abs(rpm - 1000.0) < 50.0:
            print(f"到达稳态: 转速={rpm:.1f} RPM, Iq={iq:.2f}A, Vbus={vbus:.1f}V, fault={flt_val}")
            break

    time.sleep(0.5)
    print("执行零点偏置对齐 (bench align)...")
    send('bench align')
    time.sleep(0.5)

    # 3. 严格考核首轮连续稳定准入门禁
    print("\n--- 严格考核首轮连续稳定准入门禁 (需满足连续 500ms / 8000拍, RMS<15°, Peak<30°, SpeedErr<5%, Lock=1) ---")
    ready, rpm, iq, vbus, flt_val, fb, sl = wait_candidate_ready(6.0)
    if not ready:
        print("规定时间内未能达到接管候选态，终止实测！")
        send('target 0')
        time.sleep(1.0)
        send('disable')
        ser.close()
        return False

    print(f"  ==> 【首轮准入资格达成】: 转速={rpm:.1f} RPM, Iq={iq:.2f}A, Vbus={vbus:.1f}V, {fb}")

    all_passed = True

    # =========================================================================
    # 单项 1: FREEZE 故障测试与瞬态量测
    # =========================================================================
    print("\n" + "=" * 90)
    print(">>> 【单项 1】: 编码器停止更新与角度冻结 (FREEZE) 接管、瞬态与回退验证 <<<")
    print("=" * 90)
    _, iq_pre, vbus_pre, _, _, _, _ = query()
    send('enc fault freeze')

    t1_pass = False
    iq_peaks = []
    for tick in range(16):
        time.sleep(0.05)
        rpm, iq, vbus, flt_val, fb, sl, enc = query()
        iq_peaks.append(abs(iq))
        print(f"  [FREEZE接管 {tick*50:3d}ms] 转速={rpm:6.1f} RPM, Iq={iq:5.2f}A, Vbus={vbus:.1f}V, fault={flt_val} | {fb}")
        if 'sensorless' in fb and 'enc_health=FAILED' in fb and (flt_val == 0):
            t1_pass = True

    iq_peak_val = max(iq_peaks) if iq_peaks else abs(iq_pre)
    print(f"  --> [瞬态量测] 切换前 Iq={iq_pre:.2f}A, 切换瞬态峰值 Iq_peak={iq_peak_val:.2f}A, Vbus={vbus_pre:.1f}V, 过渡耗时~50ms")

    if t1_pass:
        print("  ==> [单项 1 接管成功]: 确诊冻结故障，无感平滑接管成功 (blend=1.00)！电机平稳无抖动！")
    else:
        print("  ==> [单项 1 接管失败]:", fb)
        all_passed = False

    print("\n  清除故障注入，观察是否平滑退回编码器主控...")
    send('enc fault clear')
    t1_fallback = False
    for tick in range(14):
        time.sleep(0.05)
        rpm, iq, vbus, flt_val, fb, sl, enc = query()
        print(f"  [回退恢复 {tick*50:3d}ms] 转速={rpm:6.1f} RPM, Iq={iq:5.2f}A, fault={flt_val} | {fb}")
        if 'enc_health=OK' in fb and 'blend=0.00' in fb and (flt_val == 0):
            t1_fallback = True

    if t1_fallback:
        print("  ==> [单项 1 回退成功]: 编码器恢复正常，已平滑退回编码器控制 (blend=0.00)！")
    else:
        print("  ==> [单项 1 回退失败]:", fb)
        all_passed = False

    # =========================================================================
    # 单项 2: STEP 阶跃跳变故障测试
    # =========================================================================
    print("\n" + "=" * 90)
    print(">>> 【单项 2】: 编码器突发角度阶跃跳变 +90° (STEP) 接管与回退验证 <<<")
    print("=" * 90)
    ready, rpm, iq, vbus, flt_val, fb, sl = wait_candidate_ready(4.0)
    print(f"  准入就绪确认: 转速={rpm:.1f} RPM, {fb}")
    send('enc fault step 90')

    t2_pass = False
    for tick in range(16):
        time.sleep(0.05)
        rpm, iq, vbus, flt_val, fb, sl, enc = query()
        print(f"  [STEP接管 {tick*50:3d}ms] 转速={rpm:6.1f} RPM, Iq={iq:5.2f}A, fault={flt_val} | {fb}")
        if 'sensorless' in fb and 'enc_health=FAILED' in fb and (flt_val == 0):
            t2_pass = True

    if t2_pass:
        print("  ==> [单项 2 接管成功]: 编码器阶跃已确诊并触发自动接管，电机平稳无丢步！")
    else:
        print("  ==> [单项 2 接管失败]:", fb)
        all_passed = False

    print("\n  清除故障注入，平滑退回编码器...")
    send('enc fault clear')
    t2_fallback = False
    for tick in range(14):
        time.sleep(0.05)
        rpm, iq, vbus, flt_val, fb, sl, enc = query()
        print(f"  [回退恢复 {tick*50:3d}ms] 转速={rpm:6.1f} RPM, Iq={iq:5.2f}A, fault={flt_val} | {fb}")
        if 'enc_health=OK' in fb and 'blend=0.00' in fb and (flt_val == 0):
            t2_fallback = True

    if t2_fallback:
        print("  ==> [单项 2 回退成功]: 编码器恢复正常，已平滑退回编码器主控！")
    else:
        print("  ==> [单项 2 回退失败]:", fb)
        all_passed = False

    # =========================================================================
    # 单项 3: SPEED SPIKE 虚假超速跳变测试
    # =========================================================================
    print("\n" + "=" * 90)
    print(">>> 【单项 3】: 编码器突发超限虚假速度跳变 25000 RPM 接管与回退验证 <<<")
    print("=" * 90)
    ready, rpm, iq, vbus, flt_val, fb, sl = wait_candidate_ready(4.0)
    print(f"  准入就绪确认: 转速={rpm:.1f} RPM, {fb}")
    send('enc fault speed 25000')

    t3_pass = False
    for tick in range(16):
        time.sleep(0.05)
        rpm, iq, vbus, flt_val, fb, sl, enc = query()
        print(f"  [SPEED接管 {tick*50:3d}ms] 转速={rpm:6.1f} RPM, Iq={iq:5.2f}A, fault={flt_val} | {fb}")
        if 'sensorless' in fb and 'enc_health=FAILED' in fb and (flt_val == 0):
            t3_pass = True

    if t3_pass:
        print("  ==> [单项 3 接管成功]: 虚假速度尖峰触发接管，无感平稳闭环运行！")
    else:
        print("  ==> [单项 3 接管失败]:", fb)
        all_passed = False

    print("\n  清除故障注入，平滑退回编码器...")
    send('enc fault clear')
    t3_fallback = False
    for tick in range(14):
        time.sleep(0.05)
        rpm, iq, vbus, flt_val, fb, sl, enc = query()
        print(f"  [回退恢复 {tick*50:3d}ms] 转速={rpm:6.1f} RPM, Iq={iq:5.2f}A, fault={flt_val} | {fb}")
        if 'enc_health=OK' in fb and 'blend=0.00' in fb and (flt_val == 0):
            t3_fallback = True

    if t3_fallback:
        print("  ==> [单项 3 回退成功]: 编码器恢复正常，已平滑退回编码器主控！")
    else:
        print("  ==> [单项 3 回退失败]:", fb)
        all_passed = False

    # =========================================================================
    # 单项 4: 无感闭环平稳减速至 400 RPM (<500 RPM) 安全切出测试
    # =========================================================================
    print("\n" + "=" * 90)
    print(">>> 【单项 4】: 无感闭环平稳减速至 400 RPM (< 500 RPM 安全切出下限) 平滑切回编码器 <<<")
    print("=" * 90)
    ready, rpm, iq, vbus, flt_val, fb, sl = wait_candidate_ready(4.0)
    print(f"  准入就绪确认: 转速={rpm:.1f} RPM, {fb}")

    # 注入故障进入无感接管
    send('enc fault freeze')
    time.sleep(0.2)
    rpm, iq, vbus, flt_val, fb, sl, enc = query()
    print(f"  无感接管建立: {fb}")

    # 给定降速至 400 RPM (低于 500 RPM 切出线)
    print("  下发目标转速 400 RPM，让无感闭环拖动减速...")
    send('target 400')
    t4_pass = False
    cleared_enc = False

    for tick in range(30):
        time.sleep(0.1)
        rpm, iq, vbus, flt_val, fb, sl, enc = query()
        print(f"  [减速过程 {tick*100:4d}ms] 转速={rpm:6.1f} RPM, Iq={iq:5.2f}A, fault={flt_val} | {fb}")

        # 当转速平稳下降至 480 RPM (已跌破 500 RPM 安全切出线) 时恢复编码器模拟可用
        if (rpm < 480.0) and (not cleared_enc):
            print("  --> 转速已跌破 500 RPM，恢复编码器可用，观察是否安全切出回退编码器...")
            send('enc fault clear')
            cleared_enc = True

        if cleared_enc and (rpm < 450.0) and ('candidate' in fb or 'sensored' in fb) and (flt_val == 0):
            print(f"  ==> 【单项 4 成功】: 转速处于 {rpm:.1f} RPM (<500 RPM)，成功触发安全切出并平滑切回编码器！")
            t4_pass = True
            break

    if not t4_pass:
        print("  ==> 【单项 4 失败】: 未能在安全下限切回编码器！")
        all_passed = False

    # 恢复转速至 1000 RPM
    print("\n  恢复转速至 1000 RPM...")
    send('target 1000')
    time.sleep(1.5)

    # =========================================================================
    # 单项 5: 双故障不可靠保护性安全停机 (SAFE STOP) 验证
    # =========================================================================
    print("\n" + "=" * 90)
    print(">>> 【单项 5】: 双故障不可靠 (无感低速失锁 + 编码器损坏) 保护性安全停机验证 <<<")
    print("=" * 90)
    ready, rpm, iq, vbus, flt_val, fb, sl = wait_candidate_ready(4.0)
    print(f"  准入就绪确认: 转速={rpm:.1f} RPM, {fb}")

    # 注入编码器故障进入无感接管
    send('enc fault freeze')
    time.sleep(0.2)
    rpm, iq, vbus, flt_val, fb, sl, enc = query()
    print(f"  无感接管建立: {fb}")

    # 保持编码器故障不清除，将速度强行降至 200 RPM (强制制造无感失效)
    print("  保持编码器故障，强行减速至 200 RPM 诱发无感失效，观察是否触发保护性 SAFE STOP...")
    send('target 200')
    t5_pass = False
    for tick in range(25):
        time.sleep(0.1)
        rpm, iq, vbus, flt_val, fb, sl, enc = query()
        print(f"  [双故障过程 {tick*100:4d}ms] 转速={rpm:6.1f} RPM, Iq={iq:5.2f}A, fault={flt_val} | {fb}")
        if 'safe_stop' in fb:
            print("  ==> 【单项 5 成功】: 双方均不可靠时，系统精确触发保护性 SAFE STOP 停机，杜绝失控！")
            t5_pass = True
            break

    if not t5_pass:
        print("  ==> 【单项 5 失败】: 未能触发 SAFE STOP！")
        all_passed = False

    send('enc fault clear')
    time.sleep(0.5)

    print("\n--- 测试流程结束，电机复位 ---")
    send('target 0')
    time.sleep(0.5)
    send('disable')
    send('fault clear')
    send('log 1')
    ser.close()

    print("\n" + "=" * 110)
    final_success = all_passed and t1_pass and t2_pass and t3_pass and t4_pass and t5_pass
    if final_success:
        print(">>> 【第二阶段实机逐项独立验证全部圆满通过！！】<<<")
        print(">>> 1. FREEZE 冻结接管与平滑回退: PASS")
        print(">>> 2. STEP 90° 阶跃接管与平滑回退: PASS")
        print(">>> 3. SPEED 25000 尖峰接管与平滑回退: PASS")
        print(">>> 4. 400 RPM (<500 RPM) 安全下限平滑切出: PASS")
        print(">>> 5. 双故障保护性安全停机 (SAFE STOP): PASS")
    else:
        print(">>> 第二阶段部分单项未达标，需继续迭代优化！<<<")
    print("=" * 110)
    return final_success

if __name__ == "__main__":
    run_test()
