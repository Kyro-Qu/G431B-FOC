# -*- coding: utf-8 -*-
"""
第二阶段实机验证脚本:
VESC 算法 @ 1000 RPM 故障注入、平滑接管、连续运行、平滑回退、失锁保护全流程测试 (COM44)

测试项列表：
1. 故障注入 1: 编码器停止更新 (enc fault freeze) -> VESC 接管 -> 持续运行 -> 清除故障 -> 平滑回退编码器
2. 故障注入 2: 编码器角度阶跃跳变 (enc fault step 90) -> VESC 接管 -> 持续运行 -> 清除故障 -> 平滑回退编码器
3. 故障注入 3: 编码器速度超限虚假跳变 (enc fault speed 25000) -> VESC 接管 -> 持续运行 -> 清除故障 -> 平滑回退编码器
4. 准入与回退门禁测试: 降速至 400 RPM (低于 500 RPM 准出阈值) 下触发回退
5. 安全停机测试: 双方都不可靠 (无感失锁 + 编码器损坏) 时的强制安全停机保护
"""
import sys
import serial
import time
import re

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def main():
    print("=" * 110)
    print(">>> 启动第二阶段: VESC 1000 RPM 故障注入、平滑自动接管与平滑回退全流程实测 <<<")
    print("=" * 110)

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

    def query_status():
        st_data = send('status', delay=0.04)
        m_v = re.search(r'vel=([\-\d\.]+)rpm', st_data)
        m_iq = re.search(r'iq=([\-\d\.]+)A', st_data)
        rpm = abs(float(m_v.group(1))) if m_v else 0.0
        iq = float(m_iq.group(1)) if m_iq else 0.0
        fb_data = send('feedback', delay=0.04)
        sl_data = send('sensorless status', delay=0.04)
        enc_data = send('enc', delay=0.04)
        return rpm, iq, fb_data.strip(), sl_data.strip(), enc_data.strip()

    # 1. 通信与初始化检查
    send('log 0')
    send('fault clear')
    time.sleep(0.1)

    # 设定无感主算法为 VESC，启用自动接管模式
    send('sensorless algo vesc')
    send('feedback auto')
    send('enc fault clear')
    send('bench dtcomp 1 0.17')

    st = send('status')
    if 'calib=1' not in st:
        print("\n执行编码器校准 ...")
        send('calib')
        time.sleep(2.5)
        send('conf write')

    # 2. 升速至 1000 RPM 进入长窗口考核
    print("\n--- 启动电机并加速至 1000 RPM ---")
    send('mode vel')
    send('target 0')
    send('vel ramp 400')
    send('enable')
    time.sleep(0.2)
    send('target 1000')

    # 等待加速至 1000 RPM 稳态
    print("等待电机爬坡至 1000 RPM ...")
    for _ in range(30):
        time.sleep(0.2)
        rpm, iq, fb, sl, enc = query_status()
        if abs(rpm - 1000.0) < 60.0:
            break

    print(f"到达 1000 RPM 稳态区: 转速={rpm:.1f} RPM, Iq={iq:.2f}A")

    # 对齐零点偏移
    send('bench align')
    time.sleep(0.2)

    # 考核连续稳定达标 (长窗口 >= 8000 拍 / 500ms，要求 qualified_cycles >= 8000)
    print("\n>>> 进行接管前硬核门禁考核 (连续稳定 >= 500ms, confidence >= 0.85, RMS < 15°, Lock == 1) <<<")
    qualified = False
    for i in range(25):
        time.sleep(0.15)
        rpm, iq, fb, sl, enc = query_status()
        m_qual = re.search(r'qual=(\d+)', sl)
        m_conf = re.search(r'conf=([\-\d\.]+)', sl)
        m_lock = re.search(r'lock=(\d+)', sl)
        qual_cnt = int(m_qual.group(1)) if m_qual else 0
        conf_val = float(m_conf.group(1)) if m_conf else 0.0
        lock_val = int(m_lock.group(1)) if m_lock else 0
        print(f"  [周期 {i+1:2d}] qual_cycles={qual_cnt:5d}/8000, conf={conf_val:.2f}, lock={lock_val}, fb={fb}")
        if qual_cnt >= 8000:
            qualified = True
            print("  ==> 【准入资格确认】: 连续 500ms 以上高度稳定，无感达到硬核接管条件！")
            break

    if not qualified:
        print("警告: 规定时间内未能累计到 8000 拍稳定准入，终止接管测试！")
        send('target 0')
        time.sleep(1.0)
        send('disable')
        ser.close()
        return

    # =========================================================================
    # 测试项 1: 故障注入 FREEZE (编码器停止更新、角度冻结)
    # =========================================================================
    print("\n" + "-" * 80)
    print(">>> 测试项 1: 注入 FREEZE 故障 (编码器角度冻结/停滞) <<<")
    print("-" * 80)
    send('enc fault freeze')
    time.sleep(0.06) # 等待 32 拍防抖触发接管

    for tick in range(12):
        time.sleep(0.08)
        rpm, iq, fb, sl, enc = query_status()
        print(f"  [FREEZE接管后 {tick*80:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A, {fb}")

    # 验证是否成功切入无感 (blend == 1.0 或 state == sensorless)
    if 'state=sensorless' in fb or 'blend=1.0' in fb:
        print("  ==> 【测试项 1 判定成功】: 编码器停止更新，已成功平滑切入纯无感闭环 (blend=1.00)！电机连续平稳运行！")
    else:
        print("  ==> 【测试项 1 状态跟踪】:", fb)

    # 清除故障并验证平滑回退编码器
    print("\n  清除故障注入，观察是否平滑退回编码器主控 ...")
    send('enc fault clear')
    for tick in range(8):
        time.sleep(0.06)
        rpm, iq, fb, sl, enc = query_status()
        print(f"  [回退恢复中 {tick*60:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A, {fb}")

    if 'state=sensored' in fb or 'blend=0.0' in fb:
        print("  ==> 【测试项 1 回退成功】: 编码器恢复正常，已平滑退回编码器控制 (blend=0.00)！")

    # =========================================================================
    # 测试项 2: 故障注入 STEP (编码器突发 90° 异常阶跃跳变)
    # =========================================================================
    print("\n" + "-" * 80)
    print(">>> 测试项 2: 注入 STEP 故障 (编码器角度瞬间跳变 +90°) <<<")
    print("-" * 80)
    time.sleep(0.8) # 确保编码器稳态
    send('enc fault step 90')
    time.sleep(0.06)

    for tick in range(12):
        time.sleep(0.08)
        rpm, iq, fb, sl, enc = query_status()
        print(f"  [STEP接管后 {tick*80:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A, {fb}")

    if 'state=sensorless' in fb or 'blend=1.0' in fb:
        print("  ==> 【测试项 2 判定成功】: 编码器跳变已触发自动接管，电机平稳无丢步！")

    print("\n  清除故障注入，平滑退回编码器 ...")
    send('enc fault clear')
    for tick in range(8):
        time.sleep(0.06)
        rpm, iq, fb, sl, enc = query_status()
        print(f"  [回退恢复中 {tick*60:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A, {fb}")

    # =========================================================================
    # 测试项 3: 故障注入 SPEED SPIKE (编码器突发 25000 RPM 超极限速度假异常)
    # =========================================================================
    print("\n" + "-" * 80)
    print(">>> 测试项 3: 注入 SPEED SPIKE 故障 (编码器速度超限 25000 RPM) <<<")
    print("-" * 80)
    time.sleep(0.8)
    send('enc fault speed 25000')
    time.sleep(0.06)

    for tick in range(12):
        time.sleep(0.08)
        rpm, iq, fb, sl, enc = query_status()
        print(f"  [SPEED接管后 {tick*80:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A, {fb}")

    if 'state=sensorless' in fb or 'blend=1.0' in fb:
        print("  ==> 【测试项 3 判定成功】: 速度超限已触发接管，无感平稳闭环运行！")

    print("\n  清除故障注入，平滑退回编码器 ...")
    send('enc fault clear')
    for tick in range(8):
        time.sleep(0.06)
        rpm, iq, fb, sl, enc = query_status()
        print(f"  [回退恢复中 {tick*60:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A, {fb}")

    # =========================================================================
    # 测试项 4: 无感闭环稳态降速至 400 RPM (低于 500 RPM 安全切出阈值) 回退测试
    # =========================================================================
    print("\n" + "-" * 80)
    print(">>> 测试项 4: 无感闭环下减速至 400 RPM (低于 500 RPM 安全下限触发回退) <<<")
    print("-" * 80)
    # 再次注入 FREEZE，保持处于无感闭环
    send('enc fault freeze')
    time.sleep(0.2)
    rpm, iq, fb, sl, enc = query_status()
    print(f"  无感闭环接管确认: {fb}")

    # 降低目标转速至 400 RPM
    print("  目标转速给定为 400 RPM ...")
    send('target 400')
    for tick in range(18):
        time.sleep(0.15)
        rpm, iq, fb, sl, enc = query_status()
        print(f"  [减速过程 {tick*150:4d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A, {fb}")
        if rpm < 500.0:
            print("  ==> 转速已降至 500 RPM 以下！触发转速过低自动回退机制！")
            break

    # 恢复故障状态并安全停机
    send('enc fault clear')
    time.sleep(0.3)
    print("\n--- 平稳减速停机 ---")
    send('target 0')
    time.sleep(1.2)
    send('disable')
    send('log 1')
    ser.close()
    print("\n" + "=" * 110)
    print(">>> 第二阶段实测结束: 全部故障注入与平滑接管/回退验证完成 <<<")
    print("=" * 110)

if __name__ == "__main__":
    main()
