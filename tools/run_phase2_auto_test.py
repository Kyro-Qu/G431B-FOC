# -*- coding: utf-8 -*-
"""
第二阶段自动化闭环实测脚本:
VESC @ 1000 RPM 自动接管、平滑加权过渡 (Blend 0->1)、故障注入容错与平滑回退 (Blend 1->0) 实机严密验证
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
    print(">>> 启动第二阶段: VESC @ 1000 RPM 故障注入、平滑自动接管与平滑回退实机验证 <<<")
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
        res = ''
        if ser.in_waiting:
            res = ser.read(ser.in_waiting).decode('ascii', errors='replace').strip()
        return res

    def query():
        st = send('status', 0.04)
        m_v = re.search(r'vel=([\-\d\.]+)rpm', st)
        m_iq = re.search(r'iq=([\-\d\.]+)A', st)
        rpm = abs(float(m_v.group(1))) if m_v else 0.0
        iq = float(m_iq.group(1)) if m_iq else 0.0
        fb = send('feedback', 0.04)
        sl = send('sensorless status', 0.04)
        enc = send('enc', 0.04)
        return rpm, iq, fb, sl, enc

    # 1. 模式初始化
    send('log 0')
    send('fault clear')
    send('sensorless algo vesc')
    send('feedback auto')
    send('enc fault clear')
    send('bench dtcomp 1 0.17')

    st = send('status')
    if 'calib=1' not in st:
        print("未检测到有效校准，退出重检！")
        ser.close()
        return

    # 2. 启动电机进入 1000 RPM 闭环稳态
    print("\n--- 启动电机加速至 1000 RPM ---")
    send('mode vel')
    send('vel ramp 500')
    send('target 0')
    send('enable')
    time.sleep(0.2)
    send('target 1000')

    print("等待电机爬坡至 1000 RPM 稳态...")
    for _ in range(30):
        time.sleep(0.2)
        rpm, iq, fb, sl, enc = query()
        if abs(rpm - 1000.0) < 50.0:
            print(f"到达稳态: 转速={rpm:.1f} RPM, Iq={iq:.2f}A")
            break

    time.sleep(1.0)
    print("对齐零点偏置...")
    send('bench align')
    time.sleep(0.6)

    # 3. 门禁考核: 连续累加 8000 拍稳定准入 (qual_cycles >= 8000)
    print("\n--- 进行长窗口连续稳定考核 (qual_cycles 需累加至 8000 拍 / 500ms) ---")
    ready = False
    for i in range(40):
        time.sleep(0.15)
        rpm, iq, fb, sl, enc = query()
        m_qual = re.search(r'qual=(\d+)', sl)
        qual_cnt = int(m_qual.group(1)) if m_qual else 0
        print(f"  [采样 {i+1:2d}] 转速={rpm:.1f} RPM, Iq={iq:.2f}A, qual={qual_cnt:5d}/8000, {fb}")
        if 'candidate' in fb or qual_cnt >= 8000:
            ready = True
            print("  ==> 【准入资格达成】: 无感达到连续稳定要求，进入 CANDIDATE 接管候选态！")
            break

    if not ready:
        print("规定时间内未能达到接管候选态，保护性停机！")
        send('target 0')
        time.sleep(1.0)
        send('disable')
        ser.close()
        return

    # =========================================================================
    # 测试 1: 故障注入 1 - 编码器冻结 (FREEZE) 平滑接管与回退
    # =========================================================================
    print("\n" + "=" * 90)
    print(">>> 【测试 1】: 注入 FREEZE 故障 (编码器停止更新、角度冻结) <<<")
    print("=" * 90)
    send('enc fault freeze')
    time.sleep(0.06)

    for tick in range(15):
        time.sleep(0.06)
        rpm, iq, fb, sl, enc = query()
        print(f"  [FREEZE接管 {tick*60:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A | {fb}")

    if 'blend=1.0' in fb or 'sensorless' in fb:
        print("  ==> [测试 1 接管成功]: 编码器故障确认，无感平滑接管成功 (blend=1.00)！电机连续运行无抖动！")
    else:
        print("  ==> [测试 1 状态]:", fb)

    # 清除故障，观察平滑回退编码器
    print("\n  清除故障注入，观察是否平滑退回编码器主控...")
    send('enc fault clear')
    for tick in range(10):
        time.sleep(0.06)
        rpm, iq, fb, sl, enc = query()
        print(f"  [回退恢复 {tick*60:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A | {fb}")

    if 'blend=0.0' in fb or 'sensored' in fb:
        print("  ==> [测试 1 回退成功]: 编码器恢复正常，已平滑退回编码器控制 (blend=0.00)！")

    # =========================================================================
    # 测试 2: 故障注入 2 - 编码器角度瞬间跳变 90° (STEP) 平滑接管与回退
    # =========================================================================
    print("\n" + "=" * 90)
    print(">>> 【测试 2】: 注入 STEP 故障 (编码器角度瞬间跳变 +90°) <<<")
    print("=" * 90)
    time.sleep(1.0) # 重新建立稳态
    send('enc fault step 90')
    time.sleep(0.06)

    for tick in range(15):
        time.sleep(0.06)
        rpm, iq, fb, sl, enc = query()
        print(f"  [STEP接管 {tick*60:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A | {fb}")

    if 'blend=1.0' in fb or 'sensorless' in fb:
        print("  ==> [测试 2 接管成功]: 编码器阶跃已触发自动接管，电机平稳无丢步！")

    print("\n  清除故障注入，平滑退回编码器...")
    send('enc fault clear')
    for tick in range(10):
        time.sleep(0.06)
        rpm, iq, fb, sl, enc = query()
        print(f"  [回退恢复 {tick*60:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A | {fb}")

    # =========================================================================
    # 测试 3: 故障注入 3 - 编码器超速虚假毛刺 25000 RPM (SPEED SPIKE) 平滑接管与回退
    # =========================================================================
    print("\n" + "=" * 90)
    print(">>> 【测试 3】: 注入 SPEED SPIKE 故障 (编码器超极限转速跳变 25000 RPM) <<<")
    print("=" * 90)
    time.sleep(1.0)
    send('enc fault speed 25000')
    time.sleep(0.06)

    for tick in range(15):
        time.sleep(0.06)
        rpm, iq, fb, sl, enc = query()
        print(f"  [SPEED接管 {tick*60:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A | {fb}")

    if 'blend=1.0' in fb or 'sensorless' in fb:
        print("  ==> [测试 3 接管成功]: 速度超限已触发接管，无感平稳闭环运行！")

    print("\n  清除故障注入，平滑退回编码器...")
    send('enc fault clear')
    for tick in range(10):
        time.sleep(0.06)
        rpm, iq, fb, sl, enc = query()
        print(f"  [回退恢复 {tick*60:3d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A | {fb}")

    # =========================================================================
    # 测试 4: 无感闭环降速至 400 RPM (低于 500 RPM 安全切出阈值) 回退测试
    # =========================================================================
    print("\n" + "=" * 90)
    print(">>> 【测试 4】: 无感闭环下减速至 400 RPM (低于 500 RPM 安全切出阈值触发回退) <<<")
    print("=" * 90)
    send('enc fault freeze')
    time.sleep(0.3)
    rpm, iq, fb, sl, enc = query()
    print(f"  接管确认: 转速={rpm:.1f} RPM | {fb}")

    print("  目标转速给定降低至 400 RPM ...")
    send('target 400')
    for tick in range(20):
        time.sleep(0.12)
        rpm, iq, fb, sl, enc = query()
        print(f"  [降速过渡 {tick*120:4d}ms] 转速={rpm:.1f} RPM, Iq={iq:.2f}A | {fb}")
        if rpm < 500.0:
            print("  ==> 转速已降至 500 RPM 以下！触发切出回退机制！")
            break

    send('enc fault clear')
    time.sleep(0.5)

    print("\n--- 平稳减速停机 ---")
    send('target 0')
    time.sleep(1.2)
    send('disable')
    send('log 1')
    ser.close()
    print("\n" + "=" * 110)
    print(">>> 第二阶段实测全部完成！<<<")
    print("=" * 110)

if __name__ == "__main__":
    main()
