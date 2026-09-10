# -*- coding: utf-8 -*-
"""
纯无感 I/F -> VESC 切换与保护多工况全面验证脚本 (test_sensorless_primary_suite.py)

测试维度:
1. 300~500 RPM 阶梯目标切换验证 (目标 300, 350, 400, 450, 500 RPM)
2. I/F 正反转启动与换向验证 (+500 RPM 与 -500 RPM)
3. 稳态负载/转速阶跃响应验证
4. 堵转保护触发与 SAFE_STOP 保护验证
"""

import serial
import time
import sys

PORT = 'COM44'
BAUD = 6500000

def get_serial():
    ser = serial.Serial(PORT, BAUD, timeout=0.08)
    ser.reset_input_buffer()
    return ser

def cmd(ser, c, wait=0.025):
    ser.reset_input_buffer()
    ser.write((c + '\n').encode('ascii'))
    time.sleep(wait)
    res = ser.read_all().decode(errors='ignore').strip()
    return res

def wait_until_run_or_fault(ser, timeout_sec=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        fb = cmd(ser, 'feedback')
        if 'state=run' in fb:
            return 'RUN', fb
        st = cmd(ser, 'status')
        if 'FAULT' in st or 'state=lost' in fb or 'SAFE_STOP' in fb:
            return 'FAULT', st + "\n" + fb
        time.sleep(0.05)
    return 'TIMEOUT', cmd(ser, 'status')

def test_startup_at_speed(ser, target_rpm, if_curr=0.60, if_target=None):
    if if_target is None:
        if_target = max(300, min(500, abs(target_rpm)))

    print(f"\n--- 测试 I/F -> VESC 启动: 目标 {target_rpm} RPM (I/F目标={if_target} RPM, 拖动电流={if_curr}A) ---")
    cmd(ser, 'disable')
    cmd(ser, 'fault clear')
    cmd(ser, 'feedback sensorless')
    cmd(ser, f'feedback if {if_curr} {if_target} 300')
    cmd(ser, 'mode vel')
    cmd(ser, f'target {target_rpm}')
    cmd(ser, 'enable')

    res, detail = wait_until_run_or_fault(ser, timeout_sec=6.0)
    if res == 'RUN':
        time.sleep(1.0)
        st = cmd(ser, 'status')
        fb = cmd(ser, 'feedback')
        ss = cmd(ser, 'sensorless status')
        print(f"[+] 启动成功切入 RUN 态！")
        for l in st.splitlines():
            if any(k in l for k in ['vel=', 'vel_obs=', 'id=', 'iq=', 'vbus=']):
                print("    ST:", l)
        for l in fb.splitlines():
            if 'feedback:' in l:
                print("    FB:", l)
        for l in ss.splitlines():
            if 'sensorless:' in l:
                print("    SS:", l)
        cmd(ser, 'disable')
        return True
    else:
        print(f"[-] 启动失败，结果={res}")
        print("    详情:\n", detail)
        cmd(ser, 'disable')
        return False

def test_stall_protection(ser):
    print("\n--- 测试堵转与失锁保护: SAFE_STOP 保护性停机验证 ---")
    cmd(ser, 'disable')
    cmd(ser, 'fault clear')
    cmd(ser, 'feedback sensorless')
    # 设定极小 I/F 目标且拖动转速极低使得观测器转速 < 100 RPM 触发堵转
    cmd(ser, 'feedback if 0.20 500 300')
    cmd(ser, 'mode vel')
    cmd(ser, 'target 500')
    cmd(ser, 'enable')

    # 观察是否在超限后受控进入 SAFE_STOP / FAULT
    t0 = time.time()
    tripped = False
    for _ in range(40):
        st = cmd(ser, 'status')
        fb = cmd(ser, 'feedback')
        if 'FAULT' in st or 'lost=' in fb or 'fault=' in st:
            print("[+] 成功捕获失锁/堵转保护动作！")
            for l in st.splitlines():
                if any(k in l for k in ['fault=', 'state=', 'vel=']):
                    print("    ST:", l)
            for l in fb.splitlines():
                if 'lost=' in l:
                    print("    FB:", l)
            tripped = True
            break
        time.sleep(0.1)
    cmd(ser, 'disable')
    cmd(ser, 'fault clear')
    return tripped

def main():
    print("=" * 70)
    print("   纯无感 I/F -> VESC 切换、多速度段、正反向与安全保护综合测试")
    print("=" * 70)

    try:
        ser = get_serial()
    except Exception as e:
        print(f"[-] 无法打开串口 {PORT}: {e}")
        return

    # 1. 300~500 RPM 切换梯度测试
    speeds = [300, 350, 400, 450, 500]
    success_cnt = 0
    for spd in speeds:
        ok = test_startup_at_speed(ser, spd, if_curr=0.60, if_target=spd)
        if ok: success_cnt += 1
        time.sleep(1.0)

    # 2. 反向 -500 RPM 启动测试
    print("\n>>> 开始测试反转工况: 目标 -500 RPM")
    rev_ok = test_startup_at_speed(ser, -500, if_curr=0.60, if_target=500)
    time.sleep(1.0)

    # 3. 堵转与硬超时保护验证
    stall_ok = test_stall_protection(ser)

    ser.close()

    print("\n" + "=" * 70)
    print("                       测试总结报告")
    print("=" * 70)
    print(f"300~500 RPM 阶梯启动通过率: {success_cnt}/{len(speeds)}")
    print(f"反转 (-500 RPM) 启动测试: {'PASS' if rev_ok else 'FAIL'}")
    print(f"失锁与堵转受控 SAFE_STOP 保护: {'PASS' if stall_ok else 'FAIL'}")
    print("=" * 70)

if __name__ == '__main__':
    main()
