# -*- coding: utf-8 -*-
"""位置模式相对原点语义实机验证脚本
测试流程:
1. 连接 COM44, log 0 关闭遥测
2. 检查 status, 确保电机已校准(若未校准则执行 calib full)
3. 切换 mode pos
4. enable 使能 -> 验证当前位置锁定不动, pos_err 接近 0, 不出现持续运转
5. target 6.283 (正转 1 圈) -> 验证到达指定角度并锁定
6. target 0 (回使能原点) -> 验证回原点并锁定
7. target -3.14159 (反转半圈) -> 验证反向定位
8. disable 停机
"""
import sys
import time
import serial

PORT = "COM44"
BAUD = 6500000

def send_cmd(ser, cmd, delay=0.1):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("ascii"))
    time.sleep(delay)
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < delay + 0.1:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        time.sleep(0.01)
    return buf.decode("ascii", "replace").strip()

def get_status(ser):
    res = send_cmd(ser, "status", delay=0.15)
    return res

def main():
    print(f"Connecting to {PORT}...")
    ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=0.5)
    time.sleep(0.2)
    try:
        # 关闭遥测
        send_cmd(ser, "log 0")
        print("Version:", send_cmd(ser, "version"))

        # 停机确保安全
        send_cmd(ser, "disable")
        send_cmd(ser, "fault clear")

        # 读初始状态
        st = get_status(ser)
        print("\n--- Initial Status ---")
        print(st)

        # 检查是否校准
        if "calib=OK" not in st and "valid=1" not in st:
            print("\nTriggering calibration...")
            send_cmd(ser, "calib full")
            for _ in range(30):
                time.sleep(0.5)
                st = get_status(ser)
                if "CALIB" not in st:
                    break
            print("Post-calib status:", st)

        # 切换到位置模式
        print("\n[Step 1] Set mode pos")
        res = send_cmd(ser, "mode pos")
        print(" ->", res)
        st = get_status(ser)
        print(" ->", st)

        # 使能
        print("\n[Step 2] Enable in pos mode (should lock at current position, target=0)")
        res = send_cmd(ser, "enable")
        print(" ->", res)
        time.sleep(0.5)
        st = get_status(ser)
        print(" ->", st)

        # 观察 1.5 秒确认电机静止不持续转动
        print("\n[Step 3] Checking if stationary (sampling for 1.5s)...")
        for i in range(3):
            time.sleep(0.5)
            st = get_status(ser)
            # 提取 vel 和 pos
            print(f"  T+{0.5*(i+1):.1f}s:", [line for line in st.splitlines() if "M0 state=" in line or "pos=" in line or "vel=" in line])

        # 目标: 正转 1 圈 (6.283 rad)
        print("\n[Step 4] Move target 6.283 rad (+1 turn)...")
        res = send_cmd(ser, "target 6.283")
        print(" ->", res)
        time.sleep(1.5)
        st = get_status(ser)
        print(" -> Post-move status:")
        for line in st.splitlines():
            if any(k in line for k in ["state=", "pos=", "vel=", "iq="]):
                print("   ", line)

        # 目标: 回原点 0 rad
        print("\n[Step 5] Move target 0 rad (back to origin)...")
        res = send_cmd(ser, "target 0")
        print(" ->", res)
        time.sleep(1.5)
        st = get_status(ser)
        print(" -> Post-move status:")
        for line in st.splitlines():
            if any(k in line for k in ["state=", "pos=", "vel=", "iq="]):
                print("   ", line)

        # 目标: 反转半圈 (-3.14159 rad)
        print("\n[Step 6] Move target -3.14159 rad (-0.5 turn)...")
        res = send_cmd(ser, "target -3.14159")
        print(" ->", res)
        time.sleep(1.5)
        st = get_status(ser)
        print(" -> Post-move status:")
        for line in st.splitlines():
            if any(k in line for k in ["state=", "pos=", "vel=", "iq="]):
                print("   ", line)

        # 停机
        print("\n[Step 7] Disable motor")
        send_cmd(ser, "disable")
        print(" -> Disabled safely.")

    finally:
        try:
            send_cmd(ser, "disable")
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
