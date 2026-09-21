# -*- coding: utf-8 -*-
"""
VLink CDC 锁死故障复现脚本 (reproduce_cdc_lockup.py)

【复现机理】
在 6.5Mbps 超高波特率下，MCU 持续以高速发送遥测或状态帧。
上位机在突发大吞吐量传输期间频繁开关串口，或者在并发运行 SWD 高频轮询时，
VLink DAPLink 内部的 CDC IN 端点 FIFO 或 UART 接收器会因 Overrun Error (ORE)
或 USB IN Token 饥饿而陷入单向死锁（接收 0 字节，但下行依然能发）。

【运行模式】
1. 监控模式 (monitor): 检测当前 COM44 是否处于 0 字节死锁状态
2. 压力复现模式 (stress): 在恢复正常后，通过高频下发命令与并发读取，模拟触发该死锁
"""
import sys
import time
import serial
from pyocd.core.helpers import ConnectHelper

PORT = "COM44"
BAUD = 6500000

def check_current_deadlock():
    print("==================================================")
    print("      VLink CDC 串口单向死锁状态现场检测           ")
    print("==================================================")

    # 1. 检查 PC -> MCU 下行
    print("\n[测试 1] 验证 PC -> MCU 下行链路...")
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    t = session.target
    if t.is_halted(): t.resume()

    head_before = t.read16(0x2000008c)
    session.close()

    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(0.05)
    test_payload = b"version\n"
    ser.write(test_payload)
    time.sleep(0.05)

    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    t = session.target
    head_after = t.read16(0x2000008c)
    downlink_ok = (head_after != head_before)
    print(f"  下行结果: head_before={head_before}, head_after={head_after} -> {'[下行畅通]' if downlink_ok else '[下行失败]'}")

    # 2. 检查 MCU -> PC 上行
    print("\n[测试 2] 验证 MCU -> PC 上行链路...")
    t0 = time.time()
    rx_bytes = bytearray()
    while time.time() - t0 < 0.5:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            rx_bytes.extend(chunk)
            break
    ser.close()

    uplink_ok = len(rx_bytes) > 0
    print(f"  上行结果: 收到 {len(rx_bytes)} 字节 -> {'[上行畅通]' if uplink_ok else '[上行死锁 (0 字节)]'}")

    # 3. 诊断定性
    print("\n==================================================")
    if downlink_ok and not uplink_ok:
        print("  【确诊】: 当前 COM44 正处于典型的 VLink CDC 单向死锁状态！")
        print("           下行畅通（MCU 可收），上行彻底断绝（PC 读 0 字节）。")
    elif not downlink_ok and not uplink_ok:
        print("  【确诊】: 双向彻底断开（可能未上电或 USB 断开）。")
    elif downlink_ok and uplink_ok:
        print("  【正常】: 当前通信正常，未处于死锁状态。")
    print("==================================================")
    session.close()

def stress_test_trigger():
    print("\n开始执行压力激发测试 (在串口正常后用于诱发死锁)...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"打开 {PORT} 失败: {e}")
        return

    print("高频迸发命令并伴随 SWD 随机读取，测试极限吞吐耐受...")
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    t = session.target

    deadlock_detected = False
    for round_idx in range(50):
        # 并发 SWD 读
        _ = t.read32(0x20000018)
        # 串口写入
        ser.write(b"help\nstatus\nversion\n")
        time.sleep(0.01)
        rx = ser.read(ser.in_waiting)
        if round_idx > 5 and len(rx) == 0:
            # 连续 3 次无数据判定为锁死
            empty_count = 1
            for _ in range(3):
                ser.write(b"version\n")
                time.sleep(0.05)
                if len(ser.read(ser.in_waiting)) == 0:
                    empty_count += 1
            if empty_count >= 3:
                print(f"在第 {round_idx} 轮成功复现死锁：上行停止返回数据！")
                deadlock_detected = True
                break
        print(f"Round {round_idx}: rx={len(rx)} bytes")

    ser.close()
    session.close()
    if not deadlock_detected:
        print("未在当前循环内触发死锁。")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "stress":
        stress_test_trigger()
    else:
        check_current_deadlock()
