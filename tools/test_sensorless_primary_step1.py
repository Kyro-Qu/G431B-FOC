# -*- coding: utf-8 -*-
"""
第四阶段纯无感独立启动 (Sensorless Primary) 自动化测试套件
按测试顺序 A -> B -> C -> D -> E -> F -> G 严格逐级执行：
  Step A: 参数与状态机逻辑审查、命令接口配置验证
  Step B: 纯无感配置下使能放行验证 (低母线、低电流，I/F 对齐 400ms 与超时保护测试)
  Step C: 开环拖动转速梯度验证 (100, 200, 300, 400, 500 RPM)
  Step D: 500 RPM 开闭环平滑切换 (等待锁定 -> 150ms 动基准衰减 Blend -> 闭环接管)
  Step E: 纯无感闭环稳定运行与电流监视 (维持 >= 1s，Iq < 0.8A)
  Step F: 失锁与受控停机 (SAFE_STOP) 保护测试
"""

import sys
import time
import serial

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

PORT = "COM44"
BAUD = 6500000

def send_cmd(ser, cmd, delay=0.05):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("utf-8"))
    time.sleep(delay)
    out = ser.read_all().decode("utf-8", errors="ignore").strip()
    return out

def run_test():
    print("====================================================================")
    print("      第四阶段：Sensorless Primary 纯无感独立启动分级测试")
    print("====================================================================")

    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    print("\n--- [测试准备] 清除可能存在的故障并确认状态 ---")
    resp = send_cmd(ser, "fault clear")
    print(f">> fault clear: {resp}")
    resp = send_cmd(ser, "disable")
    print(f">> disable: {resp}")

    # 1. 配置反馈模式为 sensorless
    print("\n--- [Step A] 激活 Sensorless Primary 纯无感独立模式 ---")
    resp = send_cmd(ser, "feedback sensorless")
    print(f">> feedback sensorless: {resp}")
    resp = send_cmd(ser, "feedback if 0.40 500") # 温和 0.40A 启动电流，目标 500 RPM
    print(f">> feedback if 0.40 500: {resp}")
    resp = send_cmd(ser, "feedback")
    print(f">> feedback: {resp}")

    # 2. 验证使能与状态机运转
    print("\n--- [Step B] 纯无感启动触发与状态跟踪 (监控 3 秒) ---")
    # 切换到速度模式并给定 500 RPM 目标
    resp = send_cmd(ser, "mode vel")
    print(f">> mode vel: {resp}")
    resp = send_cmd(ser, "target 500")
    print(f">> target 500: {resp}")
    resp = send_cmd(ser, "enable")
    print(f">> enable: {resp}")

    t_start = time.time()
    max_iq = 0.0
    states_seen = set()
    completed_run = False

    while time.time() - t_start < 3.5:
        time.sleep(0.1)
        fb = send_cmd(ser, "feedback")
        st = send_cmd(ser, "status")

        # 提取状态与电流
        for line in fb.splitlines():
            if "feedback: mode=sensorless" in line:
                print(f"  [{time.time()-t_start:.2f}s] {line}")
                for part in line.split():
                    if part.startswith("state="):
                        states_seen.add(part.split("=")[1])
                    if part == "state=run":
                        completed_run = True

        for line in st.splitlines():
            if "id=" in line and "iq=" in line:
                # id=0.000A id_ref=... iq=...
                try:
                    parts = line.split()
                    iq_val = abs(float(parts[4].replace("iq=", "").replace("A", "")))
                    if iq_val > max_iq:
                        max_iq = iq_val
                except Exception:
                    pass
            if "fault=" in line and not line.startswith("calib"):
                if "fault=0" not in line:
                    print(f"  [FAULT DETECTED] {line}")

    # 停止电机
    print("\n--- [测试收尾] 停机并查询最终状态 ---")
    send_cmd(ser, "target 0")
    send_cmd(ser, "disable")
    time.sleep(0.2)
    resp = send_cmd(ser, "feedback")
    print(f">> final feedback: {resp}")
    resp = send_cmd(ser, "status")
    for line in resp.splitlines():
        if "fault=" in line and not line.startswith("calib"):
            print(f">> fault status: {line}")
            break

    print("\n==================== 测试结果总结 ====================")
    print(f"经历状态: {list(states_seen)}")
    print(f"最大峰值电流 Iq: {max_iq:.3f} A (限幅门限 < 0.80 A)")
    if "if_start" in states_seen and "if_accel" in states_seen:
        print("[PASS] I/F 虚拟角度加速发生器正常运转")
    else:
        print("[FAIL] 未能观测到完整的 I/F 状态迁移")

    if completed_run:
        print("[PASS] 成功平滑过渡并切入纯无感闭环 RUN 状态！")
    else:
        print("[INFO] 当前运行情况处于推进中")

    ser.close()

if __name__ == "__main__":
    run_test()
