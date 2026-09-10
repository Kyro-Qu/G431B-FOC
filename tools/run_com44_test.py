# -*- coding: utf-8 -*-
"""
COM44 通信自动化测试脚本
执行流程:
1. 发送 `fault clear`，读取并打印响应
2. 发送 `status`，读取并打印响应
3. 发送 `calib`，读取并打印响应，随后以 0.5s 周期循环 15s 查询 status 与校准进度，逐行打印
4. 发送 `mode vel`、`enable`、`target 500`，监控 status 运行 3 秒
5. 发送 `target 0`、`disable`，读取并打印响应
全程打印所有串口收发报文
"""

import sys
import time
import serial

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

PORT = "COM44"
BAUD = 6500000

CALIB_STATES = {
    0: "IDLE",
    1: "BOOTSTRAP",
    2: "NEUTRAL",
    3: "ALIGN",
    4: "SETTLE",
    5: "SEARCH",
    6: "DONE",
    7: "FAIL",
}


def open_serial():
    print(f"[*] 正在打开串口 {PORT} (波特率 {BAUD})...")
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    # 清空接收缓冲
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


def send_cmd(ser, cmd_str, wait_time=0.25):
    print(f"\n>>> TX: {cmd_str}")
    ser.reset_input_buffer()
    ser.write((cmd_str + "\r\n").encode("utf-8"))

    t0 = time.time()
    rx_lines = []
    buf = bytearray()

    # 持续收集直到 wait_time 结束且至少一定时间内无新增字节
    while time.time() - t0 < wait_time:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        else:
            time.sleep(0.01)

    text = buf.decode("utf-8", errors="replace")
    for line in text.splitlines():
        line = line.strip("\r\n")
        if line:
            print(f"<<< RX: {line}")
            rx_lines.append(line)

    return rx_lines, text


def parse_status_summary(lines):
    info = {}
    for l in lines:
        if l.startswith("M0 "):
            parts = l.split()
            if len(parts) >= 2:
                info["state"] = parts[1]
        elif l.startswith("vel="):
            info["vel"] = l.split("=")[1]
        elif l.startswith("tgt="):
            info["tgt"] = l.split("=")[1]
        elif l.startswith("calib="):
            info["calib_valid"] = l.split("=")[1]
        elif l.startswith("calib_state="):
            parts = l.split()
            st_val = parts[0].split("=")[1]
            try:
                st_int = int(st_val)
                info["calib_state"] = f"{st_int} ({CALIB_STATES.get(st_int, '?')})"
            except ValueError:
                info["calib_state"] = st_val
        elif l.startswith("fault="):
            info["fault"] = l.split("=")[1]
        elif l.startswith("iq="):
            info["iq_line"] = l
    return info


def main():
    try:
        ser = open_serial()
    except Exception as e:
        print(f"[!] 打开串口 {PORT} 失败: {e}")
        sys.exit(1)

    try:
        # 步骤 1: 发送 fault clear
        print("=" * 60)
        print("步骤 1: 发送 `fault clear` 并读取响应")
        print("=" * 60)
        send_cmd(ser, "fault clear", wait_time=0.3)

        # 步骤 2: 发送 status
        print("\n" + "=" * 60)
        print("步骤 2: 发送 `status` 并读取响应")
        print("=" * 60)
        lines, _ = send_cmd(ser, "status", wait_time=0.3)
        summary = parse_status_summary(lines)
        print(f"[解析] 状态概览: {summary}")

        # 步骤 3: 发送 calib，并循环查询 15 秒
        print("\n" + "=" * 60)
        print("步骤 3: 发送 `calib` 并进入 15 秒进度监控循环 (每 0.5s 查询)")
        print("=" * 60)
        send_cmd(ser, "calib", wait_time=0.3)

        start_time = time.time()
        poll_idx = 0
        while True:
            elapsed = time.time() - start_time
            if elapsed >= 15.0:
                break
            poll_idx += 1
            print(f"\n--- [校准监控 T={elapsed:05.2f}s | 次数 #{poll_idx}] ---")
            lines, _ = send_cmd(ser, "status", wait_time=0.2)
            summary = parse_status_summary(lines)
            st_desc = summary.get("calib_state", "未知")
            v_val = summary.get("calib_valid", "未知")
            f_val = summary.get("fault", "0")
            vel_val = summary.get("vel", "未知")
            print(f"[进度提取] calib_state={st_desc} | calib_valid={v_val} | vel={vel_val} | fault={f_val}")

            # 计算剩余休眠时间以保持 0.5s 周期
            next_target = start_time + poll_idx * 0.5
            rem = next_target - time.time()
            if rem > 0:
                time.sleep(rem)

        print(f"\n[*] 校准阶段监控完成 (总耗时: {time.time() - start_time:.2f}s)")

        # 步骤 4: 发送 mode vel, enable, target 500, 监控 3 秒
        print("\n" + "=" * 60)
        print("步骤 4: 切换速度模式、使能并设定 500 RPM，监控 3 秒")
        print("=" * 60)
        send_cmd(ser, "mode vel", wait_time=0.25)
        send_cmd(ser, "enable", wait_time=0.25)
        send_cmd(ser, "target 500", wait_time=0.25)

        print("\n[*] 运行监控 (3 秒):")
        mon_start = time.time()
        mon_idx = 0
        while True:
            elapsed = time.time() - mon_start
            if elapsed >= 3.0:
                break
            mon_idx += 1
            print(f"\n--- [速度运行监控 T={elapsed:04.2f}s | 次数 #{mon_idx}] ---")
            lines, _ = send_cmd(ser, "status", wait_time=0.2)
            summary = parse_status_summary(lines)
            print(f"[运行提取] state={summary.get('state')} | vel={summary.get('vel')} | tgt={summary.get('tgt')} | {summary.get('iq_line', '')}")

            next_target = mon_start + mon_idx * 0.5
            rem = next_target - time.time()
            if rem > 0:
                time.sleep(rem)

        # 步骤 5: target 0, disable
        print("\n" + "=" * 60)
        print("步骤 5: 停止并失能电机 (`target 0`, `disable`)")
        print("=" * 60)
        send_cmd(ser, "target 0", wait_time=0.25)
        send_cmd(ser, "disable", wait_time=0.25)

        # 再次读取 status 确认已回到 IDLE
        print("\n[*] 最终确认状态:")
        lines, _ = send_cmd(ser, "status", wait_time=0.25)
        summary = parse_status_summary(lines)
        print(f"[最终状态] state={summary.get('state')} | vel={summary.get('vel')} | fault={summary.get('fault')}")

        print("\n" + "=" * 60)
        print("[+] 测试流程全部顺利执行完成！")
        print("=" * 60)

    finally:
        ser.close()
        print(f"[*] 串口 {PORT} 已关闭。")


if __name__ == "__main__":
    main()
