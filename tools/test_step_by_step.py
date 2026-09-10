# -*- coding: utf-8 -*-
"""
Test each FOC command one by one and check if MCU reset or faulted:
1. fault clear
2. calib -> wait for calib=1
3. enc fault clear -> check status
4. feedback auto -> check status
5. feedback speed 420 320 -> check status
6. mode vel -> check status
7. vel ramp 800 -> check status
8. target 800 -> check status
9. enable -> check status
"""

import sys
import time
import re
import serial

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def parse_status(raw_text):
    info = {
        "raw": raw_text,
        "state": None,
        "mode": None,
        "calib": None,
        "is_calib_valid": False,
        "fault": None,
        "rst_flags": None,
        "iwdg": None,
        "sft": None,
        "bor": None,
        "pin": None,
        "chk": None,
        "vel": None,
        "tgt": None,
        "vbus": None,
    }
    for line in raw_text.splitlines():
        line = line.strip()
        # M0 RUN mode=vel or M0 FAULT mode=vel or M0 IDLE mode=...
        m_state = re.search(r"M0\s+([A-Z_]+)\s+mode=([a-z_]+)", line)
        if m_state:
            info["state"] = m_state.group(1)
            info["mode"] = m_state.group(2)

        # tgt=...
        m_tgt = re.search(r"tgt=([-\d\.]+)", line)
        if m_tgt:
            info["tgt"] = float(m_tgt.group(1))

        # vel=...
        m_vel = re.search(r"vel=([-\d\.]+)rpm", line)
        if m_vel:
            info["vel"] = float(m_vel.group(1))

        # vbus=...
        m_vbus = re.search(r"vbus=([-\d\.]+)V", line)
        if m_vbus:
            info["vbus"] = float(m_vbus.group(1))

        # calib=1* or calib=0
        m_calib = re.search(r"calib=([0-9]+)(\*?)", line)
        if m_calib:
            info["calib"] = m_calib.group(1) + m_calib.group(2)
            info["is_calib_valid"] = (m_calib.group(1) == "1")

        # fault=0 <- or fault=6 <-
        m_fault = re.search(r"fault=(\d+)", line)
        if m_fault and ("<-" in line or info["fault"] is None):
            info["fault"] = int(m_fault.group(1))

        # rst_flags=0x24000000 (IWDG=1 SFT=0 BOR=0 PIN=1) chk=17170949
        m_rst = re.search(r"rst_flags=(0x[0-9A-Fa-f]+)\s+\(IWDG=(\d+)\s+SFT=(\d+)\s+BOR=(\d+)\s+PIN=(\d+)\)\s+chk=(\d+)", line)
        if m_rst:
            info["rst_flags"] = m_rst.group(0)
            info["rst_hex"] = m_rst.group(1)
            info["iwdg"] = int(m_rst.group(2))
            info["sft"] = int(m_rst.group(3))
            info["bor"] = int(m_rst.group(4))
            info["pin"] = int(m_rst.group(5))
            info["chk"] = int(m_rst.group(6))

    return info

class Tester:
    def __init__(self, port=PORT, baud=BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self.boot_seen = False
        self.was_calibrated = False
        self.last_chk = None
        self.initial_rst = None

    def connect(self):
        print(f"正在连接 {self.port}，波特率 {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        time.sleep(0.05)
        self.drain()

    def drain(self, duration=0.1):
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < duration:
            n = self.ser.in_waiting
            if n:
                b = self.ser.read(n)
                buf += b
                if b"FOC ready" in b or b"Current offsets" in b:
                    self.boot_seen = True
            else:
                time.sleep(0.01)
        return buf.decode("ascii", errors="replace")

    def send_cmd(self, cmd_str, wait_sec=0.15):
        self.drain(0.02)
        self.ser.write((cmd_str + "\r\n").encode("ascii"))
        time.sleep(wait_sec)
        resp = self.drain(0.1)
        return resp.strip()

    def query_status(self):
        resp = self.send_cmd("status", wait_sec=0.15)
        return parse_status(resp)

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.send_cmd("disable", wait_sec=0.1)
            except Exception:
                pass
            self.ser.close()

def main():
    tester = Tester()
    try:
        tester.connect()
    except Exception as e:
        print(f"打开串口失败: {e}")
        return 1

    print("\n--- 初始化检查 ---")
    tester.send_cmd("log 0", 0.05)
    init_st = tester.query_status()
    tester.initial_rst = init_st.get("rst_flags")
    tester.last_chk = init_st.get("chk")
    tester.was_calibrated = init_st.get("is_calib_valid", False)
    print(f"初始状态: state={init_st['state']}, calib={init_st['calib']}, fault={init_st['fault']}")
    print(f"初始复位标志: {init_st['rst_flags']}")

    steps = [
        ("1. fault clear", "fault clear", False),
        ("2. calib", "calib", True),
        ("3. enc fault clear", "enc fault clear", False),
        ("4. feedback auto", "feedback auto", False),
        ("5. feedback speed 420 320", "feedback speed 420 320", False),
        ("6. mode vel", "mode vel", False),
        ("7. vel ramp 800", "vel ramp 800", False),
        ("8. target 800", "target 800", False),
        ("9. enable", "enable", False),
    ]

    results = []
    first_iwdg_step = None
    first_fault_step = None

    for step_num, (step_label, cmd_str, is_calib) in enumerate(steps, 1):
        print(f"\n========================================================")
        print(f"【执行步骤 {step_label}】")
        print(f">> 发送命令: '{cmd_str}'")

        tester.boot_seen = False
        step_fault = False
        step_reset = False
        reset_reason = []

        if is_calib:
            # 步骤 2: calib -> wait for calib=1
            reply = tester.send_cmd(cmd_str, wait_sec=0.2)
            print(f"<< 命令响应: {reply}")
            print("等待校准完成 (calib=1)...")
            calib_done = False
            t_start = time.time()
            final_st = None
            while time.time() - t_start < 8.0:
                time.sleep(0.3)
                st = tester.query_status()
                final_st = st
                print(f"   [校准中] state={st['state']}, calib={st['calib']}, fault={st['fault']}")
                if st['is_calib_valid']:
                    calib_done = True
                    tester.was_calibrated = True
                    break
                if st['fault'] != 0 or st['state'] == 'FAULT':
                    break
                if tester.boot_seen:
                    break

            st = final_st if final_st else tester.query_status()
            if calib_done:
                print(f"校准成功完成: calib={st['calib']}")
            else:
                print(f"校准未达成 calib=1 (当前 calib={st['calib']}, fault={st['fault']})")
        else:
            # 常规命令
            reply = tester.send_cmd(cmd_str, wait_sec=0.15)
            print(f"<< 命令响应: {reply}")
            time.sleep(0.05)
            st = tester.query_status()

            if step_label.startswith("9. enable"):
                print("已使能，持续监测 2 秒运行情况...")
                for i in range(6):
                    time.sleep(0.3)
                    st_mon = tester.query_status()
                    print(f"   [T+{0.3*(i+1):.1f}s] state={st_mon['state']}, vel={st_mon['vel']}rpm, calib={st_mon['calib']}, fault={st_mon['fault']}")
                    if st_mon['state'] == 'FAULT' or st_mon['fault'] != 0:
                        st = st_mon
                        break
                    if not st_mon['is_calib_valid'] and tester.was_calibrated:
                        st = st_mon
                        break
                    st = st_mon

        # 检查是否复位
        if tester.boot_seen:
            step_reset = True
            reset_reason.append("捕获到开机启动日志(FOC ready / Current offsets)")
        if tester.was_calibrated and not st['is_calib_valid']:
            step_reset = True
            reset_reason.append("校准标志丢失(calib由1变为0)")
        if st.get('chk') != tester.last_chk and tester.last_chk is not None:
            step_reset = True
            reset_reason.append(f"检查点改变 (chk {tester.last_chk} -> {st.get('chk')})")
        if st.get('iwdg') == 1 and (tester.initial_rst is None or "IWDG=1" not in tester.initial_rst):
            step_reset = True
            reset_reason.append("IWDG看门狗复位标志置位")

        # 检查是否发生 fault
        if st.get('fault') != 0:
            step_fault = True
            fault_code = st.get('fault')
        else:
            fault_code = 0

        # 更新基准
        if st.get('chk') is not None:
            tester.last_chk = st.get('chk')
        if st.get('is_calib_valid'):
            tester.was_calibrated = True

        print(f"\n--- 步骤 {step_label} 状态检查 ---")
        print(f"  电机状态: state={st.get('state')} mode={st.get('mode')}")
        print(f"  校准状态: calib={st.get('calib')} (valid={st.get('is_calib_valid')})")
        print(f"  故障代码: fault={st.get('fault')}")
        print(f"  复位寄存器: {st.get('rst_flags')}")
        print(f"  是否发生复位: {'【是!】' if step_reset else '否'}")
        if step_reset:
            print(f"    复位原因/现象: {'; '.join(reset_reason)}")
            if first_iwdg_step is None:
                first_iwdg_step = (step_label, reset_reason)
        print(f"  是否发生故障: {'【是!】' if step_fault else '否'}")
        if step_fault:
            print(f"    故障码: fault={fault_code}")
            if first_fault_step is None:
                first_fault_step = (step_label, fault_code)

        results.append({
            "step": step_label,
            "cmd": cmd_str,
            "state": st.get('state'),
            "calib": st.get('calib'),
            "fault": fault_code,
            "rst_flags": st.get('rst_flags'),
            "reset": step_reset,
            "reset_reason": reset_reason,
        })

    # 安全停机
    tester.send_cmd("disable", 0.1)
    tester.close()

    print("\n" + "=" * 70)
    print("                      测试结果总览表")
    print("=" * 70)
    print(f"{'步骤':<28} | {'状态':<6} | {'校准':<8} | {'故障':<6} | {'复位?'}")
    print("-" * 70)
    for r in results:
        rst_str = "YES (" + ",".join(r["reset_reason"]) + ")" if r["reset"] else "NO"
        print(f"{r['step']:<28} | {str(r['state']):<6} | {str(r['calib']):<8} | {str(r['fault']):<6} | {rst_str}")
    print("=" * 70)

    print("\n【关键结论输出】")
    if first_iwdg_step:
        print(f"★ 触发复位 (IWDG/Reset) 的步骤: 【{first_iwdg_step[0]}】")
        print(f"   现象/原因: {first_iwdg_step[1]}")
    else:
        print("★ 未检测到任何指令导致 MCU 复位 (IWDG Reset)。")

    if first_fault_step:
        print(f"★ 触发故障 (Fault) 的步骤: 【{first_fault_step[0]}】")
        print(f"   故障代码: fault={first_fault_step[1]}")
    else:
        print("★ 未检测到任何指令触发故障。")

    return 0

if __name__ == "__main__":
    sys.exit(main())
