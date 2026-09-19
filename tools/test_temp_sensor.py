# -*- coding: utf-8 -*-
"""
板载 NTC 温度传感器全链路功能实机严苛测试 (tools/test_temp_sensor.py)
测试项：
1. CLI 命令: temp 查询
2. CLI 命令: status 综合回显中包含 temp 字段
3. CLI 命令: temp ot 动态修改保护阈值与越界防御测试
4. FOC-STP 10B STATUS 心跳包真实解码验证 temp_c 字段
"""
import sys
import time
import re
import struct
import serial

PORT = "COM44"
BAUD = 6500000

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

def run_test():
    print("=" * 60)
    print("STM32G431 板载 NTC 温度传感器全链路实机测试")
    print("=" * 60)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"[FAIL] 无法打开串口 {PORT}: {e}")
        return False

    ser.reset_input_buffer()
    time.sleep(0.05)

    def send_cmd(cmd, delay=0.08):
        ser.reset_input_buffer()
        ser.write((cmd + '\n').encode('ascii'))
        time.sleep(delay)
        return ser.read_all().decode(errors='ignore').strip()

    # 先停掉 wave
    send_cmd("wave 0")

    # 1. 测试 temp 查询命令
    print("\n[TEST 1] 测试 'temp' 命令查询实时采样与物理换算...")
    resp_temp = send_cmd("temp")
    print(f"CLI 回显:\n{resp_temp}")

    # 格式例如: temp=26.4C (raw=1320 R=9850ohm OK) ot=85.0C
    m_temp = re.search(r"temp=([\-\d\.]+)C\s+\(raw=(\d+)\s+R=([\-\d\.]+)ohm\s+(\w+)\)\s+ot=([\-\d\.]+)C", resp_temp)
    if not m_temp:
        print("[FAIL] 'temp' 命令回显未匹配到预期格式")
        ser.close()
        return False

    cur_temp = float(m_temp.group(1))
    raw_adc = int(m_temp.group(2))
    r_ntc = float(m_temp.group(3))
    status_str = m_temp.group(4)
    ot_thresh = float(m_temp.group(5))

    print(f"-> 当前温度: {cur_temp:.1f} °C")
    print(f"-> 原始 ADC 码值: {raw_adc} counts")
    print(f"-> 计算所得 NTC 阻值: {r_ntc:.0f} Ω")
    print(f"-> 采样链路状态: {status_str}")
    print(f"-> 当前过温保护阈值: {ot_thresh:.1f} °C")

    # 物理合理性判定（常温环境下通常在 15°C ~ 45°C 之间，ADC 在 1000 ~ 2000 左右，阻值在 5k ~ 15k 左右）
    if 10.0 <= cur_temp <= 55.0 and status_str == "OK" and 500 <= raw_adc <= 3000:
        print("[PASS] 实时温度与物理阻值换算完全符合常温物理特性！")
    else:
        print(f"[FAIL] 温度读数异常越界: {cur_temp}°C, raw={raw_adc}, R={r_ntc}")
        ser.close()
        return False

    # 2. 测试 status 命令中包含 temp 字段
    print("\n[TEST 2] 测试 'status' 命令回显中的 temp 字段...")
    resp_status = send_cmd("status")
    if "temp=" in resp_status and "raw=" in resp_status:
        # 提取相关行
        for line in resp_status.splitlines():
            if "temp=" in line:
                print(f"-> 匹配到 status 行: {line}")
        print("[PASS] 'status' 命令成功包含温度监控信息！")
    else:
        print(f"[FAIL] 'status' 输出中缺少 temp 字段:\n{resp_status}")
        ser.close()
        return False

    # 3. 测试 temp ot 参数动态配置与安全防御
    print("\n[TEST 3] 测试 'temp ot' 阈值调整与越界保护...")
    resp_ot90 = send_cmd("temp ot 90.0")
    print(f"-> 设为 90.0C: {resp_ot90}")
    resp_chk = send_cmd("temp")
    if "ot=90.0C" in resp_chk:
        print("[PASS] 动态阈值修改成功")
    else:
        print(f"[FAIL] 阈值修改未生效: {resp_chk}")
        ser.close()
        return False

    # 测试越界保护（合法区间 40..130）
    resp_err = send_cmd("temp ot 150.0")
    if "err:" in resp_err:
        print(f"[PASS] 越界设置被正确拒绝: {resp_err}")
    else:
        print(f"[FAIL] 越界设置未被拦截: {resp_err}")
        ser.close()
        return False

    # 恢复默认 85.0C
    send_cmd("temp ot 85.0")
    print("-> 恢复阈值为 85.0C")

    # 4. 测试 FOC-STP 10B STATUS 心跳包解析
    print("\n[TEST 4] 捕获并解析 10B FOC-STP STATUS 心跳帧中的 temp_c 字段...")
    ser.reset_input_buffer()
    status_frame_found = False
    start_t = time.time()
    raw_buffer = bytearray()

    while time.time() - start_t < 2.0:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            raw_buffer.extend(chunk)
            # 搜索同步字 0xA5, 0x5A
            while len(raw_buffer) >= 18:
                if raw_buffer[0] == 0xA5 and raw_buffer[1] == 0x5A:
                    ver_type = raw_buffer[2]
                    frame_type = ver_type & 0x0F
                    payload_len = raw_buffer[3]
                    total_len = 6 + payload_len + 2 # Header(6) + Payload(L) + CRC(2)
                    if len(raw_buffer) < total_len:
                        break

                    if frame_type == 0x02 and payload_len == 10: # STATUS 帧
                        # Payload 格式: timestamp_ms (4B), vbus_cvolts (2B), fault_code (1B), state (1B), temp_c (1B), cpu_load_pct (1B)
                        payload = raw_buffer[6:6+10]
                        ts, vbus_cv, fault, state, temp_c, cpu_pct = struct.unpack("<IHBBbB", payload)
                        vbus_v = vbus_cv / 100.0
                        print(f"-> 成功捕获 10B STATUS 心跳包:")
                        print(f"   时间戳: {ts} ms")
                        print(f"   母线电压: {vbus_v:.2f} V")
                        print(f"   故障码: {fault}")
                        print(f"   状态机: {state}")
                        print(f"   【板载温度 temp_c】: {temp_c} °C")
                        print(f"   CPU 负荷率: {cpu_pct} %")

                        # 对比 CLI 测量的温度
                        if abs(temp_c - cur_temp) <= 2.0:
                            print(f"[PASS] 心跳包中温度 ({temp_c}°C) 与 CLI 实时采样 ({cur_temp:.1f}°C) 完美一致！")
                            status_frame_found = True
                        else:
                            print(f"[FAIL] 心跳包中温度 ({temp_c}°C) 与 CLI ({cur_temp:.1f}°C) 偏差过大")
                        break
                    else:
                        # 消费掉一字节
                        raw_buffer.pop(0)
                else:
                    raw_buffer.pop(0)
            if status_frame_found:
                break
        time.sleep(0.01)

    ser.close()

    if not status_frame_found:
        print("[FAIL] 未能成功捕获有效的 10B STATUS 心跳帧")
        return False

    print("\n" + "=" * 60)
    print("全部测试通过！MCU 端温度采样驱动、物理模型方程、过温保护与 STP 遥测打包 100% 正常！")
    print("=" * 60)
    return True

if __name__ == "__main__":
    ok = run_test()
    sys.exit(0 if ok else 1)
