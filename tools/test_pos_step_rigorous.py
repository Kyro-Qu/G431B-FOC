# -*- coding: utf-8 -*-
"""
严密位置环实机检验脚本 (COM44)
按阶段严格从小角度递进测试，实时采集 10 字节 STATUS 二进制遥测帧，
记录真实物理时域数据，杜绝任何假断言。
"""
import time
import struct
import serial

PORT = "COM44"
BAUD = 6500000

def parse_status_frame(payload):
    # 10 字节二进制 STATUS:
    # int16 vn (0.1 RPM)
    # int16 pos (0.001 rad)
    # int16 iq (1 mA)
    # uint8 state_mode: state:4, mode:4
    # uint8 fault
    # uint16 vbus (0.01 V)
    if len(payload) != 10:
        return None
    vn, pos, iq, sm, fault, vbus = struct.unpack("<hhhBBH", payload)
    return {
        "vn": vn * 0.1,
        "pos": pos * 0.001,
        "iq": iq * 0.001,
        "state": (sm >> 4) & 0x0F,
        "mode": sm & 0x0F,
        "fault": fault,
        "vbus": vbus * 0.01
    }

def read_telemetry(ser, duration_sec):
    t0 = time.time()
    samples = []
    buf = bytearray()
    while time.time() - t0 < duration_sec:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf.extend(chunk)
        while len(buf) >= 14:
            if buf[0] == 0xAA and buf[1] == 0x55 and buf[2] == 0x01 and buf[3] == 0x0A:
                payload = buf[4:14]
                frame = parse_status_frame(payload)
                if frame:
                    frame["t"] = time.time() - t0
                    samples.append(frame)
                buf = buf[14:]
            else:
                buf.pop(0)
    return samples

def send_cmd(ser, cmd_str, wait_sec=0.1):
    ser.reset_input_buffer()
    ser.write(cmd_str.encode("ascii") + b"\r\n")
    ser.flush()
    time.sleep(wait_sec)
    out = ser.read(ser.in_waiting or 1024).decode("ascii", errors="ignore")
    return out

def run_test():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("=== 1. 检查通信与状态 ===")
    out = send_cmd(ser, "status")
    print("STATUS 回复:", out.strip())

    print("\n=== 2. 执行电机校准 ===")
    out = send_cmd(ser, "calib", wait_sec=3.5)
    print("CALIB 回复:", out.strip())

    out = send_cmd(ser, "status")
    print("校准后状态:", out.strip())

    # 确保停机并开启二进制遥测流
    send_cmd(ser, "disarm")
    time.sleep(0.1)
    out_telem = send_cmd(ser, "telem 1")
    print("开启遥测流回复:", out_telem.strip())

    print("\n=== 3. 阶段 A: 原地保持测试 (target 0.000) ===")
    send_cmd(ser, "mode pos")
    send_cmd(ser, "target 0.000")
    send_cmd(ser, "enable")
    time.sleep(0.2)

    samples_a = read_telemetry(ser, 1.0)
    print(f"采集到 {len(samples_a)} 帧遥测数据")
    if samples_a:
        vns = [s["vn"] for s in samples_a]
        iqs = [s["iq"] for s in samples_a]
        poses = [s["pos"] for s in samples_a]
        print(f"阶段 A 原地保持结果:")
        print(f"  转速范围: [{min(vns):.1f}, {max(vns):.1f}] RPM, RMS: {(sum(v**2 for v in vns)/len(vns))**0.5:.2f} RPM")
        print(f"  电流范围: [{min(iqs):.3f}, {max(iqs):.3f}] A, RMS: {(sum(i**2 for i in iqs)/len(iqs))**0.5:.3f} A")
        print(f"  位置范围: [{min(poses):.4f}, {max(poses):.4f}] rad")

    print("\n=== 4. 阶段 B: 极小阶跃 +0.010 rad (约 0.57度) ===")
    send_cmd(ser, "target 0.010")
    samples_b = read_telemetry(ser, 1.5)
    print(f"采集到 {len(samples_b)} 帧遥测数据")
    if samples_b:
        vns = [s["vn"] for s in samples_b]
        iqs = [s["iq"] for s in samples_b]
        poses = [s["pos"] for s in samples_b]
        final_pos = poses[-10:]
        avg_final = sum(final_pos) / len(final_pos)
        print(f"阶段 B 极小阶跃结果:")
        print(f"  最大转速: {max(abs(v) for v in vns):.1f} RPM")
        print(f"  最大电流: {max(abs(i) for i in iqs):.3f} A (安全阈值 0.50A)")
        print(f"  最终位置: {avg_final:.4f} rad (目标: 0.0100 rad, 误差: {avg_final - 0.010:.4f} rad)")

    print("\n=== 5. 阶段 C: 小阶跃 +0.050 rad (约 2.86度) ===")
    send_cmd(ser, "target 0.050")
    samples_c = read_telemetry(ser, 1.5)
    print(f"采集到 {len(samples_c)} 帧遥测数据")
    if samples_c:
        vns = [s["vn"] for s in samples_c]
        iqs = [s["iq"] for s in samples_c]
        poses = [s["pos"] for s in samples_c]
        final_pos = poses[-10:]
        avg_final = sum(final_pos) / len(final_pos)
        print(f"阶段 C 小阶跃结果:")
        print(f"  最大转速: {max(abs(v) for v in vns):.1f} RPM")
        print(f"  最大电流: {max(abs(i) for i in iqs):.3f} A")
        print(f"  最终位置: {avg_final:.4f} rad (目标: 0.0500 rad, 误差: {avg_final - 0.050:.4f} rad)")

    print("\n=== 6. 阶段 D: 中角度阶跃 +0.200 rad (约 11.5度) ===")
    send_cmd(ser, "target 0.200")
    samples_d = read_telemetry(ser, 2.0)
    print(f"采集到 {len(samples_d)} 帧遥测数据")
    if samples_d:
        vns = [s["vn"] for s in samples_d]
        iqs = [s["iq"] for s in samples_d]
        poses = [s["pos"] for s in samples_d]
        final_pos = poses[-10:]
        avg_final = sum(final_pos) / len(final_pos)
        print(f"阶段 D 中阶跃结果:")
        print(f"  最大转速: {max(abs(v) for v in vns):.1f} RPM")
        print(f"  最大电流: {max(abs(i) for i in iqs):.3f} A")
        print(f"  最终位置: {avg_final:.4f} rad (目标: 0.2000 rad, 误差: {avg_final - 0.200:.4f} rad)")

    # 结束测试，安全停机
    print("\n=== 7. 测试结束，安全 disarm ===")
    send_cmd(ser, "telem 0")
    send_cmd(ser, "disarm")
    ser.close()
    print("串口已安全关闭。")

if __name__ == "__main__":
    run_test()
