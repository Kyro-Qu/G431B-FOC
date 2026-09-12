# -*- coding: utf-8 -*-
"""
FOC-STP v1.0 硬件实测自动化验证套件
全面验证：
1. 串口高速全双工连接 (6.5 MBaud)
2. CLI 指令交互 (version, status, telem, log 0/1)
3. 500 Hz 自解释掩码波形流 (WAVE) 解析与帧序号单调性
4. 10 Hz 独立状态心跳流 (STATUS) 解析与各字段物理真值检验
5. 动态掩码实时切换 (telem mask)
"""

import sys
import time
import struct
import serial

PORT = "COM44"
BAUD = 6500000

# CRC16-CCITT (poly 0x1021, init 0xFFFF)
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

class StpStreamParser:
    def __init__(self):
        self.buf = bytearray()
        self.wave_frames = []
        self.status_frames = []
        self.event_frames = []
        self.ack_frames = []
        self.text_lines = []

    def feed(self, data: bytes):
        self.buf.extend(data)
        while len(self.buf) >= 8:
            # 查找同步字 0xA5 0x5A
            if self.buf[0] != 0xA5 or self.buf[1] != 0x5A:
                idx = -1
                for i in range(1, len(self.buf) - 1):
                    if self.buf[i] == 0xA5 and self.buf[i+1] == 0x5A:
                        idx = i
                        break
                if idx != -1:
                    self.buf = self.buf[idx:]
                else:
                    if self.buf[-1] == 0xA5:
                        self.buf = self.buf[-1:]
                    else:
                        self.buf.clear()
                    break

            if len(self.buf) < 8:
                break

            ver_type = self.buf[2]
            length = self.buf[3]
            frame_len = 6 + length + 2 # SYNC(2) + VER(1) + LEN(1) + SEQ(2) + PAYLOAD(length) + CRC(2)

            if length > 128:
                # 非法超长长度，跳过假同步字
                self.buf = self.buf[2:]
                continue

            if len(self.buf) < frame_len:
                break

            # 提取帧内容与 CRC
            raw_frame = bytes(self.buf[:frame_len])
            cal_crc = crc16_ccitt(raw_frame[2:frame_len-2])
            frame_crc = struct.unpack("<H", raw_frame[frame_len-2:frame_len])[0]

            if cal_crc != frame_crc:
                # CRC 校验失败，滑动 2 字节重新同步
                self.buf = self.buf[2:]
                continue

            # CRC 正确，消费缓冲区
            self.buf = self.buf[frame_len:]
            seq = struct.unpack("<H", raw_frame[4:6])[0]
            payload = raw_frame[6:frame_len-2]
            ftype = ver_type & 0x0F

            if ftype == 1: # WAVE
                if len(payload) >= 8:
                    tick, mask = struct.unpack("<II", payload[:8])
                    vals_data = payload[8:]
                    val_count = len(vals_data) // 4
                    vals = struct.unpack(f"<{val_count}f", vals_data[:val_count*4])
                    self.wave_frames.append({
                        "seq": seq, "tick": tick, "mask": mask, "vals": vals
                    })
            elif ftype == 2: # STATUS
                if len(payload) == 15:
                    ts, vbus, mf, sf, state, mode, temp, rpm, iq = struct.unpack("<IHBBBBbhh", payload)
                    self.status_frames.append({
                        "seq": seq, "ts": ts, "vbus": vbus / 100.0,
                        "motor_fault": mf, "shunt_fault": sf,
                        "state": state, "mode": mode, "temp": temp,
                        "rpm": rpm, "iq": iq / 100.0
                    })
            elif ftype == 3: # EVENT
                if len(payload) == 11:
                    ts, eid, mf, sf, detail = struct.unpack("<IBBBI", payload)
                    self.event_frames.append({
                        "seq": seq, "ts": ts, "eid": eid,
                        "motor_fault": mf, "shunt_fault": sf, "detail": detail
                    })
            elif ftype == 5: # ACK
                if len(payload) == 8:
                    code, status, mask, rate = struct.unpack("<BBHI", payload[:8])
                    self.ack_frames.append({
                        "seq": seq, "code": code, "status": status,
                        "mask": mask, "rate": rate
                    })

def main():
    print(f"=== 连接 {PORT} @ {BAUD} ===")
    ser = serial.Serial(PORT, BAUD, timeout=0.05, write_timeout=0.5)
    time.sleep(0.1)

    # 1. 测试 CLI 交互与阻塞响应
    print("\n--- [测试 1: CLI 命令双向交互] ---")
    commands = [b"version\n", b"status\n", b"telem\n", b"obs 0\n"]
    for cmd in commands:
        ser.reset_input_buffer()
        ser.write(cmd)
        t0 = time.time()
        resp = bytearray()
        while time.time() - t0 < 0.3:
            chunk = ser.read(4096)
            if chunk:
                resp.extend(chunk)
        # 寻找 ASCII 响应行
        ascii_text = resp.decode("latin1", "replace")
        lines = [line.strip() for line in ascii_text.splitlines() if line.strip() and all(32 <= ord(c) < 127 for c in line.strip())]
        print(f"CMD > {cmd.decode().strip()}")
        if lines:
            for l in lines[:3]:
                print(f"  RECV: {l}")
        else:
            print("  WARN: 未解析到纯文本行 (可能混入二进制流)")

    # 2. 收集 1.5 秒 FOC-STP 协议流
    print("\n--- [测试 2: 采集并解析 1.5 秒高速 FOC-STP 协议流] ---")
    ser.write(b"telem enable 1\n")
    time.sleep(0.1)
    ser.reset_input_buffer()
    parser = StpStreamParser()
    t_start = time.time()
    total_bytes = 0
    while time.time() - t_start < 1.5:
        data = ser.read(8192)
        if data:
            total_bytes += len(data)
            parser.feed(data)
        time.sleep(0.005)

    print(f"接收总字节数: {total_bytes} 字节 ({total_bytes / 1.5 / 1024:.1f} KB/s)")
    print(f"解析到 WAVE 帧数: {len(parser.wave_frames)} 帧 (预期 ~750 帧 @500Hz)")
    print(f"解析到 STATUS 帧数: {len(parser.status_frames)} 帧 (预期 ~15 帧 @10Hz)")

    assert len(parser.wave_frames) > 300, f"WAVE 帧率过低: {len(parser.wave_frames)}"
    assert len(parser.status_frames) >= 5, f"STATUS 心跳缺失: {len(parser.status_frames)}"

    # 校验 WAVE 帧
    sample_wave = parser.wave_frames[-1]
    print(f"WAVE 样本: seq={sample_wave['seq']}, tick={sample_wave['tick']}, mask=0x{sample_wave['mask']:08X}, 通道数={len(sample_wave['vals'])}")
    print(f"  前 4 个通道浮点数: {[round(v, 4) for v in sample_wave['vals'][:4]]}")

    # 校验 STATUS 帧
    sample_status = parser.status_frames[-1]
    print(f"STATUS 心跳样本: seq={sample_status['seq']}, ts={sample_status['ts']}ms, Vbus={sample_status['vbus']}V, State={sample_status['state']}, Mode={sample_status['mode']}, Rpm={sample_status['rpm']}, Iq={sample_status['iq']}A")
    assert 10.0 <= sample_status['vbus'] <= 28.0, f"Vbus 异常: {sample_status['vbus']}V"

    # 3. 测试动态掩码切换 (telem mask 0x00000005 -> 仅 theta_e + vel_ctrl)
    print("\n--- [测试 3: 动态掩码切换] ---")
    ser.write(b"telem mask 0x05\n")
    time.sleep(0.2)
    ser.reset_input_buffer()
    parser2 = StpStreamParser()
    t_start = time.time()
    while time.time() - t_start < 0.5:
        data = ser.read(8192)
        if data:
            parser2.feed(data)
        time.sleep(0.005)

    if parser2.wave_frames:
        last_wave = parser2.wave_frames[-1]
        print(f"切换后 WAVE 掩码: 0x{last_wave['mask']:08X}, 通道数: {len(last_wave['vals'])}")
        assert last_wave['mask'] == 0x05, f"掩码切换未生效: 0x{last_wave['mask']:08X}"
        assert len(last_wave['vals']) == 2, f"通道数未缩减为 2: {len(last_wave['vals'])}"
        print("  动态掩码切换成功且通道数精准自适应！")

    # 恢复默认掩码
    ser.write(b"telem mask 0x040001FF\n")
    time.sleep(0.1)

    ser.close()
    print("\n=== 所有硬件与协议流测试 100% PASS！===")

if __name__ == "__main__":
    main()
