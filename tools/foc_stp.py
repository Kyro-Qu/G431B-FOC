# -*- coding: utf-8 -*-
"""foc_stp.py - FOC-STP v1.0 (FOC Self-describing Telemetry Protocol) Python 流式解码器

供 tools/ 下所有脚本共用（与固件 MDK-ARM/Code/foc/App/foc_stp.c、上位机 js/protocol/stp.js 同规格）。

帧格式:  A5 5A | VER_TYPE | LEN | SEQ(u16 LE) | PAYLOAD[LEN] | CRC16(u16 LE)
CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, 覆盖 VER_TYPE..PAYLOAD 末尾。

用法:
    from foc_stp import StpStreamDecoder, CHANNEL_NAMES
    dec = StpStreamDecoder()
    dec.feed(ser.read(4096))
    for w in dec.pop_waves():        # dict(seq, tick, mask, vals, channels={name: value})
        ...
    for s in dec.pop_status():       # dict(seq, ts, vbus, motor_fault, shunt_fault, state, mode, temp, rpm, iq)
        ...
    text = dec.pop_text()            # 帧间的裸 CLI 文本（str）
"""
import struct

SYNC0 = 0xA5
SYNC1 = 0x5A
VERSION = 1

TYPE_WAVE = 0x1
TYPE_STATUS = 0x2
TYPE_EVENT = 0x3
TYPE_TEXT = 0x4
TYPE_ACK = 0x5

MAX_WAVE_CHANNELS = 16

# 32 位全局通道字典（bit -> 信号名），与固件 extract_channel_value / 上位机 channels.js 一致
CHANNEL_NAMES = [
    "theta_e", "iq_raw", "vel_ctrl", "vel_ref", "id_filt", "iq_filt", "iq_ref", "vd",
    "vq", "ia", "ib", "ic", "duty_a", "id_raw", "id_ref", "vel_raw",
    "pos_ref", "position", "duty_b", "duty_c", "obs_theta", "obs_speed", "obs_err", "obs_conf",
    "obs_flux", "power_est", "vbus_fast", "torque_est", "iq_err", "id_err", "vel_err", "diag_aux",
]

# 默认订阅掩码（与固件 FOC_TELEMETRY_DEFAULT_MASK 一致）
DEFAULT_MASK = 0x040001FF


def _make_table():
    tbl = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        tbl.append(crc)
    return tbl


_CRC_TABLE = _make_table()


def crc16_ccitt(data, crc=0xFFFF):
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


def mask_to_bits(mask):
    return [bit for bit in range(32) if mask & (1 << bit)]


def mask_of(*names):
    """按通道名生成掩码: mask_of('vel_ctrl', 'iq_filt')"""
    m = 0
    for n in names:
        m |= 1 << CHANNEL_NAMES.index(n)
    return m


def _payload_len_valid(ftype, length):
    if ftype == TYPE_WAVE:
        return 8 <= length <= 8 + 4 * MAX_WAVE_CHANNELS and (length - 8) % 4 == 0
    if ftype == TYPE_STATUS:
        return length == 15
    if ftype == TYPE_EVENT:
        return length == 11
    if ftype == TYPE_TEXT:
        return 1 <= length <= 128
    if ftype == TYPE_ACK:
        return length == 8
    return False


class StpStreamDecoder(object):
    def __init__(self):
        self.buf = bytearray()
        self.waves = []
        self.status = []
        self.events = []
        self.acks = []
        self.texts = []
        self.frames_ok = 0
        self.crc_errors = 0
        self.desync = 0
        self.bytes_in = 0
        self._text_pending = bytearray()

    # ---- 取出已解析结果 ----
    def pop_waves(self):
        out, self.waves = self.waves, []
        return out

    def pop_status(self):
        out, self.status = self.status, []
        return out

    def pop_events(self):
        out, self.events = self.events, []
        return out

    def pop_acks(self):
        out, self.acks = self.acks, []
        return out

    def pop_text(self):
        out = "".join(self.texts)
        self.texts = []
        return out

    def flush_idle(self):
        """链路静默后调用：把不足帧头长度的残留字节当作文本上交"""
        if not self.buf:
            return
        idx = self.buf.find(b"\xa5\x5a")
        if idx == 0:
            return
        end = idx if idx > 0 else len(self.buf)
        keep = 1 if (idx < 0 and self.buf[end - 1] == SYNC0) else 0
        self._emit_text(self.buf[:end - keep])
        del self.buf[:end - keep]

    # ---- 输入 ----
    def feed(self, data):
        if not data:
            return
        self.bytes_in += len(data)
        self.buf.extend(data)
        self._drain()

    def _emit_text(self, raw):
        s = "".join(chr(b) for b in raw if b in (10, 13) or 32 <= b < 127)
        if s:
            self.texts.append(s)

    def _drain(self):
        buf = self.buf
        while len(buf) >= 8:
            idx = buf.find(b"\xa5\x5a")
            if idx < 0:
                keep = 1 if buf[-1] == SYNC0 else 0
                self._emit_text(buf[:len(buf) - keep])
                del buf[:len(buf) - keep]
                return
            if idx > 0:
                self._emit_text(buf[:idx])
                del buf[:idx]
                continue
            if len(buf) < 8:
                return
            ver_type = buf[2]
            length = buf[3]
            ver = (ver_type >> 4) & 0x0F
            ftype = ver_type & 0x0F
            total = 8 + length
            if ver != VERSION or not _payload_len_valid(ftype, length):
                self.desync += 1
                del buf[:1]
                continue
            if len(buf) < total:
                return
            frame = bytes(buf[:total])
            if crc16_ccitt(frame[2:total - 2]) != (frame[total - 2] | (frame[total - 1] << 8)):
                self.crc_errors += 1
                del buf[:1]
                continue
            del buf[:total]
            seq = frame[4] | (frame[5] << 8)
            payload = frame[6:total - 2]
            self.frames_ok += 1
            self._dispatch(ftype, seq, payload)

    def _dispatch(self, ftype, seq, payload):
        if ftype == TYPE_WAVE:
            tick, mask = struct.unpack_from("<II", payload, 0)
            n = (len(payload) - 8) // 4
            bits = mask_to_bits(mask)
            if len(bits) != n:
                self.desync += 1
                return
            vals = struct.unpack_from("<%df" % n, payload, 8) if n else ()
            self.waves.append({
                "seq": seq, "tick": tick, "mask": mask, "vals": vals,
                "channels": {CHANNEL_NAMES[b]: v for b, v in zip(bits, vals)},
            })
        elif ftype == TYPE_STATUS:
            ts, vbus, mf, sf, state, mode, temp, rpm, iq = struct.unpack("<IHBBBBbhh", payload)
            self.status.append({
                "seq": seq, "ts": ts, "vbus": vbus / 100.0,
                "motor_fault": mf, "shunt_fault": sf, "state": state, "mode": mode,
                "temp": temp, "rpm": rpm, "iq": iq / 100.0,
            })
        elif ftype == TYPE_EVENT:
            ts, eid, mf, sf, detail = struct.unpack("<IBBBI", payload)
            self.events.append({
                "seq": seq, "ts": ts, "event_id": eid,
                "motor_fault": mf, "shunt_fault": sf, "detail": detail,
            })
        elif ftype == TYPE_TEXT:
            self.texts.append(payload.decode("utf-8", "replace"))
        elif ftype == TYPE_ACK:
            code, status, mask, rate = struct.unpack("<BBIH", payload)
            self.acks.append({
                "seq": seq, "cmd_code": code, "status": status,
                "mask": mask, "rate_hz": rate,
            })


def cli(ser, cmd, wait=0.25, decoder=None):
    """发送一条 CLI 命令并返回纯文本回复（自动剥离交错的二进制帧）。"""
    dec = decoder or StpStreamDecoder()
    if decoder is None:
        ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("ascii"))
    import time
    t0 = time.time()
    while time.time() - t0 < wait:
        chunk = ser.read(4096)
        if chunk:
            dec.feed(chunk)
    dec.flush_idle()
    return dec.pop_text().strip()


if __name__ == "__main__":
    # 自检：CRC 向量 + 一帧 STATUS 往返
    assert crc16_ccitt(b"123456789") == 0x29B1
    payload = struct.pack("<IHBBBBbhh", 5000, 1442, 0, 0, 1, 2, -128, 1500, 52)
    hdr = bytes([SYNC0, SYNC1, (VERSION << 4) | TYPE_STATUS, len(payload), 7, 0])
    frame = hdr + payload + struct.pack("<H", crc16_ccitt(hdr[2:] + payload))
    d = StpStreamDecoder()
    d.feed(b"M0 IDLE\r\n" + frame[:5])
    d.feed(frame[5:] + b"ok\r\n")
    d.flush_idle()
    st = d.pop_status()
    assert len(st) == 1 and st[0]["vbus"] == 14.42 and st[0]["rpm"] == 1500, st
    assert d.pop_text() == "M0 IDLE\r\nok\r\n"
    print("foc_stp self-test OK")
