# -*- coding: utf-8 -*-
"""
FOC-STP v1.0 硬件实测自动化验证套件（COM44 @ 6.5 MBaud）
1. CLI 指令双向交互（version / status / telem）
2. 500 Hz WAVE 波形流：帧率、序号单调、CRC 零错误
3. 10 Hz STATUS 心跳：帧率、母线电压真值
4. 动态掩码切换（telem mask）与 ACK 应答
5. 动态速率切换（telem rate）与 ACK 应答
6. 波形流开启时 CLI 文本交错不失步
7. 快环 CPU 占用（status 的 cpu= 字段）在最大 16 通道下仍有余量
"""
import os
import re
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from foc_stp import StpStreamDecoder, cli  # noqa: E402

PORT = "COM44"
BAUD = 6500000

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

passed = 0
failed = 0


def check(label, cond, detail=""):
    global passed, failed
    if cond:
        passed += 1
        print("  PASS  %s %s" % (label, detail))
    else:
        failed += 1
        print("  FAIL  %s %s" % (label, detail))


def capture(ser, secs):
    dec = StpStreamDecoder()
    t0 = time.time()
    while time.time() - t0 < secs:
        dec.feed(ser.read(8192))
    dec.flush_idle()
    return dec


def cpu_of(txt):
    m = re.search(r"cpu=([\d\.]+)% \(max ([\d\.]+)%\)", txt)
    return (float(m.group(1)), float(m.group(2))) if m else (None, None)


def main():
    print("=== 连接 %s @ %d ===" % (PORT, BAUD))
    ser = serial.Serial(PORT, BAUD, timeout=0.02, write_timeout=0.5)
    time.sleep(0.1)
    cli(ser, "telem enable 0")

    print("\n--- [1] CLI 命令双向交互 ---")
    ver = cli(ser, "version")
    print("  " + ver.replace("\r\n", "\n  "))
    check("version 回显含固件名", "firmware=FOC_G431" in ver)
    st = cli(ser, "status", wait=0.4)
    check("status 回显含状态行", re.search(r"M0 (IDLE|RUN|CALIB|FAULT)", st) is not None)
    tl = cli(ser, "telem")
    check("telem 状态行含 mask/rate", "mask=0x" in tl and "rate=" in tl, tl)

    print("\n--- [2] 500 Hz WAVE + 10 Hz STATUS（默认掩码 10 通道，1.5s） ---")
    cli(ser, "telem mask 0x040001FF")
    cli(ser, "telem rate 500")
    cli(ser, "telem enable 1")
    ser.reset_input_buffer()
    dec = capture(ser, 1.5)
    waves = dec.pop_waves()
    status = dec.pop_status()
    print("  bytes=%d waves=%d status=%d crc_err=%d desync=%d" % (
        dec.bytes_in, len(waves), len(status), dec.crc_errors, dec.desync))
    check("WAVE 帧率 ~500Hz", 700 <= len(waves) <= 800, "%d 帧/1.5s" % len(waves))
    check("STATUS 帧率 ~10Hz", 13 <= len(status) <= 17, "%d 帧/1.5s" % len(status))
    check("CRC 零错误", dec.crc_errors == 0, "crc_err=%d" % dec.crc_errors)
    seq_ok = all(((waves[i + 1]["seq"] - waves[i]["seq"]) & 0xFFFF) == 1 for i in range(len(waves) - 1))
    check("WAVE 序号连续无丢帧", seq_ok)
    ticks_ok = all(waves[i + 1]["tick"] >= waves[i]["tick"] for i in range(len(waves) - 1))
    check("WAVE tick 单调", ticks_ok)
    check("WAVE 通道数=10 且掩码正确", waves and waves[-1]["mask"] == 0x040001FF and len(waves[-1]["vals"]) == 10)
    if status:
        s = status[-1]
        print("  STATUS: vbus=%.2fV state=%d temp=%d cpu=%s" % (
            s["vbus"], s["state"], s.get("temp", 0), s.get("cpu_pct", "N/A")))
        check("母线电压 10..28V", 10.0 <= s["vbus"] <= 28.0, "%.2fV" % s["vbus"])
        vb = waves[-1]["channels"].get("vbus_fast") if waves else None
        check("vbus_fast 波形与 STATUS 一致(±0.3V)", vb is not None and abs(vb - s["vbus"]) < 0.3,
              "wave=%.2f status=%.2f" % (vb if vb is not None else -1, s["vbus"]))

    print("\n--- [3] 动态掩码切换 + ACK ---")
    ser.reset_input_buffer()
    dec = StpStreamDecoder()
    ser.write(b"telem mask 0x05\n")
    t0 = time.time()
    while time.time() - t0 < 0.5:
        dec.feed(ser.read(8192))
    acks = dec.pop_acks()
    waves = dec.pop_waves()
    check("收到 SET_MASK ACK(cmd=1,status=OK,mask=0x5)",
          any(a["cmd_code"] == 1 and a["status"] == 0 and a["mask"] == 0x5 for a in acks), str(acks[-1:] or ""))
    check("切换后 WAVE 掩码=0x5 通道数=2", waves and waves[-1]["mask"] == 0x5 and len(waves[-1]["vals"]) == 2)
    ser.reset_input_buffer()
    dec = StpStreamDecoder()
    ser.write(b"telem mask 0xFFFFFFFF\n")
    t0 = time.time()
    while time.time() - t0 < 0.4:
        dec.feed(ser.read(8192))
    acks = dec.pop_acks()
    waves = dec.pop_waves()
    check("超 16 通道掩码被拒绝: ACK LIMITED 且保持 0x5",
          any(a["cmd_code"] == 1 and a["status"] == 2 and a["mask"] == 0x5 for a in acks), str(acks[-1:] or ""))
    check("被拒后 WAVE 掩码仍为 0x5", waves and waves[-1]["mask"] == 0x5)

    print("\n--- [4] 动态速率切换 + ACK ---")
    for rate, expect_ok in ((100, 0), (250, 0), (333, 2), (500, 0)):
        ser.reset_input_buffer()
        dec = StpStreamDecoder()
        ser.write(("telem rate %d\n" % rate).encode())
        t0 = time.time()
        while time.time() - t0 < 1.2:
            dec.feed(ser.read(8192))
        acks = dec.pop_acks()
        waves = [w for w in dec.pop_waves()]
        ack = next((a for a in acks if a["cmd_code"] == 2), None)
        eff = ack["rate_hz"] if ack else 0
        # 用 MCU tick 估算真实帧率（去掉前 0.2s 切换暂态）
        ws = [w for w in waves if w["tick"] >= waves[0]["tick"] + 200] if waves else []
        span = (ws[-1]["tick"] - ws[0]["tick"]) / 1000.0 if len(ws) > 2 else 0
        meas = (len(ws) - 1) / span if span else 0
        check("rate %d -> ACK status=%d eff=%dHz" % (rate, expect_ok, eff),
              ack is not None and ack["status"] == expect_ok and eff >= rate and eff <= rate * 1.02 + 1,
              "ack=%s" % ack)
        check("rate %d 实测帧率 %.0fHz ≈ %dHz" % (rate, meas, eff), eff and abs(meas - eff) <= max(3, eff * 0.03))

    print("\n--- [5] 波形流开启时 CLI 文本交错 ---")
    cli(ser, "telem mask 0x040001FF")
    ser.reset_input_buffer()
    dec = StpStreamDecoder()
    for _ in range(5):
        ser.write(b"version\n")
        t0 = time.time()
        while time.time() - t0 < 0.15:
            dec.feed(ser.read(8192))
    dec.flush_idle()
    txt = dec.pop_text()
    n_ver = txt.count("firmware=FOC_G431")
    waves = dec.pop_waves()
    check("5 次 version 回显全部完整", n_ver == 5, "got %d" % n_ver)
    check("交错期间波形不失步 (crc_err=0, desync=0)", dec.crc_errors == 0 and dec.desync == 0,
          "crc_err=%d desync=%d waves=%d" % (dec.crc_errors, dec.desync, len(waves)))
    lines_with_fw = [l for l in txt.splitlines() if "firmware=" in l]
    check("行尾 \\r\\n 保留（firmware= 均独占一行，无合并行）",
          len(lines_with_fw) == 5 and all(l.startswith("firmware=") for l in lines_with_fw))

    print("\n--- [6] 快环 CPU 占用（16 通道全速） ---")
    cli(ser, "telem mask 0xFFFF")
    time.sleep(0.3)
    cur, mx = None, None
    for _ in range(3):
        st = cli(ser, "status", wait=0.4)
        cur, mx = cpu_of(st)
        if cur is not None:
            break
    check("status 含 cpu 字段", cur is not None, "cur=%s max=%s" % (cur, mx))
    if cur is not None:
        check("16 通道遥测下快环负载 < 60%%（cur=%.1f%% max=%.1f%%）" % (cur, mx), max(cur, mx) < 60.0)

    # 恢复默认
    cli(ser, "telem enable 0")
    cli(ser, "telem mask 0x040001FF")
    cli(ser, "telem rate 500")
    ser.close()

    print("\nResult: %d passed, %d failed" % (passed, failed))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
