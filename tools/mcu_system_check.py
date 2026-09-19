# -*- coding: utf-8 -*-
"""
MCU 系统级功能检验（COM44 @ 6.5 MBaud）— 2026-09-19

覆盖：
 A. 链路/协议：version、10Hz STATUS、500Hz WAVE、掩码/速率 ACK
 B. CLI 覆盖：help 列出的每个查询型命令都必须有回显、不报 unknown
 C. 保护：limit / vbus uv/ov 设定与回读；未校准闭环 enable 应 fault=6；欠压保护应 fault=11；fault clear 恢复
 D. 校准：calib → calib=1 且 offset 合理；conf read 可读
 E. 四种模式：VF 200rpm；IQ 0.3A（Id≈0）；VEL 300/1000/-500（稳态误差）；POS ±3.14（到位误差）
 F. 自适应 CLI：wave 0 且 RUN 时有 [VEL]/[POS] 行；wave 1 时终端静默
 G. 急停：RUN 中 disable 后 <300ms 到 IDLE
 H. 收尾：参数恢复默认、M0 IDLE fault=0
"""
import os
import re
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from foc_stp import StpStreamDecoder, cli  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

PORT, BAUD = "COM44", 6500000
P = F = 0
FAILS = []


def check(label, cond, detail=""):
    global P, F
    if cond:
        P += 1
    else:
        F += 1
        FAILS.append(label)
    print(("  PASS  " if cond else "  FAIL  ") + label + ("  " + detail if detail else ""))


def num(txt, pat, default=None):
    m = re.search(pat, txt)
    return float(m.group(1)) if m else default


def cpu_peak(ser, reset=False):
    """读取快环峰值负载（%），reset=True 时先清零上电以来的峰值"""
    r = cli(ser, "cpu reset" if reset else "cpu", wait=0.25)
    return num(r, r"max=([0-9.]+)%")


def wave_capture(ser, secs):
    ser.reset_input_buffer()
    dec = StpStreamDecoder()
    t0 = time.time()
    while time.time() - t0 < secs:
        dec.feed(ser.read(8192))
    dec.flush_idle()
    return dec


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.02, write_timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.1)
    cli(ser, "telem enable 0")
    cli(ser, "wave 0")
    boot_peak = cpu_peak(ser, reset=True)
    print("  info  上电以来快环峰值 %.1f%%（已清零，后续分段统计）" % (boot_peak or 0))
    PEAKS = {}

    def phase_peak(name):
        PEAKS[name] = cpu_peak(ser, reset=True)

    # ---------------- A. 链路/协议 ----------------
    print("=== A. 链路与协议 ===")
    ver = cli(ser, "version", wait=0.3)
    check("version 回显", "firmware=FOC_G431" in ver, ver.splitlines()[0][:70] if ver else "")
    dec = wave_capture(ser, 1.0)
    st = dec.pop_status()
    check("10Hz STATUS 心跳（1s 内 8~12 帧）", 8 <= len(st) <= 12, "n=%d" % len(st))
    if st:
        s = st[-1]
        check("STATUS 10B 字段合理 vbus/state/cpu", 10 <= s["vbus"] <= 28 and s["state"] in (0, 1, 2, 3) and 0 <= s["cpu_pct"] <= 100,
              "vbus=%.2f state=%d cpu=%d%%" % (s["vbus"], s["state"], s["cpu_pct"]))
    cli(ser, "telem mask 0x040001FF")
    cli(ser, "telem rate 500")
    cli(ser, "telem enable 1")
    dec = wave_capture(ser, 1.0)
    w = dec.pop_waves()
    cli(ser, "telem enable 0")
    seq_ok = all(((w[i + 1]["seq"] - w[i]["seq"]) & 0xFFFF) == 1 for i in range(len(w) - 1))
    check("500Hz WAVE（1s 内 470~530 帧）且序号连续、CRC=0", 470 <= len(w) <= 530 and seq_ok and dec.crc_errors == 0,
          "n=%d crc_err=%d desync=%d" % (len(w), dec.crc_errors, dec.desync))
    phase_peak("A.协议")

    # ---------------- B. CLI 覆盖 ----------------
    print("\n=== B. CLI 命令覆盖（查询型） ===")
    queries = ["help", "status", "motor", "fault", "calib", "calib offset", "mode", "angle", "vq", "rpm", "vf",
               "limit", "vbus", "current", "tune", "vel", "pos", "ident show", "acog", "obs", "conf read",
               "wave", "log", "telem", "feedback"]
    bad = []
    for q in queries:
        r = cli(ser, q, wait=0.3)
        if not r or re.search(r"unknown|invalid|err:", r, re.I):
            bad.append("%s->%r" % (q, r[:40]))
    check("%d 个查询命令均有回显且无 unknown/err" % len(queries), not bad, "; ".join(bad))
    r = cli(ser, "definitely_not_a_cmd", wait=0.3)
    check("未知命令有明确错误提示", bool(r) and re.search(r"unknown|help", r, re.I) is not None, r[:60])
    phase_peak("B.CLI")

    # ---------------- C. 保护 ----------------
    print("\n=== C. 保护与限幅 ===")
    r0 = cli(ser, "limit", wait=0.3)
    lim0 = num(r0, r"limit=([0-9.]+)")
    cli(ser, "limit 3.0")
    r = cli(ser, "limit", wait=0.3)
    check("limit 设定 3.0A 可回读", abs((num(r, r"limit=([0-9.]+)") or 0) - 3.0) < 0.01, r)
    cli(ser, "limit %g" % lim0)
    r = cli(ser, "vbus", wait=0.3)
    uv0, ov0 = num(r, r"uv=([0-9.]+)"), num(r, r"ov=([0-9.]+)")
    vb = num(r, r"vbus=([0-9.]+)")
    cli(ser, "vbus uv 12.0")
    cli(ser, "vbus ov 20.0")
    r = cli(ser, "vbus", wait=0.3)
    check("vbus uv/ov 设定可回读", abs((num(r, r"uv=([0-9.]+)") or 0) - 12.0) < 0.01 and abs((num(r, r"ov=([0-9.]+)") or 0) - 20.0) < 0.01, r)

    st = cli(ser, "status", wait=0.4)
    calibrated = "calib=1" in st
    if not calibrated:
        cli(ser, "fault clear")
        cli(ser, "mode vel")
        cli(ser, "enable")
        time.sleep(0.3)
        r = cli(ser, "status", wait=0.4)
        check("未校准时闭环 enable 被拒 → fault=6", "fault=6" in r, re.search(r"fault=\d+", r).group(0) if re.search(r"fault=\d+", r) else r[:40])
        cli(ser, "disable")
        cli(ser, "fault clear")
    else:
        print("  info  已校准（calib=1*），跳过“未校准拒闭环”项")

    # 欠压保护：把 uv 门槛抬到高于实际母线，使能后应 fault=11
    if vb:
        cli(ser, "fault clear")
        cli(ser, "vbus uv %.1f" % (vb + 3.0))
        cli(ser, "mode vf")
        cli(ser, "enable")
        time.sleep(0.4)
        r = cli(ser, "status", wait=0.4)
        check("欠压保护：uv>vbus 时 enable → fault=11", "fault=11" in r, re.search(r"fault=\d+", r).group(0) if re.search(r"fault=\d+", r) else r[:40])
        cli(ser, "disable")
        cli(ser, "vbus uv %g" % uv0)
        cli(ser, "vbus ov %g" % ov0)
        cli(ser, "fault clear")
        r = cli(ser, "status", wait=0.4)
        check("fault clear 后恢复 IDLE fault=0", "M0 IDLE" in r and "fault=0" in r)
    phase_peak("C.保护")

    # ---------------- D. 校准 ----------------
    print("\n=== D. 零点校准与配置 ===")
    st = cli(ser, "status", wait=0.4)
    if "calib=1" not in st:
        cli(ser, "calib")
        ok = False
        t0 = time.time()
        while time.time() - t0 < 15:
            time.sleep(0.5)
            st = cli(ser, "status", wait=0.25)
            if "calib=1" in st:
                ok = True
                break
            if re.search(r"fault=[1-9]", st):
                break
        off = num(st, r"offset=([-0-9.]+)rad")
        check("calib 完成 calib=1 且 offset∈[0,2π)", ok and off is not None and 0 <= off < 6.2832, "offset=%s" % off)
    else:
        check("已校准（跳过重校）", True, re.search(r"offset=[-0-9.]+rad", st).group(0))
    r = cli(ser, "conf read", wait=0.5)
    check("conf read 含 pp/Rs/Ls", all(k in r for k in ("pp=", "Rs=", "Ls=")), r.splitlines()[0][:70] if r else "")
    phase_peak("D.校准")

    # ---------------- E. 四种控制模式 ----------------
    print("\n=== E. 控制模式 ===")
    # VF
    cli(ser, "mode vf")
    cli(ser, "vq 0.5")
    cli(ser, "rpm 200")
    cli(ser, "enable")
    time.sleep(1.2)
    r = cli(ser, "status", wait=0.4)
    vel = num(r, r"\bvel=([-0-9.]+)rpm")
    check("VF 强拖 200rpm：实测转速 120~260", vel is not None and 120 <= abs(vel) <= 260, "vel=%s" % vel)
    cli(ser, "disable")
    time.sleep(0.5)
    phase_peak("E1.VF")

    # IQ
    cli(ser, "mode iq")
    cli(ser, "enable")
    cli(ser, "target 0.3")
    time.sleep(0.8)
    r = cli(ser, "status", wait=0.4)
    iq, idv, iqref = num(r, r"\biq=([-0-9.]+)A"), num(r, r"\bid=([-0-9.]+)A"), num(r, r"iq_ref=([-0-9.]+)A")
    check("IQ 力矩环：iq_ref=±0.3 且 |id|<0.5A", iqref is not None and abs(abs(iqref) - 0.3) < 0.02 and idv is not None and abs(idv) < 0.5,
          "iq=%s id=%s iq_ref=%s" % (iq, idv, iqref))
    cli(ser, "target 0")
    cli(ser, "disable")
    time.sleep(0.5)
    phase_peak("E2.IQ")

    # VEL
    cli(ser, "mode vel")
    cli(ser, "vel ramp 400")
    cli(ser, "enable")
    for tgt, tol in ((300, 60), (1000, 80), (-500, 60)):
        cli(ser, "target %d" % tgt)
        time.sleep(3.5)
        cli(ser, "telem enable 1")
        dec = wave_capture(ser, 0.6)
        cli(ser, "telem enable 0")
        vs = [x["channels"]["vel_ctrl"] for x in dec.pop_waves() if "vel_ctrl" in x["channels"]]
        mean = sum(vs) / len(vs) if vs else float("nan")
        check("VEL %d rpm 稳态均值误差 < %d" % (tgt, tol), vs and abs(mean - tgt) < tol, "mean=%.1f n=%d" % (mean, len(vs)))
    cli(ser, "target 0")
    time.sleep(1.5)
    cli(ser, "disable")
    cli(ser, "vel ramp 100")
    time.sleep(0.5)
    phase_peak("E3.VEL")

    # POS
    cli(ser, "mode pos")
    cli(ser, "enable")
    for tgt in (3.14, -3.14, 0.0):
        cli(ser, "target %g" % tgt)
        time.sleep(3.0)
        r = cli(ser, "status", wait=0.4)
        perr = num(r, r"pos_err=([-0-9.]+)rad")
        check("POS target %g：到位误差 |pos_err| < 0.15rad" % tgt, perr is not None and abs(perr) < 0.15, "pos_err=%s" % perr)
    cli(ser, "disable")
    time.sleep(0.4)
    phase_peak("E4.POS")

    # ---------------- F. 自适应 CLI 遥测 ----------------
    print("\n=== F. wave 0/1 互斥的自适应 CLI 监视 ===")
    cli(ser, "mode vel")
    cli(ser, "enable")
    cli(ser, "target 300")
    cli(ser, "wave 0")
    dec = wave_capture(ser, 1.2)   # 连续读取，避免 Windows 串口驱动 4KB 缓冲溢出丢帧
    txt = dec.pop_text()
    n_vel = len(re.findall(r"\[VEL\] tgt=", txt))
    check("wave 0 且 RUN：约 5Hz 输出 [VEL] 行（1.2s 内 ≥4 行）", n_vel >= 4, "n=%d" % n_vel)
    cli(ser, "wave 1")
    dec = wave_capture(ser, 1.0)
    txt = dec.pop_text()
    nw = len(dec.pop_waves())
    check("wave 1：终端静默（无 [VEL] 行）", "[VEL]" not in txt, "text=%r" % txt[:60])
    check("wave 1：波形流 ≈500Hz（1s 内 ≥450 帧）", nw >= 450, "waves=%d" % nw)
    cli(ser, "wave 0")
    phase_peak("F.wave")

    # ---------------- G. 急停 ----------------
    print("\n=== G. 急停响应 ===")
    t0 = time.time()
    ser.write(b"disable\r\n")
    idle_at = None
    while time.time() - t0 < 1.0:
        r = cli(ser, "status", wait=0.08)
        if "M0 IDLE" in r:
            idle_at = time.time() - t0
            break
    check("RUN 中 disable → <300ms 进入 IDLE", idle_at is not None and idle_at < 0.3, "t=%.0fms" % ((idle_at or 1) * 1000))

    # ---------------- H. 收尾 ----------------
    print("\n=== H. 收尾 ===")
    cli(ser, "target 0")
    cli(ser, "mode vf")
    cli(ser, "telem enable 0")
    r = cli(ser, "status", wait=0.4)
    check("最终 M0 IDLE fault=0", "M0 IDLE" in r and "fault=0" in r)
    phase_peak("G.急停+收尾")
    worst = max(((v or 0), k) for k, v in PEAKS.items())
    print("  info  各阶段快环峰值: " + "  ".join("%s=%.0f%%" % (k, v or 0) for k, v in PEAKS.items()))
    check("各阶段快环峰值均 < 60%（16kHz 周期 62.5us）", worst[0] < 60, "worst=%s %.1f%%" % (worst[1], worst[0]))
    ser.close()

    print("\nResult: %d passed, %d failed" % (P, F))
    if FAILS:
        print("Failed: " + " | ".join(FAILS))
    sys.exit(1 if F else 0)


if __name__ == "__main__":
    main()
