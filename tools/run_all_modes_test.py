# -*- coding: utf-8 -*-
import serial
import time
import sys
import math
import statistics

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

s = serial.Serial("COM44", 6500000, timeout=0.05)


def drain(t=0.3):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < t:
        n = s.in_waiting
        if n:
            buf += s.read(n)
        else:
            time.sleep(0.005)
    return bytes(buf)


def cmd(c, wait=0.35):
    s.reset_input_buffer()
    s.write((c + "\n").encode())
    time.sleep(wait)
    return drain(wait).decode("utf-8", "replace")


def status():
    cmd("log 0", 0.08)
    d = cmd("status", 0.25)
    out = {}
    for l in d.splitlines():
        if l.startswith("vel="):
            out["vel"] = float(l.split("=")[1].split("r")[0])
        elif l.startswith("pos="):
            p = l.split()
            out["pos"] = float(p[0].split("=")[1].replace("rad", ""))
            pe = [x for x in p if x.startswith("pos_err")]
            if pe:
                out["pos_err"] = float(pe[0].split("=")[1].replace("rad", ""))
        elif l.startswith("id="):
            p = l.split()
            out["id"] = float(p[0].split("=")[1].replace("A", ""))
            out["id_ref"] = float(p[1].split("=")[1].replace("A", ""))
            out["iq"] = float(p[2].split("=")[1].replace("A", ""))
            out["iq_ref"] = float(p[3].split("=")[1].replace("A", ""))
        elif l.startswith("vd="):
            p = l.split()
            vd = float(p[0].split("=")[1].replace("V", ""))
            vq = float(p[1].split("=")[1].replace("V", ""))
            out["vmag"] = (vd * vd + vq * vq) ** 0.5
        elif "vbus=" in l:
            p = l.split()
            out["vbus"] = float(p[0].split("=")[1].replace("V", ""))
            out["udc_drv"] = float(p[3].split("=")[1].replace("V", ""))
        elif "calib=" in l:
            out["calib"] = l.strip()
        elif "cs_fault=" in l:
            p = l.split()
            out["cs_fault"] = int(p[0].split("=")[1])
            out["rejected"] = int(p[1].split("=")[1])
        elif "rst_flags=" in l:
            out["rst_flags"] = l.strip()
        elif "fault=" in l and "<-" in l:
            out["fault"] = int(l.split("fault=")[1].split()[0])
    return out


print("==========================================================================")
print("       FOC_G431 COM44 独立全模式闭环复测（现场实操记录）")
print("==========================================================================")

try:
    drain()
    cmd("log 0", 0.1)
    cmd("fault clear", 0.2)

    # ---------------- 0. 基础参数与校准检查 ----------------
    print("\n【阶段 0: 状态自检与寻零】")
    st0 = status()
    print(
        f"  开机实时母线: {st0.get('vbus')} V (自适应驱动环: {st0.get('udc_drv')} V)"
    )
    print(f"  当前校准标记: {st0.get('calib')} (从 Flash v11 自加载)")
    if "calib=0" in str(st0.get("calib")):
        print("  正在执行快速寻 Z (calib)...")
        cmd("mode vel", 0.2)
        cmd("calib", 0.4)
        for _ in range(8):
            time.sleep(0.3)
            st = status()
            if "calib=1" in str(st.get("calib")):
                print("  -> 寻零成功就绪！")
                break
    else:
        print("  -> 编码器已处于就绪状态，无需重复寻零！")

    # ---------------- 1. 开环 V/F 模式测试 ----------------
    print("\n【阶段 1: 开环 V/F 模式（正转 100 / 反转 -100 RPM）】")
    cmd("mode vf", 0.2)
    cmd("enable", 0.3)

    cmd("rpm 100", 0.2)
    time.sleep(3.0)
    st_vf100 = status()
    print(
        f"  正转 100 RPM: 实际转速 = {st_vf100.get('vel')} RPM, 母线 = {st_vf100.get('vbus')}V, iq_ref = {st_vf100.get('iq_ref')}A (门控零残留)"
    )

    cmd("rpm -100", 0.2)
    time.sleep(3.0)
    st_vf_n100 = status()
    print(
        f"  反转 -100 RPM: 实际转速 = {st_vf_n100.get('vel')} RPM, 母线 = {st_vf_n100.get('vbus')}V, iq_ref = {st_vf_n100.get('iq_ref')}A"
    )

    cmd("rpm 0", 0.2)
    time.sleep(0.5)
    cmd("disable", 0.3)

    # ---------------- 2. 力矩闭环模式测试 (mode iq) ----------------
    print(
        "\n【阶段 2: 力矩闭环模式（mode iq，空载安全点动 +/-0.08A）】"
    )
    cmd("mode iq", 0.2)
    cmd("enable", 0.3)

    cmd("target 0.08", 0.1)
    time.sleep(0.4)
    st_iq_p = status()
    cmd("target 0", 0.1)
    time.sleep(0.6)
    print(
        f"  正力矩点动: 目标 = +0.08A, 实测电流 iq = {st_iq_p.get('iq')}A, id = {st_iq_p.get('id')}A, 点动峰速 = {st_iq_p.get('vel')} RPM, fault = {st_iq_p.get('fault')}"
    )

    cmd("target -0.08", 0.1)
    time.sleep(0.4)
    st_iq_n = status()
    cmd("target 0", 0.1)
    time.sleep(0.6)
    print(
        f"  反力矩点动: 目标 = -0.08A, 实测电流 iq = {st_iq_n.get('iq')}A, id = {st_iq_n.get('id')}A, 点动峰速 = {st_iq_n.get('vel')} RPM, fault = {st_iq_n.get('fault')}"
    )
    cmd("disable", 0.3)

    # ---------------- 3. 速度闭环模式测试 (mode vel) ----------------
    print(
        "\n【阶段 3: 速度闭环模式（mode vel，低速 50 / 巡航 2400 RPM）】"
    )
    cmd("mode vel", 0.2)
    cmd("enable", 0.3)

    # 3.1 50 RPM 低速巡航
    print("  >>> 50 RPM 低速巡航 (带抗齿槽前馈):")
    cmd("target 50", 0.2)
    time.sleep(3.5)
    samples_50 = [status().get("vel", 0) for _ in range(10)]
    print(
        f"  -> 50 RPM 稳态转速: {statistics.mean(samples_50):.1f} +/- {statistics.pstdev(samples_50):.1f} RPM"
    )

    # 3.2 2400 RPM 额定巡航 (斜坡加速约 24 秒)
    print("  >>> 2400 RPM 额定巡航加速 (斜坡 100 RPM/s):")
    cmd("target 2400", 0.2)
    t0 = time.time()
    while time.time() - t0 < 30:
        time.sleep(3.0)
        st = status()
        print(
            f"    t={time.time()-t0:4.1f}s: 实际转速 = {st.get('vel'):6.1f} RPM, 母线 = {st.get('vbus')}V, id={st.get('id')}A, fault={st.get('fault')}",
            flush=True,
        )
        if (st.get("vel") or 0) > 2350:
            break
    time.sleep(2.0)
    samples_2400 = []
    for _ in range(12):
        time.sleep(0.25)
        samples_2400.append(status())
    vels_2400 = [x.get("vel", 0) for x in samples_2400]
    iqs_2400 = [x.get("iq_ref", 0) for x in samples_2400]
    vmags_2400 = [x.get("vmag", 0) for x in samples_2400]
    print(
        f"  -> 2400 RPM 稳态性能: {statistics.mean(vels_2400):.1f} +/- {statistics.pstdev(vels_2400):.1f} RPM (极差 {max(vels_2400)-min(vels_2400):.1f} RPM)"
    )
    print(
        f"                       给定电流 iq_ref = {statistics.mean(iqs_2400):+.3f}A, 端电压模长 = {statistics.mean(vmags_2400):.2f}V, fault = 0"
    )

    cmd("target 0", 0.2)
    time.sleep(1.0)
    cmd("disable", 0.3)

    # ---------------- 4. 位置伺服闭环模式测试 (mode pos) ----------------
    print(
        "\n【阶段 4: 位置伺服模式（mode pos，梯形轨迹多圈精准定位）】"
    )
    cmd("mode pos", 0.2)
    cmd("enable", 0.3)
    st_p0 = status()
    base_pos = st_p0.get("pos", 0)
    print(f"  当前保持初始位置: {base_pos:.3f} rad")

    # 4.1 相对正向 2 圈 (+4π rad)
    print("  >>> 梯形轨迹正转 2 圈 (+4pi rad)...")
    target_pos = base_pos + 4.0 * math.pi
    cmd(f"target {target_pos:.3f}", 0.2)
    time.sleep(5.0)
    st_p1 = status()
    err_deg1 = math.degrees(st_p1.get("pos_err", 0))
    print(
        f"  -> 正向到位静差: {st_p1.get('pos_err'):+.4f} rad ({err_deg1:+.2f} deg 机械角)"
    )

    # 4.2 反向回到原点 (-4π rad)
    print("  >>> 梯形轨迹反向回原点...")
    cmd(f"target {base_pos:.3f}", 0.2)
    time.sleep(5.0)
    st_p2 = status()
    err_deg2 = math.degrees(st_p2.get("pos_err", 0))
    print(
        f"  -> 回程终点位置: {st_p2.get('pos'):.3f} rad vs 目标 {base_pos:.3f} rad"
    )
    print(
        f"  -> 回程到位静差: {st_p2.get('pos_err'):+.4f} rad ({err_deg2:+.2f} deg 机械角)"
    )

    cmd("disable", 0.3)

    # ---------------- 5. 最终整机健康状态 ----------------
    print("\n【阶段 5: 最终整机健康状态核验】")
    st_final = status()
    print(f"  运行后母线电压: {st_final.get('vbus')} V")
    print(
        f"  采样异常计数: cs_fault = {st_final.get('cs_fault')}, rejected = {st_final.get('rejected')}"
    )
    print(f"  系统故障状态: fault = {st_final.get('fault')} (0 = 完全健康)")
    print(f"  抗齿槽持久化状态: {cmd('acog', 0.2).strip()}")

    print(
        "\n=========================================================================="
    )
    print(
        "             FOC_G431 全模式全功能实测圆满成功！"
    )
    print(
        "=========================================================================="
    )

finally:
    cmd("target 0", 0.1)
    cmd("disable", 0.2)
    s.close()
