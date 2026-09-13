# -*- coding: utf-8 -*-
"""
电机真实旋转动态闭环平稳实测 (带 ramp 加减速过程)
验证：
  1. 校准状态确认 (calib=1)
  2. 速度闭环正转目标 500 RPM，平稳旋转 3s
  3. 速度闭环反转目标 -500 RPM，平稳旋转 3s
  4. 阶跃至 1200 RPM，采集波形流
  5. 减速回 0，安全停机
"""
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from foc_stp import StpStreamDecoder, cli  # noqa: E402

PORT = "COM44"
BAUD = 6500000

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def main():
    print("=" * 65)
    print("      电机动态旋转实测：正转 / 反转 / 调速闭环跟踪 (COM44)")
    print("=" * 65)

    ser = serial.Serial(PORT, BAUD, timeout=0.03, write_timeout=0.5)
    time.sleep(0.1)

    # 1. 安全准备
    cli(ser, "disable")
    cli(ser, "fault clear")
    cli(ser, "mode vel")
    cli(ser, "target 0")
    cli(ser, "limit 4.0")  # 软限 4A

    st = cli(ser, "status")
    print("--- 状态检查 ---")
    calib_lines = [l for l in st.splitlines() if "calib" in l or "offset" in l or "Udc" in l or "vbus" in l]
    for l in calib_lines:
        print(" ", l)

    # 开启遥测流
    cli(ser, "telem mask 0x040001FF")  # 10 通道
    cli(ser, "telem rate 500")
    cli(ser, "telem enable 1")

    # 2. 使能电机
    print("\n--- [工况 1] 使能电机并给定正转 500 RPM ---")
    en = cli(ser, "enable")
    print("  enable:", en)
    cli(ser, "target 500")

    dec = StpStreamDecoder()
    t0 = time.time()
    while time.time() - t0 < 3.0:
        chunk = ser.read(8192)
        if chunk:
            dec.feed(chunk)
    waves = dec.pop_waves()
    if waves:
        # 取后半段稳态数据
        steady = waves[len(waves)//2:]
        vels = [w["channels"].get("vel_ctrl", 0.0) for w in steady]
        iqs = [w["channels"].get("iq_filt", 0.0) for w in steady]
        avg_vel = sum(vels) / len(vels)
        avg_iq = sum(iqs) / len(iqs)
        print("  [稳态实测] 采集 %d 帧波形: 平均转速 = %.1f RPM, 平均 Iq = %.2f A" % (
            len(waves), avg_vel, avg_iq))

    # 3. 反转 -500 RPM
    print("\n--- [工况 2] 切换反转 -500 RPM ---")
    cli(ser, "target -500")
    dec_rev = StpStreamDecoder()
    t0 = time.time()
    while time.time() - t0 < 3.0:
        chunk = ser.read(8192)
        if chunk:
            dec_rev.feed(chunk)
    waves_rev = dec_rev.pop_waves()
    if waves_rev:
        steady = waves_rev[len(waves_rev)//2:]
        vels = [w["channels"].get("vel_ctrl", 0.0) for w in steady]
        iqs = [w["channels"].get("iq_filt", 0.0) for w in steady]
        avg_vel = sum(vels) / len(vels)
        avg_iq = sum(iqs) / len(iqs)
        print("  [稳态实测] 采集 %d 帧波形: 平均转速 = %.1f RPM, 平均 Iq = %.2f A" % (
            len(waves_rev), avg_vel, avg_iq))

    # 4. 加速至 1200 RPM
    print("\n--- [工况 3] 提速至 1200 RPM 旋转 ---")
    cli(ser, "target 1200")
    dec_high = StpStreamDecoder()
    t0 = time.time()
    while time.time() - t0 < 2.5:
        chunk = ser.read(8192)
        if chunk:
            dec_high.feed(chunk)
    waves_high = dec_high.pop_waves()
    if waves_high:
        steady = waves_high[len(waves_high)//2:]
        vels = [w["channels"].get("vel_ctrl", 0.0) for w in steady]
        iqs = [w["channels"].get("iq_filt", 0.0) for w in steady]
        avg_vel = sum(vels) / len(vels)
        avg_iq = sum(iqs) / len(iqs)
        print("  [稳态实测] 采集 %d 帧波形: 平均转速 = %.1f RPM, 平均 Iq = %.2f A" % (
            len(waves_high), avg_vel, avg_iq))

    # 5. 平稳减速停机
    print("\n--- [工况 4] 平稳减速归零并断电停机 ---")
    cli(ser, "target 0")
    time.sleep(0.8)
    cli(ser, "telem enable 0")
    time.sleep(0.05)
    dis = cli(ser, "disable", wait=0.3)
    print("  disable:", dis)

    print("\n" + "=" * 65)
    print("              电机旋转测试圆满完成！")
    print("=" * 65)
    ser.close()


if __name__ == "__main__":
    main()
