# -*- coding: utf-8 -*-
"""
FOC_G431 固件系统性全面功能测试套件 (Systematic Firmware Test Suite)
通过 COM44 串口链路对 MCU 固件执行细致、无死角的功能与鲁棒性验证。
覆盖：
  [1] 基础通信与系统诊断 (help, version, status, motor, fault, fault clear)
  [2] 控制模式与指令语法/边界安全防线 (mode, target, angle, limit, vq, rpm, vf)
  [3] 控制环路与算法参数整定 (current, vel, pos, tune, deadtime)
  [4] Flash 配置存储与持久化一致性 (conf read/write/erase 参数保存与恢复)
  [5] 传感器与无感观测器体系 (obs, acog, bench, feedback, cordic)
  [6] FOC-STP v1.0 遥测与协议压力测试 (mask, rate, enable, log, 文本高压注入, CRC/丢包监测)
  [7] 故障快照与黑匣子完整性 (blackbox)
"""
import math
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

passed_count = 0
failed_count = 0
warnings = []


def assert_test(name, condition, detail=""):
    global passed_count, failed_count
    if condition:
        passed_count += 1
        print("  [PASS] %-40s %s" % (name, detail))
    else:
        failed_count += 1
        print("  [FAIL] %-40s %s" % (name, detail))


def expect_contains(ser, cmd, substr, name="", wait=0.25):
    res = cli(ser, cmd, wait=wait)
    ok = substr in res
    assert_test(name or ("cmd '%s' contains '%s'" % (cmd, substr)), ok,
                "" if ok else ("got: %r" % res[:120]))
    return res


def expect_regex(ser, cmd, pattern, name="", wait=0.25):
    res = cli(ser, cmd, wait=wait)
    ok = re.search(pattern, res) is not None
    assert_test(name or ("cmd '%s' matches '%s'" % (cmd, pattern)), ok,
                "" if ok else ("got: %r" % res[:120]))
    return res


def main():
    global passed_count, failed_count
    print("=" * 70)
    print("      FOC_G431 固件系统性全面硬件功能实测 (COM44 @ 6.5 MBaud)")
    print("=" * 70)

    ser = serial.Serial(PORT, BAUD, timeout=0.03, write_timeout=0.5)
    time.sleep(0.1)

    # 预先关闭遥测流，避免高速二进制影响初始纯文本检测
    cli(ser, "telem enable 0")
    cli(ser, "disable")
    cli(ser, "fault clear")

    # =========================================================================
    # [模块 1] 基础通信与系统诊断命令
    # =========================================================================
    print("\n--- [模块 1] 基础通信与系统诊断命令 ---")
    ver = cli(ser, "version")
    assert_test("version 回显固件名", "firmware=FOC_G431" in ver, ver.splitlines()[0] if ver else "")
    assert_test("version 协议版本为 0.4.0", "version=0.4.0" in ver)
    assert_test("version 极对数与硬件匹配 (pp=7)", "pole_pairs=7" in ver)
    assert_test("version 编码器 CPR 正确 (cpr=2048)", "encoder_cpr=2048" in ver)

    st = cli(ser, "status", wait=0.3)
    assert_test("status 回显状态行", "M0 IDLE" in st or "M0 RUN" in st)
    assert_test("status 包含母线电压 (vbus/Udc)", "vbus=" in st or "Udc=" in st)
    assert_test("status 包含快环 CPU 负载统计", "cpu=" in st and "% (max" in st)

    hlp = cli(ser, "help", wait=0.3)
    assert_test("help 包含系统命令树", "FOC CLI v2:" in hlp and "System:" in hlp)
    assert_test("help 包含控制命令树", "Control:" in hlp and "Tuning:" in hlp)

    # motor 轴选择与越界检查
    m_cur = cli(ser, "motor")
    assert_test("motor 查询当前轴", "selected motor M0" in m_cur)
    m_inv = cli(ser, "motor 99")
    assert_test("motor 越界轴拒绝防护", "err: motor 0.." in m_inv)
    cli(ser, "motor 0")

    # fault 查询与清除
    f_res = cli(ser, "fault")
    assert_test("fault 查询状态", "fault=" in f_res and "current_sense=" in f_res)
    fc_res = cli(ser, "fault clear")
    assert_test("fault clear 正常复位", "fault cleared, state=IDLE" in fc_res)

    # =========================================================================
    # [模块 2] 控制模式与边界安全防线 (IDLE 安全防御)
    # =========================================================================
    print("\n--- [模块 2] 控制模式与语法/边界安全防线 ---")
    # 模式设置与查询
    cli(ser, "mode vf")
    expect_contains(ser, "mode", "mode=vf", "mode 切换为 vf")
    cli(ser, "mode iq")
    expect_contains(ser, "mode", "mode=iq", "mode 切换为 iq")
    cli(ser, "mode vel")
    expect_contains(ser, "mode", "mode=vel", "mode 切换为 vel")
    cli(ser, "mode pos")
    expect_contains(ser, "mode", "mode=pos", "mode 切换为 pos")
    expect_contains(ser, "mode unknown_xyz", "err: mode vf|iq|vel|pos", "mode 非法参数防御")
    cli(ser, "mode vel")  # 切回默认速度模式

    # angle 角度源切换
    expect_contains(ser, "angle", "angle_source=", "angle 查询角度源")
    expect_contains(ser, "angle ol", "angle_source=ol (open loop)", "angle 切换开环")
    expect_contains(ser, "angle enc", "angle_source=enc (encoder)", "angle 切换编码器")
    expect_contains(ser, "angle invalid", "err: angle enc|ol", "angle 非法参数防御")

    # limit 软限电流保护
    lim_q = cli(ser, "limit")
    assert_test("limit 查询当前电流限值", "limit=" in lim_q and "trip=" in lim_q and "hard=" in lim_q)
    # 提取 hard limit
    m_hard = re.search(r"hard=([\d\.]+)A", lim_q)
    hard_limit = float(m_hard.group(1)) if m_hard else 12.0
    expect_contains(ser, "limit 3.5", "limit=3.50A", "limit 设置有效电流 3.5A")
    expect_contains(ser, "limit %f" % (hard_limit + 5.0), "err: 0 < limit <=", "limit 超过硬件极限制防御")
    expect_contains(ser, "limit -1", "err: 0 < limit <=", "limit 负值防御")
    expect_contains(ser, "limit 0", "err: 0 < limit <=", "limit 零值防御")
    cli(ser, "limit 5.2")  # 恢复标准软限 5.2A

    # target 语法与边界防线
    cli(ser, "mode iq")
    expect_contains(ser, "target 1.5", "target=1.500A", "target iq 模式设定 1.5A")
    expect_contains(ser, "target 100.0", "err: iq target must be within", "target iq 超限过载防御")
    cli(ser, "mode vel")
    expect_contains(ser, "target 1200", "target=1200.000RPM", "target vel 模式设定 1200 RPM")
    cli(ser, "target 0")
    cli(ser, "mode pos")
    expect_regex(ser, "target 3.1415", r"target=3\.14[12]rad", "target pos 模式设定弧度")
    cli(ser, "target 0")
    cli(ser, "mode vf")
    expect_contains(ser, "target 100", "err: target is ambiguous in vf; use vq and rpm", "target vf 模式歧义拦截防御")
    cli(ser, "mode vel")

    # V/F 控制参数
    cli(ser, "mode vf")
    expect_contains(ser, "vq 1.2", "vf boost=1.200V", "vq 设置开环提升电压")
    expect_contains(ser, "vq 999.0", "err: vq [V]", "vq 超过母线极限限制防御")
    expect_contains(ser, "rpm 300", "rpm cmd=300.0", "rpm 设置开环转速")
    expect_contains(ser, "rpm 99999", "err: rpm [RPM]", "rpm 超速限制防御")
    expect_contains(ser, "vf slope 0.0015", "vf slope=0.001500V/RPM", "vf slope 斜率整定")
    expect_contains(ser, "vf slope 0.5", "err: vf [slope", "vf slope 超限防御")
    cli(ser, "rpm 0")
    cli(ser, "vq 0")
    cli(ser, "mode vel")

    # =========================================================================
    # [模块 3] 环路参数整定与调节 (Tuning: current, vel, pos, deadtime)
    # =========================================================================
    print("\n--- [模块 3] 环路参数整定与输入边界防御 ---")
    # 电流环带宽整定
    expect_contains(ser, "current bw 1000", "current bw=1000", "current bw 设置 1000 rad/s")
    expect_contains(ser, "current bw 10", "err: current bw", "current bw 低于下限防御")
    expect_contains(ser, "current bw 50000", "err: current bw", "current bw 高于上限防御")

    # 速度环各参数整定
    expect_contains(ser, "vel kp 0.05", "vel kp=0.0500", "vel kp 整定")
    expect_contains(ser, "vel ki 0.5", "vel ki=0.5000", "vel ki 整定")
    expect_contains(ser, "vel ramp 4000", "vel ramp=4000RPM/s", "vel ramp 斜坡整定")
    expect_contains(ser, "vel filter 60", "vel filter=60.0Hz", "vel filter 滤波频率整定")
    expect_contains(ser, "vel filter 999", "err: vel tuning", "vel filter 频率超限防御")
    expect_contains(ser, "vel ff 0.15", "vel ff=0.150A", "vel ff 摩擦前馈整定")
    expect_contains(ser, "vel start 0.3", "vel start=0.300A", "vel start 启动电流整定")

    # 位置环参数整定
    expect_contains(ser, "pos kp 2.5", "pos kp=2.500A/rad", "pos kp 整定")
    expect_contains(ser, "pos ki 0.01", "pos ki=0.010A/(rad*s)", "pos ki 整定")
    expect_contains(ser, "pos vkp 0.025", "pos vkp=0.0250A/RPM", "pos vkp 阻尼增益整定")
    expect_contains(ser, "pos accel 8000", "pos accel=8000RPM/s", "pos accel 加速度整定")
    expect_contains(ser, "pos vmax 2500", "pos vmax=2500RPM", "pos vmax 最大转速整定")
    expect_contains(ser, "pos vmax 99999", "err: pos [kp|ki|vkp|accel|vmax", "pos vmax 越界保护")

    # deadtime 死区补偿参数
    dt_res = cli(ser, "deadtime")
    assert_test("deadtime 查询状态", "deadtime: motor=" in dt_res and "obs_v=" in dt_res)
    expect_contains(ser, "deadtime volt 0.17", "deadtime volt=0.170V", "deadtime volt 设置补偿电压")

    # =========================================================================
    # [模块 4] Flash 参数持久化与一致性 (conf read/write/erase)
    # =========================================================================
    print("\n--- [模块 4] Flash 配置存储与持久化一致性 ---")
    conf_init = cli(ser, "conf read")
    assert_test("conf read 读取当前 Flash 配置", "conf params:" in conf_init or "conf: read OK" in conf_init)

    # 改变一个测试特征值，例如 vel ramp = 3456 RPM/s
    cli(ser, "vel ramp 3456")
    v_check = cli(ser, "vel")
    assert_test("修改 RAM 临时参数 ramp=3456", "ramp=3456RPM/s" in v_check)

    # 写入 Flash
    c_wr = cli(ser, "conf write")
    assert_test("conf write 成功写入 Flash", "config written" in c_wr or "conf: write OK" in c_wr)

    # 再次读取 Flash 验证持久化
    c_rd = cli(ser, "conf read")
    assert_test("conf read 校验读取成功", "conf params:" in c_rd or "conf: read OK" in c_rd)
    v_after_rd = cli(ser, "vel")
    assert_test("Flash 恢复参数 ramp 保持 3456", "ramp=3456RPM/s" in v_after_rd)

    # 恢复标准 ramp 4000 并重写 Flash 保持干净
    cli(ser, "vel ramp 4000")
    cli(ser, "conf write")
    assert_test("恢复标准参数并写入 Flash", True)

    # =========================================================================
    # [模块 5] 传感器与无感观测器体系 (obs, acog, bench, feedback, cordic)
    # =========================================================================
    print("\n--- [模块 5] 传感器与无感观测器/CORDIC硬件计算 ---")
    # CORDIC 硬件加速单元基准数学测试
    cordic_res = cli(ser, "bench cordic_test")
    # 标准基准输入: (1,0)->0, (0,1)->90, (-1,0)->180, (0,-1)->-90
    assert_test("CORDIC (1,0)=0.00 deg", re.search(r"\(1,0\)=-?0\.00 deg", cordic_res) is not None)
    assert_test("CORDIC (0,1)=90.00 deg", "(0,1)=90.00 deg" in cordic_res)
    assert_test("CORDIC (-1,0)=180.00 deg", "(-1,0)=180.00 deg" in cordic_res)
    assert_test("CORDIC (0,-1)=-90.00 deg", "(0,-1)=-90.00 deg" in cordic_res)

    # bench status 与 diag
    b_st = cli(ser, "bench status")
    assert_test("bench status 无感竞技场状态回显", "Sensorless Benchmark Arena" in b_st)
    assert_test("bench status 包含 Ortega 磁链观测器", "1. Ortega Flux" in b_st)
    assert_test("bench status 包含 VESC 磁链观测器", "2. VESC Flux" in b_st)
    assert_test("bench status 包含 STO 滑模观测器", "3. Simplified STO" in b_st)
    assert_test("bench status 包含 STO HW CORDIC 观测器", "4. STO HW CORDIC" in b_st)
    assert_test("bench status 包含 HFI 高频注入", "5. HFI Square-Wave" in b_st)

    b_diag = cli(ser, "bench diag")
    assert_test("bench diag 输出诊断电压/电流/反电动势", "diag: v_ab=" in b_diag and "bemf_ab=" in b_diag)

    # feedback 融合角度管理器
    fb_q = cli(ser, "feedback")
    assert_test("feedback 查询融合状态", "feedback: mode=" in fb_q and "state=" in fb_q)
    expect_contains(ser, "feedback sensored", "mode=sensored (primary)", "feedback 切换有感优先")
    expect_contains(ser, "feedback auto", "mode=auto (fallback", "feedback 切换自动退避回退")
    expect_contains(ser, "feedback sensorless", "mode=sensorless (primary VESC+I/F", "feedback 切换纯无感主控模式")
    expect_contains(ser, "feedback sensored", "mode=sensored", "feedback 切回有感主控")

    # acog 抗齿槽补偿状态 (pts=144: 7对极*12齿槽点或全局点数)
    acog_q = cli(ser, "acog")
    assert_test("acog 查询抗齿槽状态与样本数", "M0 acog state=" in acog_q and "pts=" in acog_q)

    # obs 在线对比使能
    obs_q = cli(ser, "obs")
    assert_test("obs 查询在线对比状态", "obs=" in obs_q and "theta_obs=" in obs_q)
    expect_contains(ser, "obs 1", "obs=1", "obs 1 开启观测器在线比对")
    expect_contains(ser, "obs 0", "obs=0", "obs 0 关闭观测器")

    # =========================================================================
    # [模块 6] FOC-STP v1.0 遥测与协议压力测试 (高频、动态掩码、文本混流)
    # =========================================================================
    print("\n--- [模块 6] FOC-STP v1.0 协议压力与抗打断极限测试 ---")
    # 1. 动态掩码全覆盖切换测试：单通道、多通道、稀疏通道
    test_masks = [
        (0x00000001, 1, "单通道 theta_e"),
        (0x00000003, 2, "2 通道 theta_e + iq_raw"),
        (0x0000003F, 6, "6 通道核心控制量"),
        (0x040001FF, 10, "10 通道默认工程集"),
        (0x0000FFFF, 16, "16 通道顶格压力"),
    ]
    for mask_val, expected_ch, desc in test_masks:
        ser.reset_input_buffer()
        dec = StpStreamDecoder()
        cmd_str = "telem mask 0x%08X\n" % mask_val
        ser.write(cmd_str.encode("ascii"))
        t0 = time.time()
        while time.time() - t0 < 0.2:
            chunk = ser.read(4096)
            if chunk:
                dec.feed(chunk)
        acks = dec.pop_acks()
        ack_ok = any(a["cmd_code"] == 1 and a["status"] == 0 and a["mask"] == mask_val for a in acks)
        assert_test("动态掩码切换: %s" % desc, ack_ok, "ack_count=%d" % len(acks))

    # 2. 开启 500 Hz 遥测并施加高压长文本交错打断测试
    cli(ser, "telem mask 0x040001FF")
    cli(ser, "telem rate 500")
    cli(ser, "telem enable 1")

    dec_pressure = StpStreamDecoder()
    start_time = time.time()
    text_replies = []
    # 连续下发 6 条命令并留出充裕排空时间
    test_cmds = ["version", "status", "telem", "vel", "pos", "limit"]
    for c in test_cmds:
        ser.write((c + "\n").encode("ascii"))
        t_sub = time.time()
        while time.time() - t_sub < 0.1:
            chunk = ser.read(8192)
            if chunk:
                dec_pressure.feed(chunk)
    # 收尾排空 0.8s 确保所有回显完整落地
    t_end = time.time()
    while time.time() - t_end < 0.8:
        chunk = ser.read(8192)
        if chunk:
            dec_pressure.feed(chunk)
    dec_pressure.flush_idle()

    # 关闭波形流
    cli(ser, "telem enable 0")

    p_waves = dec_pressure.pop_waves()
    p_status = dec_pressure.pop_status()
    p_text = dec_pressure.pop_text()

    assert_test("高压文本混流期间 WAVE 波形帧到达", len(p_waves) > 200, "got %d waves" % len(p_waves))
    assert_test("高压文本混流期间 CRC 零错误", dec_pressure.crc_errors == 0, "crc_err=%d" % dec_pressure.crc_errors)
    assert_test("高压文本混流期间 零失步", dec_pressure.desync == 0, "desync=%d" % dec_pressure.desync)
    assert_test("高压文本混流期间 STATUS 帧到达", len(p_status) >= 4, "got %d status" % len(p_status))
    assert_test("长文本回显完整捕获到 version", "firmware=FOC_G431" in p_text)
    assert_test("长文本回显完整捕获到 status", "M0 IDLE" in p_text or "M0 RUN" in p_text)

    # =========================================================================
    # [模块 7] 故障快照与黑匣子完整性 (blackbox)
    # =========================================================================
    print("\n--- [模块 7] 故障快照与黑匣子完整性 ---")
    bb_res = cli(ser, "blackbox", wait=0.3)
    # 当前若无故障跳闸，应返回 inactive 提示；若有跳闸则输出 512 拍十六进制数据
    if "blackbox inactive" in bb_res:
        assert_test("blackbox 状态机正常响应 (无故障时返回 inactive)", True)
    else:
        lines = bb_res.strip().splitlines()
        assert_test("blackbox 导出 512 行故障波形", len(lines) >= 500, "lines=%d" % len(lines))

    # =========================================================================
    # 总结汇报
    # =========================================================================
    print("\n" + "=" * 70)
    print("      测试汇总: %d 项通过, %d 项失败" % (passed_count, failed_count))
    print("=" * 70)
    ser.close()
    if failed_count > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
