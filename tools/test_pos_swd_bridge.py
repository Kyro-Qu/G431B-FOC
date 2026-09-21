# -*- coding: utf-8 -*-
"""
基于 SWD 内存注入与高速遥测的位置环控制实测脚本
绕过串口物理连线，直接通过 SWD 通道下发 CLI 命令并采集电机运行数据。
"""
import time
import struct
import sys
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_QUEUE_HEAD_ADDR = 0x2000008c
RX_QUEUE_TAIL_ADDR = 0x2000008e
RX_QUEUE_BUF_ADDR = 0x200034fc
CMD_RX_QUEUE_SIZE = 256

OFF_STATE = 160
OFF_MODE = 164
OFF_CALIB_VALID = 208 # uint8: valid (bit 0), dir (int8 at +1), from_store (uint8 at +2)
OFF_TARGET = 216      # float
OFF_IQ_REF = 252      # float
OFF_THETA_M = 284     # float
OFF_POS = 288         # float
OFF_VEL_OBS = 296     # float
OFF_VEL_FILT = 300    # float

def inject_cmd(target, cmd_str):
    if not cmd_str.endswith('\n'):
        cmd_str += '\n'
    cmd_bytes = cmd_str.encode('ascii')

    # 读 head 和 tail
    head_tail = target.read32(RX_QUEUE_HEAD_ADDR)
    head = head_tail & 0xFFFF
    tail = (head_tail >> 16) & 0xFFFF

    for b in cmd_bytes:
        target.write8(RX_QUEUE_BUF_ADDR + head, b)
        head = (head + 1) % CMD_RX_QUEUE_SIZE

    target.write16(RX_QUEUE_HEAD_ADDR, head)
    # 等待 tail 赶上 head (命令被消费)
    t0 = time.time()
    while time.time() - t0 < 1.0:
        cur_tail = target.read16(RX_QUEUE_TAIL_ADDR)
        if cur_tail == head:
            break
        time.sleep(0.01)

def read_motor_status(target):
    # 一次性读取 160 字节覆盖 state 到 vel_filt
    data = bytes(target.read_memory_block8(MOTOR_BASE + OFF_STATE, 150))
    state = struct.unpack_from('<I', data, 0)[0]
    mode = struct.unpack_from('<I', data, OFF_MODE - OFF_STATE)[0]
    calib_valid = data[OFF_CALIB_VALID - OFF_STATE]
    target_val = struct.unpack_from('<f', data, OFF_TARGET - OFF_STATE)[0]
    pos = struct.unpack_from('<f', data, OFF_POS - OFF_STATE)[0]
    vel_obs = struct.unpack_from('<f', data, OFF_VEL_OBS - OFF_STATE)[0]
    vel_filt = struct.unpack_from('<f', data, OFF_VEL_FILT - OFF_STATE)[0]

    # 读 iq_ref
    iq_ref = struct.unpack_from('<f', data, 40)[0] # 216 - 160 = 56, iq_ref 在周围

    return {
        'state': state,
        'mode': mode,
        'calib_valid': calib_valid,
        'target': target_val,
        'pos': pos,
        'vel_obs': vel_obs,
        'vel_filt': vel_filt
    }

print("=== 1. 连接 SWD 调试通道 ===")
with ConnectHelper.session_with_chosen_probe(target_override='cortex_m') as session:
    target = session.target
    print("SWD 连接成功！")

    # 检查校准状态
    st = read_motor_status(target)
    print(f"当前电机状态: state={st['state']} (0:IDLE, 1:RUN, 2:CALIB, 3:FAULT), mode={st['mode']}, calib_valid={st['calib_valid']}")

    if st['state'] == 3:
        print("清除故障...")
        inject_cmd(target, "fault clear")
        time.sleep(0.1)

    if st['calib_valid'] == 0:
        print("未确立物理零点，正在注入 calib 命令执行校准...")
        inject_cmd(target, "calib")
        t0 = time.time()
        while time.time() - t0 < 15.0:
            st = read_motor_status(target)
            if st['calib_valid'] != 0 and st['state'] == 0:
                print(">>> 校准成功完成！物理零点确立！")
                break
            time.sleep(0.3)
        else:
            print("校准超时，退出")
            sys.exit(1)

    print("\n=== 2. 配置位置环基线参数并进入 POS 模式 ===")
    inject_cmd(target, "mode pos")
    inject_cmd(target, "pos kp 3.00")
    inject_cmd(target, "pos ki 0.15")
    inject_cmd(target, "pos vkp 0.006")
    inject_cmd(target, "pos vmax 80")
    inject_cmd(target, "pos accel 80")
    inject_cmd(target, "target 0.000")
    time.sleep(0.1)

    print("\n=== 3. 使能电机 (Enable in POS Mode) ===")
    inject_cmd(target, "enable")
    time.sleep(0.2)
    st = read_motor_status(target)
    print(f"使能后状态: state={st['state']} (1=RUN 为成功使能)")

    print("\n--- 4. 静态死区保持测试 (Target 0.000 rad，保持 1.5s) ---")
    print("验证目标：阻尼项归零(damp_factor=0)，微分毛刺斩断，0 高频蜂鸣，位置保持稳定")
    t0 = time.time()
    while time.time() - t0 < 1.5:
        st = read_motor_status(target)
        err = st['target'] - st['pos']
        print(f"  [静态保持 t={time.time()-t0:.2f}s] pos={st['pos']:+.4f} rad (err={err:+.4f} rad, {err*57.3:+.2f}°), vel_obs={st['vel_obs']:+.2f} RPM")
        time.sleep(0.15)

    print("\n--- 5. 小步长阶跃响应测试: target +0.20 rad (11.5°) ---")
    print("下发 target 0.200...")
    inject_cmd(target, "target 0.200")
    t0 = time.time()
    while time.time() - t0 < 1.5:
        st = read_motor_status(target)
        err = st['target'] - st['pos']
        print(f"  [小阶跃 t={time.time()-t0:.2f}s] pos={st['pos']:+.4f} rad, err={err:+.4f} rad ({err*57.3:+.2f}°), vel_obs={st['vel_obs']:+.2f} RPM")
        time.sleep(0.10)

    print("\n--- 6. 90° 大步长阶跃测试: target +1.5708 rad (90.0°) ---")
    print("下发 target 1.5708 (验证 0.8A 安全限幅下的平稳制动，严禁狂甩)...")
    inject_cmd(target, "target 1.5708")
    t0 = time.time()
    while time.time() - t0 < 2.2:
        st = read_motor_status(target)
        err = st['target'] - st['pos']
        print(f"  [90°阶跃 t={time.time()-t0:.2f}s] pos={st['pos']:+.4f} rad, err={err:+.4f} rad ({err*57.3:+.2f}°), vel_obs={st['vel_obs']:+.2f} RPM")
        time.sleep(0.10)

    print("\n--- 7. -180° 反向大步长阶跃测试: target -1.5708 rad (-90.0°) ---")
    print("下发 target -1.5708...")
    inject_cmd(target, "target -1.5708")
    t0 = time.time()
    while time.time() - t0 < 2.5:
        st = read_motor_status(target)
        err = st['target'] - st['pos']
        print(f"  [-90°阶跃 t={time.time()-t0:.2f}s] pos={st['pos']:+.4f} rad, err={err:+.4f} rad ({err*57.3:+.2f}°), vel_obs={st['vel_obs']:+.2f} RPM")
        time.sleep(0.10)

    print("\n--- 8. 回原点: target 0.000 rad ---")
    inject_cmd(target, "target 0.000")
    time.sleep(1.2)
    st = read_motor_status(target)
    err = st['target'] - st['pos']
    print(f"  [回原点最终稳态] pos={st['pos']:+.4f} rad, err={err:+.4f} rad ({err*57.3:+.2f}°), vel_obs={st['vel_obs']:+.2f} RPM")

    print("\n=== 9. 安全停机 (Disable) ===")
    inject_cmd(target, "disable")
    time.sleep(0.2)
    st = read_motor_status(target)
    print(f"停机后状态: state={st['state']} (0=IDLE)")
    print("\n>>> 全流程测试圆满完成！")
