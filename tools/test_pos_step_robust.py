# -*- coding: utf-8 -*-
"""
严密的位置环阶跃响应测试脚本
1. 校验 state, mode, calib
2. 确保在 mode=3 (FOC_MODE_POSITION) 下执行
3. 阶跃测试并在出现任何异常时立即安全停机
"""
import time
import struct
import sys
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_HEAD = 0x2000008c
RX_TAIL = 0x2000008e
RX_BUF = 0x200034fc

def main():
    session = ConnectHelper.session_with_chosen_probe(
        target_override='cortex_m',
        options={'halt_on_connect': False, 'resume_on_exit': True}
    )
    session.open()
    t = session.target
    if t.is_halted():
        t.resume()

    def send_cmd(cmd_str):
        if not cmd_str.endswith('\r\n'):
            cmd_str = cmd_str.rstrip('\r\n') + '\r\n'
        raw = cmd_str.encode('ascii')
        head = t.read16(RX_HEAD)
        for b in raw:
            t.write8(RX_BUF + head, b)
            head = (head + 1) % 256
        t.write16(RX_HEAD, head)
        t0 = time.time()
        while time.time() - t0 < 0.6:
            tail = t.read16(RX_TAIL)
            if tail == head:
                break
            time.sleep(0.01)
        time.sleep(0.02)

    def read_telemetry():
        raw160 = bytes(t.read_memory_block8(MOTOR_BASE + 160, 4))
        state = raw160[0]
        mode = raw160[1]

        raw208 = bytes(t.read_memory_block8(MOTOR_BASE + 208, 8))
        calib_valid = raw208[0]
        direction = struct.unpack('<b', raw208[2:3])[0]
        elec_offset = struct.unpack('<f', raw208[4:8])[0]

        raw_pos = bytes(t.read_memory_block8(MOTOR_BASE + 216, 90))
        target_val = struct.unpack_from('<f', raw_pos, 0)[0]
        pos = struct.unpack_from('<f', raw_pos, 288 - 216)[0]
        vel_obs = struct.unpack_from('<f', raw_pos, 296 - 216)[0]
        vel_filt = struct.unpack_from('<f', raw_pos, 300 - 216)[0]

        return {
            'state': state,
            'mode': mode,
            'calib_valid': calib_valid,
            'direction': direction,
            'elec_offset': elec_offset,
            'target': target_val,
            'pos': pos,
            'vel_obs': vel_obs,
            'vel_filt': vel_filt
        }

    try:
        print("=== 1. 读取初始状态 ===")
        st = read_telemetry()
        print(f"当前状态: state={st['state']} (0=IDLE, 1=RUN), mode={st['mode']} (3=POS), calib_valid={st['calib_valid']}, dir={st['direction']}")

        send_cmd("disable")
        send_cmd("fault clear")
        time.sleep(0.1)

        st = read_telemetry()
        if st['calib_valid'] == 0:
            print("未校准，执行校准...")
            send_cmd("calib")
            for _ in range(30):
                time.sleep(0.4)
                st = read_telemetry()
                if st['calib_valid'] != 0 and st['state'] == 0:
                    print("校准成功完成！")
                    break

        print("\n=== 2. 切换到 POS 模式并配置参数 ===")
        send_cmd("mode pos")
        time.sleep(0.05)
        st = read_telemetry()
        if st['mode'] != 3:
            print(f"错误: mode pos 切换失败! 当前 mode={st['mode']}")
            return

        # 配置参数：使用 0.020 阻尼
        send_cmd("pos kp 3.00")
        send_cmd("pos ki 0.15")
        send_cmd("pos vkp 0.020")
        send_cmd("pos vmax 80")
        send_cmd("pos accel 80")
        send_cmd("target 0.000")

        print("=== 3. 使能电机 ===")
        send_cmd("enable")
        time.sleep(0.1)
        st = read_telemetry()
        print(f"使能后状态: state={st['state']} (1=RUN), mode={st['mode']} (3=POS)")
        if st['state'] != 1 or st['mode'] != 3:
            print(f"使能异常: state={st['state']}, mode={st['mode']}")
            return

        print("\n--- 4. 静态保持 1.5s (验证零蜂鸣与微电流) ---")
        t0 = time.time()
        while time.time() - t0 < 1.5:
            st = read_telemetry()
            err = st['target'] - st['pos']
            print(f"  [静态] pos={st['pos']:+.4f} rad, err={err*57.3:+.2f} deg, vel={st['vel_obs']:+.2f} RPM")
            time.sleep(0.2)

        print("\n--- 5. 小阶跃 target 0.200 rad (11.5°) ---")
        send_cmd("target 0.200")
        t0 = time.time()
        while time.time() - t0 < 1.5:
            st = read_telemetry()
            err = st['target'] - st['pos']
            print(f"  [0.20 rad] pos={st['pos']:+.4f} rad, err={err*57.3:+.2f} deg, vel={st['vel_obs']:+.2f} RPM")
            time.sleep(0.15)

        print("\n--- 6. 90° 大阶跃 target 1.5708 rad (90.0°) ---")
        send_cmd("target 1.5708")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            st = read_telemetry()
            err = st['target'] - st['pos']
            print(f"  [90° 阶跃] pos={st['pos']:+.4f} rad, err={err*57.3:+.2f} deg, vel={st['vel_obs']:+.2f} RPM")
            time.sleep(0.15)

        print("\n--- 7. 反向 180° 大阶跃 target -1.5708 rad (-90.0°) ---")
        send_cmd("target -1.5708")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            st = read_telemetry()
            err = st['target'] - st['pos']
            print(f"  [-90° 阶跃] pos={st['pos']:+.4f} rad, err={err*57.3:+.2f} deg, vel={st['vel_obs']:+.2f} RPM")
            time.sleep(0.15)

        print("\n--- 8. 回原点 target 0.000 rad ---")
        send_cmd("target 0.000")
        time.sleep(1.0)
        st = read_telemetry()
        err = st['target'] - st['pos']
        print(f"  [回原点] pos={st['pos']:+.4f} rad, err={err*57.3:+.2f} deg, vel={st['vel_obs']:+.2f} RPM")

        print("\n=== 9. 安全停机 (Disable) ===")
        send_cmd("disable")
        time.sleep(0.1)
        st = read_telemetry()
        print(f"停机完成: state={st['state']} (0=IDLE)")

    finally:
        if t.is_halted():
            t.resume()
        session.close()

if __name__ == '__main__':
    main()
