#!/usr/bin/env python3
"""foc_staircase.py - 阶梯爬坡诊断脚本

从起始转速开始，每步驻留，逐步爬升，检测到故障立即停止并导出黑匣子。
每步记录帧区间/转速统计/电流纹波，用于建立"转速-风险曲线"。

用法:
  python tools/foc_staircase.py                  # 默认 2000→2450，步进 50
  python tools/foc_staircase.py --start 2200 --end 2400 --step 25
"""
import serial, struct, time, re, sys, argparse
import numpy as np

PORT = 'COM44'
BAUD = 6500000
TAIL = b'\x00\x00\x80\x7f'
FRAME = 68
NAMES = ['th_e','iq_raw','vel','vel_ref','id','iq','iq_ref','vd','vq',
         'iu','iv','iw','duty_a','fault','fw_int','vel_filt']


def open_port():
    ser = serial.Serial(PORT, BAUD, timeout=0.02)
    time.sleep(0.4)
    ser.reset_input_buffer()
    return ser


def cli(ser, cmd, wait=0.4):
    ser.write((cmd + '\n').encode())
    time.sleep(wait)
    out = b''
    t0 = time.time()
    while time.time() - t0 < 0.7:
        if ser.in_waiting:
            out += ser.read(ser.in_waiting)
        time.sleep(0.01)
    return ''.join(chr(b) if 32 <= b <= 126 or b in (10, 13) else ' '
                   for b in out).strip()


def read_frames(ser, dur, stop_on_fault=True):
    """读 dur 秒遥测帧，返回 (帧列表, 故障帧号或None)。帧列表元素含主机时间戳。"""
    frames = []
    buf = b''
    t0 = time.time()
    fault_at = None
    while time.time() - t0 < dur:
        buf += ser.read(ser.in_waiting or 8192)
        while True:
            idx = buf.find(TAIL)
            if idx < 0 or len(buf) < idx + 4:
                break
            if idx + 4 >= FRAME:
                p = buf[idx + 4 - FRAME:idx + 4 - 4]
                if len(p) == 64:
                    vals = struct.unpack('<16f', p)
                    frames.append((time.time(), vals))
                    if stop_on_fault and vals[13] >= 1.0 and fault_at is None:
                        fault_at = len(frames) - 1
            buf = buf[idx + 4:]
        if fault_at is not None and time.time() - t0 > dur:
            break
        if fault_at is not None:
            # 故障后再收 2s 用于观察停机后状态
            t_end = time.time() + 2.0
            while time.time() < t_end:
                buf += ser.read(ser.in_waiting or 8192)
                while True:
                    idx = buf.find(TAIL)
                    if idx < 0 or len(buf) < idx + 4:
                        break
                    if idx + 4 >= FRAME:
                        p = buf[idx + 4 - FRAME:idx + 4 - 4]
                        if len(p) == 64:
                            frames.append((time.time(),
                                           struct.unpack('<16f', p)))
                    buf = buf[idx + 4:]
            break
    return frames, fault_at


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=float, default=2000.0)
    ap.add_argument('--end', type=float, default=2450.0)
    ap.add_argument('--step', type=float, default=50.0)
    ap.add_argument('--dwell', type=float, default=8.0,
                    help='每步驻留秒数')
    ap.add_argument('--out', default='tools/staircase_result.npz')
    args = ap.parse_args()

    ser = open_port()
    print('=== 阶梯爬坡诊断 ===')
    print(cli(ser, 'log 0'))

    # 板子状态检查：cs_fault 锁存需复位
    st = cli(ser, 'status')
    m = re.search(r'cs_fault=(\d+)', st)
    if m and m.group(1) != '0':
        print('!! cs_fault 锁存，需要复位板子（断电重启或调试器 reset）')
        ser.close()
        sys.exit(1)

    print(cli(ser, 'calib', wait=8.0))
    print(cli(ser, 'current bw 1000'))
    print(cli(ser, 'tune fw enable 0'))
    print(cli(ser, 'mode vel'))

    steps = []
    v = args.start
    all_frames = []
    step_marks = []   # (step_rpm, frame_start, frame_end)
    fault_rpm = None

    ser.write(b'log 1\n'); time.sleep(0.15)
    print(cli(ser, f'target {v}'))
    print(cli(ser, 'enable')[:26])
    ser.reset_input_buffer()

    while v <= args.end + 0.1:
        frames, fault_at = read_frames(ser, args.dwell)
        f_start = len(all_frames)
        all_frames.extend(frames)
        f_end = len(all_frames)
        step_marks.append((v, f_start, f_end))

        if fault_at is not None:
            ch = np.array([f[1] for f in frames[:fault_at]])
            vel_at = ch[-1, 2] if ch.ndim == 2 and len(ch) else 0
            fault_rpm = vel_at
            print(f'>>> {v:.0f}RPM 档: FAULT vel={vel_at:.0f} '
                  f'(帧 {fault_at}/{len(frames)})')
            break

        if not frames:
            print(f'{v:6.0f}RPM: 无遥测帧（疑似爆发断电）')
            fault_rpm = v
            steps.append((v, 0, 0, 0, 0, 0, 1))
            break
        ch = np.array([f[1] for f in frames])
        vel_m = ch[:, 2].mean()
        iq_pk = np.abs(ch[:, 1]).max()          # 原始 iq
        iu_pk = np.abs(ch[:, 9:12]).max()
        iq_std = ch[:, 1].std()
        duty_rng = ch[:, 12].max() - ch[:, 12].min()
        print(f'{v:6.0f}RPM: vel={vel_m:6.0f} iq_raw_pk={iq_pk:5.2f} '
              f'iq_std={iq_std:5.3f} iu_pk={iu_pk:5.2f} '
              f'duty_rng={duty_rng:.3f}')
        steps.append((v, vel_m, iq_pk, iq_std, iu_pk, duty_rng, 0))
        v += args.step
        cli(ser, f'target {v}')

    ser.write(b'log 0\n'); time.sleep(0.2); ser.reset_input_buffer()

    # 故障则导出黑匣子
    if fault_rpm is not None:
        print('--- 导出黑匣子 ---')
        ser.write(b'blackbox\n')
        out = b''
        t0 = time.time()
        while time.time() - t0 < 5.0:
            if ser.in_waiting:
                out += ser.read(ser.in_waiting)
            time.sleep(0.005)
        txt = ''.join(chr(b) if 32 <= b <= 126 or b in (10, 13) else ' '
                      for b in out)
        rows = []
        for l in txt.splitlines():
            pt = l.split()
            if len(pt) == 6 and pt[0].isdigit():
                try:
                    rows.append((int(pt[0]),) +
                                tuple(int(x, 16) for x in pt[1:]))
                except ValueError:
                    pass
        print(f'黑匣子 {len(rows)} 行')
        np.save(f'tools/staircase_blackbox.npy',
                np.array(rows, dtype=object), allow_pickle=True)

    # 保存与摘要
    t_arr = np.array([f[0] for f in all_frames])
    ch_arr = np.array([f[1] for f in all_frames])
    np.savez(args.out, t=t_arr, ch=ch_arr, names=NAMES,
             step_marks=np.array(step_marks),
             steps=np.array(steps) if steps else np.empty((0, 7)))
    print(f'\n数据保存 -> {args.out}')
    if steps:
        print('=== 风险曲线摘要（iq_raw 峰值 vs 转速） ===')
        for s in steps:
            bar = '#' * int(s[2] * 10)
            print(f'{s[0]:6.0f}RPM iq_pk={s[2]:5.2f} {bar}')
    ser.close()


if __name__ == '__main__':
    main()
