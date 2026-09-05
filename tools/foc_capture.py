#!/usr/bin/env python3
"""foc_capture.py - JustFloat 遥测抓取 + CLI 控制（COM44 6.5M）
用法:
  python tools/foc_capture.py                      # 抓 12s 遥测并打印摘要
  python tools/foc_capture.py --cmd "mode vel"     # 先执行 CLI 命令再抓
"""
import serial, struct, time, sys, argparse

PORT = 'COM44'
BAUD = 6500000
TAIL = b'\x00\x00\x80\x7f'
FRAME = 68

def cli(ser, cmd, wait=0.35):
    ser.write((cmd + '\n').encode())
    time.sleep(wait)
    out = b''
    t0 = time.time()
    while time.time() - t0 < 0.5:
        if ser.in_waiting:
            out += ser.read(ser.in_waiting)
        time.sleep(0.02)
    txt = ''.join(chr(b) if 32 <= b <= 126 or b in (10, 13) else ' '
                  for b in out)
    return txt.strip()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cmd', action='append', default=[],
                    help='send CLI command(s) before capture')
    ap.add_argument('--secs', type=float, default=12.0)
    ap.add_argument('--out', default='tools/capture_data.npz')
    args = ap.parse_args()

    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.15)
    ser.reset_input_buffer()
    print(cli(ser, 'log 0'))
    for c in args.cmd:
        print('>>>', c)
        print(cli(ser, c))
        time.sleep(0.1)

    # 开始采集遥测
    print('>>>', 'log 1')
    print(cli(ser, 'log 1'))
    ser.reset_input_buffer()

    frames = []
    buf = b''
    t0 = time.time()
    while time.time() - t0 < args.secs:
        buf += ser.read(ser.in_waiting or 4096)
        while True:
            idx = buf.find(TAIL)
            if idx < 0 or len(buf) < idx + 4:
                break
            if idx + 4 >= FRAME:
                payload = buf[idx + 4 - FRAME:idx + 4 - 4]
                if len(payload) == 64:
                    frames.append((time.time() - t0,
                                   struct.unpack('<16f', payload)))
                buf = buf[idx + 4:]
            else:
                buf = buf[idx + 4:]
    # 关遥测
    ser.write(b'log 0\n')
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.close()

    if not frames:
        print('NO FRAMES CAPTURED')
        return

    t = [f[0] for f in frames]
    ch = [f[1] for f in frames]
    print(f'frames={len(frames)} span={t[-1]-t[0]:.2f}s '
          f'({len(frames)/(t[-1]-t[0]):.0f} Hz)')
    names = ['th_e','th_m','vel','vel_ref','id','iq','iq_ref',
             'vd','vq','iu','iv','iw','state','fault','pos','vel_filt']
    for i, n in enumerate(names):
        vals = [c[i] for c in ch]
        print(f'{n:9s} min={min(vals):9.3f} max={max(vals):9.3f} '
              f'last={vals[-1]:9.3f}')
    # 保存
    try:
        import numpy as np
        np.savez(args.out, t=t, ch=np.array(ch),
                 names=names)
        print(f'saved -> {args.out}')
    except ImportError:
        print('(numpy not available, summary only)')

    # 打印最后 0.3s 的关键通道（掉电前瞬间）
    n_tail = min(len(frames), 300)
    print('--- last samples (t, vel, iq_ref, iq, vd, vq, iu, fault) ---')
    for tt, c in frames[-n_tail::30]:
        print(f'{tt:7.2f} {c[2]:8.1f} {c[6]:7.3f} {c[5]:7.3f} '
              f'{c[7]:7.3f} {c[8]:7.3f} {c[9]:7.3f} {c[13]:5.0f}')

if __name__ == '__main__':
    main()
