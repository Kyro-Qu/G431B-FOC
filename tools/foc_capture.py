#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""foc_capture.py - FOC-STP 遥测抓取 + CLI 控制（COM44 6.5M）
用法:
  python tools/foc_capture.py                          # 抓 12s 默认掩码遥测并打印摘要
  python tools/foc_capture.py --cmd "mode vel"         # 先执行 CLI 命令再抓
  python tools/foc_capture.py --ch vel_ctrl,iq_filt,vq # 只订阅指定通道（名称见 foc_stp.CHANNEL_NAMES）
  python tools/foc_capture.py --mask 0x3C --rate 250   # 直接给掩码 / 降低波形速率
"""
import argparse
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from foc_stp import StpStreamDecoder, CHANNEL_NAMES, DEFAULT_MASK, mask_of, mask_to_bits, cli  # noqa: E402

PORT = 'COM44'
BAUD = 6500000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cmd', action='append', default=[],
                    help='send CLI command(s) before capture')
    ap.add_argument('--secs', type=float, default=12.0)
    ap.add_argument('--out', default='tools/capture_data.npz')
    ap.add_argument('--ch', default='', help='comma separated channel names to subscribe')
    ap.add_argument('--mask', default='', help='hex/dec channel mask (overrides --ch)')
    ap.add_argument('--rate', type=int, default=0, help='wave rate Hz (10..500), 0 = keep')
    args = ap.parse_args()

    mask = DEFAULT_MASK
    if args.mask:
        mask = int(args.mask, 0)
    elif args.ch:
        mask = mask_of(*[c.strip() for c in args.ch.split(',') if c.strip()])

    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.15)
    ser.reset_input_buffer()
    print(cli(ser, 'log 0'))
    for c in args.cmd:
        print('>>>', c)
        print(cli(ser, c))
        time.sleep(0.1)
    print(cli(ser, 'telem mask 0x%08X' % mask))
    if args.rate:
        print(cli(ser, 'telem rate %d' % args.rate))

    # 开始采集遥测
    print('>>>', 'log 1')
    print(cli(ser, 'log 1'))
    ser.reset_input_buffer()

    dec = StpStreamDecoder()
    frames = []      # (host_time, tick, vals)
    status = []
    events = []
    t0 = time.time()
    while time.time() - t0 < args.secs:
        dec.feed(ser.read(ser.in_waiting or 4096))
        now = time.time() - t0
        for w in dec.pop_waves():
            frames.append((now, w['tick'], w['vals']))
        status.extend(dec.pop_status())
        events.extend(dec.pop_events())
    # 关遥测
    ser.write(b'log 0\n')
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.close()

    if not frames:
        print('NO FRAMES CAPTURED (crc_err=%d desync=%d bytes=%d)' % (dec.crc_errors, dec.desync, dec.bytes_in))
        return

    names = [CHANNEL_NAMES[b] for b in mask_to_bits(mask)]
    t = [f[0] for f in frames]
    ticks = [f[1] for f in frames]
    ch = [f[2] for f in frames]
    span_ms = ticks[-1] - ticks[0]
    print('frames=%d span=%.2fs (%.0f Hz by MCU tick) crc_err=%d desync=%d status=%d events=%d'
          % (len(frames), span_ms / 1000.0, (len(frames) - 1) * 1000.0 / span_ms if span_ms else 0.0,
             dec.crc_errors, dec.desync, len(status), len(events)))
    for i, n in enumerate(names):
        vals = [c[i] for c in ch]
        print('%-10s min=%9.3f max=%9.3f last=%9.3f' % (n, min(vals), max(vals), vals[-1]))
    if status:
        s = status[-1]
        print('STATUS last: vbus=%.2fV state=%d mode=%d fault=M%d/S%d rpm=%d iq=%.2fA'
              % (s['vbus'], s['state'], s['mode'], s['motor_fault'], s['shunt_fault'], s['rpm'], s['iq']))
    for e in events:
        print('EVENT: id=%d M_fault=%d S_fault=%d detail=%d @%dms' % (
            e['event_id'], e['motor_fault'], e['shunt_fault'], e['detail'], e['ts']))

    # 保存
    try:
        import numpy as np
        np.savez(args.out, t=t, tick=ticks, ch=np.array(ch), names=names, mask=mask)
        print('saved -> %s' % args.out)
    except ImportError:
        print('(numpy not available, summary only)')

    # 打印最后 0.3s 的样本（掉电前瞬间）
    n_tail = min(len(frames), 300)
    print('--- last samples (t, ' + ', '.join(names[:8]) + ') ---')
    for tt, _, c in frames[-n_tail::30]:
        print('%7.2f ' % tt + ' '.join('%9.3f' % v for v in c[:8]))


if __name__ == '__main__':
    main()
