# -*- coding: utf-8 -*-
"""无感切换最终实验：渐变切换 + raw 判据 + Ls=视在电感。

流程：
  1. 校准 → obs 1（观测器后台跟踪）→ vel 模式 2000rpm 稳定
  2. 用遥测 ch14 (obs−θe) 自动测角差并修正 offset
  3. obs 2 <offset> 渐变切换（200ms blend）
  4. 监测 12s：遥测即停=复位；失步=iq 爆发降速
"""
import serial, time, struct, re
import numpy as np

PORT = 'COM44'
BAUD = 6500000
TAIL = bytes([0x00, 0x00, 0x80, 0x7f])

ser = serial.Serial(PORT, BAUD, timeout=0.5)
buf = b''


def cmd(c, wait=0.4):
    """发送命令并收 ASCII 回复（遥测流会混入，取可读行）"""
    ser.write((c + '\r\n').encode())
    time.sleep(wait)
    r = ser.read(65536)
    txt = ''.join(chr(b) if 32 <= b <= 126 or b in (10, 13) else ' '
                  for b in r)
    return txt


def read_frames(dur):
    global buf
    frames = []
    t0 = time.time()
    while (time.time() - t0) < dur:
        buf += ser.read(8192)
        while True:
            i = buf.find(TAIL)
            if i < 0 or len(buf) < i + 4:
                break
            frame = buf[:i]
            buf = buf[i + 4:]
            if len(frame) == 64:
                frames.append(struct.unpack('<16f', frame))
    return frames


def clear_buf():
    global buf
    ser.reset_input_buffer()
    buf = b''


def main():
    cmd('log 0'); time.sleep(0.3)
    cmd('disable'); cmd('fault clear'); cmd('angle enc')
    cmd('calib', wait=2.0)
    ok = False
    for i in range(15):
        txt = cmd('status', wait=0.5)
        if 'calib=1' in txt:
            ok = True
            break
        if 'FAULT' in txt.split('\r\n')[0]:
            print('CALIB FAULT')
            return 1
    if not ok:
        print('calib timeout')
        return 1
    print('校准 OK')

    cmd('obs 1'); cmd('mode vel'); cmd('target 2000')
    print('enable:', cmd('enable', wait=6.0).strip()[:40])
    cmd('log 1'); time.sleep(0.2)
    clear_buf()

    # --- 编码器闭环下测观测器角差 ---
    f = read_frames(5.0)
    if len(f) < 500:
        print(f'编码器段帧不足 {len(f)}')
    ch = np.array(f) if f else np.zeros((0, 16))
    off = -0.28
    if len(ch) > 500:
        dif = ch[:, 14]
        print(f'编码器闭环 diff: mean={dif.mean():+.3f} std={dif.std():.3f} '
              f'(|d|<0.3: {(np.abs(dif) < 0.3).mean() * 100:.0f}%)')
        off = -0.28 + dif.mean()
        # 限幅到 ±1.0 避免极端值
        off = max(-1.0, min(1.0, off))
    print(f'使用 offset: {off:+.3f}')

    # --- 渐变切换 ---
    switched = False
    for attempt in range(6):
        r = cmd(f'obs 2 {off:+.3f}', wait=0.4)
        m = re.findall(r'(err: obs 2 rejected[^\r\n]*|M\d angle->OBSERVER[^\r\n]*|err: obs 2 needs[^\r\n]*)',
                       r)
        if m:
            print(f'[try{attempt}]', m[0][:90])
            if 'rejected' not in m[0] and 'needs' not in m[0]:
                switched = True
                break
        else:
            print(f'[try{attempt}] 无回复')
        time.sleep(1.5)

    if not switched:
        print('切换未成功')
        cmd('disable')
        return 1

    # --- 监测 ---
    print('监测 12s:')
    alive_s = 0.0
    result = 'timeout'
    for w in range(24):
        f = read_frames(0.5)
        n = len(f)
        if n > 5:
            alive_s += 0.5
            c = np.array(f)
            vel = c[:, 2].mean()
            iqpk = np.abs(c[:, 5]).max()
            print(f'  [{(w + 1) * 0.5:.1f}s] vel={vel:7.1f} iqpk={iqp:.2f}'
                  if False else
                  f'  [{(w + 1) * 0.5:.1f}s] vel={vel:7.1f} iqpk={iqpk:.2f}')
            if vel < 500:
                result = '失步'
                break
        else:
            result = '遥测停(疑似复位)'
            break

    cmd('log 0')
    txt = cmd('status', wait=0.8)
    for line in txt.split('\r\n'):
        if ('M0 ' in line or 'fault' in line.lower() or 'rst' in line
                or 'cs_' in line):
            print('  |', line.strip())
    print(f'结果: {result}  无感存活 {alive_s:.1f}s')
    return 0


if __name__ == '__main__':
    exit(main())
