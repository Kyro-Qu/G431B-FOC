import serial, time, struct, re, sys
import numpy as np
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
TAIL = b'\x00\x00\x80\x7f'; FRAME = 68

def eval_obs(port='COM44', baud=6500000, target=2400):
    ser = serial.Serial(port, baud, timeout=0.02)
    time.sleep(0.4); ser.reset_input_buffer()
    def cli(cmd, wait=0.4):
        ser.write((cmd+'\n').encode()); time.sleep(wait)
        o=b''; t0=time.time()
        while time.time()-t0<0.7:
            if ser.in_waiting: o+=ser.read(ser.in_waiting)
            time.sleep(0.01)
        return ''.join(chr(b) if 32<=b<=126 or b in (10,13) else ' ' for b in o).strip()
    cli('calib', wait=8.0)
    cli('current bw 1000')
    cli('tune fw enable 0')
    cli('obs 1')
    cli('mode vel')
    cli(f'target {target}')
    ser.write(b'log 1\n'); time.sleep(0.15)
    cli('enable')
    ser.reset_input_buffer()
    frames=[]; buf=b''; t0=time.time()
    hit=False
    while time.time()-t0 < 20:
        buf += ser.read(ser.in_waiting or 8192)
        while True:
            idx = buf.find(TAIL)
            if idx < 0 or len(buf) < idx+4: break
            if idx+4 >= FRAME:
                p = buf[idx+4-FRAME:idx+4-4]
                if len(p)==64:
                    f = struct.unpack('<16f', p)
                    frames.append(f)
                    if f[13]>=1.0 and not hit:
                        hit=True; break
            buf = buf[idx+4:]
        if hit: break
    ser.write(b'log 0\n'); time.sleep(0.2); ser.reset_input_buffer()
    ch = np.array(frames) if frames else np.zeros((0,16))
    cli('disable')
    cli('obs 0')
    ser.close()
    if len(ch) < 15000:
        return None
    ds = ch[15000:,14]
    return dict(hit=hit, vel_max=ch[:,2].max(),
                mean=ds.mean(), std=ds.std(),
                p_lt02=(np.abs(ds)<0.2).mean(),
                p_lt03=(np.abs(ds)<0.3).mean())

if __name__ == '__main__':
    r = eval_obs()
    if r:
        print(f"hit={r['hit']} vel_max={r['vel_max']:.0f}")
        print(f"diff mean={r['mean']:+.3f} std={r['std']:.3f}")
        print(f"|diff|<0.2: {r['p_lt02']*100:.0f}%  <0.3: {r['p_lt03']*100:.0f}%")
    else:
        print('insufficient frames')
