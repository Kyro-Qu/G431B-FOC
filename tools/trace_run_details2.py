# -*- coding: utf-8 -*-
import serial, time

ser = serial.Serial('COM44', 6500000, timeout=0.2)
ser.reset_input_buffer()

def cmd(s):
    ser.write((s + '\n').encode('utf-8'))
    time.sleep(0.01)
    return ser.read_all().decode('utf-8', errors='ignore').strip()

cmd('disable')
cmd('fault clear')
cmd('feedback sensorless')
cmd('feedback if 0.60 500')
cmd('mode vel')
cmd('target 500')
cmd('enable')

t0 = time.time()
while time.time() - t0 < 3.2:
    fb = cmd('feedback')
    st = cmd('status')
    t = time.time() - t0

    # parse fb
    state, blend, lost = '', '', ''
    for l in fb.splitlines():
        if 'feedback: mode=sensorless' in l:
            p = dict(kv.split('=') for kv in l.split()[1:] if '=' in kv)
            state = p.get('state', '')
            blend = p.get('blend', '')
            lost = p.get('lost', '')

    # parse st
    iq_ref, iq, vel, vel_filt, ref = '', '', '', '', ''
    for l in st.splitlines():
        if 'vel_obs=' in l:
            for item in l.split():
                if item.startswith('vel_obs='): vel = item
                if item.startswith('vel_filt='): vel_filt = item
                if item.startswith('ref='): ref = item
        if 'id=' in l and 'iq=' in l:
            for item in l.split():
                if item.startswith('iq_ref='): iq_ref = item
                if item.startswith('iq='): iq = item
    if state in ('obs_locking', 'blend', 'run', 'lost', 'safe_stop'):
        print(f"[{t:.3f}] state={state:12s} bl={blend} lost={lost} | {ref} {vel_filt} {vel} | {iq_ref} {iq}")
    time.sleep(0.03)

cmd('disable')
ser.close()
