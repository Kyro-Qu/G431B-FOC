# -*- coding: utf-8 -*-
import sys
import time
import serial

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def main():
    print(f"Connecting to {PORT} at {BAUD} baud...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except Exception as e:
        print(f"Error opening port {PORT}: {e}")
        sys.exit(1)

    time.sleep(0.05)
    # Drain any existing bytes in RX buffer
    if ser.in_waiting:
        ser.read(ser.in_waiting)

    def send_cmd(cmd, delay=0.08):
        print(f">> [SENT]: '{cmd}'")
        ser.reset_input_buffer()
        ser.write((cmd + '\r\n').encode('ascii'))
        time.sleep(delay)
        buf = ''
        start = time.time()
        while time.time() - start < 0.4:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting).decode('ascii', errors='replace')
                time.sleep(0.02)
            else:
                if buf:
                    break
                time.sleep(0.02)
        print(f"<< [RAW REPLY]:\n{buf.rstrip()}\n" if buf.strip() else "<< [RAW REPLY]: (empty)\n")
        return buf

    try:
        commands = [
            'fault clear',
            'enc fault clear',
            'feedback auto',
            'feedback speed 420 320',
            'mode vel',
            'vel ramp 800',
            'target 800',
            'enable',
            'status'
        ]

        responses = {}
        for cmd in commands:
            resp = send_cmd(cmd)
            responses[cmd] = resp

        enable_resp = responses.get('enable', '')
        status_resp = responses.get('status', '')

        is_enabled = ("enabled" in enable_resp.lower()) or ("m0 run" in status_resp.lower())

        if is_enabled:
            print("Motor is enabled. Sleeping 1 second...")
            time.sleep(1.0)
            print("Querying status after 1s...")
            send_cmd('status')
            print("Disabling motor...")
            send_cmd('disable')
        else:
            print("Motor is NOT enabled. Skipping 1s spin.")

    except KeyboardInterrupt:
        print("\nInterrupted by user, sending disable for safety...")
        try:
            send_cmd('disable')
        except Exception:
            pass
    finally:
        ser.close()
        print("Serial port closed. Done.")

if __name__ == '__main__':
    main()
