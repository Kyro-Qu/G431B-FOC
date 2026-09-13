# -*- coding: utf-8 -*-
"""COM44 CLI 链路自检: 重插 USB 后跑这个
用法: python tools/foc_link_check.py
"""
import sys
import time

import serial

BAUD = 6500000
PORT = "COM44"


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05, write_timeout=0.5)
    time.sleep(0.2)
    ok = False
    for i in range(5):
        ser.reset_input_buffer()
        ser.write(b"version\n")
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < 0.5:
            ch = ser.read(4096)
            if ch:
                buf.extend(ch)
        if buf:
            print("try%d: OK %d bytes" % (i, len(buf)))
            # 二进制帧内含非 ASCII 字符，用 repr 安全打印避免 Windows 控制台 GBK 乱码
            print(repr(bytes(buf[:200])))
            ok = True
            break
        print("try%d: 0 bytes" % i)
        ser.dtr = False
        ser.rts = False
        time.sleep(0.2)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.2)
    ser.close()
    if not ok:
        print("链路仍死: VLink CDC 桥 RX 方向未恢复, 需重插 USB/断电重启板子")
        sys.exit(1)


if __name__ == "__main__":
    main()
