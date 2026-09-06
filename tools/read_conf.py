# -*- coding: utf-8 -*-
import time
import serial

ser = serial.Serial("COM44", 6500000, timeout=0.1)
ser.reset_input_buffer()
ser.write(b"log 0\n")
time.sleep(0.05)
ser.write(b"conf read\n")
time.sleep(0.2)
buf = ser.read_all().decode("ascii", "replace")
print(buf)
ser.close()
