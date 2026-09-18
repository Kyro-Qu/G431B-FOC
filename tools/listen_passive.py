import serial
import time

try:
    ser = serial.Serial('COM44', 6500000, timeout=0.1)
    print("Opened COM44 successfully. Listening passively for 2 seconds...")
    t0 = time.time()
    total_bytes = 0
    buf = bytearray()
    while time.time() - t0 < 2.0:
        data = ser.read(1024)
        if data:
            total_bytes += len(data)
            buf.extend(data)
    print(f"Total bytes received in 2s: {total_bytes}")
    if total_bytes > 0:
        print("Raw first 100 bytes:", repr(bytes(buf[:100])))
    ser.close()
except Exception as e:
    print(f"Error: {e}")
