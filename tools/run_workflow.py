# -*- coding: utf-8 -*-
import subprocess
import sys
import time
import re
import serial

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

def read_file_safely(path):
    for enc in ['gbk', 'utf-8', 'latin1']:
        try:
            with open(path, 'r', encoding=enc) as f:
                return f.read()
        except Exception:
            pass
    return ""

def main():
    print("=" * 80)
    print("Step 1: Rebuilding project...")
    print("=" * 80)
    cmd_rebuild = 'cmd /c "taskkill /f /im UV4.exe 2>nul & E:\\Keil_v5\\UV4\\UV4.exe -r MDK-ARM\\FOC_G431.uvprojx -o MDK-ARM\\build_out.txt"'
    subprocess.run(cmd_rebuild, shell=True)

    print("\n" + "=" * 80)
    print("Step 2: Read build_out.txt:")
    print("=" * 80)
    build_out = read_file_safely("MDK-ARM/build_out.txt")
    print(build_out.strip())

    m = re.search(r'(\d+)\s+Error\(s\)', build_out)
    if not m or int(m.group(1)) != 0:
        print(f"\n[FAIL] 编译存在错误: {m.group(0) if m else '未找到Error(s)标记'}")
        sys.exit(1)

    print("\n" + "=" * 80)
    print("Step 3: Flashing to MCU (0 errors confirmed)...")
    print("=" * 80)
    cmd_flash = 'cmd /c "taskkill /f /im UV4.exe 2>nul & E:\\Keil_v5\\UV4\\UV4.exe -f MDK-ARM\\FOC_G431.uvprojx -o MDK-ARM\\flash_out.txt"'
    subprocess.run(cmd_flash, shell=True)

    print("\n" + "=" * 80)
    print("Step 4: Read flash_out.txt:")
    print("=" * 80)
    flash_out = read_file_safely("MDK-ARM/flash_out.txt")
    print(flash_out.strip())

    print("\n" + "=" * 80)
    print("Step 5: Open COM44 at 6500000 baud, send 'sensorless status'...")
    print("=" * 80)
    time.sleep(1.2)

    ser = None
    try:
        ser = serial.Serial("COM44", 6500000, timeout=0.3)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(b"\r\n")
        time.sleep(0.05)
        ser.read_all()

        ser.write(b"sensorless status\r\n")
        time.sleep(0.15)
        resp = ser.read_all().decode('ascii', errors='replace')
        print("Response:")
        print(resp.strip())
    except Exception as e:
        print(f"[ERROR] COM44 通信异常: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("COM44 已关闭。")

    print("\n" + "=" * 80)
    print("Step 6: Run 'python tools/test_phase3_dynamic_sweep.py 1 2'...")
    print("=" * 80)
    time.sleep(0.5)
    p = subprocess.run([sys.executable, "tools/test_phase3_dynamic_sweep.py", "1", "2"])
    print(f"\nWorkflow finished with exit code: {p.returncode}")

if __name__ == '__main__':
    main()
