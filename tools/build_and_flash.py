# -*- coding: utf-8 -*-
import subprocess
import sys
import time
import os

uv4 = r"E:\Keil_v5\UV4\UV4.exe"
base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
proj = os.path.join(base_dir, r"MDK-ARM\FOC_G431.uvprojx")
log_build = os.path.join(base_dir, r"MDK-ARM\build_out.txt")
log_flash = os.path.join(base_dir, r"MDK-ARM\flash_out.txt")

def run_uv4(args, log_file, success_marker, timeout=30):
    if os.path.exists(log_file):
        try:
            os.remove(log_file)
        except Exception:
            pass

    cmd = [uv4] + args + ["-o", log_file]
    p = subprocess.Popen(cmd)
    t0 = time.time()
    success = False

    while time.time() - t0 < timeout:
        if os.path.exists(log_file):
            try:
                with open(log_file, "r", encoding="gbk", errors="ignore") as f:
                    content = f.read()
                if success_marker in content:
                    success = True
                    print(content.strip())
                    break
                if "Error(s)" in content and "0 Error(s)" not in content:
                    print(content.strip())
                    break
            except Exception:
                pass
        if p.poll() is not None:
            break
        time.sleep(0.5)

    if p.poll() is None:
        try:
            p.terminate()
            time.sleep(0.2)
            if p.poll() is None:
                p.kill()
        except Exception:
            pass

    if not success and os.path.exists(log_file):
        with open(log_file, "r", encoding="gbk", errors="ignore") as f:
            print(f.read().strip())

    return success

# 如果指定了 --rebuild，则使用 -r，否则使用 -b
mode = "-r" if ("--rebuild" in sys.argv) else "-b"

# 先清理掉可能未触发增量编译的几个核心 .o 文件以防缓存
for obj in ["foc_cmd.o", "foc_app.o", "foc_telemetry.o", "foc_board_g431.o"]:
    p = os.path.join(r"MDK-ARM\FOC_G431", obj)
    if os.path.exists(p):
        try:
            os.remove(p)
        except Exception:
            pass

print(f">>> Building Keil project ({mode})...")
ok_b = run_uv4([mode, proj], log_build, "0 Error(s)", timeout=35)

if not ok_b:
    print("[FAIL] Build failed or had errors!")
    sys.exit(1)

if "--build-only" in sys.argv:
    print("\n>>> Build successful (--build-only specified). Skipping flash.")
    sys.exit(0)

print("\n>>> Flashing to MCU...")
ok_f = run_uv4(["-f", proj], log_flash, "Flash Load finished", timeout=25)
if not ok_f:
    print("[FAIL] Flash download failed!")
    sys.exit(1)

print("\n>>> Build & Flash Done successfully!")
