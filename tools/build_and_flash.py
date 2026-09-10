import subprocess
import sys
import time

uv4 = r"E:\Keil_v5\UV4\UV4.exe"
proj = r"MDK-ARM\FOC_G431.uvprojx"
log_build = r"MDK-ARM\build_out.txt"
log_flash = r"MDK-ARM\flash_out.txt"

print(">>> Building Keil project...")
ret_b = subprocess.run([uv4, "-b", proj, "-o", log_build])
with open(log_build, "r", encoding="gbk", errors="ignore") as f:
    b_txt = f.read()
print(b_txt.strip())

if "0 Error(s)" not in b_txt:
    print("[FAIL] Build had errors!")
    sys.exit(1)

if "--build-only" in sys.argv:
    print("\n>>> Build successful (--build-only specified). Skipping flash.")
    sys.exit(0)

print("\n>>> Flashing to MCU...")
ret_f = subprocess.run([uv4, "-f", proj, "-o", log_flash])
with open(log_flash, "r", encoding="gbk", errors="ignore") as f:
    f_txt = f.read()
print(f_txt.strip())
print("\n>>> Build & Flash Done!")
