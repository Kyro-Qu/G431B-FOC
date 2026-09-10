# -*- coding: utf-8 -*-
"""
PC 端 FOC 数学算法单元测试运行器 (run_unit_tests.py)
自动调用本地 gcc 编译并运行 tests/test_foc_math.c
"""

import subprocess
import sys
import os

def run_tests():
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    test_src = os.path.join(root_dir, "tests", "test_foc_math.c")
    core_dir = os.path.join(root_dir, "MDK-ARM", "Code", "foc", "Core")

    c_sources = [
        test_src,
        os.path.join(core_dir, "foc_svm.c"),
        os.path.join(core_dir, "foc_pid.c"),
        os.path.join(core_dir, "foc_traj.c"),
    ]

    out_bin = os.path.join(root_dir, "tests", "test_foc_math.exe")

    print(">>> 正在编译 PC 本地单元测试 (GCC)...")
    cmd_compile = ["gcc", "-O2", "-Wall", "-I", core_dir] + c_sources + ["-o", out_bin, "-lm"]
    res = subprocess.run(cmd_compile, capture_output=True, text=True)
    if res.returncode != 0:
        print("[FAIL] 编译单元测试失败:")
        print(res.stderr)
        sys.exit(1)

    print(">>> 编译成功，正在执行单元测试套件...")
    res_run = subprocess.run([out_bin], capture_output=True)
    out_txt = res_run.stdout.decode('utf-8', errors='replace')
    err_txt = res_run.stderr.decode('utf-8', errors='replace')
    print(out_txt)
    if err_txt:
        print(err_txt)

    if res_run.returncode == 0:
        print("[SUCCESS] 单元测试顺利通过！")
    else:
        print("[FAIL] 单元测试执行失败！")
        sys.exit(res_run.returncode)

if __name__ == "__main__":
    run_tests()
