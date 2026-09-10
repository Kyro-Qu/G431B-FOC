# -*- coding: utf-8 -*-
"""
tools/run_multispeed_eval.py
纯无感多速度 (500, 600, 800, 1000 RPM) 30秒稳态全量程验证
"""
import sys
from eval_single_experiment import run_evaluation

speeds = [500.0, 600.0, 800.0, 1000.0]
results = []

print("======================================================================")
print("=== 纯无感多速度 (500, 600, 800, 1000 RPM) 30秒长跑稳态验证 ===")
print("======================================================================")

for spd in speeds:
    name = f"Multi-Speed Test: {spd:.0f} RPM"
    res = run_evaluation(exp_name=name, target_rpm=spd, filter_hz=40.0, vel_kp=0.0010)
    if res is not None:
        results.append(res)
    else:
        print(f"[FAIL] {name} 运行失败！")
        sys.exit(1)

print("\n" + "=" * 80)
print("                           多速度 30秒长跑稳态验证汇总表")
print("=" * 80)
print(f"{'Target':8s} | {'Enc Spd(True)':14s} | {'VESC Spd(Mean/RMS)':18s} | {'P2P':7s} | {'Theta(RMS/Peak)':16s} | {'Iq_peak':8s} | {'Status'}")
print("-" * 80)
for r in results:
    t_spd = f"{r['exp_name'].split(': ')[1]}"
    enc_str = f"{r['enc_mean']:.1f} / {r['enc_rms']:.2f}"
    vesc_str = f"{r['vesc_mean']:.1f} / {r['vesc_std']:.2f}"
    p2p_str = f"{r['vesc_p2p']:.1f}"
    th_str = f"{r['theta_rms']:.1f}° / {r['theta_peak']:.1f}°"
    iq_str = f"{r['iq_peak']:.3f}A"
    stat = "PASS" if (r['pass_target_err'] and r['pass_theta_rms'] and r['pass_theta_peak'] and r['pass_conf_min'] and r['pass_lock'] and r['pass_iq_peak']) else "CHECK"
    print(f"{t_spd:8s} | {enc_str:14s} | {vesc_str:18s} | {p2p_str:7s} | {th_str:16s} | {iq_str:8s} | {stat}")
print("=" * 80)
