# -*- coding: utf-8 -*-
"""
tools/test_streak_logic.py
连续准入数学语义与状态机离线单元测试脚本

验证目标:
1. 前 4000 拍通过、后 1 拍 conf_window 失败 (例如 0.69)，streak 必须单拍回到 0；
2. 8000 拍连续全部通过，streak 严格达到 8000 并触发 takeover_ready；
3. 轻度纹波 (err_deg 处于 20°~26°，例如 22°) 时，qualified_cycles 允许动态爬坡容忍/小幅扣减，
   但 qualified_streak 必须立即清零，严禁虚假累积！
"""
import sys

def simulate_step(state, inputs):
    """
    模拟单拍 (1/16000 s) 角度仲裁逻辑中的 streak 与 cycles 计算
    inputs:
      - conf_window: float
      - obs_lock: int (0 or 1)
      - window_rms: float
      - window_peak: float
      - spd_err_pct: float (e.g. 0.02 for 2%)
      - speed_dir_match: int (0 or 1)
      - flux_healthy: int (0 or 1)
      - err_deg: float
      - true_spd_abs: float
      - exit_speed_rpm: float (default 320)
      - is_dynamic_ramp: int (0 or 1)
    """
    conf_window = inputs['conf_window']
    obs_lock = inputs['obs_lock']
    window_rms = inputs['window_rms']
    window_peak = inputs['window_peak']
    spd_err_pct = inputs['spd_err_pct']
    speed_dir_match = inputs['speed_dir_match']
    flux_healthy = inputs['flux_healthy']
    err_deg = inputs['err_deg']
    true_spd_abs = inputs['true_spd_abs']
    exit_speed = inputs.get('exit_speed_rpm', 320.0)
    is_dynamic_ramp = inputs.get('is_dynamic_ramp', 0)

    # 1. 严格连续准入门禁 (qualified_streak): 严格执行 7 大硬门禁，任一超限立即单拍清零
    strict_spd_ok = 1 if spd_err_pct < 0.05 else 0
    strict_pass = (
        (conf_window >= 0.70) and
        (obs_lock == 1) and
        (window_rms < 18.0) and
        (window_peak < 30.0) and
        (strict_spd_ok == 1) and
        (speed_dir_match == 1) and
        (flux_healthy == 1) and
        (err_deg < 30.0) and
        (true_spd_abs >= (exit_speed - 30.0))
    )

    if strict_pass:
        if state['qualified_streak'] < 32000:
            state['qualified_streak'] += 1
    else:
        state['qualified_streak'] = 0

    # 2. 动态容忍准入评分 (qualified_cycles): 加减速爬坡允许转速误差放宽至 12%，并对 20°~26° 纹波进行容忍
    max_spd_err_ratio = 0.12 if is_dynamic_ramp else 0.05
    spd_err_ok = 1 if spd_err_pct < max_spd_err_ratio else 0
    rms_ok = 1 if window_rms < 18.0 else 0

    if obs_lock == 0 or err_deg > 35.0 or flux_healthy == 0 or speed_dir_match == 0:
        state['over_26_streak'] = 0
        state['ramp_ripple_streak'] = 0
        state['over_limit_streak'] += 1
        state['qualified_cycles'] = 0
    elif err_deg > 26.0:
        state['over_26_streak'] += 1
        state['over_limit_streak'] += 1
        state['ramp_ripple_streak'] = 0
        if state['over_26_streak'] >= 16:
            state['qualified_cycles'] = 0
        else:
            if state['qualified_cycles'] >= 2:
                state['qualified_cycles'] -= 2
            else:
                state['qualified_cycles'] = 0
    elif err_deg >= 20.0:
        state['over_26_streak'] = 0
        state['over_limit_streak'] += 1
        can_hold_ramp = (
            (is_dynamic_ramp == 1) and
            (obs_lock == 1) and
            (conf_window >= 0.70) and
            (window_rms < 18.0) and
            (window_peak < 30.0) and
            (state['ramp_ripple_streak'] < 1600)
        )
        if can_hold_ramp:
            state['ramp_ripple_streak'] += 1
        else:
            state['ramp_ripple_streak'] = 0
            if state['qualified_cycles'] > 0:
                state['qualified_cycles'] -= 1
    else:
        state['over_26_streak'] = 0
        state['over_limit_streak'] = 0
        state['ramp_ripple_streak'] = 0
        conf_ok = 1 if conf_window >= 0.70 else 0
        if (true_spd_abs >= (exit_speed - 30.0)) and (spd_err_ok == 1) and (rms_ok == 1) and (conf_ok == 1):
            if state['qualified_cycles'] < 32000:
                state['qualified_cycles'] += 1
        else:
            if state['qualified_cycles'] > 0:
                state['qualified_cycles'] -= 1

    # 3. 正式接管门禁计算与状态机语义
    takeover_ready = 1 if state['qualified_streak'] >= 8000 else 0
    takeover_drop = 1 if state['qualified_cycles'] < 4000 else 0
    obs_healthy_now = (
        (obs_lock == 1) and
        (conf_window >= 0.70) and
        (true_spd_abs >= exit_speed) and
        (speed_dir_match == 1) and
        (flux_healthy == 1)
    )
    obs_ready_to_relay = (state['qualified_streak'] >= 8000) and obs_healthy_now

    return takeover_ready, takeover_drop, obs_ready_to_relay, obs_healthy_now

def run_tests():
    print("=" * 80)
    print(">>> 开始离线单元测试: qualified_streak 严格连续准入数学语义 <<<")
    print("=" * 80)

    # -------------------------------------------------------------
    # 测试 1: 前 4000 拍通过、后 1 拍 conf_window 失败，streak 必须回到 0
    # -------------------------------------------------------------
    state = {'qualified_streak': 0, 'qualified_cycles': 0, 'over_26_streak': 0, 'over_limit_streak': 0, 'ramp_ripple_streak': 0}
    perfect_input = {
        'conf_window': 0.75,
        'obs_lock': 1,
        'window_rms': 10.0,
        'window_peak': 22.0,
        'spd_err_pct': 0.02,
        'speed_dir_match': 1,
        'flux_healthy': 1,
        'err_deg': 12.0,
        'true_spd_abs': 800.0,
        'exit_speed_rpm': 320.0,
        'is_dynamic_ramp': 0
    }

    for i in range(4000):
        simulate_step(state, perfect_input)

    assert state['qualified_streak'] == 4000, f"前4000拍 streak 预期 4000，实际 {state['qualified_streak']}"
    assert state['qualified_cycles'] == 4000, f"前4000拍 cycles 预期 4000，实际 {state['qualified_cycles']}"

    # 第 4001 拍: conf_window 跌至 0.69 (门禁不达标)
    fail_input = dict(perfect_input)
    fail_input['conf_window'] = 0.69
    simulate_step(state, fail_input)

    assert state['qualified_streak'] == 0, f"第4001拍 conf_window=0.69，streak 必须清零，实际 {state['qualified_streak']}"
    assert state['qualified_cycles'] == 3999, f"第4001拍 cycles 预期扣减为 3999，实际 {state['qualified_cycles']}"
    print("[PASS] 测试 1 通过: 前 4000 拍达标，后 1 拍 conf_window=0.69 时 streak 立即清零 (0)，cycles 平滑扣减 (3999)")

    # -------------------------------------------------------------
    # 测试 2: 8000 拍全部通过，streak 严格达到 8000 并触发接管资格
    # -------------------------------------------------------------
    state = {'qualified_streak': 0, 'qualified_cycles': 0, 'over_26_streak': 0, 'over_limit_streak': 0, 'ramp_ripple_streak': 0}
    for i in range(8000):
        to_ready, to_drop, relay_ready, obs_healthy = simulate_step(state, perfect_input)
        if i < 7999:
            assert to_ready == 0, f"在第 {i+1} 拍时不应提前触发 takeover_ready"

    assert state['qualified_streak'] == 8000, f"8000拍后 streak 预期 8000，实际 {state['qualified_streak']}"
    assert state['qualified_cycles'] == 8000, f"8000拍后 cycles 预期 8000，实际 {state['qualified_cycles']}"
    assert to_ready == 1, "第 8000 拍必须触发 takeover_ready == 1"
    assert relay_ready == 1, "第 8000 拍必须触发 obs_ready_to_relay == 1"
    assert to_drop == 0, "8000 拍稳态下绝不能触发 takeover_drop"
    print("[PASS] 测试 2 通过: 连续 8000 拍全部达标，streak 严格达到 8000 并精准触发正式接管资格")

    # -------------------------------------------------------------
    # 测试 3: 轻度纹波 (err_deg=22°)，动态爬坡时 cycles 可以容忍，但 streak 绝不累积
    # -------------------------------------------------------------
    # 先连续通过 2000 拍
    state = {'qualified_streak': 0, 'qualified_cycles': 0, 'over_26_streak': 0, 'over_limit_streak': 0, 'ramp_ripple_streak': 0}
    for i in range(2000):
        simulate_step(state, perfect_input)

    assert state['qualified_streak'] == 2000
    assert state['qualified_cycles'] == 2000

    # 进入动态爬坡轻度纹波区: spd_err_pct = 0.08 (8% > 5% 严限, 但 < 12% 容忍限), err_deg = 22.0° (20°~26°), is_dynamic_ramp = 1
    ripple_input = dict(perfect_input)
    ripple_input['spd_err_pct'] = 0.08
    ripple_input['err_deg'] = 22.0
    ripple_input['is_dynamic_ramp'] = 1

    # 执行 500 拍轻度纹波
    for i in range(500):
        simulate_step(state, ripple_input)

    # 验证: streak 必须在第 1 拍纹波出现时就清零，并且在纹波期间始终保持为 0
    assert state['qualified_streak'] == 0, f"纹波期间 streak 必须保持为 0，实际 {state['qualified_streak']}"
    # 验证: cycles 在动态爬坡期 (ramp_ripple_streak < 1600) 被容忍保持
    assert state['qualified_cycles'] == 2000, f"动态爬坡纹波容忍期间 cycles 应保持 2000，实际 {state['qualified_cycles']}"
    print("[PASS] 测试 3 通过: 爬坡动态纹波 (SpeedErr=8%, err_deg=22°) 时 cycles 被动态容忍保持 (2000)，但 streak 单拍立即清零 (0)，严禁虚假累积")

    # -------------------------------------------------------------
    # 测试 4: 动态加减速爬坡中纹波抖动导致 streak 复位，但 cycles >= 4000 期间不发生状态机颤振退出 (takeover_drop == 0)
    # -------------------------------------------------------------
    state = {'qualified_streak': 0, 'qualified_cycles': 0, 'over_26_streak': 0, 'over_limit_streak': 0, 'ramp_ripple_streak': 0}
    for i in range(8000):
        simulate_step(state, perfect_input)
    assert state['qualified_streak'] == 8000
    assert state['qualified_cycles'] == 8000

    # 突发加减速动态瞬态纹波: err_deg = 23° (轻微超 20° 限) 持续 10 拍
    for i in range(10):
        to_ready, to_drop, relay_ready, obs_healthy = simulate_step(state, ripple_input)

    assert state['qualified_streak'] == 0, "瞬态纹波下 streak 必须严格清零"
    assert state['qualified_cycles'] == 8000, "动态爬坡容忍窗口内 cycles 保持 8000"
    assert to_drop == 0, "cycles >= 4000 时严禁触发 takeover_drop，防止状态机在爬坡过程中颤振切回退出"
    assert obs_healthy == 1, "观测器核心指标健康，支持无感接管"
    print("[PASS] 测试 4 通过: 爬坡纹波致 streak 清零时，迟滞门限 (cycles>=4000) 成功阻止状态机颤振退出，接管稳定性得到数学保障")

    print("=" * 80)
    print(">>> 全部 4 项离线数学逻辑单元测试 100% PASS！数学语义严格一致！ <<<")
    print("=" * 80)

if __name__ == '__main__':
    run_tests()
