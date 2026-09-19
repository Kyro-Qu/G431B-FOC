/**
 * @file    foc_angle_manager.c
 * @brief   反馈角度仲裁与无感自动接管管理器实现
 */

#include "foc_angle_manager.h"
#include "foc_motor.h"
#include "foc_sensorless_bench.h"
#include "foc_pid.h"
#include "foc_utils.h"
#include <math.h>

#ifndef _DEG2RAD
#define _DEG2RAD (_PI / 180.0f)
#endif
#ifndef _RAD2DEG
#define _RAD2DEG (180.0f / _PI)
#endif

foc_angle_manager_t g_angle_mgr;

/* 快速角度规范化到 [-PI, PI] */
static inline float wrap_pm_pi(float a)
{
    while (a > _PI)  { a -= _2PI; }
    while (a < -_PI) { a += _2PI; }
    return a;
}

void foc_angle_mgr_init(void)
{
    g_angle_mgr.mode              = FOC_FEEDBACK_SENSORED_PRIMARY;
    g_angle_mgr.active_algo       = SENSORLESS_ALGO_VESC; /* 自动接管首选仅使用 VESC */
    g_angle_mgr.enter_speed_rpm   = 450.0f; /* 适应 500~2000 RPM 全速度范围接管 */
    g_angle_mgr.exit_speed_rpm    = 350.0f; /* 适应 350 RPM 以上收敛锁定特性 */
    g_angle_mgr.blend_time_s      = 0.20f;

    g_angle_mgr.state             = FOC_ANGLE_SENSORED;
    g_angle_mgr.enc_health        = ENCODER_HEALTH_NORMAL;
    g_angle_mgr.fallback_reason   = FALLBACK_NONE;
    g_angle_mgr.handover_blend    = 0.0f;
    g_angle_mgr.handover_delta_rad= 0.0f;

    g_angle_mgr.theta_encoder     = 0.0f;
    g_angle_mgr.theta_sensorless  = 0.0f;
    g_angle_mgr.theta_control     = 0.0f;
    g_angle_mgr.angle_error_deg   = 0.0f;
    g_angle_mgr.speed_encoder_rpm = 0.0f;
    g_angle_mgr.speed_obs_rpm     = 0.0f;
    g_angle_mgr.speed_error_rpm   = 0.0f;

    g_angle_mgr.obs_lock          = 0U;
    g_angle_mgr.conf_inst         = 0.0f;
    g_angle_mgr.conf_window       = 0.0f;
    g_angle_mgr.obs_confidence    = 0.0f;
    g_angle_mgr.qualified_cycles  = 0U;
    g_angle_mgr.qualified_streak  = 0U;
    g_angle_mgr.over_limit_streak = 0U;
    g_angle_mgr.over_26_streak    = 0U;
    g_angle_mgr.ramp_ripple_streak= 0U;

    g_angle_mgr.window_samples    = 0U;
    g_angle_mgr.window_err_sum    = 0.0f;
    g_angle_mgr.window_err_sq_sum = 0.0f;
    g_angle_mgr.window_conf_sum   = 0.0f;
    g_angle_mgr.window_peak_err   = 0.0f;
    g_angle_mgr.window_mean_deg   = 0.0f;
    g_angle_mgr.window_rms_deg    = 0.0f;
    g_angle_mgr.window_min_conf   = 1.0f;
    g_angle_mgr.window_unlock_cnt = 0U;

    g_angle_mgr.obs_unlock_cnt    = 0U;
    g_angle_mgr.enc_stagnant_cnt  = 0U;
    g_angle_mgr.enc_err_streak    = 0U;
    g_angle_mgr.enc_step_latch    = 0U;
    g_angle_mgr.enc_skip_streak   = 80U; /* 开机首拍跳过 80 拍 (~5ms) 避免单拍阶跃伪影 */
    g_angle_mgr.enc_last_raw_rad  = 0.0f;
    g_angle_mgr.enc_freeze_rad    = 0.0f;
    g_angle_mgr.speed_control     = 0.0f;
    g_angle_mgr.speed_obs_filt_rpm= 0.0f;

    g_angle_mgr.inject_type       = INJECT_NONE;
    g_angle_mgr.inject_param      = 0.0f;

    g_angle_mgr.if_current_a      = 0.50f; /* 默认 0.50A 开环电流，温和启动且低于 0.8A 限幅 */
    g_angle_mgr.if_target_rpm     = 500.0f;/* 默认 500 RPM 切换转速 */
    g_angle_mgr.if_accel_rpm_s    = 300.0f;/* 默认 300 RPM/s 温和加速度斜坡 */
    g_angle_mgr.theta_open        = 0.0f;
    g_angle_mgr.open_speed_rpm    = 0.0f;
    g_angle_mgr.state_ticks       = 0U;
    g_angle_mgr.lost_reason       = 0U;
}

void foc_angle_mgr_reset(void)
{
    if (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY) {
        g_angle_mgr.state         = FOC_ANGLE_SENSORLESS_IF_START;
    } else {
        g_angle_mgr.state         = FOC_ANGLE_SENSORED;
    }
    g_angle_mgr.enc_health        = ENCODER_HEALTH_NORMAL;
    g_angle_mgr.fallback_reason   = FALLBACK_NONE;
    g_angle_mgr.handover_blend    = 0.0f;
    g_angle_mgr.handover_delta_rad= 0.0f;
    g_angle_mgr.qualified_cycles  = 0U;
    g_angle_mgr.qualified_streak  = 0U;
    g_angle_mgr.conf_inst         = 0.0f;
    g_angle_mgr.conf_window       = 0.0f;
    g_angle_mgr.obs_confidence    = 0.0f;
    g_angle_mgr.over_limit_streak = 0U;
    g_angle_mgr.over_26_streak    = 0U;
    g_angle_mgr.obs_unlock_cnt    = 0U;
    g_angle_mgr.enc_stagnant_cnt  = 0U;
    g_angle_mgr.enc_err_streak    = 0U;
    g_angle_mgr.enc_step_latch    = 0U;
    g_angle_mgr.enc_skip_streak   = 80U;
    g_angle_mgr.speed_control     = 0.0f;
    g_angle_mgr.speed_obs_filt_rpm= 0.0f;
    g_angle_mgr.inject_type       = INJECT_NONE;
    g_angle_mgr.inject_param      = 0.0f;

    g_angle_mgr.theta_open        = 0.0f;
    g_angle_mgr.open_speed_rpm    = 0.0f;
    g_angle_mgr.state_ticks       = 0U;
    g_angle_mgr.lost_reason       = 0U;

    g_angle_mgr.window_samples    = 0U;
    g_angle_mgr.window_err_sum    = 0.0f;
    g_angle_mgr.window_err_sq_sum = 0.0f;
    g_angle_mgr.window_conf_sum   = 0.0f;
    g_angle_mgr.window_peak_err   = 0.0f;
    g_angle_mgr.window_min_conf   = 1.0f;
    g_angle_mgr.window_unlock_cnt = 0U;
}

void foc_angle_mgr_set_mode(foc_feedback_mode_t mode)
{
    g_angle_mgr.mode = mode;
}

void foc_angle_mgr_set_algo(foc_sensorless_algo_t algo)
{
    g_angle_mgr.active_algo = algo;
}

void foc_angle_mgr_inject_fault(foc_enc_fault_inject_t type, float param)
{
    g_angle_mgr.inject_type = type;
    g_angle_mgr.inject_param = param;
    if (type == INJECT_FREEZE) {
        g_angle_mgr.enc_freeze_rad = g_angle_mgr.enc_last_raw_rad;
    } else if (type == INJECT_STEP) {
        g_angle_mgr.enc_step_latch = 1U;
    } else if (type == INJECT_NONE) {
        /* 清除故障注入：立即复位锁存与防抖计数，编码器恢复健康 */
        g_angle_mgr.enc_step_latch = 0U;
        g_angle_mgr.enc_stagnant_cnt = 0U;
        g_angle_mgr.enc_err_streak = 0U;
        g_angle_mgr.enc_health = ENCODER_HEALTH_NORMAL;
        g_angle_mgr.enc_skip_streak = 16U; /* 忽略恢复前 16 拍 (~1ms) 角度跳变微分伪影 */
    }
}

/**
 * @brief 编码器健康检查（防抖判定，单次毛刺不触发 FAILED）
 */
static void check_encoder_health(foc_motor_t *m, float enc_mech_rad, float enc_spd_rpm, float spd_obs)
{
    if (g_angle_mgr.enc_skip_streak > 0U) {
        g_angle_mgr.enc_skip_streak--;
        g_angle_mgr.enc_last_raw_rad = enc_mech_rad;
        g_angle_mgr.enc_step_latch = 0U;
        g_angle_mgr.enc_stagnant_cnt = 0U;
        g_angle_mgr.enc_err_streak = 0U;
        g_angle_mgr.enc_health = ENCODER_HEALTH_NORMAL;
        return;
    }

    float d_mech = fabsf(wrap_pm_pi(enc_mech_rad - g_angle_mgr.enc_last_raw_rad));
    g_angle_mgr.enc_last_raw_rad = enc_mech_rad;

    /* 1. 速度是否超物理极限 (如 > 15000 RPM) */
    uint8_t spd_bad = (fabsf(enc_spd_rpm) > (m->params.max_rpm * 1.25f)) ? 1U : 0U;

    /* 2. 单拍机械角是否产生不可能的巨大阶跃 (例如单拍在 16kHz 下变化 > 0.5 rad，等效 > 7600 RPM 冲击)
     *    由于物理阶跃失位后不可盲目信任，必须保持阶跃异常锁存，直到故障显式清除 */
    if (d_mech > 0.5f) {
        g_angle_mgr.enc_step_latch = 1U;
    }
    uint8_t step_bad = (g_angle_mgr.enc_step_latch != 0U) ? 1U : 0U;

    /* 3. 检查是否长时间机械角停滞 (FREEZE):
     * 区分过零换向与真实停更：
     * 仅当转子正在中高速旋转 (|enc_spd| > 300 RPM 或 |spd_obs| > 350 RPM 且未处于过零换向区) 时，
     * 若机械角度完全不走 (d_mech < 1e-5f) 且电机处于 RUN 态，才计入停更！
     * 在候选态 (CANDIDATE) 下 4 拍 (0.25ms) 确诊，常态下 400 拍 (25ms) 确诊！ */
    uint8_t in_crossover = ((fabsf(enc_spd_rpm) < 250.0f) && (fabsf(spd_obs) < 250.0f)) ? 1U : 0U;
    uint8_t is_high_speed = ((fabsf(enc_spd_rpm) > 300.0f) || (fabsf(spd_obs) > 350.0f)) ? 1U : 0U;
    uint8_t freeze_bad = 0U;
    uint32_t freeze_thresh = (g_angle_mgr.state == FOC_ANGLE_SENSORLESS_CANDIDATE) ? 4U : 400U;

    if ((m->state == FOC_STATE_RUN) && (in_crossover == 0U) && (is_high_speed != 0U) && (d_mech < 1e-5f)) {
        if (++g_angle_mgr.enc_stagnant_cnt >= freeze_thresh) {
            freeze_bad = 1U;
        }
    } else {
        g_angle_mgr.enc_stagnant_cnt = 0U;
    }

    if ((spd_bad != 0U) || (step_bad != 0U) || (freeze_bad != 0U)) {
        if (g_angle_mgr.enc_err_streak < 64U) {
            g_angle_mgr.enc_err_streak++;
        }
        uint32_t streak_thresh = (g_angle_mgr.state == FOC_ANGLE_SENSORLESS_CANDIDATE) ? 4U : 32U;
        if (g_angle_mgr.enc_err_streak >= streak_thresh) {
            g_angle_mgr.enc_health = ENCODER_HEALTH_FAILED;
            if (freeze_bad != 0U) {
                g_angle_mgr.fallback_reason = FALLBACK_ENC_TIMEOUT;
            } else if (spd_bad != 0U) {
                g_angle_mgr.fallback_reason = FALLBACK_ENC_SPEED_FAULT;
            } else {
                g_angle_mgr.fallback_reason = FALLBACK_ENC_STEP_FAULT;
            }
        } else {
            g_angle_mgr.enc_health = ENCODER_HEALTH_SUSPECT;
        }
    } else {
        if (g_angle_mgr.enc_err_streak > 0U) {
            g_angle_mgr.enc_err_streak--;
        }
        if (g_angle_mgr.enc_err_streak == 0U) {
            g_angle_mgr.enc_health = ENCODER_HEALTH_NORMAL;
        }
    }
}

/**
 * @brief 提取选定无感算法的实时数据 (自动接管优先只使用 VESC)
 */
static void fetch_sensorless_data(const foc_motor_t *m, foc_sensorless_algo_t algo, float *theta_out,
                                 float *speed_out, uint8_t *lock_out, float *flux_out)
{
    foc_sensorless_bench_t *b = &g_sensorless_bench;
    foc_bench_obs_metrics_t *obs = &b->obs2_vesc;

    switch (algo) {
    case SENSORLESS_ALGO_ORTEGA:
        obs = &b->obs1_ortega;
        *flux_out = b->obs1_ortega.flux_mag;
        break;

    case SENSORLESS_ALGO_STO:
        obs = &b->obs3_sto_pll;
        *flux_out = b->diag_sto_bemf_mag;
        break;

    case SENSORLESS_ALGO_VESC:
    default:
        obs = &b->obs2_vesc;
        *flux_out = b->obs2_vesc.flux_mag;
        break;
    }

    float th = obs->theta_e;

    if (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY) {
        /* 纯无感模式 (Sensorless Primary):
         * 彻底脱离物理编码器及其校准零点偏置，直接使用 VESC 观测器基于定子电压电流解算的真实转子永磁物理电角度，
         * 严禁叠加编码器相对补偿 advance_rad 或 theta_offset，杜绝相位虚假偏移导致的制动冲击！ */
    } else {
        /* 有感与自动接管模式: 补偿与编码器校准零点的系统差模/共模偏置 */
        if (obs->direction < 0) {
            th = foc_wrap_0_2pi(_2PI - th);
        }
        th = foc_wrap_0_2pi(th + obs->theta_offset);

        float spd = obs->speed_rpm;
        float spd_abs = fabsf(spd);
        float dir_blend = (spd > 150.0f) ? 1.0f : ((spd < -150.0f) ? -1.0f : (spd / 150.0f));

        float fwd_adv = 0.800f;
        float rev_adv = -0.530f;
        if (spd_abs < 1200.0f) {
            float delta_spd = 1200.0f - spd_abs;
            fwd_adv += delta_spd * 0.00035f;
            if (fwd_adv > 1.02f) { fwd_adv = 1.02f; }
            rev_adv -= delta_spd * 0.00030f;
            if (rev_adv < -0.72f) { rev_adv = -0.72f; }
        }

        float bias_0 = (fwd_adv + rev_adv) * 0.5f;
        float delta_dir = (fwd_adv - rev_adv) * 0.5f;
        float advance_rad = bias_0 + (dir_blend * delta_dir);
        th = foc_wrap_0_2pi(th + advance_rad);
    }

    *theta_out = th;
    *speed_out = obs->speed_rpm;
    *lock_out  = obs->converged;
}

float foc_angle_mgr_update(foc_motor_t *m)
{
    /* 0. 有感主控常态极速通路：当处于 SENSORED_PRIMARY 且未开启影子评测时，
     * 直接计算经校准电角度并返回，消除后台无感提取、角差滤波与状态机等 2000+ cycles 开销 */
    if ((g_angle_mgr.mode == FOC_FEEDBACK_SENSORED_PRIMARY) && (g_sensorless_bench.shadow_enabled == 0U)) {
        float enc_mech_rad = m->theta_mech;
        float we = (float)m->calib.direction * m->velocity_observer_rpm * FOC_RPM_TO_RADS * m->params.pole_pairs;
        float th_enc = foc_wrap_0_2pi(
            ((float)m->calib.direction * enc_mech_rad * m->params.pole_pairs) + m->calib.electrical_offset_rad +
            (m->runtime.angle_delay_cycles * we * m->dt_fast));
        g_angle_mgr.theta_encoder = th_enc;
        g_angle_mgr.speed_encoder_rpm = m->velocity_filt_rpm;
        g_angle_mgr.theta_control = th_enc;
        g_angle_mgr.speed_control = m->velocity_filt_rpm;
        return th_enc;
    }

    /* 1. 获取无感观测器后台数据 (提前用于健康检测与过零判断) */
    float th_obs = 0.0f;
    float spd_obs = 0.0f;
    uint8_t obs_lock = 0U;
    float flux_val = 0.0f;
    fetch_sensorless_data(m, g_angle_mgr.active_algo, &th_obs, &spd_obs, &obs_lock, &flux_val);

    g_angle_mgr.theta_sensorless = th_obs;
    g_angle_mgr.speed_obs_rpm    = spd_obs;
    g_angle_mgr.obs_lock         = obs_lock;

    /* 2. 读取编码器原始机械角与物理转速 */
    float enc_mech_rad = (m->sensor != 0) ? m->sensor->angle_rad() : 0.0f;
    float enc_spd_rpm = m->velocity_filt_rpm;

    /* 故障注入模拟拦截 */
    if (g_angle_mgr.inject_type == INJECT_FREEZE) {
        enc_mech_rad = g_angle_mgr.enc_freeze_rad;
        enc_spd_rpm = 0.0f;
    } else if (g_angle_mgr.inject_type == INJECT_STEP) {
        enc_mech_rad = foc_wrap_0_2pi(enc_mech_rad + (g_angle_mgr.inject_param * (_PI / 180.0f)));
    } else if (g_angle_mgr.inject_type == INJECT_SPEED_SPIKE) {
        enc_spd_rpm = g_angle_mgr.inject_param;
    }

    /* 3. 编码器健康检查 (结合无感转速以解耦过零停顿与高速FREEZE) */
    if (m->sensor != 0) {
        check_encoder_health(m, enc_mech_rad, enc_spd_rpm, spd_obs);
    }

    /* 计算经校准的编码器电角度 (含超前补偿) */
    float we = (float)m->calib.direction * m->velocity_observer_rpm * FOC_RPM_TO_RADS * m->params.pole_pairs;
    float th_enc = foc_wrap_0_2pi(
        ((float)m->calib.direction * enc_mech_rad * m->params.pole_pairs) + m->calib.electrical_offset_rad +
        (m->runtime.angle_delay_cycles * we * m->dt_fast));

    g_angle_mgr.theta_encoder     = th_enc;
    g_angle_mgr.speed_encoder_rpm = enc_spd_rpm;

    /* 4. 计算角差与速度偏差 (保留带符号瞬时误差以便精确诊断超前/滞后)
     * 速度误差计算：对无感观测器瞬时转速建立 50Hz 匹配低通滤波，与编码器滤波速度对齐带宽，
     * 滤除 16kHz 快环中的 PWM 斩波与单拍离散量化白噪声，真实反映机电转速估计偏差 */
    float spd_alpha = 0.02f; /* 约 51Hz 带宽 @16kHz */
    if ((fabsf(g_angle_mgr.speed_obs_filt_rpm) < 1.0f) && (fabsf(spd_obs) > 1.0f)) {
        g_angle_mgr.speed_obs_filt_rpm = spd_obs;
    } else {
        g_angle_mgr.speed_obs_filt_rpm += spd_alpha * (spd_obs - g_angle_mgr.speed_obs_filt_rpm);
    }

    float err_rad = wrap_pm_pi(th_obs - th_enc);
    float err_signed_deg = err_rad * (180.0f / _PI);
    float err_deg = fabsf(err_signed_deg);
    float spd_err = fabsf(fabsf(g_angle_mgr.speed_obs_filt_rpm) - fabsf(enc_spd_rpm));

    g_angle_mgr.angle_error_deg = err_signed_deg;
    g_angle_mgr.speed_error_rpm = spd_err;

    /* 5. 准入资格与置信度评估:
     * 明确区分瞬时保护置信度 conf_inst 与 500ms 滑动窗口准入置信度 conf_window */
    float conf_inst = 1.0f;
    uint8_t is_nan_or_inf = ((err_deg != err_deg) || (err_deg > 360.0f)) ? 1U : 0U;

    /* 计算电机相对标称磁链有效范围 (0.4 * flux_nominal <= flux <= 2.5 * flux_nominal) */
    float flux_nominal = 0.00115f;
    if ((m->params.ke > 0.01f) && (m->params.pole_pairs > 0.0f)) {
        flux_nominal = m->params.ke / (1.73205f * (m->params.pole_pairs * 1000.0f * _2PI / 60.0f));
    }
    float flux_min_rel = 0.40f * flux_nominal;
    float flux_max_rel = 2.50f * flux_nominal;
    if (flux_min_rel < 0.0001f) { flux_min_rel = 0.0001f; }
    if (flux_max_rel > 0.0500f) { flux_max_rel = 0.0500f; }
    uint8_t flux_healthy = ((flux_val >= flux_min_rel) && (flux_val <= flux_max_rel)) ? 1U : 0U;

    if (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY) {
        /* 纯无感模式 (SENSORLESS_PRIMARY):
         * 编码器仅作为后台无害诊断对比，严禁进入控制链或参与置信度判定。
         * 内生物理特性独立置信度评估：
         * 1. 观测器锁定 obs_lock == 1
         * 2. 磁链模长正常 flux_healthy == 1
         * 3. 估计速度与开环/设定速度同号且误差合理 */
        float spd_ref_check = (g_angle_mgr.state < FOC_ANGLE_SENSORLESS_RUN)
                              ? g_angle_mgr.open_speed_rpm : m->vel_ref_rpm;
        uint8_t dir_match = 1U;
        if (fabsf(spd_ref_check) > 50.0f) {
            dir_match = ((spd_obs * spd_ref_check) > 0.0f) ? 1U : 0U;
        }
        if ((obs_lock == 0U) || (flux_healthy == 0U) || (dir_match == 0U) || (is_nan_or_inf != 0U)) {
            conf_inst = 0.0f;
        } else {
            float flux_err_ratio = fabsf(flux_val - flux_nominal) / flux_nominal;
            float spd_err_ratio = fabsf(spd_obs - spd_ref_check) / (fabsf(spd_ref_check) + 50.0f);
            conf_inst = 1.0f - foc_clampf(flux_err_ratio * 0.8f, 0.0f, 0.4f)
                             - foc_clampf(spd_err_ratio * 1.5f, 0.0f, 0.4f);
            if (conf_inst < 0.0f) { conf_inst = 0.0f; }
        }
        g_angle_mgr.conf_inst = conf_inst;

        /* 纯无感模式下的滑动窗口 EMA 平滑追踪 */
        float ema_alpha = 0.001f;
        if (g_angle_mgr.conf_window < 0.01f) {
            g_angle_mgr.conf_window = conf_inst;
        } else {
            g_angle_mgr.conf_window = (g_angle_mgr.conf_window * (1.0f - ema_alpha)) + (conf_inst * ema_alpha);
        }
        g_angle_mgr.obs_confidence = g_angle_mgr.conf_window;
    } else {
        /* 原有的有感主控 (SENSORED_PRIMARY) 与 自动接管 (AUTO_FALLBACK) 模式 */
        if ((obs_lock == 0U) || (is_nan_or_inf != 0U)) {
            conf_inst = 0.0f;
        } else if (g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL) {
            if (err_deg > 30.0f) {
                conf_inst = 0.0f;
            } else {
                conf_inst = 1.0f - (err_deg / 30.0f);
            }
            float true_spd_abs = fabsf(enc_spd_rpm);
            if (true_spd_abs > 100.0f) {
                float spd_err_pct = spd_err / true_spd_abs;
                if (spd_err_pct > 0.10f) {
                    conf_inst *= 0.5f;
                }
            }
        } else {
            /* 编码器异常期间：瞬时置信度由观测器转速与锁定评估 */
            if (fabsf(spd_obs) < (g_angle_mgr.exit_speed_rpm - 50.0f)) {
                conf_inst = 0.2f;
            } else {
                conf_inst = 1.0f;
            }
        }
        g_angle_mgr.conf_inst = conf_inst;

        /* 6. 长窗口 (500ms 统计：500 拍 @1kHz) 滑动统计与硬核接管资格判定
         * 仅在编码器健康时统计对齐品质。
         * 采用 16 分频 (1kHz) 采样：保持 500ms 统计物理时间跨度，数学方差与 RMS 严格等价，
         * 将每拍长窗浮点平方、除法与开方开销彻底从 16kHz 快环中解除！ */
        static uint8_t s_mgr_decim = 0U;
        s_mgr_decim = (s_mgr_decim + 1U) & 0x0FU;

        if ((s_mgr_decim == 0U) && (g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL)) {
            g_angle_mgr.window_samples++;
            g_angle_mgr.window_err_sum += err_deg;
            g_angle_mgr.window_err_sq_sum += err_deg * err_deg;
            g_angle_mgr.window_conf_sum += conf_inst;
            if (err_deg > g_angle_mgr.window_peak_err) {
                g_angle_mgr.window_peak_err = err_deg;
            }
            if (conf_inst < g_angle_mgr.window_min_conf) {
                g_angle_mgr.window_min_conf = conf_inst;
            }
            if (obs_lock == 0U) {
                g_angle_mgr.window_unlock_cnt++;
            }

            /* 窗口 EMA 平滑滤波 (1kHz 下 alpha = 0.016f，与原 16kHz alpha=0.001f 时间常数完全一致) */
            float ema_alpha = 0.016f;
            if (g_angle_mgr.conf_window < 0.01f) {
                g_angle_mgr.conf_window = conf_inst;
            } else {
                g_angle_mgr.conf_window = (g_angle_mgr.conf_window * (1.0f - ema_alpha)) + (conf_inst * ema_alpha);
            }

            if (g_angle_mgr.window_samples >= 500U) {
                g_angle_mgr.window_mean_deg = g_angle_mgr.window_err_sum / 500.0f;
                g_angle_mgr.window_rms_deg  = sqrtf(g_angle_mgr.window_err_sq_sum / 500.0f);
                /* 窗口真实平均置信度结算 */
                float full_win_conf = g_angle_mgr.window_conf_sum / 500.0f;
                g_angle_mgr.conf_window = full_win_conf;

                /* 重置下一个窗口周期累加器 */
                g_angle_mgr.window_samples = 0U;
                g_angle_mgr.window_err_sum = 0.0f;
                g_angle_mgr.window_err_sq_sum = 0.0f;
                g_angle_mgr.window_conf_sum = 0.0f;
                g_angle_mgr.window_peak_err = 0.0f;
                g_angle_mgr.window_min_conf = 1.0f;
                g_angle_mgr.window_unlock_cnt = 0U;
            }

            /* 保持向下兼容映射 */
            g_angle_mgr.obs_confidence = g_angle_mgr.conf_window;
        }

        /* 接管资格准入门禁 (qualified_streak) 与动态容忍评分 (qualified_cycles):
         * 仅在自动接管模式 (AUTO_FALLBACK) 下执行评定；有感主控常态下无需状态机接管，直接跳过！ */
        if (g_angle_mgr.mode == FOC_FEEDBACK_AUTO_FALLBACK) {
            float true_spd_abs = fabsf(enc_spd_rpm);
            uint8_t speed_dir_match = 1U;
            if (fabsf(m->vel_ref_rpm) > 100.0f) {
                speed_dir_match = ((spd_obs * m->vel_ref_rpm) > 0.0f) ? 1U : 0U;
            }
            uint8_t strict_spd_err_ok = (true_spd_abs > 100.0f) ? ((spd_err / true_spd_abs) < 0.05f) : 1U;
            uint8_t streak_gate_pass = ((g_angle_mgr.conf_window >= 0.70f) &&
                                        (obs_lock == 1U) &&
                                        (g_angle_mgr.window_rms_deg < 18.0f) &&
                                        (g_angle_mgr.window_peak_err < 30.0f) &&
                                        (strict_spd_err_ok != 0U) &&
                                        (speed_dir_match != 0U) &&
                                        (flux_healthy != 0U) &&
                                        (err_deg < 30.0f) &&
                                        (true_spd_abs >= (g_angle_mgr.exit_speed_rpm - 30.0f))) ? 1U : 0U;

            if (streak_gate_pass != 0U) {
                if (g_angle_mgr.qualified_streak < 32000U) {
                    g_angle_mgr.qualified_streak++;
                }
            } else {
                if (g_angle_mgr.qualified_streak > 200U) {
                    /* 记录被清零的原因 */
                    g_angle_mgr.streak_fail_count++;
                    if (g_angle_mgr.conf_window < 0.70f) { g_angle_mgr.streak_fail_reason = 1U; }
                    else if (obs_lock == 0U) { g_angle_mgr.streak_fail_reason = 2U; }
                    else if (g_angle_mgr.window_rms_deg >= 18.0f) { g_angle_mgr.streak_fail_reason = 3U; }
                    else if (g_angle_mgr.window_peak_err >= 30.0f) { g_angle_mgr.streak_fail_reason = 4U; }
                    else if (strict_spd_err_ok == 0U) { g_angle_mgr.streak_fail_reason = 5U; }
                    else if (speed_dir_match == 0U) { g_angle_mgr.streak_fail_reason = 6U; }
                    else if (flux_healthy == 0U) { g_angle_mgr.streak_fail_reason = 7U; }
                    else if (err_deg >= 20.0f) { g_angle_mgr.streak_fail_reason = 8U; }
                    else { g_angle_mgr.streak_fail_reason = 9U; }
                }
                g_angle_mgr.qualified_streak = 0U;
            }

            /* 分级扣减与连续超限时间处理 (qualified_cycles - 动态容忍准入评分) */
            uint8_t is_dynamic_ramp = ((m->mode == FOC_MODE_VELOCITY) && (fabsf(m->target - m->vel_ref_rpm) > 20.0f)) ? 1U : 0U;
            float max_spd_err_ratio = (is_dynamic_ramp != 0U) ? 0.12f : 0.05f;
            uint8_t spd_err_ok = (true_spd_abs > 100.0f) ? ((spd_err / true_spd_abs) < max_spd_err_ratio) : 1U;
            uint8_t rms_ok = ((g_angle_mgr.window_rms_deg < 18.0f) || (g_angle_mgr.window_samples < 60U)) ? 1U : 0U;

            if ((obs_lock == 0U) || (is_nan_or_inf != 0U) || (err_deg > 35.0f) ||
                (flux_healthy == 0U) || (speed_dir_match == 0U)) {
                g_angle_mgr.over_26_streak = 0U;
                g_angle_mgr.ramp_ripple_streak = 0U;
                g_angle_mgr.over_limit_streak++;
                g_angle_mgr.qualified_cycles = 0U;
            } else if (err_deg > 26.0f) {
                g_angle_mgr.over_26_streak++;
                g_angle_mgr.over_limit_streak++;
                g_angle_mgr.ramp_ripple_streak = 0U;
                if (g_angle_mgr.over_26_streak >= 16U) {
                    g_angle_mgr.qualified_cycles = 0U;
                } else {
                    if (g_angle_mgr.qualified_cycles >= 2U) {
                        g_angle_mgr.qualified_cycles -= 2U;
                    } else {
                        g_angle_mgr.qualified_cycles = 0U;
                    }
                }
            } else if (err_deg >= 20.0f) {
                g_angle_mgr.over_26_streak = 0U;
                g_angle_mgr.over_limit_streak++;
                uint8_t can_hold_ramp = ((is_dynamic_ramp != 0U) &&
                                         (obs_lock == 1U) &&
                                         (g_angle_mgr.conf_window >= 0.70f) &&
                                         (g_angle_mgr.window_rms_deg < 18.0f) &&
                                         (g_angle_mgr.window_peak_err < 30.0f) &&
                                         (g_angle_mgr.ramp_ripple_streak < 1600U)) ? 1U : 0U;
                if (can_hold_ramp != 0U) {
                    g_angle_mgr.ramp_ripple_streak++;
                } else {
                    g_angle_mgr.ramp_ripple_streak = 0U;
                    if (g_angle_mgr.qualified_cycles > 0U) {
                        g_angle_mgr.qualified_cycles--;
                    }
                }
            } else {
                g_angle_mgr.over_26_streak = 0U;
                g_angle_mgr.over_limit_streak = 0U;
                g_angle_mgr.ramp_ripple_streak = 0U;
                uint8_t conf_ok = (g_angle_mgr.conf_window >= 0.70f) ? 1U : 0U;
                if ((true_spd_abs >= (g_angle_mgr.exit_speed_rpm - 30.0f)) &&
                    (spd_err_ok != 0U) && (rms_ok != 0U) && (conf_ok != 0U)) {
                    if (g_angle_mgr.qualified_cycles < 32000U) {
                        g_angle_mgr.qualified_cycles++;
                    }
                } else {
                    if (g_angle_mgr.qualified_cycles > 0U) {
                        g_angle_mgr.qualified_cycles--;
                    }
                }
            }
        }
    }

    /* 接管资格候选状态与正式接管门禁:
     * 必须使用严格连续准入 qualified_streak >= 8000 (500ms) 作为接管准入门限！
     * 迟滞退出门限 (qualified_cycles < 4000 拍)：动态加减速纹波容忍，彻底杜绝门限边缘单拍抖动导致状态机颤振退出！ */
    uint8_t takeover_ready = (g_angle_mgr.qualified_streak >= 8000U) ? 1U : 0U;
    uint8_t takeover_drop  = (g_angle_mgr.qualified_cycles < 4000U) ? 1U : 0U;

    /* 多维硬与门门禁判定：长窗连续达标 + 观测器锁定 + 窗口置信度严格>=0.70 + 速度在接管区 + 磁链模长正常 + 转速方向同号一致 */
    uint8_t speed_dir_ok = 1U;
    if (fabsf(m->vel_ref_rpm) > 100.0f) {
        speed_dir_ok = ((spd_obs * m->vel_ref_rpm) > 0.0f) ? 1U : 0U;
    }
    uint8_t obs_healthy_now = ((obs_lock == 1U) &&
                               (g_angle_mgr.conf_window >= 0.70f) &&
                               (fabsf(spd_obs) >= g_angle_mgr.exit_speed_rpm) &&
                               (speed_dir_ok != 0U) &&
                               (flux_healthy != 0U)) ? 1U : 0U;
    uint8_t obs_ready_to_relay = ((g_angle_mgr.qualified_streak >= 8000U) && (obs_healthy_now != 0U)) ? 1U : 0U;

    /* 7. 核心仲裁状态机与平滑过渡 (Blend) */
    float dt = m->dt_fast;
    float blend_step = dt / ((g_angle_mgr.blend_time_s > 0.01f) ? g_angle_mgr.blend_time_s : 0.20f);

    if (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY) {
        /* 第四阶段：Sensorless Primary 纯无感独立启动状态机 (最小安全闭环: VESC + I/F) */
        switch (g_angle_mgr.state) {
        case FOC_ANGLE_SENSORLESS_IF_START: {
            /* 阶段 1: 直流吸附与转子对齐 (Align)
             * 虚拟角置 0，Id 从 0 斜坡升至 if_current_a (100ms)，并在 if_current_a 保持 300ms (总共 400ms / 6400 拍)
             * Iq 置 0，消除突跳与火花 */
            g_angle_mgr.state_ticks++;
            g_angle_mgr.theta_open = 0.0f;
            g_angle_mgr.open_speed_rpm = 0.0f;
            g_angle_mgr.handover_blend = 0.0f;
            g_angle_mgr.qualified_streak = 0U;

            float id_cmd = 0.0f;
            if (g_angle_mgr.state_ticks < 1600U) { /* 前 100ms 斜坡起步 */
                id_cmd = ((float)g_angle_mgr.state_ticks / 1600.0f) * g_angle_mgr.if_current_a;
            } else {
                id_cmd = g_angle_mgr.if_current_a;
            }
            if (id_cmd > 0.80f) { id_cmd = 0.80f; } /* 硬限幅 <=0.80A */
            m->id_ref = id_cmd;
            m->iq_ref = 0.0f;

            g_angle_mgr.theta_control = 0.0f;
            g_angle_mgr.speed_control = 0.0f;

            /* 磁链观测器物理初值对齐注入 (彻底消除零速启动纯积分产生的 90° 直流偏置虚影):
             * 在对齐吸附期间，定子注入直流 Id，转子物理定向于电角度 0° 处保持静止。
             * 此时真实的定子全磁链为: x1 = Ls * id_cmd + psi_m, x2 = 0
             * 将物理真值注入 VESC 观测器初态，确保加速开始时拥有 100% 准确的角度基准 */
            float psi_m = (m->params.ke > 0.01f)
                          ? (m->params.ke / (1.73205f * _2PI * m->params.pole_pairs * 1000.0f / 60.0f))
                          : 0.00080f;
            g_sensorless_bench.vesc_x1 = (m->params.ls_henry * id_cmd) + psi_m;
            g_sensorless_bench.vesc_x2 = 0.0f;
            g_sensorless_bench.vesc_pll_pos = 0.0f;
            g_sensorless_bench.vesc_pll_vel = 0.0f;
            g_sensorless_bench.obs2_vesc.theta_e = 0.0f;
            g_sensorless_bench.obs2_vesc.speed_rpm = 0.0f;
            g_sensorless_bench.obs2_vesc.flux_mag = psi_m;

            /* 硬超时保护: 对齐阶段最大 1.0s (16000 拍) */
            if (g_angle_mgr.state_ticks > 16000U) {
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                g_angle_mgr.lost_reason = 5; /* Timeout */
                foc_motor_fault(m, FOC_FAULT_OBSERVER);
            } else if (g_angle_mgr.state_ticks >= 6400U) { /* 400ms 对齐完成 */
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_IF_ACCEL;
                g_angle_mgr.state_ticks = 0U;
            }
            break;
        }

        case FOC_ANGLE_SENSORLESS_IF_ACCEL: {
            /* 阶段 2: 虚拟电角度开环加速拖动 (Ramp)
             * 速度以 if_accel_rpm_s 斜坡平滑升至 if_target_rpm (默认 500 RPM)
             * Id 保持 if_current_a, Iq 保持 0 (由定子矢量连续旋转牵引转子) */
            g_angle_mgr.state_ticks++;
            int8_t dir = (m->target >= 0.0f) ? 1 : -1;
            float target_rpm = (float)dir * g_angle_mgr.if_target_rpm;
            float spd_step = g_angle_mgr.if_accel_rpm_s * dt;

            if (dir > 0) {
                g_angle_mgr.open_speed_rpm += spd_step;
                if (g_angle_mgr.open_speed_rpm >= target_rpm) {
                    g_angle_mgr.open_speed_rpm = target_rpm;
                }
            } else {
                g_angle_mgr.open_speed_rpm -= spd_step;
                if (g_angle_mgr.open_speed_rpm <= target_rpm) {
                    g_angle_mgr.open_speed_rpm = target_rpm;
                }
            }

            /* 虚拟电角度步进: we = pole_pairs * open_speed_rpm * 2pi / 60 */
            float we_open = (float)m->params.pole_pairs * g_angle_mgr.open_speed_rpm * FOC_RPM_TO_RADS;
            g_angle_mgr.theta_open = foc_wrap_0_2pi(g_angle_mgr.theta_open + (we_open * dt));

            m->id_ref = g_angle_mgr.if_current_a;
            if (m->id_ref > 0.80f) { m->id_ref = 0.80f; }
            m->iq_ref = 0.0f;

            g_angle_mgr.theta_control = g_angle_mgr.theta_open;
            g_angle_mgr.speed_control = g_angle_mgr.open_speed_rpm;

            /* 硬超时保护: 开环加速阶段最大 3.0s (48000 拍) */
            if (g_angle_mgr.state_ticks > 48000U) {
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                g_angle_mgr.lost_reason = 5; /* Timeout */
                foc_motor_fault(m, FOC_FAULT_OBSERVER);
            } else if (fabsf(g_angle_mgr.open_speed_rpm) >= (g_angle_mgr.if_target_rpm - 1.0f)) {
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_OBS_LOCKING;
                g_angle_mgr.state_ticks = 0U;
                g_angle_mgr.qualified_streak = 0U;
            }
            break;
        }

        case FOC_ANGLE_SENSORLESS_OBS_LOCKING: {
            /* 阶段 3: 等待 VESC 观测器完全收敛并硬锁定
             * 恒速拖动在 if_target_rpm (500 RPM)
             * 快环逐拍严审 7 大准入门禁，必须严格连续满足至少 500ms (8000 拍) */
            g_angle_mgr.state_ticks++;

            float we_open = (float)m->params.pole_pairs * g_angle_mgr.open_speed_rpm * FOC_RPM_TO_RADS;
            g_angle_mgr.theta_open = foc_wrap_0_2pi(g_angle_mgr.theta_open + (we_open * dt));

            m->id_ref = g_angle_mgr.if_current_a;
            if (m->id_ref > 0.80f) { m->id_ref = 0.80f; }
            m->iq_ref = 0.0f;

            g_angle_mgr.theta_control = g_angle_mgr.theta_open;
            g_angle_mgr.speed_control = g_angle_mgr.open_speed_rpm;

            /* 7 大准入门禁逐拍检验:
             * 1. 观测器锁定标志 obs_lock == 1
             * 2. 滑动窗口置信度 conf_window >= 0.70
             * 3. 估计速度达到切入转速 (|spd_obs| >= 450 RPM)
             * 4. 磁链模长在标称有效范围内 (0.4 ~ 2.5)
             * 5. 估计速度与开环速度同号一致
             * 6. 相位差检查 (防止 180° 反相歧义)
             */
            float delta_th = wrap_pm_pi(th_obs - g_angle_mgr.theta_open);
            uint8_t lock_ok  = (obs_lock == 1U) ? 1U : 0U;
            uint8_t conf_ok  = (g_angle_mgr.conf_window >= 0.70f) ? 1U : 0U;
            uint8_t spd_ok   = (fabsf(spd_obs) >= (g_angle_mgr.if_target_rpm - 50.0f)) ? 1U : 0U;
            uint8_t flux_ok  = (flux_healthy != 0U) ? 1U : 0U;
            uint8_t dir_ok   = ((spd_obs * g_angle_mgr.open_speed_rpm) > 0.0f) ? 1U : 0U;
            uint8_t delta_ok = (fabsf(delta_th) < (150.0f * _DEG2RAD)) ? 1U : 0U;

            if ((lock_ok != 0U) && (conf_ok != 0U) && (spd_ok != 0U) &&
                (flux_ok != 0U) && (dir_ok != 0U) && (delta_ok != 0U)) {
                g_angle_mgr.qualified_streak++;
            } else {
                if (lock_ok == 0U) { g_angle_mgr.streak_fail_reason = 2U; }
                else if (conf_ok == 0U) { g_angle_mgr.streak_fail_reason = 1U; }
                else if (spd_ok == 0U) { g_angle_mgr.streak_fail_reason = 9U; }
                else if (flux_ok == 0U) { g_angle_mgr.streak_fail_reason = 7U; }
                else if (dir_ok == 0U) { g_angle_mgr.streak_fail_reason = 6U; }
                else { g_angle_mgr.streak_fail_reason = 8U; }
                g_angle_mgr.qualified_streak = 0U;
                g_angle_mgr.streak_fail_count++;
            }

            /* 堵转检测: 若拖动持续超过 2.0s (32000 拍) 且观测器转速 < 100 RPM，判定堵转 */
            if ((g_angle_mgr.state_ticks > 32000U) && (fabsf(spd_obs) < 100.0f)) {
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                g_angle_mgr.lost_reason = 9; /* Stall detected */
                foc_motor_fault(m, FOC_FAULT_OBSERVER);
            } else if (g_angle_mgr.state_ticks > 80000U) { /* 硬超时保护: 锁定等待阶段最大 5.0s (80000 拍) */
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                g_angle_mgr.lost_reason = 5; /* Timeout */
                foc_motor_fault(m, FOC_FAULT_OBSERVER);
            } else if (g_angle_mgr.qualified_streak >= 8000U) {
                /* 连续 500ms 达标，且检查无 180° 极性歧义 */
                if (fabsf(delta_th) > (150.0f * _DEG2RAD)) {
                    g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                    g_angle_mgr.lost_reason = 6; /* 180-deg polarity ambiguity */
                    foc_motor_fault(m, FOC_FAULT_OBSERVER);
                } else {
                    g_angle_mgr.handover_delta_rad = delta_th;
                    g_angle_mgr.handover_blend = 0.0f;
                    g_angle_mgr.state = FOC_ANGLE_SENSORLESS_BLEND;
                    g_angle_mgr.state_ticks = 0U;
                }
            }
            break;
        }

        case FOC_ANGLE_SENSORLESS_BLEND: {
            /* 阶段 4: 开闭环 150ms 平滑无顿挫加权融合
             * 控制角连续衰减: theta_control = th_obs - (1 - blend) * delta_0
             * 首拍严格等于 theta_open，零突变！
             * 电流矢量同时通过定子磁场不变旋转投影平滑过渡到转子坐标系
             * 监视母线电压、角差与锁定，任一异常立即停机 */
            g_angle_mgr.state_ticks++;
            float blend_dt = dt / 0.150f; /* 150ms 过渡 */
            g_angle_mgr.handover_blend += blend_dt;
            if (g_angle_mgr.handover_blend >= 1.0f) {
                g_angle_mgr.handover_blend = 1.0f;
            }

            float decay = 1.0f - g_angle_mgr.handover_blend;
            g_angle_mgr.theta_control = foc_wrap_0_2pi(th_obs - (decay * g_angle_mgr.handover_delta_rad));
            g_angle_mgr.speed_control = spd_obs;

            /* 动基准衰减平滑电流过渡:
             * 开环拖动时，定子牵引电流为 (Id = if_current_a, Iq = 0)。
             * 在进入纯无感闭环稳态后，电磁系统由转子磁极精准定向，不再需要大 Id 强行牵引，
             * 且空载电机维持恒速只需克服轴承机械摩擦 (约 0.12A~0.15A 转矩电流)。
             * 在 150ms BLEND 期间:
             *   Id: 从 if_current_a 平滑衰减至 0.0A;
             *   Iq: 从 0.0A 平滑建立至稳态摩擦补偿电流 (保证转矩连续无缝衔接，消除空载超调加速)。 */
            int8_t dir_cmd = (m->target >= 0.0f) ? 1 : -1;
            float iq_target = (float)dir_cmd * ((m->cfg.vel_friction_a > 0.05f) ? m->cfg.vel_friction_a : 0.120f);
            m->id_ref = decay * g_angle_mgr.if_current_a;
            m->iq_ref = g_angle_mgr.handover_blend * iq_target;

            /* 安全监视门禁:
             * 1. 观测器锁定状态维持 (增加 80 拍 / 5ms 防抖滤波，防止瞬态跳变误判)
             * 2. 磁链幅值正常
             * 3. 瞬时合成电流模长不超过 2.50A (开环设定电流为 0.60A，切换期允许短暂动态调节)
             * 4. 母线电压在合理范围内 (8V ~ 32V)
             * 5. 角差收敛性检查 */
            float max_allowed_delta = (fabsf(g_angle_mgr.handover_delta_rad) * decay) + (60.0f * _DEG2RAD);
            uint8_t delta_bad   = (fabsf(wrap_pm_pi(th_obs - g_angle_mgr.theta_control)) > max_allowed_delta) ? 1U : 0U;
            uint8_t vdc_bad     = ((m->drv->u_dc < 8.0f) || (m->drv->u_dc > 32.0f)) ? 1U : 0U;

            float i_mag_sq = (m->i_dq.d * m->i_dq.d) + (m->i_dq.q * m->i_dq.q);
            uint8_t i_overflow = 0U;
            if (g_angle_mgr.state_ticks > 400U) { /* 25ms 建立期之后再行监视 */
                if (i_mag_sq > (2.50f * 2.50f)) {
                    i_overflow = 1U;
                }
            }

            if (i_overflow != 0U) {
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                g_angle_mgr.lost_reason = 71; /* Current overflow in BLEND */
                foc_motor_fault(m, FOC_FAULT_OBSERVER);
            } else if (obs_lock == 0U) {
                if (++g_angle_mgr.obs_unlock_cnt > 80U) { /* 5ms 失锁防抖 */
                    g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                    g_angle_mgr.lost_reason = 11; /* Observer unlock in BLEND */
                    foc_motor_fault(m, FOC_FAULT_OBSERVER);
                }
            } else if (flux_healthy == 0U) {
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                g_angle_mgr.lost_reason = 12; /* Flux abnormal in BLEND */
                foc_motor_fault(m, FOC_FAULT_OBSERVER);
            } else if (vdc_bad != 0U) {
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                g_angle_mgr.lost_reason = 14; /* Vdc abnormal in BLEND */
                foc_motor_fault(m, FOC_FAULT_OBSERVER);
            } else if (delta_bad != 0U) {
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                g_angle_mgr.lost_reason = 18; /* Phase diff divergence in BLEND */
                foc_motor_fault(m, FOC_FAULT_OBSERVER);
            } else if (g_angle_mgr.state_ticks > 4800U) { /* 硬超时 300ms */
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                g_angle_mgr.lost_reason = 15; /* Timeout in BLEND */
                foc_motor_fault(m, FOC_FAULT_OBSERVER);
            } else if (g_angle_mgr.handover_blend >= 1.0f) {
                /* 过渡完成，正式切入纯无感闭环运行 */
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_RUN;
                g_angle_mgr.state_ticks = 0U;

                /* 速度环 PI 无扰重平衡 (Bumpless Transfer) */
                m->vel_ref_rpm = m->target;
                m->velocity_filt_rpm = g_angle_mgr.speed_control;
                foc_speed_filter_reset(&m->vel_filter, g_angle_mgr.speed_control);
                foc_speed_filter_reset(&m->vel_filter_low, g_angle_mgr.speed_control);
                float spd_err_now = m->target - g_angle_mgr.speed_control;
                float p_term = m->pid_vel.kp * spd_err_now;
                float i_term = m->iq_ref - p_term;
                if (m->pid_vel.out_limit > 0.0f) {
                    i_term = foc_clampf(i_term, -m->pid_vel.out_limit, m->pid_vel.out_limit);
                }
                m->pid_vel.integral = i_term;
                m->pid_vel.prev_output = m->iq_ref;
                m->pid_vel.prev_error = spd_err_now;
            } else {
                g_angle_mgr.obs_unlock_cnt = 0U;
            }
            break;
        }

        case FOC_ANGLE_SENSORLESS_RUN: {
            /* 阶段 5: 纯无感闭环稳定运行
             * 编码器仅作为后台诊断对比，严禁进入控制链
             * 严密监控观测器内生状态: lock, flux, speed divergence, NaN */
            g_angle_mgr.state_ticks++;
            g_angle_mgr.handover_blend = 1.0f;
            g_angle_mgr.theta_control = th_obs;
            g_angle_mgr.speed_control = spd_obs;

            /* Id 残余电流在 200ms 内平滑归零，由速度环 PI 自然接管全部转矩 */
            if (fabsf(m->id_ref) > 0.001f) {
                float id_decay_step = (g_angle_mgr.if_current_a / 0.200f) * dt;
                if (m->id_ref > id_decay_step) {
                    m->id_ref -= id_decay_step;
                } else if (m->id_ref < -id_decay_step) {
                    m->id_ref += id_decay_step;
                } else {
                    m->id_ref = 0.0f;
                }
            }

            uint8_t hard_fault = 0U;
            if (flux_healthy == 0U) {
                hard_fault = 2U;
            } else if ((m->vel_ref_rpm * spd_obs < -1e-3f) && (fabsf(spd_obs) > 200.0f)) {
                hard_fault = 3U;
            } else if ((th_obs != th_obs) || (spd_obs != spd_obs) || (is_nan_or_inf != 0U)) {
                hard_fault = 4U;
            }

            if (hard_fault != 0U) {
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                g_angle_mgr.lost_reason = hard_fault;
                foc_motor_fault(m, FOC_FAULT_OBSERVER);
            } else if (obs_lock == 0U) {
                if (++g_angle_mgr.obs_unlock_cnt > 160U) { /* 10ms 失锁滤波 */
                    g_angle_mgr.state = FOC_ANGLE_SENSORLESS_LOST;
                    g_angle_mgr.lost_reason = 1U;
                    foc_motor_fault(m, FOC_FAULT_OBSERVER);
                }
            } else {
                g_angle_mgr.obs_unlock_cnt = 0U;
            }
            break;
        }

        case FOC_ANGLE_SENSORLESS_LOST: {
            /* 纯无感失锁确认态: 保持当前状态一拍记录快照，并在下一拍转入 SAFE_STOP */
            g_angle_mgr.handover_blend = 0.0f;
            g_angle_mgr.theta_control = 0.0f;
            g_angle_mgr.speed_control = 0.0f;
            m->id_ref = 0.0f;
            m->iq_ref = 0.0f;
            foc_motor_fault(m, FOC_FAULT_OBSERVER);
            if (++g_angle_mgr.state_ticks > 1600U) { /* 保持 100ms 快照后切入 SAFE_STOP */
                g_angle_mgr.state = FOC_ANGLE_SAFE_STOP;
            }
            break;
        }

        case FOC_ANGLE_SAFE_STOP:
        default: {
            /* 停机保护保持态: 维持 PWM 关断，不重复报错 */
            g_angle_mgr.handover_blend = 0.0f;
            g_angle_mgr.theta_control = 0.0f;
            g_angle_mgr.speed_control = 0.0f;
            m->id_ref = 0.0f;
            m->iq_ref = 0.0f;
            break;
        }
        }
    } else {
        /* 原有的有感主控 (SENSORED_PRIMARY) 与 自动接管 (AUTO_FALLBACK) 状态机 */
        switch (g_angle_mgr.state) {
        case FOC_ANGLE_SENSORED:
            g_angle_mgr.handover_blend = 0.0f;
            if ((g_angle_mgr.mode == FOC_FEEDBACK_AUTO_FALLBACK) &&
                (g_angle_mgr.enc_health == ENCODER_HEALTH_FAILED)) {
                if (obs_ready_to_relay != 0U) {
                    g_angle_mgr.state = FOC_ANGLE_BLEND_TO_SENSORLESS;
                    g_angle_mgr.handover_delta_rad = wrap_pm_pi(th_enc - th_obs);
                } else {
                    /* 编码器损坏但无感未满足硬门禁 (双故障/未就绪)：坚决禁止切入无感，立即执行 SAFE_STOP 停机保护 */
                    g_angle_mgr.state = FOC_ANGLE_SAFE_STOP;
                    g_angle_mgr.fallback_reason = FALLBACK_OBS_UNLOCK;
                    foc_motor_fault(m, FOC_FAULT_CONTROL_NAN);
                }
            } else if (takeover_ready != 0U) {
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS_CANDIDATE;
            }
            break;

        case FOC_ANGLE_SENSORLESS_CANDIDATE:
            g_angle_mgr.handover_blend = 0.0f;
            if (g_angle_mgr.mode == FOC_FEEDBACK_AUTO_FALLBACK) {
                uint8_t enc_fault_present = ((g_angle_mgr.enc_health == ENCODER_HEALTH_FAILED) ||
                                             (g_angle_mgr.enc_stagnant_cnt >= 4U)) ? 1U : 0U;
                if (enc_fault_present != 0U) {
                    if (obs_healthy_now != 0U) {
                        /* 单故障：已获准入资格期间编码器故障，且无感观测器瞬时健康 -> 平滑接管 */
                        g_angle_mgr.state = FOC_ANGLE_BLEND_TO_SENSORLESS;
                        g_angle_mgr.handover_delta_rad = wrap_pm_pi(th_enc - th_obs);
                        g_angle_mgr.enc_health = ENCODER_HEALTH_FAILED;
                    } else {
                        /* 双故障：编码器故障，且无感亦未满足硬门禁(如conf<0.70或失锁) -> 坚决禁止切入无感，立即安全停机！ */
                        g_angle_mgr.state = FOC_ANGLE_SAFE_STOP;
                        g_angle_mgr.fallback_reason = FALLBACK_OBS_UNLOCK;
                        foc_motor_fault(m, FOC_FAULT_CONTROL_NAN);
                    }
                } else if ((takeover_drop != 0U) && (g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL)) {
                    g_angle_mgr.state = FOC_ANGLE_SENSORED;
                }
            } else if ((takeover_drop != 0U) && (g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL)) {
                g_angle_mgr.state = FOC_ANGLE_SENSORED;
            }
            break;

        case FOC_ANGLE_BLEND_TO_SENSORLESS:
            /* 平滑线性切向无感: 0.0 -> 1.0 */
            g_angle_mgr.handover_blend += blend_step;
            if (g_angle_mgr.handover_blend >= 1.0f) {
                g_angle_mgr.handover_blend = 1.0f;
                g_angle_mgr.state = FOC_ANGLE_SENSORLESS;
            }
            /* 切换期间安全监视：引入 800 拍 (50ms) 工业级防抖滤波，防止过渡期瞬态波动误停机 */
            if (obs_lock == 0U) {
                if (++g_angle_mgr.obs_unlock_cnt > 800U) {
                    if (g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL) {
                        g_angle_mgr.state = FOC_ANGLE_BLEND_TO_SENSORED;
                        g_angle_mgr.handover_delta_rad = wrap_pm_pi(th_obs - th_enc);
                        g_angle_mgr.fallback_reason = FALLBACK_OBS_UNLOCK;
                    } else {
                        g_angle_mgr.state = FOC_ANGLE_SAFE_STOP;
                        foc_motor_fault(m, FOC_FAULT_CONTROL_NAN);
                    }
                }
            } else {
                g_angle_mgr.obs_unlock_cnt = 0U;
            }
            break;

        case FOC_ANGLE_SENSORLESS: {
            g_angle_mgr.handover_blend = 1.0f;

            /* 无感闭环运行中的回退逻辑:
             * 1. 编码器已完全恢复健康 -> 启动平滑淡回编码器。
             * 2. 转速低于安全切出转速 -> 若编码器健康则回退，若编码器亦损坏则触发安全停机 */
            uint8_t enc_ready_to_return = (g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL) ? 1U : 0U;
            if (enc_ready_to_return != 0U) {
                g_angle_mgr.state = FOC_ANGLE_BLEND_TO_SENSORED;
                g_angle_mgr.handover_delta_rad = wrap_pm_pi(th_obs - th_enc);
            } else if (fabsf(spd_obs) < g_angle_mgr.exit_speed_rpm) {
                if (g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL) {
                    g_angle_mgr.state = FOC_ANGLE_BLEND_TO_SENSORED;
                    g_angle_mgr.handover_delta_rad = wrap_pm_pi(th_obs - th_enc);
                    g_angle_mgr.fallback_reason = FALLBACK_SPEED_TOO_LOW;
                } else {
                    if (++g_angle_mgr.obs_unlock_cnt > 800U) {
                        g_angle_mgr.state = FOC_ANGLE_SAFE_STOP;
                        g_angle_mgr.fallback_reason = FALLBACK_SPEED_TOO_LOW;
                        foc_motor_fault(m, FOC_FAULT_CONTROL_NAN);
                    }
                }
            }

            /* 若无感闭环期间自身出现异常：区分软故障防抖与硬故障瞬时停机 */
            uint8_t hard_fault = 0U;
            if (flux_healthy == 0U) {
                hard_fault = 1U;
            }
            if ((m->vel_ref_rpm * spd_obs < -1e-3f) && (fabsf(spd_obs) > 400.0f)) {
                hard_fault = 1U;
            }
            if ((th_obs != th_obs) || (spd_obs != spd_obs) || (is_nan_or_inf != 0U)) {
                hard_fault = 1U;
            }
            if ((g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL) &&
                ((err_deg > 35.0f) || (g_angle_mgr.over_26_streak >= 16U))) {
                hard_fault = 1U;
            }

            if (hard_fault != 0U) {
                if (++g_angle_mgr.obs_unlock_cnt > 5U) {
                    if (g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL) {
                        g_angle_mgr.state = FOC_ANGLE_BLEND_TO_SENSORED;
                        g_angle_mgr.handover_delta_rad = wrap_pm_pi(th_obs - th_enc);
                        g_angle_mgr.fallback_reason = FALLBACK_OBS_UNLOCK;
                    } else {
                        g_angle_mgr.state = FOC_ANGLE_SAFE_STOP;
                        foc_motor_fault(m, FOC_FAULT_CONTROL_NAN);
                    }
                }
            } else if (obs_lock == 0U) {
                if (++g_angle_mgr.obs_unlock_cnt > 800U) {
                    if (g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL) {
                        g_angle_mgr.state = FOC_ANGLE_BLEND_TO_SENSORED;
                        g_angle_mgr.handover_delta_rad = wrap_pm_pi(th_obs - th_enc);
                        g_angle_mgr.fallback_reason = FALLBACK_OBS_UNLOCK;
                    } else {
                        g_angle_mgr.state = FOC_ANGLE_SAFE_STOP;
                        foc_motor_fault(m, FOC_FAULT_CONTROL_NAN);
                    }
                }
            } else {
                g_angle_mgr.obs_unlock_cnt = 0U;
            }
            break;
        }

        case FOC_ANGLE_BLEND_TO_SENSORED:
            /* 平滑线性切回编码器: 1.0 -> 0.0 */
            g_angle_mgr.handover_blend -= blend_step;
            if (g_angle_mgr.handover_blend <= 0.0f) {
                g_angle_mgr.handover_blend = 0.0f;
                g_angle_mgr.state = FOC_ANGLE_SENSORED;
            }
            break;

        case FOC_ANGLE_SAFE_STOP:
        default:
            g_angle_mgr.handover_blend = 0.0f;
            break;
        }
    }

    /* 8. 生成最终换相控制角度 theta_control 与 控制转速 speed_control
     * 采用严谨连续的相位衰减与无缝转速接管：
     * 接管时：以高速连续旋转的 th_obs 为基准，平滑衰减初始相位差 handover_delta_rad；
     * 回退时：以健康连续旋转的 th_enc 为基准，平滑衰减初始相位差 handover_delta_rad；
     * 绝不将冻结停滞的异常编码器读数卷入加权旋转！确保角速度始终与真实转速一致！ */
    if ((m->mode == FOC_MODE_OPENLOOP_VF) || (m->state == FOC_STATE_CALIB)) {
        g_angle_mgr.theta_control = m->theta_e;
        g_angle_mgr.speed_control = enc_spd_rpm;
    } else if (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY) {
        /* SENSORLESS_PRIMARY 各状态内部已完成控制电角度与控制速度的逐拍平滑构建 */
    } else {
        if ((g_angle_mgr.state == FOC_ANGLE_SENSORLESS) || (g_angle_mgr.handover_blend >= 0.999f)) {
            /* 处于纯无感接管态：控制量直接由平滑且正确的无感观测器全面驱动 */
            g_angle_mgr.theta_control = th_obs;
            g_angle_mgr.speed_control = spd_obs;
        } else if (g_angle_mgr.state == FOC_ANGLE_BLEND_TO_SENSORLESS) {
            /* 切向无感过渡区：以高速旋转的 th_obs 为基准，将初始残差 handover_delta_rad 衰减至 0 */
            float decay = 1.0f - g_angle_mgr.handover_blend;
            g_angle_mgr.theta_control = foc_wrap_0_2pi(th_obs + (decay * g_angle_mgr.handover_delta_rad));
            g_angle_mgr.speed_control = spd_obs;
        } else if (g_angle_mgr.state == FOC_ANGLE_BLEND_TO_SENSORED) {
            /* 切回编码器过渡区：以已恢复健康且高速旋转的 th_enc 为基准，将初始残差 handover_delta_rad 衰减至 0 */
            g_angle_mgr.theta_control = foc_wrap_0_2pi(th_enc + (g_angle_mgr.handover_blend * g_angle_mgr.handover_delta_rad));
            g_angle_mgr.speed_control = enc_spd_rpm;
        } else {
            /* 编码器主控态 (SENSORED 或 SENSORLESS_CANDIDATE)：严格输出 th_enc，绝不单拍阶跃！ */
            g_angle_mgr.theta_control = th_enc;
            g_angle_mgr.speed_control = enc_spd_rpm;
        }
    }

    return g_angle_mgr.theta_control;
}
