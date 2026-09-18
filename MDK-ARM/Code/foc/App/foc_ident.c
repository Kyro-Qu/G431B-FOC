/**
 * @file    foc_ident.c
 * @brief   电机参数自动测量实现（Rs 升压法 / Ls 方波注入法 / 极对数开环位移法 / 磁链稳态拟合法）
 */

#include "foc_ident.h"
#include "foc_cmd.h"
#include "foc_config.h"
#include "foc_utils.h"
#include "main.h"
#include <math.h>

/* ---- 测量参数（保守默认值） ---- */
#define IDENT_TEST_CURRENT_A   1.0f    /* Rs 目标电流 */
#define IDENT_MAX_VOLTAGE_V    2.0f    /* Rs 电压上限 */
#define IDENT_MIN_CURRENT_A    0.1f    /* 平均电流低于此值判失败 */
#define IDENT_RAMP_GAIN        0.003f  /* 升压积分增益 V/(A·ms) */
#define IDENT_BOOTSTRAP_MS     10U
#define IDENT_NEUTRAL_MS       200U
#define IDENT_RAMP_MS          1000U   /* 升压+稳定总时长 */
#define IDENT_MEASURE_MS       400U    /* Rs 平均窗口 */
#define IDENT_LS_VOLTAGE_V     0.15f   /* Ls 方波幅值 */
#define IDENT_LS_CYCLES        4096U   /* Ls 记账拍数 */
#define IDENT_LS_SKIP          8U      /* 起始丢弃拍数 */
#define IDENT_LS_MIN_H         0.0000005f /* 0.5 µH */
#define IDENT_LS_MAX_H         0.02f      /* 20 mH */

/* ---- 极对数与磁链辨识参数 ---- */
#define IDENT_PP_ALIGN_MS      400U    /* 极对数起始吸附时间 */
#define IDENT_PP_TARGET_ELEC   (4.0f * _2PI) /* 旋转 4 个完整电周期 (8*pi) */
#define IDENT_PP_FREQ_HZ       2.0f    /* 电频率 2Hz (平稳转动不失步) */
#define IDENT_FLUX_SPEED_RPM   300.0f  /* 磁链辨识转速 RPM */
#define IDENT_FLUX_RAMP_MS     1200U   /* 磁链辨识加速稳定时间 */
#define IDENT_FLUX_SAMPLE_MS   600U    /* 磁链稳态采样时间 */

static foc_motor_t *ident_motor = 0;
static foc_ident_state_t ident_state = FOC_IDENT_IDLE;
static foc_ident_mode_t ident_mode = FOC_IDENT_MODE_FULL;
static foc_ident_result_t ident_result = {0};
static uint32_t ident_tick = 0U;
static uint32_t ident_last_ms = 0U;

/* Rs 阶段累计 */
static float rs_voltage = 0.0f;
static float rs_v_sum = 0.0f;
static float rs_i_sum = 0.0f;
static uint32_t rs_samples = 0U;

/* Ls 阶段累计 */
static volatile float ls_vs_sum = 0.0f;
static volatile float ls_di_sum = 0.0f;
static volatile uint32_t ls_count = 0U;
static float ls_i_prev = 0.0f;
static int8_t ls_pol_applied = 0;
static uint8_t ls_phase = 0U;

/* 极对数辨识累计 */
static float pp_start_mech = 0.0f;
static float pp_total_elec = 0.0f;
static float pp_hold_v = 0.35f;

/* 磁链辨识累计 */
static float flux_v_sum = 0.0f;
static float flux_i_sum = 0.0f;
static float flux_speed_sum = 0.0f;
static uint32_t flux_samples = 0U;

static void ident_set_state(foc_ident_state_t s)
{
    ident_state = s;
}

static void ident_cleanup(foc_motor_t *m)
{
    m->test_hook = 0;
    m->drv->disable();
    foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
    foc_motor_restore_current_limits(m);
    m->pwm_hold = 0U;
    m->id_ref = 0.0f;
    m->iq_ref = 0.0f;
    m->v_dq.d = 0.0f;
    m->v_dq.q = 0.0f;
    m->v_openloop.d = 0.0f;
    m->v_openloop.q = 0.0f;
    m->ol_angle_step = 0.0f;
    if (m->state != FOC_STATE_FAULT) {
        m->state = FOC_STATE_IDLE;
    }
}

static void ident_fail(const char *reason)
{
    if (ident_motor != 0) {
        ident_cleanup(ident_motor);
    }
    ident_set_state(FOC_IDENT_FAIL);
    foc_cmd_print("ident FAIL: %s\r\n", reason);
}

/* Ls 测量钩子：在快环（16kHz）中逐拍翻转方波电压 */
static void ident_ls_hook(foc_motor_t *m)
{
    float i_now = m->i_dq.d;

    if (ls_count < (IDENT_LS_CYCLES + IDENT_LS_SKIP)) {
        if ((ls_count >= IDENT_LS_SKIP) && (ls_phase == 1U)) {
            float pol = (float)ls_pol_applied;
            float applied_v = pol * IDENT_LS_VOLTAGE_V;

            ls_vs_sum += pol *
                (applied_v - (ident_result.rs_ohm * ls_i_prev)) *
                m->dt_fast;
            ls_di_sum += pol * (i_now - ls_i_prev);
        }
        ++ls_count;
    }
    ls_i_prev = i_now;

    if ((ls_pol_applied != 0) && (ls_phase == 0U)) {
        ls_phase = 1U;
        return;
    }

    ls_pol_applied = (ls_pol_applied == 1) ? -1 : 1;
    ls_phase = 0U;
    m->v_openloop.d = (float)ls_pol_applied * IDENT_LS_VOLTAGE_V;
    m->v_openloop.q = 0.0f;
}

/* Lq 测量钩子：在快环（16kHz）中沿 q 轴逐拍翻转方波电压 */
static void ident_lq_hook(foc_motor_t *m)
{
    float i_now = m->i_dq.q;

    if (ls_count < (IDENT_LS_CYCLES + IDENT_LS_SKIP)) {
        if ((ls_count >= IDENT_LS_SKIP) && (ls_phase == 1U)) {
            float pol = (float)ls_pol_applied;
            float applied_v = pol * IDENT_LS_VOLTAGE_V;

            ls_vs_sum += pol *
                (applied_v - (ident_result.rs_ohm * ls_i_prev)) *
                m->dt_fast;
            ls_di_sum += pol * (i_now - ls_i_prev);
        }
        ++ls_count;
    }
    ls_i_prev = i_now;

    if ((ls_pol_applied != 0) && (ls_phase == 0U)) {
        ls_phase = 1U;
        return;
    }

    ls_pol_applied = (ls_pol_applied == 1) ? -1 : 1;
    ls_phase = 0U;
    m->v_openloop.d = 0.0f;
    m->v_openloop.q = (float)ls_pol_applied * IDENT_LS_VOLTAGE_V;
}

void foc_ident_start_mode(foc_motor_t *m, foc_ident_mode_t mode)
{
    if ((m == 0) || (m->state != FOC_STATE_IDLE)) {
        foc_cmd_print("err: ident needs IDLE\r\n");
        return;
    }
    if ((m->cur == 0) || (m->cur->is_ready() == 0U)) {
        foc_cmd_print("err: ident needs current sensing\r\n");
        return;
    }
    if (((mode == FOC_IDENT_MODE_FULL) || (mode == FOC_IDENT_MODE_PP) ||
         (mode == FOC_IDENT_MODE_FLUX)) && (m->sensor == 0)) {
        foc_cmd_print("err: Pp/Flux ident needs encoder sensor\r\n");
        return;
    }

    ident_motor = m;
    ident_mode = mode;

    if (mode == FOC_IDENT_MODE_FULL) {
        ident_result.valid = 0U;
        ident_result.has_rs_ls = 0U;
        ident_result.has_pp = 0U;
        ident_result.has_flux = 0U;
        ident_result.has_ld_lq = 0U;
    } else if (mode == FOC_IDENT_MODE_LD_LQ) {
        ident_result.valid = 0U;
        ident_result.has_ld_lq = 0U;
    }

    rs_voltage = 0.0f;
    rs_v_sum = 0.0f;
    rs_i_sum = 0.0f;
    rs_samples = 0U;

    /* 辨识期间软限 4.5A，硬限 7.5A（给旋转瞬态留出安全裕量，防止误跳闸） */
    foc_motor_override_current_limits(m, 4.5f, 7.5f);

    ident_tick = HAL_GetTick();
    ident_last_ms = ident_tick;
    ident_set_state(FOC_IDENT_BOOTSTRAP);

    foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
    m->pwm_hold = 1U;
    m->state = FOC_STATE_CALIB;
    m->drv->bootstrap();

    switch (mode) {
    case FOC_IDENT_MODE_FULL:
        foc_cmd_print("ident start: Full Suite (Rs->Ld->Lq->Pp->Flux, ~7s)\r\n");
        break;
    case FOC_IDENT_MODE_RS_LS:
        foc_cmd_print("ident start: Rs & Ls only (~2s, rotor locks)\r\n");
        break;
    case FOC_IDENT_MODE_LD_LQ:
        foc_cmd_print("ident start: Ld/Lq Saliency Suite (~3s, rotor locks)\r\n");
        break;
    case FOC_IDENT_MODE_PP:
        foc_cmd_print("ident start: Pole Pairs only (~3s, rotor turns)\r\n");
        break;
    case FOC_IDENT_MODE_FLUX:
        foc_cmd_print("ident start: Flux & Ke only (~3s, rotor spins)\r\n");
        break;
    default:
        break;
    }
}

void foc_ident_start(foc_motor_t *m)
{
    foc_ident_start_mode(m, FOC_IDENT_MODE_FULL);
}

void foc_ident_task(void)
{
    foc_motor_t *m = ident_motor;
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms;

    if ((m == 0) || (foc_ident_is_active() == 0U)) {
        return;
    }

    if (m->state == FOC_STATE_FAULT) {
        m->test_hook = 0;
        foc_motor_restore_current_limits(m);
        m->pwm_hold = 0U;
        ident_set_state(FOC_IDENT_FAIL);
        foc_cmd_print("ident FAIL: fault=%u\r\n",
                      (unsigned)m->safety.fault_code);
        return;
    }
    if (m->state != FOC_STATE_CALIB) {
        ident_cleanup(m);
        ident_set_state(FOC_IDENT_FAIL);
        return;
    }
    if (m->cur->is_ready() == 0U) {
        ident_fail("current sense lost");
        return;
    }

    dt_ms = now - ident_last_ms;

    switch (ident_state) {
    case FOC_IDENT_BOOTSTRAP:
        if ((uint32_t)(now - ident_tick) >= IDENT_BOOTSTRAP_MS) {
            m->drv->disable();
            m->pwm_hold = 0U;
            foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
            ident_tick = now;
            ident_set_state(FOC_IDENT_NEUTRAL);
            m->drv->enable();
        }
        break;

    case FOC_IDENT_NEUTRAL:
        if ((uint32_t)(now - ident_tick) >= IDENT_NEUTRAL_MS) {
            ident_tick = now;
            if ((ident_mode == FOC_IDENT_MODE_FULL) ||
                (ident_mode == FOC_IDENT_MODE_RS_LS) ||
                (ident_mode == FOC_IDENT_MODE_LD_LQ)) {
                ident_set_state(FOC_IDENT_RS_RAMP);
            } else if (ident_mode == FOC_IDENT_MODE_PP) {
                pp_hold_v = 0.35f;
                ident_set_state(FOC_IDENT_PP_ALIGN);
            } else if (ident_mode == FOC_IDENT_MODE_FLUX) {
                ident_set_state(FOC_IDENT_FLUX_SPIN);
            }
        }
        break;

    case FOC_IDENT_RS_RAMP:
        if (dt_ms > 0U) {
            rs_voltage += IDENT_RAMP_GAIN * (float)dt_ms *
                          (IDENT_TEST_CURRENT_A - m->i_dq_filt.d);
            rs_voltage = foc_clampf(rs_voltage, 0.0f, IDENT_MAX_VOLTAGE_V);
            foc_motor_openloop_hold(m, 0.0f, rs_voltage, 0.0f);
        }
        if ((uint32_t)(now - ident_tick) >= IDENT_RAMP_MS) {
            ident_tick = now;
            ident_set_state(FOC_IDENT_RS_MEASURE);
        }
        break;

    case FOC_IDENT_RS_MEASURE:
        if (dt_ms > 0U) {
            rs_v_sum += rs_voltage;
            rs_i_sum += m->i_dq_filt.d;
            ++rs_samples;
        }
        if ((uint32_t)(now - ident_tick) >= IDENT_MEASURE_MS) {
            float i_avg;

            if (rs_samples == 0U) {
                ident_fail("no samples");
                break;
            }
            i_avg = rs_i_sum / (float)rs_samples;

            if (i_avg < IDENT_MIN_CURRENT_A) {
                ident_fail("no measurable current (check wiring)");
                break;
            }
            ident_result.rs_ohm = (rs_v_sum / (float)rs_samples) / i_avg;
            ident_result.test_current_a = i_avg;

            if ((ident_mode == FOC_IDENT_MODE_LD_LQ) || (ident_mode == FOC_IDENT_MODE_FULL)) {
                /* 进入 Ld 测量：先撤压，由钩子接管 d 轴高频方波 */
                foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
                ls_vs_sum = 0.0f;
                ls_di_sum = 0.0f;
                ls_count = 0U;
                ls_i_prev = 0.0f;
                ls_pol_applied = 0;
                ls_phase = 0U;
                m->test_hook = ident_ls_hook;
                ident_tick = now;
                ident_set_state(FOC_IDENT_LD_MEASURE);
                break;
            }

            /* 进入 Ls：先撤压，由钩子接管 d 轴电压 */
            foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
            ls_vs_sum = 0.0f;
            ls_di_sum = 0.0f;
            ls_count = 0U;
            ls_i_prev = 0.0f;
            ls_pol_applied = 0;
            ls_phase = 0U;
            m->test_hook = ident_ls_hook;
            ident_tick = now;
            ident_set_state(FOC_IDENT_LS);
        }
        break;

    case FOC_IDENT_LD_MEASURE:
        if (ls_count >= (IDENT_LS_CYCLES + IDENT_LS_SKIP)) {
            float di = ls_di_sum;
            float ld;

            m->test_hook = 0;
            if ((di < 1e-3f) && (di > -1e-3f)) {
                ident_fail("no current response on d-axis");
                break;
            }
            ld = ls_vs_sum / di;
            if (!((ld >= IDENT_LS_MIN_H) && (ld <= IDENT_LS_MAX_H))) {
                ident_fail("Ld out of range or noisy");
                break;
            }
            ident_result.ld_henry = ld;

            /* 准备进入 Lq 测量：转子仍保持在当前对齐位置，由 ident_lq_hook 接管 q 轴方波 */
            foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
            ls_vs_sum = 0.0f;
            ls_di_sum = 0.0f;
            ls_count = 0U;
            ls_i_prev = 0.0f;
            ls_pol_applied = 0;
            ls_phase = 0U;
            m->test_hook = ident_lq_hook;
            ident_tick = now;
            ident_set_state(FOC_IDENT_LQ_MEASURE);
        } else if ((uint32_t)(now - ident_tick) >= 2000U) {
            ident_fail("Ld timeout");
        }
        break;

    case FOC_IDENT_LQ_MEASURE:
        if (ls_count >= (IDENT_LS_CYCLES + IDENT_LS_SKIP)) {
            float di = ls_di_sum;
            float lq;

            m->test_hook = 0;
            if ((di < 1e-3f) && (di > -1e-3f)) {
                ident_fail("no current response on q-axis");
                break;
            }
            lq = ls_vs_sum / di;
            if (!((lq >= IDENT_LS_MIN_H) && (lq <= IDENT_LS_MAX_H))) {
                ident_fail("Lq out of range or noisy");
                break;
            }
            ident_result.lq_henry = lq;
            ident_result.ls_henry = 0.5f * (ident_result.ld_henry + lq);
            ident_result.delta_l_henry = lq - ident_result.ld_henry;
            ident_result.saliency_ratio = (ident_result.ld_henry > 1e-6f)
                                          ? (ident_result.delta_l_henry / ident_result.ld_henry)
                                          : 0.0f;
            ident_result.has_ld_lq = 1U;
            ident_result.has_rs_ls = 1U;
            ident_result.valid = 1U;

            foc_cmd_print("ident: Rs=%.4f ohm  Ld=%.2f uH  Lq=%.2f uH  (I=%.2fA)\r\n",
                          (double)ident_result.rs_ohm,
                          (double)(ident_result.ld_henry * 1e6f),
                          (double)(ident_result.lq_henry * 1e6f),
                          (double)ident_result.test_current_a);
            foc_cmd_print("ident: Delta_L=%+.2f uH, Saliency Ratio=%+.2f%% (Lq/Ld=%.3f)\r\n",
                          (double)(ident_result.delta_l_henry * 1e6f),
                          (double)(ident_result.saliency_ratio * 100.0f),
                          (double)(ident_result.lq_henry / ident_result.ld_henry));

            if (ident_mode == FOC_IDENT_MODE_LD_LQ) {
                ident_cleanup(m);
                ident_set_state(FOC_IDENT_DONE);
                foc_cmd_print("ident DONE: Ld/Lq ready.\r\n");
            } else {
                /* 全套辨识顺利过渡到极对数测量：按 Rs * 1.5A 计算对齐电压 */
                pp_hold_v = foc_clampf(ident_result.rs_ohm * 1.5f, 0.20f, 0.50f);
                ident_tick = now;
                ident_set_state(FOC_IDENT_PP_ALIGN);
            }
        } else if ((uint32_t)(now - ident_tick) >= 2000U) {
            ident_fail("Lq timeout");
        }
        break;

    case FOC_IDENT_LS:
        if (ls_count >= (IDENT_LS_CYCLES + IDENT_LS_SKIP)) {
            float di = ls_di_sum;
            float ls;

            m->test_hook = 0;
            if ((di < 1e-3f) && (di > -1e-3f)) {
                ident_fail("no current response (Ls too big or wiring)");
                break;
            }
            ls = ls_vs_sum / di;
            if (!((ls >= IDENT_LS_MIN_H) && (ls <= IDENT_LS_MAX_H))) {
                ident_fail("Ls out of range or noisy");
                break;
            }
            ident_result.ls_henry = ls;
            ident_result.has_rs_ls = 1U;

            foc_cmd_print("ident: Rs=%.4f ohm  Ls=%.2f uH  (I=%.2fA)\r\n",
                          (double)ident_result.rs_ohm,
                          (double)(ident_result.ls_henry * 1e6f),
                          (double)ident_result.test_current_a);

            if (ident_mode == FOC_IDENT_MODE_RS_LS) {
                ident_result.valid = 1U;
                ident_cleanup(m);
                ident_set_state(FOC_IDENT_DONE);
                foc_cmd_print("ident DONE: Rs/Ls ready. Apply with 'ident apply'\r\n");
            } else {
                /* 顺利过渡到极对数测量：按 Rs * 1.5A 计算对齐电压 */
                pp_hold_v = foc_clampf(ident_result.rs_ohm * 1.5f, 0.20f, 0.50f);
                ident_tick = now;
                ident_set_state(FOC_IDENT_PP_ALIGN);
            }
        } else if ((uint32_t)(now - ident_tick) >= 2000U) {
            ident_fail("Ls timeout");
        }
        break;

    case FOC_IDENT_PP_ALIGN: {
        /* 在电角度 0 施加缓升吸附电压，让转子可靠平稳对齐归正 */
        uint32_t align_elapsed = now - ident_tick;
        float v_cur = pp_hold_v;
        if (align_elapsed < 200U) {
            v_cur = pp_hold_v * ((float)align_elapsed / 200.0f);
        }
        foc_motor_openloop_hold(m, 0.0f, v_cur, 0.0f);
        if (align_elapsed >= IDENT_PP_ALIGN_MS) {
            pp_start_mech = m->position_rad;
            pp_total_elec = 0.0f;
            ident_tick = now;
            ident_set_state(FOC_IDENT_PP_SPIN);
        }
        break;
    }

    case FOC_IDENT_PP_SPIN:
        /* 以 2Hz 电角频率匀速旋转 4 个电周期 (8*pi) */
        if (dt_ms > 0U) {
            float d_elec = _2PI * IDENT_PP_FREQ_HZ * (float)dt_ms * 0.001f;
            pp_total_elec += d_elec;
            foc_motor_openloop_hold(m, pp_total_elec, pp_hold_v, 0.0f);
        }
        if (pp_total_elec >= IDENT_PP_TARGET_ELEC) {
            float delta_mech = fabsf(m->position_rad - pp_start_mech);
            float pp_raw;
            float pp_est;
            float residual;

            if (delta_mech < 0.15f) {
                ident_fail("rotor did not rotate (stalled or encoder lost)");
                break;
            }

            pp_raw = pp_total_elec / delta_mech;
            pp_est = roundf(pp_raw);
            residual = fabsf(pp_raw - pp_est) / pp_est;

            ident_result.pole_pairs = pp_est;
            ident_result.pp_calc_raw = pp_raw;
            ident_result.pp_residual = residual;

            if ((pp_est < 1.0f) || (pp_est > 32.0f) || (residual > 0.12f)) {
                foc_cmd_print("ident WARN: Pp=%.2f (est=%.0f, residual=%.1f%%) doubtful!\r\n",
                              (double)pp_raw, (double)pp_est, (double)(residual * 100.0f));
            } else {
                foc_cmd_print("ident: Pole Pairs = %.0f (raw=%.2f, residual=%.1f%%)\r\n",
                              (double)pp_est, (double)pp_raw, (double)(residual * 100.0f));
            }
            ident_result.has_pp = 1U;

            if (ident_mode == FOC_IDENT_MODE_PP) {
                ident_result.valid = 1U;
                ident_cleanup(m);
                ident_set_state(FOC_IDENT_DONE);
                foc_cmd_print("ident DONE: Pole Pairs ready. Apply with 'ident apply'\r\n");
            } else {
                /* 平滑过渡：先撤压缓冲 250ms，待转子平稳停顿后再启动磁链旋转 */
                foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
                ident_tick = now;
                ident_set_state(FOC_IDENT_PP_SETTLE);
            }
        }
        break;

    case FOC_IDENT_PP_SETTLE:
        foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
        if ((uint32_t)(now - ident_tick) >= 250U) {
            flux_v_sum = 0.0f;
            flux_i_sum = 0.0f;
            flux_speed_sum = 0.0f;
            flux_samples = 0U;
            ident_tick = now;
            ident_set_state(FOC_IDENT_FLUX_SPIN);
        }
        break;

    case FOC_IDENT_FLUX_SPIN: {
        float pp = (ident_result.has_pp != 0U)
                       ? ident_result.pole_pairs
                       : m->params.pole_pairs;
        float rs = (ident_result.has_rs_ls != 0U)
                       ? ident_result.rs_ohm
                       : m->params.rs_ohm;
        uint32_t elapsed = now - ident_tick;
        /* 端电压动态自适应：高于内阻降约 0.35V 反电势裕量，保证 Eq 稳态正向可测 */
        float v_target = foc_clampf((rs * 1.2f) + 0.35f, 0.75f, 1.10f);
        float v_applied = v_target;

        if (elapsed < IDENT_FLUX_RAMP_MS) {
            v_applied = v_target * ((float)elapsed / (float)IDENT_FLUX_RAMP_MS);
        }

        /* 开环平滑驱动电机以恒定转速旋转 */
        foc_motor_openloop_spin(m, IDENT_FLUX_SPEED_RPM, 0.0f, v_applied);

        if (elapsed > IDENT_FLUX_RAMP_MS) {
            /* 加速稳定后，进入稳态采样阶段 */
            if (dt_ms > 0U) {
                flux_v_sum += v_applied;
                flux_i_sum += fabsf(m->i_dq_filt.q);
                flux_speed_sum += fabsf(m->velocity_observer_rpm);
                ++flux_samples;
            }

            if (elapsed >= (IDENT_FLUX_RAMP_MS + IDENT_FLUX_SAMPLE_MS)) {
                if (flux_samples > 10U) {
                    float v_mag = v_applied;
                    float iq_avg = flux_i_sum / (float)flux_samples;
                    float rpm_avg = flux_speed_sum / (float)flux_samples;
                    float omega_e = _2PI * (rpm_avg / 60.0f) * pp;
                    float ir_drop = rs * iq_avg;
                    float eq_sq = (v_mag * v_mag) - (ir_drop * ir_drop);
                    float eq = (eq_sq > 0.01f) ? sqrtf(eq_sq) : (0.40f * v_mag);
                    float psi_f;
                    float ke;

                    if ((omega_e > 10.0f) && (eq > 0.02f)) {
                        psi_f = eq / omega_e;
                        /* Line-to-line peak EMF per 1000 RPM */
                        ke = 1.7320508f * psi_f * (_2PI * pp * 1000.0f / 60.0f);

                        ident_result.flux_linkage_wb = psi_f;
                        ident_result.ke_v_krpm = ke;
                        ident_result.has_flux = 1U;

                        foc_cmd_print("ident: Flux=%.5f Wb, Ke=%.2f V/krpm (rpm=%.0f, Iq=%.2fA)\r\n",
                                      (double)psi_f, (double)ke,
                                      (double)rpm_avg, (double)iq_avg);
                    } else {
                        foc_cmd_print("ident WARN: low EMF response (Eq=%.3fV)\r\n", (double)eq);
                    }
                }
                ident_result.valid = 1U;
                ident_cleanup(m);
                ident_set_state(FOC_IDENT_DONE);
                foc_cmd_print("ident DONE: Identification complete!\r\n"
                              "apply with 'ident apply', then 'conf write'\r\n");
            }
        }
        break;
    }

    default:
        ident_fail("bad state");
        break;
    }

    ident_last_ms = now;
}

uint8_t foc_ident_is_active(void)
{
    return ((ident_state == FOC_IDENT_BOOTSTRAP) ||
            (ident_state == FOC_IDENT_NEUTRAL) ||
            (ident_state == FOC_IDENT_RS_RAMP) ||
            (ident_state == FOC_IDENT_RS_MEASURE) ||
            (ident_state == FOC_IDENT_LS) ||
            (ident_state == FOC_IDENT_LD_MEASURE) ||
            (ident_state == FOC_IDENT_LQ_MEASURE) ||
            (ident_state == FOC_IDENT_PP_ALIGN) ||
            (ident_state == FOC_IDENT_PP_SPIN) ||
            (ident_state == FOC_IDENT_PP_SETTLE) ||
            (ident_state == FOC_IDENT_FLUX_SPIN)) ? 1U : 0U;
}

foc_ident_state_t foc_ident_get_state(void)
{
    return ident_state;
}

const foc_ident_result_t *foc_ident_get_result(void)
{
    return &ident_result;
}

uint8_t foc_ident_apply(foc_motor_t *m)
{
    float v_max;
    float bw;

    if ((m == 0) || (ident_result.valid == 0U) ||
        (m->state == FOC_STATE_RUN) || (m->state == FOC_STATE_CALIB)) {
        return 0U;
    }

    if (ident_result.has_rs_ls != 0U) {
        m->params.rs_ohm = ident_result.rs_ohm;
        m->params.ls_henry = ident_result.ls_henry;

        /* 重新自整定电流环 */
        bw = m->cfg.current_bw_rads;
        v_max = m->drv->u_dc * INV_SQRT_3;
        foc_pid_init(&m->pid_id, m->params.ls_henry * bw,
                     m->params.rs_ohm * bw, 0.0f, v_max, 0.0f);
        foc_pid_init(&m->pid_iq, m->params.ls_henry * bw,
                     m->params.rs_ohm * bw, 0.0f, v_max, 0.0f);
    }

    if (ident_result.has_pp != 0U) {
        m->params.pole_pairs = ident_result.pole_pairs;
    }

    if (ident_result.has_flux != 0U) {
        m->params.ke = ident_result.ke_v_krpm;
    }

    return 1U;
}
