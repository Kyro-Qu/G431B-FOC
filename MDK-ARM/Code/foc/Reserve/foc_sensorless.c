/**
 * @file    foc_sensorless.c
 * @brief   【预留区】无感 FOC 绑定层实现
 *
 * 数学说明：观测器需要 αβ 系的电压和电流。快环里它们是局部变量，
 * 为了不改动 foc_motor 的结构，这里用 v_dq/i_dq + 当前使用的 θe
 * 反变换重建 αβ——与快环内部的值数学等价（多花 8 次乘法）。
 */

#include "foc_sensorless.h"
#include "../Core/foc_transform.h"
#include "../Core/foc_utils.h"

void foc_sensorless_init(foc_sensorless_t *sl,
                         const foc_motor_params_t *params,
                         float flux_wb, float dt_fast)
{
    (void)dt_fast;

    foc_observer_init(&sl->obs,
                      params->rs_ohm, params->ls_henry, flux_wb,
                      /* PLL 增益：VESC 默认量级 kp=2000, ki=40000 */
                      2000.0f, 40000.0f,
                      0.0f /* gamma 用默认 25/λ² */);

    sl->state = FOC_SL_IDLE;
    sl->ramp_target_rpm = 0.15f * params->max_rpm;  /* 15% 额定起步 */
    sl->ramp_rpm_s = 500.0f;
    sl->switch_err_rad = 0.3f;      /* ~17°，实测后可收紧 */
    sl->switch_ticks = 1600U;       /* 16kHz 下 100ms */
    sl->ramp_rpm_now = 0.0f;
    sl->agree_cnt = 0U;
    sl->last_err_rad = 0.0f;
}

void foc_sensorless_start(foc_sensorless_t *sl)
{
    foc_observer_reset(&sl->obs);
    sl->ramp_rpm_now = 0.0f;
    sl->agree_cnt = 0U;
    sl->state = FOC_SL_RAMP;
}

float foc_sensorless_update(foc_sensorless_t *sl, foc_motor_t *m)
{
    ab_t v_ab;
    ab_t i_ab;
    float sin_th;
    float cos_th;
    float theta_obs;

    if (sl->state == FOC_SL_IDLE) {
        return m->theta_e;
    }

    /* 用"上一拍实际使用的角度"重建 αβ 量喂观测器 */
    sin_th = foc_sin(m->theta_e);
    cos_th = foc_cos(m->theta_e);
    foc_inv_park(&m->v_dq, sin_th, cos_th, &v_ab);
    foc_inv_park(&m->i_dq, sin_th, cos_th, &i_ab);

    theta_obs = foc_observer_update(&sl->obs, &v_ab, &i_ab, m->dt_fast);

    switch (sl->state) {
    case FOC_SL_RAMP: {
        /* 开环角度推进（等效 foc_motor_openloop_spin 的斜坡版） */
        float err;

        sl->ramp_rpm_now += sl->ramp_rpm_s * m->dt_fast;
        if (sl->ramp_rpm_now > sl->ramp_target_rpm) {
            sl->ramp_rpm_now = sl->ramp_target_rpm;
        }
        m->ol_angle_step = _2PI * (sl->ramp_rpm_now / 60.0f) *
                           m->params.pole_pairs * m->dt_fast;

        /* 切换判据：观测器角度与开环角度持续一致 */
        err = foc_wrap_pm_pi(theta_obs - m->theta_e);
        sl->last_err_rad = err;
        if ((err < sl->switch_err_rad) && (err > -sl->switch_err_rad) &&
            (sl->ramp_rpm_now >= (0.9f * sl->ramp_target_rpm))) {
            if (++sl->agree_cnt >= sl->switch_ticks) {
                sl->state = FOC_SL_CLOSED;
            }
        } else {
            sl->agree_cnt = 0U;
        }
        /* RAMP 阶段仍然用开环角度输出 */
        return foc_wrap_0_2pi(m->theta_e + m->ol_angle_step);
    }

    case FOC_SL_CLOSED: {
        /* 失锁检测：估计转速掉到启动转速一半以下说明观测器丢了 */
        float rpm = (sl->obs.speed_e_rads / m->params.pole_pairs) *
                    FOC_RADS_TO_RPM;

        if ((rpm < (0.5f * sl->ramp_target_rpm)) &&
            (rpm > (-0.5f * sl->ramp_target_rpm))) {
            sl->state = FOC_SL_LOST;
        }
        return theta_obs;
    }

    case FOC_SL_LOST:
    default:
        /* 保持返回观测器角度，等待上层处理（disarm 或重启动） */
        return theta_obs;
    }
}

float foc_sensorless_speed_e_rads(const foc_sensorless_t *sl)
{
    return sl->obs.speed_e_rads;
}

foc_sensorless_state_t foc_sensorless_get_state(const foc_sensorless_t *sl)
{
    return sl->state;
}
