/**
 * @file    foc_observer.c
 * @brief   【预留模块】非线性磁链观测器 + PLL 实现（VESC 风格）
 */

#include "foc_observer.h"
#include "foc_utils.h"
#include "foc_transform.h"

void foc_observer_init(foc_observer_t *obs,
                       float rs_ohm, float ls_henry, float flux_wb,
                       float pll_kp, float pll_ki, float gamma)
{
    obs->rs_ohm = rs_ohm;
    obs->ls_henry = ls_henry;
    obs->flux_wb = flux_wb;
    obs->pll_kp = pll_kp;
    obs->pll_ki = pll_ki;

    /* VESC 经验值：γ 与磁链平方成反比时收敛速度基本恒定 */
    if (gamma > 0.0f) {
        obs->gamma = gamma;
    } else {
        obs->gamma = 25.0f / (flux_wb * flux_wb);
    }

    foc_observer_reset(obs);
}

void foc_observer_reset(foc_observer_t *obs)
{
    /* 从一个非零小值起步，避免 atan2(0,0) */
    obs->x1 = obs->flux_wb;
    obs->x2 = 0.0f;
    obs->theta_e = 0.0f;
    obs->pll_theta = 0.0f;
    obs->speed_e_rads = 0.0f;
}

float foc_observer_update(foc_observer_t *obs,
                          const ab_t *v_ab, const ab_t *i_ab, float dt)
{
    const float R = obs->rs_ohm;
    const float L = obs->ls_henry;
    const float lambda_sq = obs->flux_wb * obs->flux_wb;

    /* 定子磁链中扣除电感分量，剩下的就是转子磁链投影 */
    float e1 = obs->x1 - (L * i_ab->alpha);
    float e2 = obs->x2 - (L * i_ab->beta);

    /* 幅值约束误差：真实转子磁链幅值恒为 λm */
    float err = lambda_sq - ((e1 * e1) + (e2 * e2));

    /* 磁链积分 + 非线性修正（Ortega 观测器核心公式） */
    float x1_dot = v_ab->alpha - (R * i_ab->alpha) +
                   (0.5f * obs->gamma * e1 * err);
    float x2_dot = v_ab->beta - (R * i_ab->beta) +
                   (0.5f * obs->gamma * e2 * err);

    obs->x1 += x1_dot * dt;
    obs->x2 += x2_dot * dt;

    /* 转子磁链方向 = 电角度 */
    e1 = obs->x1 - (L * i_ab->alpha);
    e2 = obs->x2 - (L * i_ab->beta);
    obs->theta_e = foc_wrap_0_2pi(atan2f(e2, e1));

    /* ---- PLL 提取转速 ---- */
    {
        float delta = foc_wrap_pm_pi(obs->theta_e - obs->pll_theta);

        obs->pll_theta = foc_wrap_0_2pi(
            obs->pll_theta +
            ((obs->speed_e_rads + (obs->pll_kp * delta)) * dt));
        obs->speed_e_rads += obs->pll_ki * delta * dt;
    }

    return obs->theta_e;
}

float foc_observer_speed_e_rads(const foc_observer_t *obs)
{
    return obs->speed_e_rads;
}
