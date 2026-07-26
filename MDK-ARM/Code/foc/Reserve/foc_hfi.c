/**
 * @file    foc_hfi.c
 * @brief   【预留区】HFI 高频注入实现（教学版）
 *
 * 解调思路（对应 update() 里的步骤号）：
 *   ① 注入：d 轴电压叠加 ±V 方波（每极性保持 2 拍抗 CCR 预装载）；
 *   ② 响应：q 轴电流的"高频差分" Δiq = iq[k] - iq[k-1] 在测量拍
 *     携带角度误差信息：Δiq ≈ pol · V·dt·(1/Ld - 1/Lq)·sin(2Δθ)/2；
 *   ③ 解调：err = pol × Δiq，符号即角度误差方向；
 *   ④ PLL：θ += (ω + Kp·err)·dt，ω += Ki·err·dt，把误差收敛到 0。
 *
 * 教学版简化点（实测后可升级）：
 *   - 未做基波电流解耦（低速下基波变化慢，差分自然抑制）；
 *   - 极性判别用 d 轴正负脉冲的电流幅值差（饱和效应：顺磁方向
 *     电感略小、电流略大）；对饱和弱的电机可能需要加大脉冲。
 */

#include "foc_hfi.h"
#include "../Core/foc_utils.h"

#define HFI_POLARITY_TICKS 512U   /* 极性判别时长：32ms@16kHz */

void foc_hfi_init(foc_hfi_t *h, float v_inject, float pll_kp, float pll_ki)
{
    h->v_inject = v_inject;
    h->pll_kp = pll_kp;
    h->pll_ki = pll_ki;
    h->state = FOC_HFI_IDLE;
    h->theta_e = 0.0f;
    h->speed_e_rads = 0.0f;
    h->pol = 0;
    h->phase = 0U;
    h->iq_prev = 0.0f;
    h->demod = 0.0f;
    h->pol_cnt = 0U;
    h->pol_acc = 0.0f;
}

void foc_hfi_start(foc_hfi_t *h, float theta_guess)
{
    h->theta_e = foc_wrap_0_2pi(theta_guess);
    h->speed_e_rads = 0.0f;
    h->pol = 0;
    h->phase = 0U;
    h->iq_prev = 0.0f;
    h->pol_cnt = 0U;
    h->pol_acc = 0.0f;
    h->state = FOC_HFI_POLARITY;
}

float foc_hfi_update(foc_hfi_t *h, foc_motor_t *m)
{
    float iq = m->i_dq.q;
    float id = m->i_dq.d;

    if (h->state == FOC_HFI_IDLE) {
        return h->theta_e;
    }

    /* ---- ②③ 测量拍解调（当前拍电流反映上一拍注入的响应） ---- */
    if ((h->pol != 0) && (h->phase == 1U)) {
        float d_iq = iq - h->iq_prev;

        h->demod = (float)h->pol * d_iq;

        if (h->state == FOC_HFI_TRACK) {
            /* ④ PLL：角度误差信号 → 锁相 */
            h->speed_e_rads += h->pll_ki * h->demod * m->dt_fast;
            h->theta_e = foc_wrap_0_2pi(
                h->theta_e +
                ((h->speed_e_rads + (h->pll_kp * h->demod)) * m->dt_fast));
        } else {
            /* 极性判别：累计 |id| 在正负半周的差。
             * 猜的 d+ 方向若真是 N 极（顺磁），正脉冲电流略大 */
            h->pol_acc += (float)h->pol * id;
            if (++h->pol_cnt >= HFI_POLARITY_TICKS) {
                if (h->pol_acc < 0.0f) {
                    /* 反了：磁极在猜测方向的对面 */
                    h->theta_e = foc_wrap_0_2pi(h->theta_e + _PI);
                }
                h->state = FOC_HFI_TRACK;
            }
        }
    }
    h->iq_prev = iq;

    /* ---- ① 注入：每极性保持 2 拍（时序说明见 foc_ident.c） ---- */
    if ((h->pol != 0) && (h->phase == 0U)) {
        h->phase = 1U;
    } else {
        h->pol = (h->pol == 1) ? -1 : 1;
        h->phase = 0U;
    }
    m->v_dq.d += (float)h->pol * h->v_inject;

    return h->theta_e;
}
