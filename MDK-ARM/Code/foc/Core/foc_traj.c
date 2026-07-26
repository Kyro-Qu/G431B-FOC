/**
 * @file    foc_traj.c
 * @brief   梯形速度轨迹规划器实现（ODrive trap_traj 的 C 移植）
 */

#include "foc_traj.h"
#include "foc_utils.h"

/* 带死区的符号函数：|x| 很小时返回 0，避免方向抖动 */
static float traj_sign_hard(float x)
{
    if (x > 1e-6f) {
        return 1.0f;
    }
    if (x < -1e-6f) {
        return -1.0f;
    }
    return 0.0f;
}

void foc_traj_plan(foc_traj_t *tr, float x_target, float x_now, float v_now,
                   float vmax, float amax, float dmax)
{
    float dx = x_target - x_now;
    float stop_dist;
    float dx_stop;
    float s;
    float dx_min;

    tr->xi = x_now;
    tr->xf = x_target;
    tr->vi = v_now;
    tr->t = 0.0f;

    /* 若按最大减速度刹车会停在哪？决定本次运动的"净方向" s。
     * 这样处理"运动中反向改目标"时会先减速、过冲一点再折返，
     * 而不是硬性瞬间反向。 */
    stop_dist = (v_now * v_now) / (2.0f * dmax);
    dx_stop = (v_now >= 0.0f) ? stop_dist : -stop_dist;
    s = traj_sign_hard(dx - dx_stop);
    if (s == 0.0f) {
        /* 已经在目标处且速度≈0 */
        tr->ar = 0.0f;
        tr->vr = 0.0f;
        tr->dr = 0.0f;
        tr->t_acc = 0.0f;
        tr->t_vel = 0.0f;
        tr->t_dec = 0.0f;
        tr->t_total = 0.0f;
        tr->y_acc_end = x_target;
        tr->active = (v_now != 0.0f) ? 1U : 0U;
        if (tr->active == 0U) {
            return;
        }
        /* 有余速：做一段纯减速 */
        s = (v_now > 0.0f) ? 1.0f : -1.0f;
        tr->dr = -s * dmax;
        tr->t_dec = -v_now / tr->dr;
        tr->t_total = tr->t_dec;
        tr->xf = x_now + (0.5f * v_now * tr->t_dec);
        return;
    }

    tr->ar = s * amax;
    tr->dr = -s * dmax;
    tr->vr = s * vmax;

    /* 初速度已超过巡航速度：加速段实际上是减速 */
    if ((s * v_now) > (s * tr->vr)) {
        tr->ar = -tr->ar;
    }

    tr->t_acc = (tr->vr - v_now) / tr->ar;
    tr->t_dec = -tr->vr / tr->dr;

    /* 达到巡航速度所需的最短行程；不够就退化为三角形曲线 */
    dx_min = (0.5f * tr->t_acc * (tr->vr + v_now)) +
             (0.5f * tr->t_dec * tr->vr);
    if ((s * dx) < (s * dx_min)) {
        float vr_sq = ((tr->dr * v_now * v_now) +
                       (2.0f * tr->ar * tr->dr * dx)) /
                      (tr->dr - tr->ar);

        if (vr_sq < 0.0f) {
            vr_sq = 0.0f;
        }
        tr->vr = s * sqrtf(vr_sq);
        tr->t_acc = (tr->vr - v_now) / tr->ar;
        tr->t_dec = -tr->vr / tr->dr;
        if (tr->t_acc < 0.0f) {
            tr->t_acc = 0.0f;
        }
        if (tr->t_dec < 0.0f) {
            tr->t_dec = 0.0f;
        }
        tr->t_vel = 0.0f;
    } else {
        tr->t_vel = (dx - dx_min) / tr->vr;
    }

    tr->t_total = tr->t_acc + tr->t_vel + tr->t_dec;
    tr->y_acc_end = tr->xi + (v_now * tr->t_acc) +
                    (0.5f * tr->ar * tr->t_acc * tr->t_acc);
    tr->active = 1U;
}

uint8_t foc_traj_eval(foc_traj_t *tr, float dt, float *pos_ref, float *vel_ff)
{
    float t;

    if (tr->active == 0U) {
        *pos_ref = tr->xf;
        *vel_ff = 0.0f;
        return 0U;
    }

    tr->t += dt;
    t = tr->t;

    if (t >= tr->t_total) {
        *pos_ref = tr->xf;
        *vel_ff = 0.0f;
        tr->active = 0U;
        return 0U;
    }

    if (t < tr->t_acc) {
        *pos_ref = tr->xi + (tr->vi * t) + (0.5f * tr->ar * t * t);
        *vel_ff = tr->vi + (tr->ar * t);
    } else if (t < (tr->t_acc + tr->t_vel)) {
        *pos_ref = tr->y_acc_end + (tr->vr * (t - tr->t_acc));
        *vel_ff = tr->vr;
    } else {
        /* 减速段：从终点倒着算，保证 t=t_total 时恰好落在 xf、速度 0 */
        float td = t - tr->t_total;

        *pos_ref = tr->xf + (0.5f * tr->dr * td * td);
        *vel_ff = tr->dr * td;
    }
    return 1U;
}
