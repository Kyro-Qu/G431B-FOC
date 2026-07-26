/**
 * @file    foc_motor.c
 * @brief   电机轴对象实现：快环（电流环+调制）与慢环（速度/位置环）
 *
 * 本文件是纯算法层：不 include 任何 STM32 头文件，
 * 所有硬件访问都通过 foc_types.h 里的接口表。
 */

#include "foc_motor.h"
#include "foc_transform.h"
#include "foc_utils.h"

/* dq 电流遥测低通时间常数（仅用于观察，不进控制环） */
#define FOC_IDQ_TELEM_LPF_TF 0.002f

/* ======================== 内部函数 ======================== */

/**
 * 电压圆限幅：保证 |v_dq| ≤ v_max（SVPWM 线性区上限 U_dc/√3）。
 * d 轴优先（MCSDK/VESC 惯例）：d 轴电压维持磁场定向，被裁剪的
 * 应该是产生转矩的 q 轴，这样过载时牺牲加速度而不是失去定向。
 */
static void foc_voltage_circle_limit(dq_t *v, float v_max)
{
    float vd = foc_clampf(v->d, -v_max, v_max);
    float vq_max_sq = (v_max * v_max) - (vd * vd);
    float vq = v->q;

    if ((vq * vq) > vq_max_sq) {
        float vq_max = sqrtf(vq_max_sq);
        vq = foc_clampf(vq, -vq_max, vq_max);
    }
    v->d = vd;
    v->q = vq;
}

/**
 * 过流保护（继承自本工程验证过的策略）：
 *   软限制：连续 N 拍超过 soft limit 才跳闸（容忍毛刺）；
 *   硬限制：单拍超过 hard limit 立即跳闸。
 * 返回 0 表示已触发保护（本拍不要继续输出）。
 */
#define FOC_OVERCURRENT_TRIP_SAMPLES 8U

static uint8_t foc_motor_check_current(foc_motor_t *m)
{
    float ia = m->i_abc.a;
    float ib = m->i_abc.b;
    float ic = m->i_abc.c;
    float peak = fabsf(ia);
    float soft = m->safety.current_limit_a;
    float hard = m->safety.hard_current_limit_a;
    uint8_t hard_trip;

    if (fabsf(ib) > peak) {
        peak = fabsf(ib);
    }
    if (fabsf(ic) > peak) {
        peak = fabsf(ic);
    }

    m->safety.peak_current_a = peak;
    if (peak > m->safety.max_observed_current_a) {
        m->safety.max_observed_current_a = peak;
    }

    hard_trip = (peak > hard) ? 1U : 0U;
    if (peak > soft) {
        if (m->safety.consecutive_over_limit < 0xFFFFU) {
            ++m->safety.consecutive_over_limit;
        }
    } else {
        m->safety.consecutive_over_limit = 0U;
    }

    if ((hard_trip == 0U) &&
        (m->safety.consecutive_over_limit < FOC_OVERCURRENT_TRIP_SAMPLES)) {
        return 1U;
    }

    /* 锁存触发瞬间的电流快照，FAULT 后可在 Keil Watch 里查看 */
    m->safety.trip_current_u_a = ia;
    m->safety.trip_current_v_a = ib;
    m->safety.trip_current_w_a = ic;

    foc_motor_fault(m, (m->state == FOC_STATE_CALIB)
                           ? FOC_FAULT_CALIB_OVERCURRENT
                           : FOC_FAULT_RUN_OVERCURRENT);
    return 0U;
}

/**
 * 慢环（默认 1 kHz）：速度斜坡 → 速度 PI / 位置 P。
 * 输出写入 m->iq_ref，由快环的电流环去执行。
 */
static void foc_motor_slow_loop(foc_motor_t *m)
{
    const float dt = m->dt_fast * (float)m->slow_div;

    m->velocity_filt_rpm = foc_lpf_update(&m->lpf_vel, m->velocity_rpm, dt);

    switch (m->mode) {
    case FOC_MODE_VELOCITY: {
        /* 目标速度斜坡：限制加速度，避免阶跃目标产生电流冲击 */
        float tgt = m->target;
        if (m->cfg.vel_ramp_rpm_s > 0.0f) {
            float step = m->cfg.vel_ramp_rpm_s * dt;
            m->vel_ref_rpm = foc_clampf(tgt,
                                        m->vel_ref_rpm - step,
                                        m->vel_ref_rpm + step);
        } else {
            m->vel_ref_rpm = tgt;
        }
        m->vel_ref_rpm = foc_clampf(m->vel_ref_rpm,
                                    -m->params.max_rpm, m->params.max_rpm);
        m->iq_ref = foc_pid_update(&m->pid_vel,
                                   m->vel_ref_rpm - m->velocity_filt_rpm, dt);
        break;
    }

    case FOC_MODE_POSITION: {
        /* 位置环 P：输出速度给定，再进速度 PI（级联） */
        float vel_cmd = foc_pid_update(&m->pid_pos,
                                       m->target - m->position_rad, dt);
        m->vel_ref_rpm = foc_clampf(vel_cmd,
                                    -m->cfg.pos_vel_limit_rpm,
                                    m->cfg.pos_vel_limit_rpm);
        m->iq_ref = foc_pid_update(&m->pid_vel,
                                   m->vel_ref_rpm - m->velocity_filt_rpm, dt);
        break;
    }

    case FOC_MODE_TORQUE:
        m->iq_ref = m->target;
        break;

    case FOC_MODE_OPENLOOP_VF:
    default:
        break;
    }

    m->iq_ref = foc_clampf(m->iq_ref,
                           -m->params.max_current_a, m->params.max_current_a);
}

/* ======================== 生命周期 ======================== */

void foc_motor_init(foc_motor_t *m,
                    const foc_driver_if_t *drv,
                    const foc_current_if_t *cur,
                    const foc_sensor_if_t *sensor,
                    const foc_motor_params_t *params,
                    const foc_ctrl_cfg_t *cfg,
                    float dt_fast,
                    uint16_t slow_div)
{
    float v_max;
    float kp_i;
    float ki_i;

    m->drv = drv;
    m->cur = cur;
    m->sensor = sensor;
    m->params = *params;
    m->cfg = *cfg;
    m->dt_fast = dt_fast;
    m->slow_div = (slow_div == 0U) ? 1U : slow_div;

    m->state = FOC_STATE_IDLE;
    m->mode = FOC_MODE_OPENLOOP_VF;
    m->angle_source = FOC_ANGLE_OPEN_LOOP;
    m->pwm_hold = 0U;

    m->calib.valid = 0U;
    m->calib.direction = 1;
    m->calib.electrical_offset_rad = 0.0f;

    m->target = 0.0f;
    m->v_openloop.d = 0.0f;
    m->v_openloop.q = 0.0f;
    m->vel_ref_rpm = 0.0f;
    m->iq_ref = 0.0f;

    m->theta_e = 0.0f;
    m->theta_mech = 0.0f;
    m->position_rad = 0.0f;
    m->velocity_rpm = 0.0f;
    m->velocity_filt_rpm = 0.0f;
    m->i_abc.a = m->i_abc.b = m->i_abc.c = 0.0f;
    m->i_dq.d = m->i_dq.q = 0.0f;
    m->i_dq_filt.d = m->i_dq_filt.q = 0.0f;
    m->v_dq.d = m->v_dq.q = 0.0f;
    m->svm.duty_a = m->svm.duty_b = m->svm.duty_c = 0.5f;
    m->svm.sector = 0U;
    m->ol_angle_step = 0.0f;
    m->slow_cnt = 0U;

    /* 安全限制默认取电机参数 */
    m->safety.peak_current_a = 0.0f;
    m->safety.max_observed_current_a = 0.0f;
    m->safety.current_limit_a = params->max_current_a;
    m->safety.hard_current_limit_a = params->hard_current_a;
    m->safety.trip_current_u_a = 0.0f;
    m->safety.trip_current_v_a = 0.0f;
    m->safety.trip_current_w_a = 0.0f;
    m->safety.consecutive_over_limit = 0U;
    m->safety.fault_code = (uint8_t)FOC_FAULT_NONE;

    /* ---- 电流环带宽整定：Kp = Ls·ω, Ki = Rs·ω ---- */
    v_max = drv->u_dc * INV_SQRT_3;
    kp_i = params->ls_henry * cfg->current_bw_rads;
    ki_i = params->rs_ohm * cfg->current_bw_rads;
    foc_pid_init(&m->pid_id, kp_i, ki_i, 0.0f, v_max, 0.0f);
    foc_pid_init(&m->pid_iq, kp_i, ki_i, 0.0f, v_max, 0.0f);

    /* 速度环：输出限制 = 电流限制 */
    foc_pid_init(&m->pid_vel, cfg->vel_kp, cfg->vel_ki, 0.0f,
                 params->max_current_a, 0.0f);

    /* 位置环：纯 P，输出限制 = 位置模式速度上限 */
    foc_pid_init(&m->pid_pos, cfg->pos_kp, 0.0f, 0.0f,
                 cfg->pos_vel_limit_rpm, 0.0f);

    foc_lpf_init(&m->lpf_vel, cfg->vel_lpf_tf);
    foc_lpf_init(&m->lpf_id, FOC_IDQ_TELEM_LPF_TF);
    foc_lpf_init(&m->lpf_iq, FOC_IDQ_TELEM_LPF_TF);
}

/* ======================== 快环 ======================== */

void foc_motor_fast_loop(foc_motor_t *m)
{
    foc_state_t st;
    float sin_th;
    float cos_th;
    ab_t i_ab;
    ab_t v_ab;

    /* 1. 传感器更新：任何状态都执行，保证角度/速度随时可观测 */
    if (m->sensor != 0) {
        float th;

        m->sensor->update();
        th = m->sensor->angle_rad();
        m->position_rad += foc_wrap_pm_pi(th - m->theta_mech);
        m->theta_mech = th;
        m->velocity_rpm = m->sensor->velocity_rpm();
    }

    st = m->state;
    if ((st != FOC_STATE_RUN) && (st != FOC_STATE_CALIB)) {
        m->safety.consecutive_over_limit = 0U;
        return;
    }

    /* 2. 读取电流并做过流保护（先保护后控制） */
    if (m->cur != 0) {
        m->cur->get(&m->i_abc.a, &m->i_abc.b, &m->i_abc.c);
        if (foc_motor_check_current(m) == 0U) {
            return;
        }
    }

    /* 自举充电等阶段：保护照跑，但不允许控制环覆盖 PWM */
    if (m->pwm_hold != 0U) {
        return;
    }

    /* 3. 电角度选择
     *    CALIB 状态一律用开环角度（校准状态机负责推进/保持）；
     *    RUN 状态按 angle_source 选择开环或编码器角度。 */
    if ((st == FOC_STATE_CALIB) ||
        (m->angle_source == FOC_ANGLE_OPEN_LOOP) ||
        (m->calib.valid == 0U)) {
        m->theta_e = foc_wrap_0_2pi(m->theta_e + m->ol_angle_step);
    } else {
        m->theta_e = foc_motor_encoder_theta_e(m);
    }

    sin_th = foc_sin(m->theta_e);
    cos_th = foc_cos(m->theta_e);

    /* 4. Clarke + Park：三相电流 → dq 电流 */
    foc_clarke(&m->i_abc, &i_ab);
    foc_park(&i_ab, sin_th, cos_th, &m->i_dq);
    m->i_dq_filt.d = foc_lpf_update(&m->lpf_id, m->i_dq.d, m->dt_fast);
    m->i_dq_filt.q = foc_lpf_update(&m->lpf_iq, m->i_dq.q, m->dt_fast);

    /* 5. 慢环分频：速度/位置环 */
    if (++m->slow_cnt >= m->slow_div) {
        m->slow_cnt = 0U;
        foc_motor_slow_loop(m);
    }

    /* 6. 电压命令 */
    if ((st == FOC_STATE_CALIB) || (m->mode == FOC_MODE_OPENLOOP_VF)) {
        /* 开环/校准：直接使用外部给的 vd/vq */
        m->v_dq = m->v_openloop;
    } else {
        /* 电流闭环：Id 恒 0（表贴电机不弱磁），Iq 跟随给定 */
        float vd = foc_pid_update(&m->pid_id, 0.0f - m->i_dq.d, m->dt_fast);
        float vq = foc_pid_update(&m->pid_iq, m->iq_ref - m->i_dq.q, m->dt_fast);

        /* dq 解耦前馈：抵消旋转坐标系带来的交叉耦合项 ω·L·i */
        if (m->cfg.decouple_enable != 0U) {
            float we = m->velocity_rpm * FOC_RPM_TO_RADS * m->params.pole_pairs;

            vd -= we * m->params.ls_henry * m->i_dq.q;
            vq += we * m->params.ls_henry * m->i_dq.d;
        }

        m->v_dq.d = vd;
        m->v_dq.q = vq;
    }

    foc_voltage_circle_limit(&m->v_dq, m->drv->u_dc * INV_SQRT_3);

    /* 7. 反 Park + SVPWM + 输出 */
    foc_inv_park(&m->v_dq, sin_th, cos_th, &v_ab);
    foc_svm_calc(&v_ab, m->drv->u_dc, &m->svm);
    m->drv->set_compare(
        (uint32_t)(m->svm.duty_a * (float)m->drv->full_count),
        (uint32_t)(m->svm.duty_b * (float)m->drv->full_count),
        (uint32_t)(m->svm.duty_c * (float)m->drv->full_count),
        m->svm.sector);
}

/* ======================== 命令接口 ======================== */

uint8_t foc_motor_arm(foc_motor_t *m)
{
    if (m->state != FOC_STATE_IDLE) {
        return 0U;
    }

    /* 闭环模式必须先有有效校准（或明确切到开环角度源） */
    if ((m->mode != FOC_MODE_OPENLOOP_VF) &&
        ((m->calib.valid == 0U) ||
         (m->angle_source != FOC_ANGLE_ENCODER_CALIBRATED))) {
        foc_motor_fault(m, FOC_FAULT_NOT_CALIBRATED);
        return 0U;
    }

    /* 清干净旧状态，从零起步 */
    foc_pid_reset(&m->pid_id);
    foc_pid_reset(&m->pid_iq);
    foc_pid_reset(&m->pid_vel);
    foc_pid_reset(&m->pid_pos);
    m->vel_ref_rpm = 0.0f;
    m->iq_ref = 0.0f;
    m->slow_cnt = 0U;
    m->safety.consecutive_over_limit = 0U;

    /* 位置模式上电即"保持当前位置"，避免使能瞬间飞车 */
    if (m->mode == FOC_MODE_POSITION) {
        m->target = m->position_rad;
    }

    m->state = FOC_STATE_RUN;
    m->drv->enable();
    return 1U;
}

void foc_motor_disarm(foc_motor_t *m)
{
    if (m->state == FOC_STATE_FAULT) {
        return; /* FAULT 只能通过 clear_fault 离开 */
    }
    m->drv->disable();
    m->v_dq.d = 0.0f;
    m->v_dq.q = 0.0f;
    m->v_openloop.d = 0.0f;
    m->v_openloop.q = 0.0f;
    m->state = FOC_STATE_IDLE;
}

void foc_motor_set_mode(foc_motor_t *m, foc_mode_t mode)
{
    if (m->mode == mode) {
        return;
    }
    m->mode = mode;
    m->target = 0.0f;
    foc_pid_reset(&m->pid_vel);
    foc_pid_reset(&m->pid_pos);
    m->vel_ref_rpm = 0.0f;
    m->iq_ref = 0.0f;

    if (mode == FOC_MODE_POSITION) {
        m->target = m->position_rad;
    }
    if ((mode != FOC_MODE_OPENLOOP_VF) && (m->calib.valid != 0U)) {
        m->angle_source = FOC_ANGLE_ENCODER_CALIBRATED;
    }
}

void foc_motor_set_target(foc_motor_t *m, float value)
{
    m->target = value;
}

void foc_motor_set_angle_source(foc_motor_t *m, foc_angle_source_t src)
{
    m->angle_source = src;
}

void foc_motor_openloop_hold(foc_motor_t *m, float theta_e, float vd, float vq)
{
    m->theta_e = foc_wrap_0_2pi(theta_e);
    m->ol_angle_step = 0.0f;
    m->v_openloop.d = vd;
    m->v_openloop.q = vq;
}

void foc_motor_openloop_spin(foc_motor_t *m, float rpm, float vd, float vq)
{
    /* Δθe = 2π·(rpm/60)·pp·dt */
    m->ol_angle_step = _2PI * (rpm / 60.0f) * m->params.pole_pairs * m->dt_fast;
    m->v_openloop.d = vd;
    m->v_openloop.q = vq;
}

/* ======================== 故障处理 ======================== */

void foc_motor_fault(foc_motor_t *m, foc_fault_t fault)
{
    m->drv->disable();
    m->v_dq.d = 0.0f;
    m->v_dq.q = 0.0f;
    m->v_openloop.d = 0.0f;
    m->v_openloop.q = 0.0f;
    m->state = FOC_STATE_FAULT;
    if ((fault != FOC_FAULT_NONE) &&
        (m->safety.fault_code == (uint8_t)FOC_FAULT_NONE)) {
        m->safety.fault_code = (uint8_t)fault; /* 只锁存第一个故障 */
    }
}

void foc_motor_clear_fault(foc_motor_t *m)
{
    if (m->state != FOC_STATE_FAULT) {
        return;
    }
    m->safety.fault_code = (uint8_t)FOC_FAULT_NONE;
    m->safety.consecutive_over_limit = 0U;
    m->state = FOC_STATE_IDLE;
}

/* ======================== 辅助 ======================== */

void foc_motor_override_current_limits(foc_motor_t *m, float soft_a, float hard_a)
{
    m->safety.current_limit_a = soft_a;
    m->safety.hard_current_limit_a = hard_a;
}

void foc_motor_restore_current_limits(foc_motor_t *m)
{
    m->safety.current_limit_a = m->params.max_current_a;
    m->safety.hard_current_limit_a = m->params.hard_current_a;
}

float foc_motor_encoder_theta_e(const foc_motor_t *m)
{
    /* θe = dir·pp·θm + offset：
     * direction 校正编码器计数方向与电角度方向的关系，
     * offset 来自 D 轴对齐 + Z 脉冲搜索，保证 d 轴对准转子磁链 */
    return foc_wrap_0_2pi(
        ((float)m->calib.direction * m->params.pole_pairs * m->theta_mech) +
        m->calib.electrical_offset_rad);
}
