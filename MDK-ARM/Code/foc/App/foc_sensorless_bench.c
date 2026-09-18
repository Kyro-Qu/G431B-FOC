/**
 * @file    foc_sensorless_bench.c
 * @brief   无感观测器多算法影子初测与性能评估平台实现
 */

#include "foc_sensorless_bench.h"
#include "foc_angle_manager.h"
#include "foc_motor.h"
#include "foc_transform.h"
#include "foc_utils.h"
#include "main.h"
#include <math.h>

foc_sensorless_bench_t g_sensorless_bench = {0};
static uint8_t s_bench_slot = 0U;

/* 快速角度规范化到 [-PI, PI] */
static inline float wrap_pm_pi(float a)
{
    while (a > _PI)  { a -= _2PI; }
    while (a < -_PI) { a += _2PI; }
    return a;
}

static inline uint32_t get_dwt_cycles(void)
{
    return DWT->CYCCNT;
}

/* =========================================================================
 * 真实 STM32G4 片上硬件 CORDIC 协处理器操作接口
 * ========================================================================= */
/**
 * 配置硬件 CORDIC 为 PHASE (反正切角度与模长) 模式
 * Function: PHASE (0x02)
 * Precision: 15 cycles (最高精度)
 * Scale: 0
 * NbWrite: 2 (写 x, 写 y)
 * NbRead: 1 (仅读角度)
 * InSize: 32 bits (Q1.31)
 * OutSize: 32 bits (Q1.31, -1.0 -> -pi, +1.0 -> +pi)
 */
static void cordic_hardware_init(void)
{
    /* 确保时钟开启 */
    __HAL_RCC_CORDIC_CLK_ENABLE();

    /* CSR 配置: FUNC=PHASE(2), PRECISION=6CYCLES(6<<4), NARGS=2(1<<20), NRES=1(0<<19) */
    CORDIC->CSR = (2U << CORDIC_CSR_FUNC_Pos) |
                  (6U << CORDIC_CSR_PRECISION_Pos) |
                  (0U << CORDIC_CSR_SCALE_Pos) |
                  (1U << CORDIC_CSR_NARGS_Pos) |   /* 2 arguments: Arg1=x, Arg2=y */
                  (0U << CORDIC_CSR_NRES_Pos) |    /* 1 result: angle */
                  (0U << CORDIC_CSR_ARGSIZE_Pos) | /* 32-bit */
                  (0U << CORDIC_CSR_RESSIZE_Pos);  /* 32-bit */
}

float foc_cordic_calc_phase(float y, float x)
{
    float max_val = fabsf(x);
    float abs_y = fabsf(y);
    if (abs_y > max_val) {
        max_val = abs_y;
    }
    if (max_val < 1e-6f) {
        return 0.0f;
    }

    /* 归一化输入到 [-1.0, 1.0) 并转为 Q1.31 */
    float inv_m = 0.99999f / max_val;
    int32_t q_x = (int32_t)((x * inv_m) * 2147483647.0f);
    int32_t q_y = (int32_t)((y * inv_m) * 2147483647.0f);

    /* 写入硬件 CORDIC: 先写 x (ARG1), 再写 y (ARG2) */
    CORDIC->WDATA = (uint32_t)q_x;
    CORDIC->WDATA = (uint32_t)q_y;

    /* 等待结果就绪 (带超时防死循环保护) */
    uint32_t timeout = 50U;
    while (((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) && (--timeout > 0U)) {
    }

    /* 硬件流水线计算并读出角度结果 (Q1.31, 1.0 对应 PI) */
    int32_t q_res = (int32_t)CORDIC->RDATA;
    float angle = ((float)q_res / 2147483648.0f) * _PI;
    return angle;
}

void foc_sensorless_bench_init(foc_motor_t *m)
{
    (void)m;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cordic_hardware_init();

    /* 默认方向与偏置 */
    g_sensorless_bench.obs1_ortega.direction = 1;
    g_sensorless_bench.obs2_vesc.direction = 1;
    g_sensorless_bench.obs3_sto_pll.direction = 1;
    g_sensorless_bench.obs4_sto_cordic.direction = 1;
    g_sensorless_bench.obs5_hfi.direction = 1;

    /* HFI 影子注入默认参数 */
    g_sensorless_bench.hfi_enabled = 0U;
    g_sensorless_bench.hfi_inj_volt = 1.0f;
    g_sensorless_bench.hfi_inj_pol = 1;
    g_sensorless_bench.hfi_pll_kp = 100.0f;
    g_sensorless_bench.hfi_pll_ki = 2500.0f;
    g_sensorless_bench.hfi_pll_pos = 0.0f;
    g_sensorless_bench.hfi_pll_vel = 0.0f;
    g_sensorless_bench.hfi_i_prev_q = 0.0f;
    g_sensorless_bench.hfi_demod_err_filt = 0.0f;
    g_sensorless_bench.hfi_confidence = 0.0f;
    g_sensorless_bench.hfi_carrier_mag = 0.0f;
    g_sensorless_bench.hfi_iq_ripple = 0.0f;

    g_sensorless_bench.obs_deadtime_comp_enable = 1U;
    g_sensorless_bench.deadtime_comp_v = 0.17f;

    foc_sensorless_bench_reset_metrics();
    g_sensorless_bench.enabled = 1U;
    g_sensorless_bench.shadow_enabled = 0U;   /* 影子对比默认关，`bench start` 再开 */
    g_sensorless_bench.steady_state = 0U;
}

void foc_sensorless_bench_reset_metrics(void)
{
    g_sensorless_bench.total_samples = 0U;

    /* Obs 1 */
    g_sensorless_bench.obs1_ortega.err_sum_deg = 0.0f;
    g_sensorless_bench.obs1_ortega.err_sq_sum = 0.0f;
    g_sensorless_bench.obs1_ortega.err_peak_deg = 0.0f;
    g_sensorless_bench.obs1_ortega.samples = 0U;
    g_sensorless_bench.obs1_ortega.unlock_count = 0U;

    /* Obs 2 */
    g_sensorless_bench.obs2_vesc.err_sum_deg = 0.0f;
    g_sensorless_bench.obs2_vesc.err_sq_sum = 0.0f;
    g_sensorless_bench.obs2_vesc.err_peak_deg = 0.0f;
    g_sensorless_bench.obs2_vesc.samples = 0U;
    g_sensorless_bench.obs2_vesc.unlock_count = 0U;

    /* Obs 3 */
    g_sensorless_bench.obs3_sto_pll.err_sum_deg = 0.0f;
    g_sensorless_bench.obs3_sto_pll.err_sq_sum = 0.0f;
    g_sensorless_bench.obs3_sto_pll.err_peak_deg = 0.0f;
    g_sensorless_bench.obs3_sto_pll.samples = 0U;
    g_sensorless_bench.obs3_sto_pll.unlock_count = 0U;

    /* Obs 4 */
    g_sensorless_bench.obs4_sto_cordic.err_sum_deg = 0.0f;
    g_sensorless_bench.obs4_sto_cordic.err_sq_sum = 0.0f;
    g_sensorless_bench.obs4_sto_cordic.err_peak_deg = 0.0f;
    g_sensorless_bench.obs4_sto_cordic.samples = 0U;
    g_sensorless_bench.obs4_sto_cordic.unlock_count = 0U;

    /* Obs 5: HFI */
    g_sensorless_bench.obs5_hfi.err_sum_deg = 0.0f;
    g_sensorless_bench.obs5_hfi.err_sq_sum = 0.0f;
    g_sensorless_bench.obs5_hfi.err_peak_deg = 0.0f;
    g_sensorless_bench.obs5_hfi.samples = 0U;
    g_sensorless_bench.obs5_hfi.unlock_count = 0U;
    g_sensorless_bench.hfi_demod_err_filt = 0.0f;
    g_sensorless_bench.hfi_carrier_mag = 0.0f;
    g_sensorless_bench.hfi_iq_ripple = 0.0f;
}

void foc_sensorless_bench_set_steady(uint8_t steady)
{
    g_sensorless_bench.steady_state = steady;
    if (steady != 0U) {
        foc_sensorless_bench_reset_metrics();
    }
}

void foc_sensorless_bench_enable(uint8_t en)
{
    /* bench start/stop 只控制影子对比观测器。主观测器 (VESC / active_algo)
     * 是角度仲裁的依赖，必须常开——否则 `bench stop` 会让无感接管失锁。 */
    g_sensorless_bench.enabled = 1U;
    g_sensorless_bench.shadow_enabled = (en != 0U) ? 1U : 0U;
    if (en != 0U) {
        foc_sensorless_bench_reset_metrics();
    }
}

void foc_sensorless_bench_hfi_enable(uint8_t en, float inj_volt)
{
    g_sensorless_bench.hfi_enabled = en;
    if (inj_volt > 0.05f) {
        g_sensorless_bench.hfi_inj_volt = inj_volt;
    }
    if (en == 0U) {
        g_sensorless_bench.hfi_v_inj_alpha = 0.0f;
        g_sensorless_bench.hfi_v_inj_beta = 0.0f;
        g_sensorless_bench.hfi_inj_pol = 1;
        g_sensorless_bench.obs5_hfi.converged = 0U;
        g_sensorless_bench.hfi_confidence = 0.0f;
    } else {
        /* 使能时重置内部 PLL 状态 */
        g_sensorless_bench.hfi_demod_err_filt = 0.0f;
        g_sensorless_bench.hfi_carrier_mag = 0.0f;
        g_sensorless_bench.obs5_hfi.samples = 0U;
        g_sensorless_bench.obs5_hfi.err_sum_deg = 0.0f;
        g_sensorless_bench.obs5_hfi.err_sq_sum = 0.0f;
        g_sensorless_bench.obs5_hfi.err_peak_deg = 0.0f;
    }
}

/* =========================================================================
 * 1. Obs 1: Ortega 非线性磁链观测器 + 二阶 PLL
 * ========================================================================= */
static void update_obs1_ortega(foc_sensorless_bench_t *b, foc_motor_t *m,
                              float va, float vb, float ia, float ib, float dt)
{
    float rs = m->params.rs_ohm;
    float ls = m->params.ls_henry;
    float psi = (m->params.ke > 0.01f) ? (m->params.ke / (1.73205f * _2PI * m->params.pole_pairs * 1000.0f / 60.0f))
                                       : 0.00080f;
    float psi_sq = psi * psi;
    float observer_gain = 1200.0f;

    b->od_flux_a += (va - (rs * ia)) * dt;
    b->od_flux_b += (vb - (rs * ib)) * dt;

    float eta_a = b->od_flux_a - (ls * ia);
    float eta_b = b->od_flux_b - (ls * ib);
    float est_psi_sq = (eta_a * eta_a) + (eta_b * eta_b);

    /* 相对无量纲误差归一化: 避免微小磁链导致修正增益数值爆炸 */
    float err = (psi_sq - est_psi_sq) / psi_sq;
    float correction_gain = 0.5f * observer_gain * err;

    b->od_flux_a += correction_gain * eta_a * dt;
    b->od_flux_b += correction_gain * eta_b * dt;

    eta_a = b->od_flux_a - (ls * ia);
    eta_b = b->od_flux_b - (ls * ib);
    est_psi_sq = (eta_a * eta_a) + (eta_b * eta_b);

    /* 磁链圆诊断：幅值与中心漂移低通跟踪 */
    b->obs1_ortega.flux_mag = sqrtf(est_psi_sq);
    b->obs1_ortega.flux_center_a += (eta_a - b->obs1_ortega.flux_center_a) * (dt * 15.0f);
    b->obs1_ortega.flux_center_b += (eta_b - b->obs1_ortega.flux_center_b) * (dt * 15.0f);

    /* 记录 Ortega 底层快照 */
    b->diag_od_eta_a = eta_a;
    b->diag_od_eta_b = eta_b;

    if (est_psi_sq > (psi_sq * 1e-4f)) {
        float raw_angle = atan2f(eta_b, eta_a);
        if (raw_angle < 0.0f) {
            raw_angle += _2PI;
        }
        b->diag_od_theta_raw = raw_angle;

        float pll_kp = 1200.0f;
        float pll_ki = 4.0e5f;
        b->od_pll_pos = wrap_pm_pi(b->od_pll_pos + (b->od_pll_vel * dt));
        float delta_theta = wrap_pm_pi(raw_angle - b->od_pll_pos);
        b->od_pll_pos = wrap_pm_pi(b->od_pll_pos + (pll_kp * delta_theta * dt));
        b->od_pll_vel += pll_ki * delta_theta * dt;

        float theta_out = b->od_pll_pos;
        if (theta_out < 0.0f) {
            theta_out += _2PI;
        }
        float dir_scale = (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY)
                          ? 1.0f : ((m->calib.direction < 0) ? -1.0f : 1.0f);
        b->obs1_ortega.theta_e = theta_out;
        b->obs1_ortega.speed_rpm = dir_scale * (b->od_pll_vel / (_2PI * m->params.pole_pairs)) * 60.0f;
    }
}

/* =========================================================================
 * 2. Obs 2: VESC 单向约束鲁棒磁链观测器 (Benjamin Vedder 约束版)
 * ========================================================================= */
static void update_obs2_vesc(foc_sensorless_bench_t *b, foc_motor_t *m,
                             float va, float vb, float ia, float ib, float theta_enc, float dt)
{
    float rs = m->params.rs_ohm;
    float ls = m->params.ls_henry;
    /* 真实永磁磁链基准: 标称约 0.80 mWb */
    float lambda = (m->params.ke > 0.01f) ? (m->params.ke / (1.73205f * _2PI * m->params.pole_pairs * 1000.0f / 60.0f))
                                          : 0.00080f;
    float lambda_sq = lambda * lambda;
    float gamma = 1600.0f;

    float L_ia = ls * ia;
    float L_ib = ls * ib;

    /* 低速起步/静止辅助校正:
     * 仅当处于编码器主控状态 (SENSORED) 且转速低于 250 RPM 时，对转子磁链进行极弱软牵引，
     * 消除起步瞬间零速反电势不可观测引起的初始积分偏置（90° 虚影）。
     * 高速运行或编码器异常后立即断开，杜绝任何外部污染。 */
    if ((g_angle_mgr.mode == FOC_FEEDBACK_SENSORED_PRIMARY) &&
        (g_angle_mgr.state == FOC_ANGLE_SENSORED) &&
        (g_angle_mgr.enc_health == ENCODER_HEALTH_NORMAL) &&
        (fabsf(m->velocity_observer_rpm) < 250.0f)) {
        float sin_th = sinf(theta_enc);
        float cos_th = cosf(theta_enc);
        b->vesc_x1 = L_ia + (lambda * cos_th);
        b->vesc_x2 = L_ib + (lambda * sin_th);
        b->vesc_pll_pos = theta_enc;
        float dir_scale = (m->calib.direction < 0) ? -1.0f : 1.0f;
        b->vesc_pll_vel = dir_scale * (m->velocity_observer_rpm * (_2PI * m->params.pole_pairs / 60.0f));
    }

    float eta_a = b->vesc_x1 - L_ia;
    float eta_b = b->vesc_x2 - L_ib;
    float mag_sq = (eta_a * eta_a) + (eta_b * eta_b);

    /* 约束误差项: 严格归一化消除微小磁链 (0.8mWb) 导致的增益尺度衰减 */
    float err = (lambda_sq - mag_sq) / lambda_sq;
    if (err > 0.0f) {
        err *= 0.25f; /* 弱向外拉伸，强向内收缩 */
    }

    float x1_dot = va - (rs * ia) + (gamma * eta_a * err);
    float x2_dot = vb - (rs * ib) + (gamma * eta_b * err);

    b->vesc_x1 += x1_dot * dt;
    b->vesc_x2 += x2_dot * dt;

    /* 重新解耦定子漏感计算本拍转子磁链 */
    eta_a = b->vesc_x1 - L_ia;
    eta_b = b->vesc_x2 - L_ib;
    mag_sq = (eta_a * eta_a) + (eta_b * eta_b);

    /* 磁链圆诊断：幅值与中心漂移低通跟踪 */
    b->obs2_vesc.flux_mag = sqrtf(mag_sq);
    b->obs2_vesc.flux_center_a += (eta_a - b->obs2_vesc.flux_center_a) * (dt * 15.0f);
    b->obs2_vesc.flux_center_b += (eta_b - b->obs2_vesc.flux_center_b) * (dt * 15.0f);

    if (mag_sq > (lambda_sq * 1e-4f)) {
        float theta_raw = atan2f(eta_b, eta_a);
        if (theta_raw < 0.0f) {
            theta_raw += _2PI;
        }

        float pll_kp = 300.0f;
        float pll_ki = 25000.0f;
        b->vesc_pll_pos = wrap_pm_pi(b->vesc_pll_pos + (b->vesc_pll_vel * dt));
        float d_theta = wrap_pm_pi(theta_raw - b->vesc_pll_pos);
        b->vesc_pll_pos = wrap_pm_pi(b->vesc_pll_pos + (pll_kp * d_theta * dt));
        b->vesc_pll_vel += pll_ki * d_theta * dt;

        float theta_out = b->vesc_pll_pos;
        if (theta_out < 0.0f) {
            theta_out += _2PI;
        }
        float dir_scale = (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY)
                          ? 1.0f : ((m->calib.direction < 0) ? -1.0f : 1.0f);
        b->obs2_vesc.theta_e = theta_out;
        float est_rpm = dir_scale * (b->vesc_pll_vel / (_2PI * m->params.pole_pairs)) * 60.0f;
        /* 由 50Hz 带宽 PLL 锁相环直接输出估计转速，消除过度低通导致的滞后振荡 */
        b->obs2_vesc.speed_rpm = est_rpm;
    }
}

/* =========================================================================
 * 3. Obs 3: 简化状态观测器 (Simplified STO + 连续指数离散化)
 * ========================================================================= */
static void update_obs3_sto_pll(foc_sensorless_bench_t *b, foc_motor_t *m,
                               float va, float vb, float ia, float ib, float dt)
{
    float rs = m->params.rs_ohm;
    float ls = m->params.ls_henry;

    /* 精确一阶指数离散化：杜绝因 Ls 极小 (20uH) 欧拉离散产生数值爆炸 */
    float tau = ls / rs;
    float a1 = expf(-dt / tau);
    float b1 = (1.0f - a1) / rs;

    float i_err_a = b->sto_i_est_a - ia;
    float i_err_b = b->sto_i_est_b - ib;

    /* 状态观测器反馈校正项: 提高反电势自适应带宽 */
    float k_bemf = 200.0f;
    float k_curr = 200.0f;

    b->sto_i_est_a = (a1 * b->sto_i_est_a) + (b1 * (va - b->sto_bemf_a)) - (k_curr * i_err_a * dt);
    b->sto_i_est_b = (a1 * b->sto_i_est_b) + (b1 * (vb - b->sto_bemf_b)) - (k_curr * i_err_b * dt);

    /* 严谨负反馈: 消除交叉项 -i_err * e_err */
    b->sto_bemf_a += (k_bemf * i_err_a) * dt;
    b->sto_bemf_b += (k_bemf * i_err_b) * dt;

    /* 安全防溢出软限幅 (母线电压 14.4V 下反电势不可能超过 15V) */
    if (b->sto_bemf_a > 15.0f)  { b->sto_bemf_a = 15.0f; }
    if (b->sto_bemf_a < -15.0f) { b->sto_bemf_a = -15.0f; }
    if (b->sto_bemf_b > 15.0f)  { b->sto_bemf_b = 15.0f; }
    if (b->sto_bemf_b < -15.0f) { b->sto_bemf_b = -15.0f; }

    float bemf_sq = (b->sto_bemf_a * b->sto_bemf_a) + (b->sto_bemf_b * b->sto_bemf_b);
    b->diag_sto_bemf_a = b->sto_bemf_a;
    b->diag_sto_bemf_b = b->sto_bemf_b;
    b->diag_sto_bemf_mag = sqrtf(bemf_sq);

    if (bemf_sq > 0.0001f) {
        float raw_angle = atan2f(-b->sto_bemf_a, b->sto_bemf_b);
        if (raw_angle < 0.0f) {
            raw_angle += _2PI;
        }

        float pll_kp = 1200.0f;
        float pll_ki = 4.0e5f;
        b->sto_pll_pos = wrap_pm_pi(b->sto_pll_pos + (b->sto_pll_vel * dt));
        float d_theta = wrap_pm_pi(raw_angle - b->sto_pll_pos);
        b->sto_pll_pos = wrap_pm_pi(b->sto_pll_pos + (pll_kp * d_theta * dt));
        b->sto_pll_vel += pll_ki * d_theta * dt;

        float theta_out = b->sto_pll_pos;
        if (theta_out < 0.0f) {
            theta_out += _2PI;
        }
        float dir_scale = (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY)
                          ? 1.0f : ((m->calib.direction < 0) ? -1.0f : 1.0f);
        b->obs3_sto_pll.theta_e = theta_out;
        b->obs3_sto_pll.speed_rpm = dir_scale * (b->sto_pll_vel / (_2PI * m->params.pole_pairs)) * 60.0f;
    }
}

/* =========================================================================
 * 4. Obs 4: 简化状态观测器 + STM32G4 真实片上硬件 CORDIC 协处理器求相
 * ========================================================================= */
static void update_obs4_sto_cordic(foc_sensorless_bench_t *b, foc_motor_t *m, float dt)
{
    float x = b->sto_bemf_b;
    float y = -b->sto_bemf_a;

    if (((x * x) + (y * y)) > 0.001f) {
        /* 调用真正的片上硬件 CORDIC 寄存器解算反正切 [-PI, PI] */
        float raw_phase = foc_cordic_calc_phase(y, x);
        if (raw_phase < 0.0f) {
            raw_phase += _2PI;
        }
        b->cordic_theta = raw_phase;

        /* 独立 PLL 提取转速与平滑滤波角度 */
        float pll_kp = 1200.0f;
        float pll_ki = 4.0e5f;
        b->cordic_pll_pos = wrap_pm_pi(b->cordic_pll_pos + (b->cordic_pll_vel * dt));
        float d_theta = wrap_pm_pi(raw_phase - b->cordic_pll_pos);
        b->cordic_pll_pos = wrap_pm_pi(b->cordic_pll_pos + (pll_kp * d_theta * dt));
        b->cordic_pll_vel += pll_ki * d_theta * dt;

        float theta_out = b->cordic_pll_pos;
        if (theta_out < 0.0f) {
            theta_out += _2PI;
        }
        b->obs4_sto_cordic.theta_e = theta_out;

        float dir_scale = (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY)
                          ? 1.0f : ((m->calib.direction < 0) ? -1.0f : 1.0f);
        b->obs4_sto_cordic.speed_rpm = dir_scale * (b->cordic_pll_vel / (_2PI * m->params.pole_pairs)) * 60.0f;
    }
}

/* =========================================================================
 * 5. Obs 5: HFI 影子模块 (高频方波脉动注入与正交解调锁相)
 * ========================================================================= */
static void update_obs5_hfi(foc_sensorless_bench_t *b, foc_motor_t *m,
                            float ia, float ib, float theta_enc, float dt)
{
    if (b->hfi_enabled == 0U) {
        b->hfi_v_inj_alpha = 0.0f;
        b->hfi_v_inj_beta = 0.0f;
        b->obs5_hfi.converged = 0U;
        b->hfi_confidence = 0.0f;
        return;
    }

    /* 1. 当前估计角度与三角函数 */
    float th_est = b->hfi_pll_pos;
    if (th_est < 0.0f) {
        th_est += _2PI;
    }
    float s_th = sinf(th_est);
    float c_th = cosf(th_est);

    /* 2. 将实际定子电流采样投影至当前估计 dq 坐标系 */
    float i_est_d =  ia * c_th + ib * s_th;
    float i_est_q = -ia * s_th + ib * c_th;

    /* 3. 正交高频电流差分响应计算: delta_iq = i_q(k) - i_q(k-1) */
    float delta_iq = i_est_q - b->hfi_i_prev_q;
    b->hfi_i_prev_q = i_est_q;

    /* 4. 同步正交解调:
     * 上一拍施加的高频电压极性为 hfi_inj_pol。
     * 根据表贴/弱凸极电机特性 (实测 Ld > Lq, delta_L < 0):
     * 当估算角度超前或滞后时，delta_iq 与注入极性的乘积反映误差角 2*theta_err。
     * 解调原始误差信号: raw_err = - (inj_pol) * delta_iq */
    float demod_raw = -((float)b->hfi_inj_pol) * delta_iq;

    /* 载波信号模长统计（用于信噪比与置信度评估） */
    float carrier_abs = fabsf(delta_iq);
    b->hfi_carrier_mag += 0.02f * (carrier_abs - b->hfi_carrier_mag);
    b->hfi_iq_ripple = b->hfi_carrier_mag * 2.0f; /* 峰峰值近似估算 */

    /* 一阶低通平滑解调误差信号 (截止频率约 200Hz) */
    float alpha_filt = dt / (dt + (1.0f / (_2PI * 200.0f)));
    b->hfi_demod_err_filt += alpha_filt * (demod_raw - b->hfi_demod_err_filt);

    /* 5. 专有二阶自适应跟踪 PLL 锁相环:
     * 增益根据实测极性进行缩放。鉴于 DJI 2312S 为弱凸极 (Ld > Lq)，
     * 误差斜率为负，故 PLL 跟踪增益乘负号反向闭环 */
    float err_hfi = b->hfi_demod_err_filt;
    /* 软限幅防止电流突变造成 PLL 积分溢出发散 */
    err_hfi = foc_clampf(err_hfi, -0.5f, 0.5f);

    /* 将解调电流误差通过电感增益折算为角度误差尺度 (rad) */
    /* 理论折算因子: scale ~ 2 * Ld * Lq / (V_inj * dt * |Ld - Lq|) */
    float rad_scale = 10.0f;
    float pll_err_rad = -err_hfi * rad_scale;

    /* PLL 状态积分步进 */
    b->hfi_pll_pos = wrap_pm_pi(b->hfi_pll_pos + (b->hfi_pll_vel * dt));
    b->hfi_pll_pos = wrap_pm_pi(b->hfi_pll_pos + (b->hfi_pll_kp * pll_err_rad * dt));
    b->hfi_pll_vel += b->hfi_pll_ki * pll_err_rad * dt;

    /* 转速限幅保护: HFI 严格面向低速 (<500 RPM) */
    float max_we = 500.0f * FOC_RPM_TO_RADS * (float)m->params.pole_pairs;
    b->hfi_pll_vel = foc_clampf(b->hfi_pll_vel, -max_we, max_we);

    /* 规范化估计电角度到 [0, 2PI) */
    float theta_out = b->hfi_pll_pos;
    if (theta_out < 0.0f) {
        theta_out += _2PI;
    }
    b->obs5_hfi.theta_e = theta_out;

    /* 转速换算 (RPM) */
    float dir_scale = (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY)
                      ? 1.0f : ((m->calib.direction < 0) ? -1.0f : 1.0f);
    b->obs5_hfi.speed_rpm = dir_scale * (b->hfi_pll_vel / (_2PI * m->params.pole_pairs)) * 60.0f;

    /* 6. 置信度与内生锁定判定:
     * 门槛依据：
     * a. 载波响应幅值是否大于最小探测门限 (15mA ~ 0.5 LSB) 且不过度饱和 (<0.4A)；
     * b. 解调残差是否收敛稳定 (|demod_err| < 0.08A)；
     * c. 估计转速是否处于有效低速区间 (<400 RPM)。 */
    if ((b->hfi_carrier_mag >= 0.015f) && (b->hfi_carrier_mag <= 0.400f) &&
        (fabsf(b->hfi_demod_err_filt) < 0.080f) && (fabsf(b->obs5_hfi.speed_rpm) <= 400.0f)) {
        b->hfi_confidence = b->hfi_confidence + 0.005f * (1.0f - b->hfi_confidence);
    } else {
        b->hfi_confidence = b->hfi_confidence * 0.995f;
    }

    if (b->hfi_confidence >= 0.60f) {
        b->obs5_hfi.converged = 1U;
    } else {
        b->obs5_hfi.converged = 0U;
        b->obs5_hfi.unlock_count++;
    }

    /* 7. 生成下一拍待注入的估计 d 轴方波高频电压并旋转到定子 alpha-beta:
     * 隔拍翻转极性 (+1 -> -1 -> +1) */
    b->hfi_inj_pol = (b->hfi_inj_pol == 1) ? -1 : 1;
    float v_inj_d = (float)b->hfi_inj_pol * b->hfi_inj_volt;
    float v_inj_q = 0.0f;

    /* 反 Park 变换到 alpha-beta 坐标系 */
    b->hfi_v_inj_alpha = v_inj_d * c_th - v_inj_q * s_th;
    b->hfi_v_inj_beta  = v_inj_d * s_th + v_inj_q * c_th;
}

/* =========================================================================
 * 单个观测器指标评估与严格收敛判定
 * ========================================================================= */
static void evaluate_metric(foc_bench_obs_metrics_t *m, float theta_enc, uint8_t is_steady)
{
    /* 1. 独立极性与偏置校正 */
    float obs_th = m->theta_e;
    if (m->direction < 0) {
        obs_th = foc_wrap_0_2pi(_2PI - obs_th);
    }
    obs_th = foc_wrap_0_2pi(obs_th + m->theta_offset);

    /* 2. 计算与真实编码器电角度误差 */
    float err_rad = wrap_pm_pi(obs_th - theta_enc);
    float err_deg = err_rad * (180.0f / _PI);
    float abs_deg = fabsf(err_deg);

    m->err_deg = err_deg;
    m->err_abs_deg = abs_deg;

    /* 3. 严格收敛与失锁判定门槛 (内生物理特性判据):
     * 架构原则：观测器自身是否锁定，仅取决于内生磁链模长与估算转速是否处于有效物理区间；
     * 严禁将与外部编码器的角差作为无感失锁依据，避免编码器损坏时无感被连带误判为失锁！
     * DJI 2312S 标称磁链约 0.0008 Wb，安全有效窗口 [0.0001 Wb, 0.0100 Wb]
     * 注：若当前评估对象为 HFI (obs5_hfi)，则由 update_obs5_hfi() 内生置信度决定，不覆盖 */
    if (m != &g_sensorless_bench.obs5_hfi) {
        float min_spd_thresh = (g_angle_mgr.exit_speed_rpm > 150.0f) ? (g_angle_mgr.exit_speed_rpm - 80.0f) : 250.0f;
        if ((fabsf(m->speed_rpm) >= min_spd_thresh) &&
            (m->flux_mag >= 0.0001f) && (m->flux_mag <= 0.010f)) {
            m->converged = 1U;
        } else {
            m->converged = 0U;
            m->unlock_count++;
        }
    }

    /* 4. 仅在稳态采样窗口内进行统计累加，坚决剔除启动过渡与变速阶段 */
    if (is_steady != 0U) {
        m->err_sum_deg += abs_deg;
        m->err_sq_sum += err_deg * err_deg;
        if (abs_deg > m->err_peak_deg) {
            m->err_peak_deg = abs_deg;
        }
        m->samples++;
    }
}

void foc_sensorless_bench_auto_align(foc_motor_t *m)
{
    float enc_th = foc_motor_encoder_theta_e(m);

    /* 自动将当前的瞬时误差对齐为零点 offset */
    float diff1 = wrap_pm_pi(enc_th - g_sensorless_bench.obs1_ortega.theta_e);
    float diff2 = wrap_pm_pi(enc_th - g_sensorless_bench.obs2_vesc.theta_e);
    float diff3 = wrap_pm_pi(enc_th - g_sensorless_bench.obs3_sto_pll.theta_e);
    float diff4 = wrap_pm_pi(enc_th - g_sensorless_bench.obs4_sto_cordic.theta_e);
    float diff5 = wrap_pm_pi(enc_th - g_sensorless_bench.obs5_hfi.theta_e);

    g_sensorless_bench.obs1_ortega.theta_offset = diff1;
    g_sensorless_bench.obs2_vesc.theta_offset = diff2;
    g_sensorless_bench.obs3_sto_pll.theta_offset = diff3;
    g_sensorless_bench.obs4_sto_cordic.theta_offset = diff4;
    g_sensorless_bench.obs5_hfi.theta_offset = diff5;
}

void foc_sensorless_bench_update(foc_motor_t *m, float v_alpha, float v_beta,
                                 float i_alpha, float i_beta, float theta_enc, float dt)
{
    uint32_t t0, t1;
    uint8_t is_steady = g_sensorless_bench.steady_state;

    if (g_sensorless_bench.enabled == 0U) {
        return;
    }

    g_sensorless_bench.diag_v_alpha = v_alpha;
    g_sensorless_bench.diag_v_beta = v_beta;
    g_sensorless_bench.diag_i_alpha = i_alpha;
    g_sensorless_bench.diag_i_beta = i_beta;
    g_sensorless_bench.diag_enc_theta = theta_enc;

    /* 真实端电压死区降落修正 (Reconstructed Terminal Voltage Deadtime Compensation) */
    float va_obs = v_alpha;
    float vb_obs = v_beta;
    if (g_sensorless_bench.obs_deadtime_comp_enable != 0U) {
        const float vc = g_sensorless_bench.deadtime_comp_v;
        const float th_i = 0.05f;
        abc_t i_abc;
        abc_t v_loss;
        ab_t v_loss_ab;

        /* 计算相电流三相物理方向 */
        foc_inv_clarke(&(ab_t){i_alpha, i_beta}, &i_abc);
        v_loss.a = (i_abc.a > th_i) ? vc : ((i_abc.a < -th_i) ? -vc : 0.0f);
        v_loss.b = (i_abc.b > th_i) ? vc : ((i_abc.b < -th_i) ? -vc : 0.0f);
        v_loss.c = (i_abc.c > th_i) ? vc : ((i_abc.c < -th_i) ? -vc : 0.0f);
        foc_clarke(&v_loss, &v_loss_ab);

        /* 从理想平均电压中扣除死区消耗的压降 */
        va_obs -= v_loss_ab.alpha;
        vb_obs -= v_loss_ab.beta;
    }

    /* 1. 第一主接管算法 VESC 约束磁链观测器：逐拍以快环周期 (16kHz, dt) 执行！
     * 保证闭环接管时电角度完全连续、无任何 4 拍停滞阶梯波，控制绝对平滑 */
    t0 = get_dwt_cycles();
    update_obs2_vesc(&g_sensorless_bench, m, va_obs, vb_obs, i_alpha, i_beta, theta_enc, dt);
    t1 = get_dwt_cycles();
    g_sensorless_bench.obs2_vesc.exec_cycles = t1 - t0;
    evaluate_metric(&g_sensorless_bench.obs2_vesc, theta_enc, is_steady);

    /* 2. HFI 影子模块：方波注入与高频解调必须逐拍执行 (16kHz Nyquist 采样)！
     * 若 HFI 使能，在此执行电流采样差分投影、解调、PLL 更新与下一拍注入电压生成 */
    if (g_sensorless_bench.hfi_enabled != 0U) {
        t0 = get_dwt_cycles();
        update_obs5_hfi(&g_sensorless_bench, m, i_alpha, i_beta, theta_enc, dt);
        t1 = get_dwt_cycles();
        g_sensorless_bench.obs5_hfi.exec_cycles = t1 - t0;
        evaluate_metric(&g_sensorless_bench.obs5_hfi, theta_enc, is_steady);
    } else {
        g_sensorless_bench.hfi_v_inj_alpha = 0.0f;
        g_sensorless_bench.hfi_v_inj_beta = 0.0f;
    }

    /* 3. 其余后台对比算法按槽轮询分频执行 (等效 5.3kHz 刷新率)。
     * 默认只跑角度仲裁真正选用的那一个（active_algo），全套影子对比仅在
     * `bench start` 后开启——实测省 ~30% 快环 CPU，消除 max>100% 的超周期。 */
    {
        uint8_t all = g_sensorless_bench.shadow_enabled;
        uint8_t algo = g_angle_mgr.active_algo;

        switch (s_bench_slot) {
        case 0U:
            if ((all != 0U) || (algo == SENSORLESS_ALGO_ORTEGA)) {
                t0 = get_dwt_cycles();
                update_obs1_ortega(&g_sensorless_bench, m, va_obs, vb_obs, i_alpha, i_beta, dt * 3.0f);
                t1 = get_dwt_cycles();
                g_sensorless_bench.obs1_ortega.exec_cycles = t1 - t0;
                evaluate_metric(&g_sensorless_bench.obs1_ortega, theta_enc, is_steady);
            }
            break;

        case 1U:
            if ((all != 0U) || (algo == SENSORLESS_ALGO_STO)) {
                t0 = get_dwt_cycles();
                update_obs3_sto_pll(&g_sensorless_bench, m, va_obs, vb_obs, i_alpha, i_beta, dt * 3.0f);
                t1 = get_dwt_cycles();
                g_sensorless_bench.obs3_sto_pll.exec_cycles = t1 - t0;
                evaluate_metric(&g_sensorless_bench.obs3_sto_pll, theta_enc, is_steady);
            }
            break;

        case 2U:
            if (all != 0U) {
                t0 = get_dwt_cycles();
                update_obs4_sto_cordic(&g_sensorless_bench, m, dt * 3.0f);
                t1 = get_dwt_cycles();
                g_sensorless_bench.obs4_sto_cordic.exec_cycles = t1 - t0;
                evaluate_metric(&g_sensorless_bench.obs4_sto_cordic, theta_enc, is_steady);
            }
            break;

        default:
            break;
        }
    }

    s_bench_slot = (s_bench_slot + 1U) % 3U;
    g_sensorless_bench.total_samples++;
}
