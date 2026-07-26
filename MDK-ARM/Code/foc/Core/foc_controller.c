#include "foc_controller.h"
#include <math.h>
#include "../../vofa/vofa.h"  // VOFA数据上传接口
#include "../App/foc_app.h"
#include "../App/foc_calib.h"
#include "../Driver/current/current_shunt.h"
#include "../Driver/encoder/abz_encoder.h"   // FOC应用层状态管�?
foc_vf_state_t vf = {0};
foc_feedback_t foc_feedback = {0};
volatile foc_safety_diag_t g_foc_safety_diag = {0};
static foc_angle_source_t angle_source = FOC_ANGLE_OPEN_LOOP;
static uint16_t vofa_sample_counter = 0;  // VOFA采样计数器（16kHz降频�?kHz�?

void foc_safety_latch_fault(foc_safety_fault_t fault)
{
    if ((fault != FOC_SAFETY_FAULT_NONE) &&
        (g_foc_safety_diag.fault_code == (uint8_t)FOC_SAFETY_FAULT_NONE)) {
        g_foc_safety_diag.fault_code = (uint8_t)fault;
    }
}

static uint8_t foc_current_limit_ok(foc_state_t state)
{
    float iu;
    float iv;
    float iw;
    float peak;
    float limit;
    float hard_limit;
    uint8_t hard_trip;

    foc_get_currents(&iu, &iv, &iw);
    peak = fabsf(iu);
    if (fabsf(iv) > peak) {
        peak = fabsf(iv);
    }
    if (fabsf(iw) > peak) {
        peak = fabsf(iw);
    }

    limit = (state == FOC_STATE_CALIB) ? FOC_CALIB_CURRENT_LIMIT_A
                                        : foc_motor_info.max_current;
    hard_limit = (state == FOC_STATE_CALIB) ? FOC_CALIB_HARD_CURRENT_LIMIT_A
                                             : foc_motor_info.max_current;
    g_foc_safety_diag.peak_current_a = peak;
    g_foc_safety_diag.current_limit_a = limit;
    g_foc_safety_diag.hard_current_limit_a = hard_limit;
    if (peak > g_foc_safety_diag.max_observed_current_a) {
        g_foc_safety_diag.max_observed_current_a = peak;
    }

    hard_trip = (peak > hard_limit) ? 1U : 0U;

    if (peak > limit) {
        if (g_foc_safety_diag.consecutive_over_limit < 0xFFFFU) {
            ++g_foc_safety_diag.consecutive_over_limit;
        }
    } else {
        g_foc_safety_diag.consecutive_over_limit = 0U;
    }

    if ((hard_trip == 0U) &&
        (g_foc_safety_diag.consecutive_over_limit < FOC_OVERCURRENT_TRIP_SAMPLES)) {
        return 1U;
    }

    /* Latch the coherent sample that caused the trip. It remains unchanged
     * after FOC_STATE_FAULT and is therefore safe to inspect in Keil Watch. */
    g_foc_safety_diag.trip_current_u_a = iu;
    g_foc_safety_diag.trip_current_v_a = iv;
    g_foc_safety_diag.trip_current_w_a = iw;
    g_foc_safety_diag.trip_tim_cnt = TIM1->CNT;
    g_foc_safety_diag.trip_adc1_raw = g_current_shunt_diag.adc1_raw;
    g_foc_safety_diag.trip_adc2_raw = g_current_shunt_diag.adc2_raw;
    g_foc_safety_diag.trip_calib_state = (uint8_t)foc_calib_get_state();
    g_foc_safety_diag.trip_pwm_stage = g_foc_pwm_stage;
    g_foc_safety_diag.trip_active_pair = g_current_shunt_diag.active_pair;
    g_foc_safety_diag.trip_hard_limit = hard_trip;

    foc_set_dq_voltage(0.0f, 0.0f);
    if (state == FOC_STATE_CALIB) {
        foc_safety_latch_fault(FOC_SAFETY_FAULT_CALIB_OVERCURRENT);
        foc_calib_abort();
    } else {
        foc_safety_latch_fault(FOC_SAFETY_FAULT_RUN_OVERCURRENT);
        foc_pwm_disable();
        foc_set_state(FOC_STATE_FAULT);
    }
    return 0U;
}

/**
 * @brief V/f 开环控制初始化（RPM版本�? * @param pole_pairs  极对�? * @param init_rpm    初始机械转速（RPM�? */
void foc_contr_init(float pole_pairs, float init_rpm)
{
    g_foc_safety_diag.peak_current_a = 0.0f;
    g_foc_safety_diag.max_observed_current_a = 0.0f;
    g_foc_safety_diag.current_limit_a = 0.0f;
    g_foc_safety_diag.hard_current_limit_a = 0.0f;
    g_foc_safety_diag.trip_current_u_a = 0.0f;
    g_foc_safety_diag.trip_current_v_a = 0.0f;
    g_foc_safety_diag.trip_current_w_a = 0.0f;
    g_foc_safety_diag.trip_tim_cnt = 0U;
    g_foc_safety_diag.consecutive_over_limit = 0U;
    g_foc_safety_diag.trip_adc1_raw = 0U;
    g_foc_safety_diag.trip_adc2_raw = 0U;
    g_foc_safety_diag.fault_code = (uint8_t)FOC_SAFETY_FAULT_NONE;
    g_foc_safety_diag.trip_calib_state = 0U;
    g_foc_safety_diag.trip_pwm_stage = 0U;
    g_foc_safety_diag.trip_active_pair = 0U;
    g_foc_safety_diag.trip_hard_limit = 0U;
    vf.pole_pairs = pole_pairs;

    /* 统一通过 RPM setter 计算角度步进 */
    foc_set_speed_rpm(init_rpm);

    /* dq 初始�?*/
    vf.dq.d = 0.0f;  /* V/f开环模式，d轴电压命令为0 */
    vf.dq.q = 0.0f;

    /* 初始电压命令 */
    foc_vf_set_voltage(0.5f);

    /* 角度清零 */
    vf.shaft_angle_rad      = 0.0f;
    vf.electrical_angle_rad = 0.0f;

    /* αβ 清零 */
    vf.ab.alpha = 0.0f;
    vf.ab.beta  = 0.0f;
}

/**
 * @brief 设置机械转速（RPM�? * @param rpm  机械转速：�?分钟
 */
void foc_set_speed_rpm(float rpm)
{
    const float dt = 1.0f / PWM_FREQ_HZ;
    // Δθe = 2π * (rpm/60) * pole_pairs * dt
    vf.angle_step_rad = _2PI * (rpm * (1.0f / 60.0f)) * vf.pole_pairs * dt;
}

/**
 * @brief 设置 Vq 命令，限幅至 SVPWM 线性调制最大�? *        SVPWM 最大相电压幅�?= Udc / �?
 * @param v_q 目标 Vq（V�? */
static float foc_limit_voltage(float voltage)
{
    const float max_v = U_DC * INV_SQRT_3;

    if (voltage > max_v) {
        return max_v;
    }
    if (voltage < -max_v) {
        return -max_v;
    }
    return voltage;
}
void foc_vf_set_voltage(float v_q)
{
    foc_set_dq_voltage(vf.dq.d, v_q);
}

void foc_set_dq_voltage(float vd, float vq)
{
    vf.dq.d = foc_limit_voltage(vd);
    vf.dq.q = foc_limit_voltage(vq);
}

void foc_openloop_hold(float theta_e, float vd, float vq)
{
    vf.electrical_angle_rad = limit_angle_rad(theta_e);
    vf.shaft_angle_rad = vf.electrical_angle_rad / vf.pole_pairs;
    vf.angle_step_rad = 0.0f;
    
    angle_source = FOC_ANGLE_OPEN_LOOP;
    foc_set_dq_voltage(vd, vq);
}

void foc_openloop_spin(float rpm, float vd, float vq)
{
    foc_set_speed_rpm(rpm);
    
    angle_source = FOC_ANGLE_OPEN_LOOP;
    foc_set_dq_voltage(vd, vq);
}

void foc_set_angle_source(foc_angle_source_t source)
{
    angle_source = source;
}

foc_angle_source_t foc_get_angle_source(void)
{
    return angle_source;
}

float foc_get_calibrated_electrical_angle(void)
{
    const foc_calib_result_t *calib = foc_calib_get_result();

    if ((calib == 0) || (calib->valid == 0U)) {
        return vf.electrical_angle_rad;
    }

    /* Encoder angle is mechanical angle. pole_pairs converts it to electrical
     * angle. electrical_offset_rad comes from D-axis alignment + Z/index search,
     * so the d-axis follows rotor flux and q-axis stays perpendicular to it. */
    return limit_angle_rad(
        ((float)calib->direction * vf.pole_pairs * foc_feedback.angle_rad) +
        calib->electrical_offset_rad);
}
/**
 * @brief Step open-loop electrical angle.
 */
static float foc_step_electrical_angle(void)
{
    vf.electrical_angle_rad = limit_angle_rad(
        vf.electrical_angle_rad + vf.angle_step_rad);

    /* 机械角度由电角度折算 */
    vf.shaft_angle_rad = vf.electrical_angle_rad / vf.pole_pairs;

    return vf.electrical_angle_rad;
}

/**
 * @brief 定时器中断回调（16 kHz�? * �?HAL_TIM_PeriodElapsedCallback 中调�? */
void foc_tim_irq(void)
{
    foc_state_t state = foc_get_state();

    /* 仅在RUN状态下执行FOC控制 */
    if ((state == FOC_STATE_RUN) || (state == FOC_STATE_CALIB))
    {
        float theta;

        if (foc_current_limit_ok(state) == 0U) {
            return;
        }

        /* Do not let the control ISR overwrite CCR=0 during bootstrap charge. */
        if ((state != FOC_STATE_CALIB) ||
            (foc_calib_get_state() != FOC_CALIB_CHARGE_BOOTSTRAP)) {
            /* CALIB always uses open-loop angle so it can align/search safely.
             * RUN can use calibrated encoder angle after calibration is valid. */
            if ((state == FOC_STATE_RUN) &&
                (angle_source == FOC_ANGLE_ENCODER_CALIBRATED) &&
                (foc_calib_is_valid())) {
                theta = foc_get_calibrated_electrical_angle();
                vf.electrical_angle_rad = theta;
                vf.shaft_angle_rad = foc_feedback.angle_rad;
            } else {
                theta = foc_step_electrical_angle();
            }

            inverse_park_transform(&vf.dq, theta, &vf.ab);
            vf.svpwm_sector = svpwm_calc(&vf.ab);
        }
    } else {
        g_foc_safety_diag.consecutive_over_limit = 0U;
        g_foc_safety_diag.current_limit_a = 0.0f;
    }

    /* VOFA数据上传�?6kHz -> 1kHz�?*/
    if (++vofa_sample_counter >= 16) // 16kHz / 16 = 1kHz
    {
        vofa_sample_counter = 0;
        VOFA_Task();
    }

    abz_encoder_update();
}
