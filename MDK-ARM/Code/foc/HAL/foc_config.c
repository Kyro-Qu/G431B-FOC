#include "foc_config.h"
#include "../Core/foc_controller.h"
#include "../Driver/current/current_shunt.h"
#include "../App/foc_app.h"
#include "../App/foc_calib.h"

foc_motor_info_t foc_motor_info;
foc_log_monitor_t foc_log_monitor;
volatile uint8_t g_foc_pwm_enabled = 0U;
volatile uint8_t g_foc_pwm_stage = 0U;

#define FOC_PWM_CCER_MASK (TIM_CCER_CC1E  | TIM_CCER_CC1NE | \
                           TIM_CCER_CC2E  | TIM_CCER_CC2NE | \
                           TIM_CCER_CC3E  | TIM_CCER_CC3NE)

/*
 * Update all three active compare registers while the power stage is gated
 * off. Temporarily disabling preload avoids re-enabling MOE with stale duty
 * values from the previous run.
 */
static void foc_pwm_set_neutral_now(void)
{
    CLEAR_BIT(htim1.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    CLEAR_BIT(htim1.Instance->CCMR2, TIM_CCMR2_OC3PE);

    htim1.Instance->CCR1 = PWM_NEUTRAL_CNT;
    htim1.Instance->CCR2 = PWM_NEUTRAL_CNT;
    htim1.Instance->CCR3 = PWM_NEUTRAL_CNT;

    SET_BIT(htim1.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    SET_BIT(htim1.Instance->CCMR2, TIM_CCMR2_OC3PE);
}

/* Match MCSDK R3_2_TurnOnLowSides(..., 0): CHx is inactive and CHxN is active. */
static void foc_pwm_set_low_sides_now(void)
{
    CLEAR_BIT(htim1.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    CLEAR_BIT(htim1.Instance->CCMR2, TIM_CCMR2_OC3PE);

    htim1.Instance->CCR1 = 0U;
    htim1.Instance->CCR2 = 0U;
    htim1.Instance->CCR3 = 0U;

    SET_BIT(htim1.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    SET_BIT(htim1.Instance->CCMR2, TIM_CCMR2_OC3PE);
}

void foc_timer_init(void)
{
    system_power_checkpoint(SYSTEM_CHECKPOINT_TIMER_READY,
                            0U,
                            (uint32_t)g_foc_calib_state,
                            (uint32_t)g_foc_state_diag);
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    foc_pwm_set_neutral_now();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_CNT - 1U);

    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);

    /*
     * Match the validated MCSDK startup phase: begin one tick before ARR with
     * the initial up-count direction.  This keeps the update ISR that submits
     * JSQR away from the CCR4 trigger placed near the counter peak.
     */
    CLEAR_BIT(htim1.Instance->CR1, TIM_CR1_DIR);
    __HAL_TIM_SET_COUNTER(&htim1, PWM_CNT - 1U);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }

    /* HAL starts an advanced-timer channel by setting MOE. CH1..3 are still
     * disabled here, so clear MOE before arming all six phase outputs. */
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    SET_BIT(htim1.Instance->BDTR, TIM_BDTR_OSSR | TIM_BDTR_OSSI);
    SET_BIT(htim1.Instance->CCER, FOC_PWM_CCER_MASK);
    g_foc_pwm_enabled = 0U;
    g_foc_pwm_stage = 0U;
}

void foc_pwm_bootstrap_start(void)
{
    if (current_shunt_is_ready() == 0U)
    {
        return;
    }

    system_power_checkpoint(SYSTEM_CHECKPOINT_BOOTSTRAP_ON,
                            1U,
                            (uint32_t)g_foc_calib_state,
                            (uint32_t)g_foc_state_diag);
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    foc_pwm_set_low_sides_now();
    SET_BIT(htim1.Instance->CCER, FOC_PWM_CCER_MASK);
    SET_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    g_foc_pwm_enabled = 1U;
    g_foc_pwm_stage = 1U;
}

void foc_pwm_enable(void)
{
    if (current_shunt_is_ready() == 0U)
    {
        return;
    }

    system_power_checkpoint(SYSTEM_CHECKPOINT_NORMAL_PWM_ON,
                            2U,
                            (uint32_t)g_foc_calib_state,
                            (uint32_t)g_foc_state_diag);
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    foc_pwm_set_neutral_now();
    SET_BIT(htim1.Instance->CCER, FOC_PWM_CCER_MASK);
    SET_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    g_foc_pwm_enabled = 1U;
    g_foc_pwm_stage = 2U;
}

void foc_pwm_disable(void)
{
    system_power_checkpoint(SYSTEM_CHECKPOINT_PWM_DISABLE,
                            (uint32_t)g_foc_pwm_stage,
                            (uint32_t)g_foc_calib_state,
                            (uint32_t)g_foc_state_diag);
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    g_foc_pwm_enabled = 0U;
    g_foc_pwm_stage = 0U;
    foc_pwm_set_neutral_now();
}

inline void foc_set_pwm(uint32_t ccr_a, uint32_t ccr_b, uint32_t ccr_c, uint8_t sector)
{
    if ((ccr_a > PWM_CNT) || (ccr_b > PWM_CNT) || (ccr_c > PWM_CNT))
    {
        foc_pwm_disable();
        return;
    }

    current_shunt_prepare_pwm(ccr_a, ccr_b, ccr_c, sector);
    if (current_shunt_is_ready() == 0U)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
}

void foc_motor_init(void)
{
    /* DJI 2312S parameters from the validated MCSDK project. */
    foc_motor_info.pole_pairs = 6.0f;
    foc_motor_info.Rs = 0.1f;
    foc_motor_info.Ls = 0.00002f;
    foc_motor_info.max_current = 5.2f;
    foc_motor_info.max_voltage = 14.23f;
    foc_motor_info.Ke = 0.9f;
    foc_motor_info.max_rpm = 12450.0f;
}

void foc_log_monitor_init(void)
{
    foc_log_monitor.angle = &vf.electrical_angle_rad;
    foc_log_monitor.dq = &vf.dq;
    foc_log_monitor.ab = &vf.ab;
    foc_log_monitor.pwm_a = (uint16_t *)&htim1.Instance->CCR1;
    foc_log_monitor.pwm_b = (uint16_t *)&htim1.Instance->CCR2;
    foc_log_monitor.pwm_c = (uint16_t *)&htim1.Instance->CCR3;
}

__attribute__((weak)) float foc_get_electrical_angle(void)
{
    return 0.0f;
}

void foc_get_currents(float *ia, float *ib, float *ic)
{
    current_shunt_get_currents(ia, ib, ic);
}
