/**
 * @file    foc_board_g431.c
 * @brief   STM32G431 板级绑定实现
 *
 * PWM 上电时序继承自本板验证过的 MCSDK 风格流程：
 *   - 修改比较值前先临时关闭预装载，避免带着上一次运行的旧占空比
 *     重新使能 MOE；
 *   - 上电顺序：MOE 关 → 预置中点 → 计数器定位 → 启动 CH4 触发 →
 *     清 MOE（HAL 启动通道时会顺手置位）→ 装配六路 CCER；
 *   - 自举充电：CHx 低边全导通（CCR=0，等效 MCSDK TurnOnLowSides）。
 */

#include "foc_board_g431.h"
#include "main.h"
#include "../App/foc_app.h"
#include "../App/foc_calib.h"
#include "../Driver/current/current_shunt.h"
#include "../Driver/encoder/abz_encoder.h"

extern TIM_HandleTypeDef htim1;

volatile uint8_t g_foc_pwm_stage = 0U;
volatile uint8_t g_foc_pwm_enabled = 0U;

#define FOC_PWM_NEUTRAL_CNT (FOC_PWM_ARR / 2U)

#define FOC_PWM_CCER_MASK (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
                           TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                           TIM_CCER_CC3E | TIM_CCER_CC3NE)

/* ======================== 轴 0：TIM1 PWM ======================== */

/*
 * 在功率级关闭期间同步刷新三路比较值。
 * 临时关闭预装载，保证写入立即生效，不残留旧占空比。
 */
static void m0_pwm_set_neutral_now(void)
{
    CLEAR_BIT(htim1.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    CLEAR_BIT(htim1.Instance->CCMR2, TIM_CCMR2_OC3PE);

    htim1.Instance->CCR1 = FOC_PWM_NEUTRAL_CNT;
    htim1.Instance->CCR2 = FOC_PWM_NEUTRAL_CNT;
    htim1.Instance->CCR3 = FOC_PWM_NEUTRAL_CNT;

    SET_BIT(htim1.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    SET_BIT(htim1.Instance->CCMR2, TIM_CCMR2_OC3PE);
}

/* 等效 MCSDK R3_2_TurnOnLowSides：CHx 无效、CHxN 有效（低边全通） */
static void m0_pwm_set_low_sides_now(void)
{
    CLEAR_BIT(htim1.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    CLEAR_BIT(htim1.Instance->CCMR2, TIM_CCMR2_OC3PE);

    htim1.Instance->CCR1 = 0U;
    htim1.Instance->CCR2 = 0U;
    htim1.Instance->CCR3 = 0U;

    SET_BIT(htim1.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    SET_BIT(htim1.Instance->CCMR2, TIM_CCMR2_OC3PE);
}

void foc_board_init(void)
{
    system_power_checkpoint(SYSTEM_CHECKPOINT_TIMER_READY, 0U,
                            (uint32_t)g_foc_calib_state,
                            (uint32_t)foc_app_diag_state());
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    m0_pwm_set_neutral_now();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, FOC_PWM_ARR - 1U);

    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);

    /*
     * 与验证过的 MCSDK 启动相位一致：从 ARR-1 开始向上计数。
     * 这样提交 JSQR 的更新中断与放在计数峰值附近的 CCR4 触发点
     * 保持安全距离。
     */
    CLEAR_BIT(htim1.Instance->CR1, TIM_CR1_DIR);
    __HAL_TIM_SET_COUNTER(&htim1, FOC_PWM_ARR - 1U);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }

    /* HAL 启动高级定时器通道时会置位 MOE，而 CH1..3 尚未使能，
     * 先清掉 MOE 再装配六路输出 */
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    SET_BIT(htim1.Instance->BDTR, TIM_BDTR_OSSR | TIM_BDTR_OSSI);
    SET_BIT(htim1.Instance->CCER, FOC_PWM_CCER_MASK);
    g_foc_pwm_enabled = 0U;
    g_foc_pwm_stage = 0U;
}

static void m0_pwm_bootstrap(void)
{
    if (current_shunt_is_ready() == 0U)
    {
        return;
    }

    system_power_checkpoint(SYSTEM_CHECKPOINT_BOOTSTRAP_ON, 1U,
                            (uint32_t)g_foc_calib_state,
                            (uint32_t)foc_app_diag_state());
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    m0_pwm_set_low_sides_now();
    SET_BIT(htim1.Instance->CCER, FOC_PWM_CCER_MASK);
    SET_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    g_foc_pwm_enabled = 1U;
    g_foc_pwm_stage = 1U;
}

static void m0_pwm_enable(void)
{
    if (current_shunt_is_ready() == 0U)
    {
        return;
    }

    system_power_checkpoint(SYSTEM_CHECKPOINT_NORMAL_PWM_ON, 2U,
                            (uint32_t)g_foc_calib_state,
                            (uint32_t)foc_app_diag_state());
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    m0_pwm_set_neutral_now();
    SET_BIT(htim1.Instance->CCER, FOC_PWM_CCER_MASK);
    SET_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    g_foc_pwm_enabled = 1U;
    g_foc_pwm_stage = 2U;
}

static void m0_pwm_disable(void)
{
    system_power_checkpoint(SYSTEM_CHECKPOINT_PWM_DISABLE,
                            (uint32_t)g_foc_pwm_stage,
                            (uint32_t)g_foc_calib_state,
                            (uint32_t)foc_app_diag_state());
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    g_foc_pwm_enabled = 0U;
    g_foc_pwm_stage = 0U;
    m0_pwm_set_neutral_now();
}

/*
 * 写三相比较值。顺序：
 *   1. 越界保护（越界说明上游算法异常，直接关功率级）；
 *   2. 让电流采样驱动根据新占空比规划下一拍的采样点和通道路由；
 *   3. 采样链路健康才真正写入 CCR。
 */
static void m0_pwm_set_compare(uint32_t ccr_a, uint32_t ccr_b,
                               uint32_t ccr_c, uint8_t sector)
{
    if ((ccr_a > FOC_PWM_ARR) || (ccr_b > FOC_PWM_ARR) || (ccr_c > FOC_PWM_ARR))
    {
        m0_pwm_disable();
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

/* ======================== 轴 0：接口表 ======================== */

const foc_driver_if_t g_board_m0_driver = {
    .enable      = m0_pwm_enable,
    .disable     = m0_pwm_disable,
    .bootstrap   = m0_pwm_bootstrap,
    .set_compare = m0_pwm_set_compare,
    .full_count  = FOC_PWM_ARR,
    .u_dc        = FOC_UDC_V,
};

const foc_current_if_t g_board_m0_current = {
    .is_ready = current_shunt_is_ready,
    .get      = current_shunt_get_currents,
};

const foc_sensor_if_t g_board_m0_sensor = {
    .update            = abz_encoder_update,
    .angle_rad         = abz_encoder_angle_rad,
    .velocity_rpm      = abz_encoder_velocity_rpm,
    .force_zero        = abz_encoder_force_zero,
    .set_zero_on_index = abz_encoder_set_zero_on_index,
    .consume_index     = abz_encoder_consume_index,
    .rad_per_cnt       = ABZ_RAD_PER_CNT,
};

/* ======================== 轴 1：虚拟接口（演示双轴框架） ======================== */

#if FOC_NUM_AXES >= 2

/*
 * 虚拟功率级：什么都不做。
 * 接入真实硬件时，把这四个函数替换为第二个高级定时器
 * （如 TIM8）的对应操作，接口表指针换掉即可。
 */
static void m1_pwm_enable(void)  {}
static void m1_pwm_disable(void) {}
static void m1_pwm_bootstrap(void) {}
static void m1_pwm_set_compare(uint32_t ccr_a, uint32_t ccr_b,
                               uint32_t ccr_c, uint8_t sector)
{
    (void)ccr_a;
    (void)ccr_b;
    (void)ccr_c;
    (void)sector;
}

const foc_driver_if_t g_board_m1_driver = {
    .enable      = m1_pwm_enable,
    .disable     = m1_pwm_disable,
    .bootstrap   = m1_pwm_bootstrap,
    .set_compare = m1_pwm_set_compare,
    .full_count  = FOC_PWM_ARR,
    .u_dc        = FOC_UDC_V,
};

#endif /* FOC_NUM_AXES >= 2 */
