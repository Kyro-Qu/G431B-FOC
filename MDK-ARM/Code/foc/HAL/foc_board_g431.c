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
    .velocity_control_rpm = abz_encoder_pll_velocity_rpm,
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

/* ======================== 系统级稳定性设施 ======================== */

foc_cpu_diag_t g_foc_cpu_diag = {0};

/* 一拍快环的周期预算：一个 PWM 周期的 CPU 时钟数 */
#define FOC_CYCLES_PER_TICK (170000000.0f / FOC_PWM_FREQ_HZ)

void foc_board_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t foc_board_cycles(void)
{
    return DWT->CYCCNT;
}

void foc_board_cpu_sample(uint32_t cycles)
{
    g_foc_cpu_diag.last_cycles = cycles;
    if (cycles > g_foc_cpu_diag.max_cycles) {
        g_foc_cpu_diag.max_cycles = cycles;
    }
    g_foc_cpu_diag.load_pct = 100.0f * (float)cycles / FOC_CYCLES_PER_TICK;
}

/*
 * IWDG 直接寄存器操作（HAL IWDG 模块未启用，寄存器序列很短）：
 * LSI ≈ 32 kHz，预分频 /32 → 计数节拍 1 ms，RLR 直接填毫秒数。
 * LSI 精度 ±5% 左右，超时按标称值放余量即可。
 */
void foc_board_watchdog_init(uint32_t timeout_ms)
{
    uint32_t reload = timeout_ms;

    if (reload > 4095U) {
        reload = 4095U;
    }
    if (reload == 0U) {
        reload = 1U;
    }

    /* 调试器停在断点时冻结看门狗计数，避免调试即复位 */
    SET_BIT(DBGMCU->APB1FZR1, DBGMCU_APB1FZR1_DBG_IWDG_STOP);

    IWDG->KR = 0x0000CCCCU;   /* 启动看门狗 */
    IWDG->KR = 0x00005555U;   /* 解锁 PR/RLR */
    IWDG->PR = 3U;            /* LSI/32 → 1 kHz */
    IWDG->RLR = reload;
    /* 等待寄存器更新完成：正常 <0.3ms（LSI 起振 + 5 个 LSI 周期）。
     * 加 10ms 上限，LSI 硅片级失效时放弃等待而不是卡死上电 */
    {
        uint32_t t0 = HAL_GetTick();

        while ((IWDG->SR != 0U) &&
               ((uint32_t)(HAL_GetTick() - t0) < 10U)) {
        }
    }
    IWDG->KR = 0x0000AAAAU;   /* 首次喂狗，装载 RLR */
}

void foc_board_watchdog_kick(void)
{
    IWDG->KR = 0x0000AAAAU;
}

/* ======================== 母线电压实时采样（PA0 / ADC1_IN1） ======================== */

volatile foc_vbus_diag_t g_foc_vbus_diag = {
    .raw_adc = 0U,
    .voltage_v = FOC_UDC_V,
    .valid = 0U,
    .sample_count = 0U
};

static uint32_t s_vbus_last_tick = 0U;

void foc_board_vbus_init(void)
{
    g_foc_vbus_diag.raw_adc = 0U;
    g_foc_vbus_diag.voltage_v = FOC_UDC_V;
    g_foc_vbus_diag.valid = 0U;
    g_foc_vbus_diag.sample_count = 0U;
    s_vbus_last_tick = HAL_GetTick();

    /* 确保 PA0（ADC1_IN1）规则通道在 current_shunt_init() 重新校准后正确配置：
     * 1. 序列长度 = 1（单个转换）；
     * 2. Rank 1 = Channel 1 (PA0)；
     * 3. 采样时间 = 92.5 cycles（匹配 16.3kΩ 分压阻抗）。
     */
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_92CYCLES_5);
}

void foc_board_vbus_update(void)
{
#if FOC_VBUS_ENABLE
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - s_vbus_last_tick) < FOC_VBUS_SAMPLE_INTERVAL_MS) {
        return;
    }
    s_vbus_last_tick = now;

    /* 非阻塞状态机式转换：
     * 1. 若当前没有规则转换正在进行，启动一次软件单次转换；
     * 2. 若已有转换完成标志（EOC），读取结果并应用一阶滤波。
     * 绝不使用阻塞等待 while(EOC)，完全零开销防中断饥饿。 */
    if (LL_ADC_IsActiveFlag_EOC(ADC1) != 0U) {
        uint32_t raw = LL_ADC_REG_ReadConversionData12(ADC1);
        LL_ADC_ClearFlag_EOC(ADC1);

        /* 换算母线电压: Vbus = Vadc / 分压比
         * Vadc = raw * VREF / 4095.0f */
        float vadc = (float)raw * (FOC_VBUS_ADC_VREF / 4095.0f);
        float vbus_meas = vadc / FOC_VBUS_PARTITIONING_FACTOR;

        if (g_foc_vbus_diag.valid == 0U) {
            g_foc_vbus_diag.voltage_v = vbus_meas;
            g_foc_vbus_diag.valid = 1U;
        } else {
            g_foc_vbus_diag.voltage_v += FOC_VBUS_LPF_ALPHA * (vbus_meas - g_foc_vbus_diag.voltage_v);
        }
        g_foc_vbus_diag.raw_adc = (uint16_t)raw;
        g_foc_vbus_diag.sample_count++;
    }

    /* 若规则通道处于空闲（没有正在转换），触发下一次转换 */
    if (LL_ADC_REG_IsConversionOngoing(ADC1) == 0U) {
        LL_ADC_REG_StartConversion(ADC1);
    }
#endif
}

float foc_board_get_vbus_v(void)
{
#if FOC_VBUS_ENABLE
    if (g_foc_vbus_diag.valid != 0U) {
        return g_foc_vbus_diag.voltage_v;
    }
#endif
    return FOC_UDC_V;
}

