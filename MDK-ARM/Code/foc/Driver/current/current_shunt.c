#include "current_shunt.h"

#include "main.h"
#include "foc_config.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_opamp.h"
#include <string.h>

/* 本文件内沿用 PWM_CNT 简写指代定时器 ARR。 */
#define PWM_CNT FOC_PWM_ARR

/*
 * STM32G431 R3_2 低边电流采样驱动。
 *
 * TIM1 CH4 产生 OC4REF，TIM1 把 OC4REF 路由到 TRGO，两路 ADC 的注入组
 * 在同一个触发沿采样。驱动为每个 PWM 周期在 ADC1 和 ADC2 各配置一个
 * 注入序列，再利用 Iu + Iv + Iw = 0 重建第三相电流。
 *
 * 对时序敏感的 ADC 上下文更新由本模块独占；FOC 代码只通过
 * current_shunt_prepare_pwm() 提供 PWM 比较值。
 */

/* ADC 码值到电流的换算常量。
 *
 * 硬件真值（火柴FOC bd6s40a_mini_g431 V2 原理图确认）：
 *   采样电阻 20mΩ；放大网络与 ST B-G431B-ESC1 等效——
 *   电阻是 ESC1 的 20/3 倍，增益是 ESC1 的 3/20：
 *   理论有效增益 = 9.14 × 3/20 ≈ 1.371（本值 1.367 为实测标定）。
 *   两块板的 RSHUNT×GAIN 乘积相同，因此固件电流换算可以通用。 */
#define CURRENT_ADC_VREF_V      3.3f
#define CURRENT_ADC_FULL_SCALE  4095.0f
#define CURRENT_SHUNT_OHM       0.02f
#define CURRENT_EFFECTIVE_GAIN  1.367f
#define CURRENT_AMPS_PER_COUNT (CURRENT_ADC_VREF_V /      \
                                (CURRENT_ADC_FULL_SCALE * \
                                 CURRENT_SHUNT_OHM *      \
                                 CURRENT_EFFECTIVE_GAIN))

/* 零电流校准策略。 */
#define CURRENT_OFFSET_SAMPLES   1024UL
#define CURRENT_OFFSET_MIN_COUNT 2048U
#define CURRENT_OFFSET_MAX_COUNT 3072U

/*
 * 距离 PWM 开关沿的安全间隔，单位为 TIM1 计数。
 * 覆盖死区时间、开关噪声、OPAMP 建立时间、ADC 采样时间，
 * 以及采样点算法使用的设计裕量。
 * TAFTER 对齐 ST MCSDK R3_2 的 TW_AFTER（死区 750ns + 振铃 1000ns
 * = 1747ns = 297 tick）：下桥开通后必须等死区结束且开关振铃衰减，
 * 否则高速大 di/dt 时相电压瞬态耦合进运放输出形成电流尖峰假象
 * （实测 2400 RPM 以上单拍 7~10A 假尖峰触发硬保护）。
 */
#define CURRENT_SAMPLE_TBEFORE 41U
#define CURRENT_SAMPLE_TAFTER  297U

/* 14.4 V / 300 uH limits a physical one-period DC step to about 3.0 A.
 * A 4.5 A threshold safely rejects switching noise and false spikes while allowing full bandwidth.
 * 连续无效样本上限：黑匣子实测 2330 RPM 振荡发散时采样 step 连续超限
 * 6 拍——电流环对"发布旧值"积分 6 拍后电压指令漂移出安全区，
 * 单拍冲到 21 A 硬跳。8 拍太宽容，降到 3 拍在积分失控前停机。
 * 诊断（2026-09-02）：振荡起振根因仍待查，此为防爆炸的安全限。 */
#define CURRENT_MAX_STEP_A      3.0f
/* SDK 式裸奔复现实验（2026-09-03）：连续拒绝上限放宽到 255（等于
 * 关闭 DISCONTINUITY 保护），硬过流 12A 仍保留为最后防线。 */
#define DIAG_BARE_MODE 0
#if DIAG_BARE_MODE
#define CURRENT_MAX_REJECTED_CONSECUTIVE 255U
#else
/* 2026-09-03：单路 ADC 偶发假偏差（V 相 OPAMP2 受扰 -3.3A）会连续
 * 3 拍触发旧阈值而误停机；真爆发发展需 >1ms。30 拍 = 1ms 是
 * "毛刺通过/爆发停机"的分界。硬过流 12A 仍单拍兜底。
 * （用户授权调整 2026-09-03） */
#define CURRENT_MAX_REJECTED_CONSECUTIVE 30U
#endif

/* 同一 PWM 周期内转换的两相组合。 */
typedef enum {
    CURRENT_PAIR_UV = 0,
    CURRENT_PAIR_UW = 1,
    CURRENT_PAIR_VW = 2
} current_sample_pair_t;

/*
 * 预期暂态宽限（无感角度切换等）：宽限期内 step 判据放宽到
 * transient_step_limit_a，任一拍回到正常判据内立即结束。
 * 单向计数避免中断/主循环竞争；grace_a<=0 时立即失效。
 * 2026-09-04：角度切换暂态是真实物理暂态——14.4V/300µH 一个周期
 * 的物理极限 ≈3.0A，与 CURRENT_MAX_STEP_A 相等，连续 30 拍必然
 * 触发 DISCONTINUITY 停机（切换实验遥测即停的根因）。
 */
static volatile float transient_step_limit_a = 0.0f;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;
extern TIM_HandleTypeDef htim1;

/* 导出的诊断量；所有字段都在中断上下文中写入。 */
volatile current_shunt_diag_t g_current_shunt_diag;

/*
 * pending_pair 由下一组 PWM 比较值计算得出；active_pair 与产生当前
 * 正在读取的 ADC 结果的那份 JSQR 上下文一同锁存。
 */
static volatile current_sample_pair_t pending_pair = CURRENT_PAIR_UV;
static volatile current_sample_pair_t active_pair = CURRENT_PAIR_UV;
static volatile uint32_t pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_RISING;

/* 上下文归属标志与模块生命周期标志。 */
static volatile uint8_t contexts_armed = 0U;
static volatile uint8_t adc_deferred_consecutive = 0U;
static volatile uint8_t initialized = 0U;
static volatile uint8_t ready = 0U;
static volatile uint8_t suspended = 0U;
static volatile uint8_t rejected_consecutive = 0U;

/* 仅在 PWM 主输出关闭期间使用的零偏累加器。 */
static uint32_t offset_sum_u;
static uint32_t offset_sum_v;
static uint32_t offset_sum_w;
static uint32_t offset_sum_w_internal;
static uint32_t offset_sample_count;

/* 抓取可解释采样故障原因的寄存器快照。 */
static void current_shunt_snapshot(void)
{
    g_current_shunt_diag.adc1_isr = ADC1->ISR;
    g_current_shunt_diag.adc2_isr = ADC2->ISR;
    g_current_shunt_diag.adc1_jsqr = ADC1->JSQR;
    g_current_shunt_diag.adc2_jsqr = ADC2->JSQR;
    g_current_shunt_diag.adc1_cr = ADC1->CR;
    g_current_shunt_diag.adc2_cr = ADC2->CR;
    g_current_shunt_diag.tim_cnt = htim1.Instance->CNT;
    g_current_shunt_diag.tim_cr1 = htim1.Instance->CR1;
    g_current_shunt_diag.active_pair = (uint8_t)active_pair;
    g_current_shunt_diag.pending_pair = (uint8_t)pending_pair;
    g_current_shunt_diag.contexts_armed = contexts_armed;
    g_current_shunt_diag.adc_deferred_consecutive =
        adc_deferred_consecutive;
}

/*
 * 失效安全：停止后续 TRGO 边沿并关闭 TIM1 主输出。
 * 第一个详细故障原因被锁存，供 Keil Watch 查看。
 * 应用层观察到 ready == 0 后将 FOC 状态切换为 FAULT。
 */
static void current_shunt_fail(current_shunt_fault_t reason)
{
    if (g_current_shunt_diag.fault_code ==
        (uint8_t)CURRENT_SHUNT_FAULT_NONE) {
        g_current_shunt_diag.fault_code = (uint8_t)reason;
        current_shunt_snapshot();
    }
    ready = 0U;
    g_current_shunt_diag.state = (uint8_t)CURRENT_SHUNT_IDLE;

    if (htim1.Instance != 0) {
        CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
        MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_RESET);
    }
}

/*
 * 构造单序列注入 JSQR 上下文。
 * JL 清零表示单次转换；JEXTSEL/JEXTEN 将其接到 TIM1_TRGO 的选定边沿。
 * 调用方必须只在 TRGO 关闭时写入。
 */
static uint32_t current_shunt_jsqr(uint32_t channel, uint32_t edge)
{
    return ((channel << ADC_JSQR_JSQ1_Pos) & ADC_JSQR_JSQ1) |
           (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ADC_JSQR_JEXTSEL) |
           (edge & ADC_JSQR_JEXTEN);
}

/*
 * 换算两路 ADC 原始码值，并用 KCL 重建第三相。
 * 每种组合的 ADC 路由固定：
 *   UV: ADC1=U, ADC2=V
 *   UW: ADC1=U, ADC2=W（经 OPAMP3 内部输出）
 *   VW: ADC1=W, ADC2=V
 */
static void current_shunt_reconstruct(current_sample_pair_t pair,
                                      uint16_t adc1_raw,
                                      uint16_t adc2_raw,
                                      uint16_t offset_u,
                                      uint16_t offset_v,
                                      uint16_t offset_w_external,
                                      uint16_t offset_w_internal,
                                      float *iu,
                                      float *iv,
                                      float *iw)
{
    float u = 0.0f;
    float v = 0.0f;
    float w = 0.0f;

    switch (pair) {
    case CURRENT_PAIR_UV:
        u = ((float)offset_u - (float)adc1_raw) * CURRENT_AMPS_PER_COUNT;
        v = ((float)offset_v - (float)adc2_raw) * CURRENT_AMPS_PER_COUNT;
        w = -(u + v);
        break;

    case CURRENT_PAIR_UW:
        u = ((float)offset_u - (float)adc1_raw) * CURRENT_AMPS_PER_COUNT;
        w = ((float)offset_w_internal - (float)adc2_raw) *
            CURRENT_AMPS_PER_COUNT;
        v = -(u + w);
        break;

    case CURRENT_PAIR_VW:
        w = ((float)offset_w_external - (float)adc1_raw) *
            CURRENT_AMPS_PER_COUNT;
        v = ((float)offset_v - (float)adc2_raw) * CURRENT_AMPS_PER_COUNT;
        u = -(v + w);
        break;

    default:
        break;
    }

    *iu = u;
    *iv = v;
    *iw = w;
}

/*
 * 使能一路 ADC 并将其注入组挂到外部触发上。
 * 简短的启动/停止序列让注入状态机在 TIM1 触发使能前回到空闲。
 * 队列模式采用 end-empty：上下文被消费后硬件自动清空 JSQR，
 * 下一次 TIM1 更新即可安全地装入一个新上下文。
 */
static uint8_t current_shunt_enable_adc(ADC_TypeDef *adc,
                                        current_shunt_fault_t ready_fault,
                                        current_shunt_fault_t stop_fault,
                                        current_shunt_fault_t flush_fault)
{
    uint32_t timeout = 100000UL;

    /*
     * 勘误 ES0431 ADC5-140924：ADCAL 清零后过快写入 ADEN 可能被忽略。
     * 按 ST 已验证的 MCSDK R3_2 驱动做法，在轮询 ADRDY 的同时重试 ADEN。
     */
    while ((LL_ADC_IsActiveFlag_ADRDY(adc) == 0U) && (timeout != 0U)) {
        LL_ADC_Enable(adc);
        --timeout;
    }
    if (timeout == 0U) {
        current_shunt_fail(ready_fault);
        return 0U;
    }

    LL_ADC_INJ_StartConversion(adc);
    LL_ADC_INJ_StopConversion(adc);
    timeout = 100000UL;
    while ((LL_ADC_INJ_IsStopConversionOngoing(adc) != 0U) && (timeout != 0U)) {
        --timeout;
    }
    if (timeout == 0U) {
        current_shunt_fail(stop_fault);
        return 0U;
    }

    LL_ADC_INJ_SetTriggerEdge(adc, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_StartConversion(adc);
    LL_ADC_INJ_SetQueueMode(adc, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);

    /* 修改 JQDIS 必须冲刷掉 CubeMX 初始的双序列 JSQR 上下文。 */
    if (adc->JSQR != 0U) {
        current_shunt_fail(flush_fault);
        return 0U;
    }

    /* ST MCSDK 针对 STM32G4 勘误使用的首次转换规避措施。 */
    LL_ADC_REG_SetSequencerLength(adc, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_StartConversion(adc);
    return 1U;
}

/*
 * 启动模拟链路并准备注入 ADC 中断通路。
 * 本函数不开启 PWM 主输出，也不采集零偏。
 */
uint8_t current_shunt_init(void)
{
    memset((void *)&g_current_shunt_diag, 0, sizeof(g_current_shunt_diag));
    pending_pair = CURRENT_PAIR_UV;
    active_pair = CURRENT_PAIR_UV;
    pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_RISING;
    contexts_armed = 0U;
    adc_deferred_consecutive = 0U;
    initialized = 0U;
    ready = 0U;
    suspended = 0U;
    rejected_consecutive = 0U;
    g_current_shunt_diag.state = (uint8_t)CURRENT_SHUNT_IDLE;
    g_current_shunt_diag.fault_code = (uint8_t)CURRENT_SHUNT_FAULT_NONE;
    g_current_shunt_diag.init_stage = (uint8_t)CURRENT_SHUNT_STAGE_RESET;
    g_current_shunt_diag.active_pair = (uint8_t)active_pair;
    g_current_shunt_diag.pending_pair = (uint8_t)pending_pair;

    /* ADC 幂等重建：恢复路径重进本函数时，上一次的注入转换可能仍处于
     * 启动态（JADSTART=1，queue END_EMPTY 等上下文时由 INJ_Start 置位）。
     * JADSTP 在该状态下清不掉 JADSTART，ADDIS 又要求无转换进行——
     * 三者互相死锁（2026-09-05 实测 CR 卡在 JADSTART|JADSTP）。
     * 唯一干净的出路是 RCC 外设复位：把 ADC12 全部寄存器打回上电状态，
     * 之后按冷启动流程重新建链。MX_ADCx_Init 不重跑（HAL 句柄状态已在
     * fail 路径之外保持一致，寄存器重配由本函数后续步骤完成）。 */
    MODIFY_REG(RCC->AHB2RSTR, 0U, RCC_AHB2RSTR_ADC12RST);
    SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_ADC12RST);
    CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_ADC12RST);
    /* 复位后 ADC 时钟仍使能（AHB2ENR 不动），等待时钟稳定 */
    HAL_Delay(1U);

    /* RCC 复位清掉了 GPIO 复用之外的全部 ADC 配置：SMPR/SQR/JSQR/CFGR
     * 都回零。本函数后续 enable_adc 会重建注入触发路径，但常规组的
     * 采样时间等需要 MX 层配置的项不在本函数职责内——当前固件的常规
     * 组只在 enable_adc 末尾用默认时序做一次"首转换规避"，不需要 SMPR。
     * （若未来常规组承担采样，必须在这里重跑 MX_ADC1/2_Init。） */
    HAL_ADC_DeInit(&hadc1);
    HAL_ADC_DeInit(&hadc2);
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_ADC1_CALIBRATION);
        return 0U;
    }
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_ADC2_CALIBRATION);
        return 0U;
    }

    /* OPAMP 幂等重启：恢复路径（cs 故障自动恢复）在 OPAMP 仍处于
     * BUSY 状态时重新进入本函数，HAL_OPAMP_Start 只接受 READY，
     * 不先 Stop 会永远失败（fault=OPAMP1_START 死锁，2026-09-05 定位）。
     * Stop 对 RESET/READY 返回 ERROR 但无副作用，对 BUSY 正常关闭。 */
    (void)HAL_OPAMP_Stop(&hopamp1);
    (void)HAL_OPAMP_Stop(&hopamp2);
    (void)HAL_OPAMP_Stop(&hopamp3);

    g_current_shunt_diag.init_stage = (uint8_t)CURRENT_SHUNT_STAGE_OPAMP1;
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_OPAMP1_START);
        return 0U;
    }
    g_current_shunt_diag.init_stage = (uint8_t)CURRENT_SHUNT_STAGE_OPAMP2;
    if (HAL_OPAMP_Start(&hopamp2) != HAL_OK) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_OPAMP2_START);
        return 0U;
    }
    g_current_shunt_diag.init_stage = (uint8_t)CURRENT_SHUNT_STAGE_OPAMP3;
    if (HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_OPAMP3_START);
        return 0U;
    }
    HAL_Delay(1U);

    g_current_shunt_diag.init_stage =
        (uint8_t)CURRENT_SHUNT_STAGE_ADC1_CALIBRATION;
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_ADC1_CALIBRATION);
        return 0U;
    }
    g_current_shunt_diag.init_stage =
        (uint8_t)CURRENT_SHUNT_STAGE_ADC2_CALIBRATION;
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_ADC2_CALIBRATION);
        return 0U;
    }

    g_current_shunt_diag.init_stage =
        (uint8_t)CURRENT_SHUNT_STAGE_ADC1_ENABLE;
    if (current_shunt_enable_adc(ADC1,
                                 CURRENT_SHUNT_FAULT_ADC1_READY_TIMEOUT,
                                 CURRENT_SHUNT_FAULT_ADC1_STOP_TIMEOUT,
                                 CURRENT_SHUNT_FAULT_ADC1_CONTEXT_FLUSH) == 0U) {
        return 0U;
    }
    g_current_shunt_diag.init_stage =
        (uint8_t)CURRENT_SHUNT_STAGE_ADC2_ENABLE;
    if (current_shunt_enable_adc(ADC2,
                                 CURRENT_SHUNT_FAULT_ADC2_READY_TIMEOUT,
                                 CURRENT_SHUNT_FAULT_ADC2_STOP_TIMEOUT,
                                 CURRENT_SHUNT_FAULT_ADC2_CONTEXT_FLUSH) == 0U) {
        return 0U;
    }

    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);
    LL_ADC_ClearFlag_JQOVF(ADC1);
    LL_ADC_ClearFlag_JQOVF(ADC2);

    /* ADC2 JEOS 是唯一的控制节拍中断源。 */
    LL_ADC_EnableIT_JEOS(ADC2);

    /* 上下文由 TIM1 更新中断装填，在此之前不产生触发。 */
    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_RESET);
    initialized = 1U;
    g_current_shunt_diag.init_stage =
        (uint8_t)CURRENT_SHUNT_STAGE_INITIALIZED;
    current_shunt_snapshot();
    return 1U;
}

/*
 * 分两段采集三相零电流偏置：
 *   1. UV: ADC1=U、ADC2=V，各 1024 个样本。
 *   2. VW: ADC1=W、ADC2=V，取 1024 个 W 样本。
 * 整个阻塞式启动流程期间 PWM 主输出保持关闭。
 */
uint8_t current_shunt_calibrate(uint32_t timeout_ms)
{
    uint32_t start_tick;

    if (initialized == 0U) {
        return 0U;
    }

    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    offset_sum_u = 0U;
    offset_sum_v = 0U;
    offset_sum_w = 0U;
    offset_sum_w_internal = 0U;
    offset_sample_count = 0U;
    pending_pair = CURRENT_PAIR_UV;
    pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_RISING;
    g_current_shunt_diag.state = (uint8_t)CURRENT_SHUNT_CAL_UV;
    g_current_shunt_diag.init_stage = (uint8_t)CURRENT_SHUNT_STAGE_CAL_UV;
    g_current_shunt_diag.sample_count = 0U;
    g_current_shunt_diag.pending_pair = (uint8_t)pending_pair;
    start_tick = HAL_GetTick();

    while ((g_current_shunt_diag.state == (uint8_t)CURRENT_SHUNT_CAL_UV) ||
           (g_current_shunt_diag.state == (uint8_t)CURRENT_SHUNT_CAL_W) ||
           (g_current_shunt_diag.state ==
            (uint8_t)CURRENT_SHUNT_CAL_W_INTERNAL)) {
        if ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms) {
            current_shunt_fail(CURRENT_SHUNT_FAULT_CALIBRATION_TIMEOUT);
            break;
        }
    }

    return ready;
}

/*
 * ADC 完成路径，在 ADC1_2_IRQHandler 清除 ADC2 JEOS 后调用。
 * 校验 ADC1 完成了同一次触发，取走两路 ADC 的 JDR1，
 * 然后累加零偏或发布重建后的相电流。
 */
uint8_t current_shunt_adc_irq(void)
{
    uint16_t adc1_raw;
    uint16_t adc2_raw;

    /* 本次结果消费期间不允许再来一个触发沿。 */
    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_RESET);

    /*
     * 主循环主动发起 Flash 操作暂停时，可能已有一个 JEOS 中断悬起。
     * 直接清掉它，不把被打断的上下文当作流水线故障；
     * suspend() 会冲刷并重新装填队列。
     */
    if (suspended != 0U) {
        LL_ADC_ClearFlag_JEOS(ADC1);
        LL_ADC_ClearFlag_JEOS(ADC2);
        contexts_armed = 0U;
        g_current_shunt_diag.contexts_armed = 0U;
        return 0U;
    }

    /* 保留首次故障时抓取的寄存器快照和计数器，不再覆盖。 */
    if (g_current_shunt_diag.fault_code !=
        (uint8_t)CURRENT_SHUNT_FAULT_NONE) {
        return 0U;
    }

    ++g_current_shunt_diag.adc_irq_count;
    current_shunt_snapshot();

    if ((initialized == 0U) || (contexts_armed == 0U)) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_CONTEXT_NOT_ARMED);
        return 0U;
    }

    if (LL_ADC_IsActiveFlag_JEOS(ADC1) == 0U) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_ADC1_JEOS_MISSING);
        return 0U;
    }
    LL_ADC_ClearFlag_JEOS(ADC1);

    if ((LL_ADC_IsActiveFlag_JQOVF(ADC1) != 0U) ||
        (LL_ADC_IsActiveFlag_JQOVF(ADC2) != 0U)) {
        LL_ADC_ClearFlag_JQOVF(ADC1);
        LL_ADC_ClearFlag_JQOVF(ADC2);
        current_shunt_fail(CURRENT_SHUNT_FAULT_QUEUE_OVERFLOW);
        return 0U;
    }

    adc1_raw = (uint16_t)ADC1->JDR1;
    adc2_raw = (uint16_t)ADC2->JDR1;
    g_current_shunt_diag.adc1_raw = adc1_raw;
    g_current_shunt_diag.adc2_raw = adc2_raw;
    contexts_armed = 0U;
    adc_deferred_consecutive = 0U;
    g_current_shunt_diag.contexts_armed = 0U;
    g_current_shunt_diag.adc_deferred_consecutive = 0U;

    if (g_current_shunt_diag.state == (uint8_t)CURRENT_SHUNT_CAL_UV) {
        if (active_pair != CURRENT_PAIR_UV) {
            current_shunt_fail(CURRENT_SHUNT_FAULT_CAL_UV_PAIR);
            return 0U;
        }

        offset_sum_u += adc1_raw;
        offset_sum_v += adc2_raw;
        ++offset_sample_count;
        g_current_shunt_diag.sample_count = offset_sample_count;
        if (offset_sample_count >= CURRENT_OFFSET_SAMPLES) {
            g_current_shunt_diag.offset_u =
                (uint16_t)(offset_sum_u / CURRENT_OFFSET_SAMPLES);
            g_current_shunt_diag.offset_v =
                (uint16_t)(offset_sum_v / CURRENT_OFFSET_SAMPLES);
            offset_sample_count = 0U;
            pending_pair = CURRENT_PAIR_VW;
            g_current_shunt_diag.state = (uint8_t)CURRENT_SHUNT_CAL_W;
            g_current_shunt_diag.init_stage =
                (uint8_t)CURRENT_SHUNT_STAGE_CAL_W;
            g_current_shunt_diag.sample_count = 0U;
            g_current_shunt_diag.pending_pair = (uint8_t)pending_pair;
        }
        return 0U;
    }

    if (g_current_shunt_diag.state == (uint8_t)CURRENT_SHUNT_CAL_W) {
        if (active_pair != CURRENT_PAIR_VW) {
            current_shunt_fail(CURRENT_SHUNT_FAULT_CAL_W_PAIR);
            return 0U;
        }

        offset_sum_w += adc1_raw;
        ++offset_sample_count;
        g_current_shunt_diag.sample_count = offset_sample_count;
        if (offset_sample_count >= CURRENT_OFFSET_SAMPLES) {
            g_current_shunt_diag.offset_w =
                (uint16_t)(offset_sum_w / CURRENT_OFFSET_SAMPLES);
            offset_sample_count = 0U;
            pending_pair = CURRENT_PAIR_UW;
            g_current_shunt_diag.state =
                (uint8_t)CURRENT_SHUNT_CAL_W_INTERNAL;
            g_current_shunt_diag.init_stage =
                (uint8_t)CURRENT_SHUNT_STAGE_CAL_W_INTERNAL;
            g_current_shunt_diag.sample_count = 0U;
            g_current_shunt_diag.pending_pair = (uint8_t)pending_pair;
        }
        return 0U;
    }

    if (g_current_shunt_diag.state ==
        (uint8_t)CURRENT_SHUNT_CAL_W_INTERNAL) {
        if (active_pair != CURRENT_PAIR_UW) {
            current_shunt_fail(CURRENT_SHUNT_FAULT_CAL_W_PAIR);
            return 0U;
        }

        offset_sum_w_internal += adc2_raw;
        ++offset_sample_count;
        g_current_shunt_diag.sample_count = offset_sample_count;
        if (offset_sample_count >= CURRENT_OFFSET_SAMPLES) {
            g_current_shunt_diag.offset_w_internal =
                (uint16_t)(offset_sum_w_internal / CURRENT_OFFSET_SAMPLES);

            if ((g_current_shunt_diag.offset_u < CURRENT_OFFSET_MIN_COUNT) ||
                (g_current_shunt_diag.offset_u > CURRENT_OFFSET_MAX_COUNT) ||
                (g_current_shunt_diag.offset_v < CURRENT_OFFSET_MIN_COUNT) ||
                (g_current_shunt_diag.offset_v > CURRENT_OFFSET_MAX_COUNT) ||
                (g_current_shunt_diag.offset_w < CURRENT_OFFSET_MIN_COUNT) ||
                (g_current_shunt_diag.offset_w > CURRENT_OFFSET_MAX_COUNT) ||
                (g_current_shunt_diag.offset_w_internal <
                 CURRENT_OFFSET_MIN_COUNT) ||
                (g_current_shunt_diag.offset_w_internal >
                 CURRENT_OFFSET_MAX_COUNT)) {
                current_shunt_fail(CURRENT_SHUNT_FAULT_OFFSET_RANGE);
            } else {
                ready = 1U;
                g_current_shunt_diag.state = (uint8_t)CURRENT_SHUNT_READY;
                g_current_shunt_diag.init_stage =
                    (uint8_t)CURRENT_SHUNT_STAGE_RUNNING;
            }
        }
        return 0U;
    }

    if (ready != 0U) {
        float iu;
        float iv;
        float iw;
        float du;
        float dv;
        float dw;
        float max_step;
        static float last_u = 0.0f;
        static float last_v = 0.0f;
        static float last_w = 0.0f;
        static current_sample_pair_t last_active_pair = CURRENT_PAIR_UV;
        uint8_t pair_just_switched = (active_pair != last_active_pair);
        last_active_pair = active_pair;

        current_shunt_reconstruct(active_pair,
                                  adc1_raw,
                                  adc2_raw,
                                  g_current_shunt_diag.offset_u,
                                  g_current_shunt_diag.offset_v,
                                  g_current_shunt_diag.offset_w,
                                  g_current_shunt_diag.offset_w_internal,
                                  &iu, &iv, &iw);

        if (pair_just_switched != 0U) {
            /* 采样对切换拍：硬件 OPAMP/ADC 路由建立，更新基准并重置连续计数 */
            last_u = iu;
            last_v = iv;
            last_w = iw;
            rejected_consecutive = 0U;
            g_current_shunt_diag.rejected_consecutive = 0U;
        } else {
            du = iu - last_u;
            dv = iv - last_v;
            dw = iw - last_w;
            last_u = iu;
            last_v = iv;
            last_w = iw;

            if (du < 0.0f) {
                du = -du;
            }
            if (dv < 0.0f) {
                dv = -dv;
            }
            if (dw < 0.0f) {
                dw = -dw;
            }
            max_step = du;
            if (dv > max_step) {
                max_step = dv;
            }
            if (dw > max_step) {
                max_step = dw;
            }

            if (max_step > CURRENT_MAX_STEP_A) {
                /* 预期暂态宽限：放宽判据放行，不拒绝不累计。
                 * 任一拍回到正常判据内则由下方代码结束宽限。 */
                float step_limit = transient_step_limit_a;
                if ((step_limit > CURRENT_MAX_STEP_A) &&
                    (max_step <= step_limit)) {
                    return 1U;
                }

                ++g_current_shunt_diag.rejected_sample_count;
                if (rejected_consecutive == 0U) {
                    g_current_shunt_diag.rejected_current_u = iu;
                    g_current_shunt_diag.rejected_current_v = iv;
                    g_current_shunt_diag.rejected_current_w = iw;
                    g_current_shunt_diag.rejected_adc1_raw = adc1_raw;
                    g_current_shunt_diag.rejected_adc2_raw = adc2_raw;
                    g_current_shunt_diag.rejected_ccr1 =
                        (uint16_t)htim1.Instance->CCR1;
                    g_current_shunt_diag.rejected_ccr2 =
                        (uint16_t)htim1.Instance->CCR2;
                    g_current_shunt_diag.rejected_ccr3 =
                        (uint16_t)htim1.Instance->CCR3;
                    g_current_shunt_diag.rejected_ccr4 =
                        (uint16_t)htim1.Instance->CCR4;
                    g_current_shunt_diag.rejected_pair =
                        (uint8_t)active_pair;
                }
                if (rejected_consecutive < 0xFFU) {
                    ++rejected_consecutive;
                }
                g_current_shunt_diag.rejected_consecutive =
                    rejected_consecutive;

                if (rejected_consecutive >=
                    CURRENT_MAX_REJECTED_CONSECUTIVE) {
                    current_shunt_fail(
                        CURRENT_SHUNT_FAULT_CURRENT_DISCONTINUITY);
                    return 0U;
                }
                return 1U;
            }

            rejected_consecutive = 0U;
            g_current_shunt_diag.rejected_consecutive = 0U;
            /* 正常样本到达：结束预期暂态宽限 */
            transient_step_limit_a = 0.0f;
        }

        /* 将通过连续性检查的有效三相电流发布给控制环 */
        g_current_shunt_diag.current_u = iu;
        g_current_shunt_diag.current_v = iv;
        g_current_shunt_diag.current_w = iw;
        return 1U;
    }

    return 0U;
}

/*
 * TIM1 更新路径。在下一个 CH4 边沿之前为两路 ADC 各装入一个注入上下文。
 * 修改 JSQR 和 OPAMP3 路由期间保持 TRGO 关闭。
 */
void current_shunt_tim_update_irq(void)
{
    uint32_t adc1_channel;
    uint32_t adc2_channel;

    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_RESET);

    /* Flash 擦除/编程期间主动暂停本流水线。 */
    if (suspended != 0U) {
        return;
    }

    /* 保留 current_shunt_fail() 抓取的定时器/ADC 原始状态。 */
    if (g_current_shunt_diag.fault_code !=
        (uint8_t)CURRENT_SHUNT_FAULT_NONE) {
        return;
    }

    ++g_current_shunt_diag.tim_update_count;
    current_shunt_snapshot();

    if (initialized == 0U) {
        return;
    }

    if ((ready == 0U) &&
        (g_current_shunt_diag.state != (uint8_t)CURRENT_SHUNT_CAL_UV) &&
        (g_current_shunt_diag.state != (uint8_t)CURRENT_SHUNT_CAL_W) &&
        (g_current_shunt_diag.state !=
         (uint8_t)CURRENT_SHUNT_CAL_W_INTERNAL)) {
        return;
    }

    /*
     * 排队的转换启动后硬件会清空 JSQR。若 ADC 中断尚未取走 JDR1，
     * 则保持 active_pair 不变并跳过这一次上下文提交；
     * CCR4 紧邻定时器更新点时可能出现这种情况。
     * 容忍一次延迟提交；连续两次更新都拿不到 ADC 结果说明快环已停转。
     */
    if (contexts_armed != 0U) {
        if ((ADC1->JSQR != 0U) || (ADC2->JSQR != 0U)) {
            current_shunt_fail(CURRENT_SHUNT_FAULT_CONTEXT_NOT_CONSUMED);
            return;
        }

        ++g_current_shunt_diag.adc_deferred_count;
        ++adc_deferred_consecutive;
        current_shunt_snapshot();
        if (adc_deferred_consecutive >= 2U) {
            current_shunt_fail(CURRENT_SHUNT_FAULT_ADC_RESULT_TIMEOUT);
        }
        return;
    }

    /* 此时没有任何软件属主，非空的 JSQR 是真正的残留上下文。 */
    if ((ADC1->JSQR != 0U) || (ADC2->JSQR != 0U)) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_CONTEXT_NOT_CONSUMED);
        return;
    }

    switch (pending_pair) {
    case CURRENT_PAIR_UV:
        /* ADC1_IN3 = U（OPAMP1 外部输出），ADC2_IN3 = V。 */
        adc1_channel = 3U;
        adc2_channel = 3U;
        LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
        break;

    case CURRENT_PAIR_UW:
        /* ADC1_IN3 = U，ADC2_CH18 = OPAMP3 内部 W 输出。 */
        adc1_channel = 3U;
        adc2_channel = 18U;
        LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
        break;

    case CURRENT_PAIR_VW:
        /* ADC1_IN12 = W（OPAMP3 外部输出），ADC2_IN3 = V。 */
        adc1_channel = 12U;
        adc2_channel = 3U;
        LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
        break;

    default:
        current_shunt_fail(CURRENT_SHUNT_FAULT_INVALID_PAIR);
        return;
    }

    ADC1->JSQR = current_shunt_jsqr(adc1_channel, pending_trigger_edge);
    ADC2->JSQR = current_shunt_jsqr(adc2_channel, pending_trigger_edge);
    active_pair = pending_pair;
    contexts_armed = 1U;
    g_current_shunt_diag.active_pair = (uint8_t)active_pair;
    g_current_shunt_diag.pending_pair = (uint8_t)pending_pair;
    g_current_shunt_diag.contexts_armed = 1U;

    /* 下降沿选择只生效一个周期，用后即恢复上升沿。 */
    pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_RISING;
    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_OC4REF);
}

/*
 * 根据下一组 PWM 比较值规划路由和采样点。
 * CCR 最大的相低边窗口最短，因此该相由另外两相重建。
 * sector 仅为 FOC 接口保留；实际路由按 CCR 排序决定，
 * 避免扇区编号约定带来的歧义。
 */
void current_shunt_prepare_pwm(uint32_t ccr_u,
                               uint32_t ccr_v,
                               uint32_t ccr_w,
                               uint8_t sector)
{
    uint32_t max_duty;
    uint32_t mid_duty;
    uint32_t sampling_point;

    g_current_shunt_diag.sector = sector;

    if (ready == 0U) {
        return;
    }

    /* 找出最大和次大的 CCR（即下桥开通最短与次短的相） */
    max_duty = ccr_u;
    mid_duty = ccr_v;
    if (ccr_v > max_duty) {
        mid_duty = max_duty;
        max_duty = ccr_v;
    }
    if (ccr_w > max_duty) {
        mid_duty = max_duty;
        max_duty = ccr_w;
    } else if (ccr_w > mid_duty) {
        mid_duty = ccr_w;
    }

    if ((max_duty > PWM_CNT) || (mid_duty > PWM_CNT)) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_INVALID_WINDOW);
        return;
    }

    /*
     * 对齐 ST MCSDK R3_2 核心策略：
     * 1. 只要在周期中点（公共零矢量 V0）的低边时间足够 (> TAFTER)，
     *    永远固定采样 UV 相（CURRENT_PAIR_UV），OPAMP3 始终保持外部直通，
     *    彻底杜绝因每个 PWM 周期剧烈切换 OPAMP 模式产生的模拟前端充放电假尖峰！
     * 2. 只有当调制比极高、公共窗口不足时，才根据下桥时间最短的相动态切换采样对与采样点。
     */
    pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_RISING;
    if ((PWM_CNT - max_duty) > CURRENT_SAMPLE_TAFTER) {
        pending_pair = CURRENT_PAIR_UV;
        sampling_point = PWM_CNT - 1U;
    } else {
        if ((ccr_u >= ccr_v) && (ccr_u >= ccr_w)) {
            pending_pair = CURRENT_PAIR_VW;
        } else if ((ccr_v >= ccr_u) && (ccr_v >= ccr_w)) {
            pending_pair = CURRENT_PAIR_UW;
        } else {
            pending_pair = CURRENT_PAIR_UV;
        }

        if ((max_duty - mid_duty) > ((PWM_CNT - max_duty) * 2U)) {
            if (max_duty <= CURRENT_SAMPLE_TBEFORE) {
                current_shunt_fail(CURRENT_SHUNT_FAULT_INVALID_WINDOW);
                return;
            }
            sampling_point = max_duty - CURRENT_SAMPLE_TBEFORE;
        } else {
            sampling_point = max_duty + CURRENT_SAMPLE_TAFTER;
            if (sampling_point >= PWM_CNT) {
                /* 镜像到向下计数半周，改用 OC4REF 下降沿触发。 */
                pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_FALLING;
                sampling_point = (2U * PWM_CNT) - sampling_point - 1U;
            }
        }
    }
    g_current_shunt_diag.pending_pair = (uint8_t)pending_pair;

    if ((sampling_point == 0U) || (sampling_point >= PWM_CNT)) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_INVALID_WINDOW);
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sampling_point);
}

/* 拷贝最新发布的电流，不暴露内部采样组合状态。 */
void current_shunt_get_currents(float *iu, float *iv, float *iw)
{
    if (iu != 0) {
        *iu = g_current_shunt_diag.current_u;
    }
    if (iv != 0) {
        *iv = g_current_shunt_diag.current_v;
    }
    if (iw != 0) {
        *iw = g_current_shunt_diag.current_w;
    }
}

uint8_t current_shunt_is_ready(void)
{
    return ready;
}

void current_shunt_allow_transient(float grace_a)
{
    /* 单向放宽：只升不降，避免主循环旧值覆盖中断里刚结束的宽限。
     * 立即结束用 0；float 单写原子，中断侧读取无撕裂。 */
    if (grace_a > transient_step_limit_a) {
        transient_step_limit_a = grace_a;
    }
}

/*
 * Flash 页擦除/编程会阻塞同一 Flash bank 的取指约 20 ms。
 * 不显式暂停的话，TIM1 仍在运行而其中断无法执行，
 * 注入上下文看门狗会误报 ADC 超时。
 * 进入该区间前先停止并冲刷注入组。
 */
uint8_t current_shunt_suspend(void)
{
    uint32_t primask;
    uint32_t timeout;

    if ((initialized == 0U) || (ready == 0U) ||
        (g_current_shunt_diag.fault_code !=
         (uint8_t)CURRENT_SHUNT_FAULT_NONE)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    suspended = 1U;
    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_RESET);
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    LL_ADC_DisableIT_JEOS(ADC2);
    if (primask == 0U) {
        __enable_irq();
    }

    if (LL_ADC_INJ_IsConversionOngoing(ADC1) != 0U) {
        LL_ADC_INJ_StopConversion(ADC1);
    }
    if (LL_ADC_INJ_IsConversionOngoing(ADC2) != 0U) {
        LL_ADC_INJ_StopConversion(ADC2);
    }

    timeout = 100000UL;
    while (((LL_ADC_INJ_IsConversionOngoing(ADC1) != 0U) ||
            (LL_ADC_INJ_IsStopConversionOngoing(ADC1) != 0U) ||
            (LL_ADC_REG_IsConversionOngoing(ADC1) != 0U)) &&
           (timeout != 0U)) {
        --timeout;
    }
    if (timeout == 0U) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_ADC1_STOP_TIMEOUT);
        return 0U;
    }

    timeout = 100000UL;
    while (((LL_ADC_INJ_IsConversionOngoing(ADC2) != 0U) ||
            (LL_ADC_INJ_IsStopConversionOngoing(ADC2) != 0U) ||
            (LL_ADC_REG_IsConversionOngoing(ADC2) != 0U)) &&
           (timeout != 0U)) {
        --timeout;
    }
    if (timeout == 0U) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_ADC2_STOP_TIMEOUT);
        return 0U;
    }

    /* 修改 JQDIS 会冲刷排队的上下文并清空 JSQR。 */
    LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);
    LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);
    LL_ADC_ClearFlag_JQOVF(ADC1);
    LL_ADC_ClearFlag_JQOVF(ADC2);
    contexts_armed = 0U;
    adc_deferred_consecutive = 0U;
    g_current_shunt_diag.contexts_armed = 0U;
    g_current_shunt_diag.adc_deferred_consecutive = 0U;
    return 1U;
}

uint8_t current_shunt_resume(void)
{
    uint32_t primask;

    if (suspended == 0U) {
        return ready;
    }
    if ((initialized == 0U) || (ready == 0U) ||
        (g_current_shunt_diag.fault_code !=
         (uint8_t)CURRENT_SHUNT_FAULT_NONE)) {
        return 0U;
    }

    /*
     * 重新挂回初始化时使用的 end-empty 注入队列。
     * 在下一次 TIM1 更新装入新 JSQR 之前保持 TRGO 关闭。
     */
    LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_StartConversion(ADC1);
    LL_ADC_INJ_StartConversion(ADC2);
    LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);
    LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);

    if ((ADC1->JSQR != 0U) || (ADC2->JSQR != 0U)) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_CONTEXT_NOT_CONSUMED);
        return 0U;
    }

    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);
    LL_ADC_ClearFlag_JQOVF(ADC1);
    LL_ADC_ClearFlag_JQOVF(ADC2);
    contexts_armed = 0U;
    adc_deferred_consecutive = 0U;
    g_current_shunt_diag.contexts_armed = 0U;
    g_current_shunt_diag.adc_deferred_consecutive = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    suspended = 0U;
    LL_ADC_EnableIT_JEOS(ADC2);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}
