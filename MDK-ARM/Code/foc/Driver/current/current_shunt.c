#include "current_shunt.h"

#include "main.h"
#include "foc_config.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_opamp.h"
#include <string.h>

/* 本文件内沿用 PWM_CNT 简写指代定时器 ARR */
#define PWM_CNT FOC_PWM_ARR

/*
 * STM32G431 R3_2 low-side current feedback.
 *
 * TIM1 CH4 generates OC4REF. TIM1 routes OC4REF to TRGO and both ADC injected
 * groups sample from the same trigger edge. The driver configures one injected
 * rank in ADC1 and one injected rank in ADC2 for every PWM period, then uses
 * Iu + Iv + Iw = 0 to reconstruct the third phase current.
 *
 * The module owns the timing-sensitive ADC context update. The FOC code only
 * supplies PWM compare values through current_shunt_prepare_pwm().
 */

/* ADC-code-to-current conversion constants.
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

/* Zero-current calibration policy. */
#define CURRENT_OFFSET_SAMPLES   1024UL
#define CURRENT_OFFSET_MIN_COUNT 2048U
#define CURRENT_OFFSET_MAX_COUNT 3072U

/*
 * Safe distances from a PWM switching edge, expressed in TIM1 counter ticks.
 * They cover dead time, switching noise, OPAMP settling, ADC sample time, and
 * the design margin used by the sampling-point algorithm.
 */
#define CURRENT_SAMPLE_TBEFORE 41U
#define CURRENT_SAMPLE_TAFTER  298U

/* Two phases converted in the same PWM period. */
typedef enum {
    CURRENT_PAIR_UV = 0,
    CURRENT_PAIR_UW = 1,
    CURRENT_PAIR_VW = 2
} current_sample_pair_t;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;
extern TIM_HandleTypeDef htim1;

/* Exported diagnostic values; all fields are written from interrupt context. */
volatile current_shunt_diag_t g_current_shunt_diag;

/*
 * pending_pair is calculated with the next PWM compare values. active_pair is
 * latched with the JSQR context that produced the ADC results being read.
 */
static volatile current_sample_pair_t pending_pair = CURRENT_PAIR_UV;
static volatile current_sample_pair_t active_pair = CURRENT_PAIR_UV;
static volatile uint32_t pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_RISING;

/* Context ownership and module lifecycle flags. */
static volatile uint8_t contexts_armed = 0U;
static volatile uint8_t initialized = 0U;
static volatile uint8_t ready = 0U;

/* Offset accumulators used only while PWM main outputs are disabled. */
static uint32_t offset_sum_u;
static uint32_t offset_sum_v;
static uint32_t offset_sum_w;
static uint32_t offset_sample_count;

/* Capture registers that explain why a sampling failure was raised. */
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
}

/*
 * Fail closed: stop future TRGO edges and disable TIM1 main outputs. The
 * first detailed reason is latched for Keil Watch.  The application observes
 * ready == 0 and transitions its FOC state to FAULT.
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
 * Build a single-rank injected JSQR context.
 * Leaving JL clear selects one conversion. JEXTSEL/JEXTEN connect it to the
 * selected edge of TIM1_TRGO; the caller must write this only while TRGO is off.
 */
static uint32_t current_shunt_jsqr(uint32_t channel, uint32_t edge)
{
    return ((channel << ADC_JSQR_JSQ1_Pos) & ADC_JSQR_JSQ1) |
           (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ADC_JSQR_JEXTSEL) |
           (edge & ADC_JSQR_JEXTEN);
}

/*
 * Convert two raw ADC codes and reconstruct the third phase with KCL.
 * ADC routing is fixed for each pair:
 *   UV: ADC1=U, ADC2=V
 *   UW: ADC1=U, ADC2=W through OPAMP3 internal output
 *   VW: ADC1=W, ADC2=V
 */
static void current_shunt_reconstruct(current_sample_pair_t pair,
                                      uint16_t adc1_raw,
                                      uint16_t adc2_raw,
                                      uint16_t offset_u,
                                      uint16_t offset_v,
                                      uint16_t offset_w,
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
        w = ((float)offset_w - (float)adc2_raw) * CURRENT_AMPS_PER_COUNT;
        v = -(u + w);
        break;

    case CURRENT_PAIR_VW:
        w = ((float)offset_w - (float)adc1_raw) * CURRENT_AMPS_PER_COUNT;
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
 * Enable one ADC and arm its injected group for external triggering.
 * The short start/stop sequence leaves the injected state machine idle before
 * the TIM1 trigger is enabled. Queue mode is end-empty so a consumed context
 * clears JSQR and the next TIM1 update can safely install one new context.
 */
static uint8_t current_shunt_enable_adc(ADC_TypeDef *adc,
                                        current_shunt_fault_t ready_fault,
                                        current_shunt_fault_t stop_fault,
                                        current_shunt_fault_t flush_fault)
{
    uint32_t timeout = 100000UL;

    /*
     * ES0431 ADC5-140924: ADEN can be ignored when it is written too soon
     * after ADCAL clears.  Retry ADEN while polling ADRDY, as the validated
     * MCSDK R3_2 driver does for STM32G4.
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

    /* Changing JQDIS must flush CubeMX's initial two-rank JSQR context. */
    if (adc->JSQR != 0U) {
        current_shunt_fail(flush_fault);
        return 0U;
    }

    /* First-conversion workaround used by ST MCSDK for STM32G4 errata. */
    LL_ADC_REG_SetSequencerLength(adc, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_StartConversion(adc);
    return 1U;
}

/*
 * Start the analog chain and prepare the injected ADC interrupt path. This
 * function does not start PWM main outputs and does not collect offsets.
 */
uint8_t current_shunt_init(void)
{
    memset((void *)&g_current_shunt_diag, 0, sizeof(g_current_shunt_diag));
    pending_pair = CURRENT_PAIR_UV;
    active_pair = CURRENT_PAIR_UV;
    pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_RISING;
    contexts_armed = 0U;
    initialized = 0U;
    ready = 0U;
    g_current_shunt_diag.state = (uint8_t)CURRENT_SHUNT_IDLE;
    g_current_shunt_diag.fault_code = (uint8_t)CURRENT_SHUNT_FAULT_NONE;
    g_current_shunt_diag.init_stage = (uint8_t)CURRENT_SHUNT_STAGE_RESET;
    g_current_shunt_diag.active_pair = (uint8_t)active_pair;
    g_current_shunt_diag.pending_pair = (uint8_t)pending_pair;

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

    /* ADC2 JEOS is the single control-rate interrupt source. */
    LL_ADC_EnableIT_JEOS(ADC2);

    /* Contexts are armed by the TIM1 update IRQ; no trigger before then. */
    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_RESET);
    initialized = 1U;
    g_current_shunt_diag.init_stage =
        (uint8_t)CURRENT_SHUNT_STAGE_INITIALIZED;
    current_shunt_snapshot();
    return 1U;
}

/*
 * Collect three zero-current offsets in two stages:
 *   1. UV: ADC1=U and ADC2=V, 1024 samples each.
 *   2. VW: ADC1=W and ADC2=V, 1024 W samples.
 * PWM main outputs remain disabled throughout this blocking startup routine.
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
    offset_sample_count = 0U;
    pending_pair = CURRENT_PAIR_UV;
    pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_RISING;
    g_current_shunt_diag.state = (uint8_t)CURRENT_SHUNT_CAL_UV;
    g_current_shunt_diag.init_stage = (uint8_t)CURRENT_SHUNT_STAGE_CAL_UV;
    g_current_shunt_diag.sample_count = 0U;
    g_current_shunt_diag.pending_pair = (uint8_t)pending_pair;
    start_tick = HAL_GetTick();

    while ((g_current_shunt_diag.state == (uint8_t)CURRENT_SHUNT_CAL_UV) ||
           (g_current_shunt_diag.state == (uint8_t)CURRENT_SHUNT_CAL_W)) {
        if ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms) {
            current_shunt_fail(CURRENT_SHUNT_FAULT_CALIBRATION_TIMEOUT);
            break;
        }
    }

    return ready;
}

/*
 * ADC completion path, called after ADC2 JEOS was cleared by ADC1_2_IRQHandler.
 * Verify ADC1 completed the same trigger, consume JDR1 from both ADCs, then
 * either accumulate offsets or publish reconstructed phase currents.
 */
uint8_t current_shunt_adc_irq(void)
{
    uint16_t adc1_raw;
    uint16_t adc2_raw;

    /* Do not allow another edge while this result is being consumed. */
    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_RESET);

    /* Preserve the register snapshot and counters captured at first fault. */
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
    g_current_shunt_diag.contexts_armed = 0U;

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

            if ((g_current_shunt_diag.offset_u < CURRENT_OFFSET_MIN_COUNT) ||
                (g_current_shunt_diag.offset_u > CURRENT_OFFSET_MAX_COUNT) ||
                (g_current_shunt_diag.offset_v < CURRENT_OFFSET_MIN_COUNT) ||
                (g_current_shunt_diag.offset_v > CURRENT_OFFSET_MAX_COUNT) ||
                (g_current_shunt_diag.offset_w < CURRENT_OFFSET_MIN_COUNT) ||
                (g_current_shunt_diag.offset_w > CURRENT_OFFSET_MAX_COUNT)) {
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

        current_shunt_reconstruct(active_pair,
                                  adc1_raw,
                                  adc2_raw,
                                  g_current_shunt_diag.offset_u,
                                  g_current_shunt_diag.offset_v,
                                  g_current_shunt_diag.offset_w,
                                  &iu, &iv, &iw);

        g_current_shunt_diag.current_u = iu;
        g_current_shunt_diag.current_v = iv;
        g_current_shunt_diag.current_w = iw;
        return 1U;
    }

    return 0U;
}

/*
 * TIM1 update path. Install one injected context for each ADC before the next
 * CH4 edge. TRGO is disabled while JSQR and OPAMP3 routing are modified.
 */
void current_shunt_tim_update_irq(void)
{
    uint32_t adc1_channel;
    uint32_t adc2_channel;

    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_RESET);

    /* Preserve the exact timer/ADC state captured by current_shunt_fail(). */
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
        (g_current_shunt_diag.state != (uint8_t)CURRENT_SHUNT_CAL_W)) {
        return;
    }

    /*
     * JSQR is cleared by hardware when the queued conversion starts.  If the
     * ADC IRQ has not yet consumed JDR1, keep active_pair unchanged and skip
     * this one context submission.  This can occur when CCR4 is immediately
     * before a timer update; it is not a hardware queue failure.
     */
    if (contexts_armed != 0U) {
        if ((ADC1->JSQR != 0U) || (ADC2->JSQR != 0U)) {
            current_shunt_fail(CURRENT_SHUNT_FAULT_CONTEXT_NOT_CONSUMED);
            return;
        }

        ++g_current_shunt_diag.adc_deferred_count;
        current_shunt_snapshot();
        return;
    }

    /* No software owner remains, so a non-empty JSQR is genuinely stale. */
    if ((ADC1->JSQR != 0U) || (ADC2->JSQR != 0U)) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_CONTEXT_NOT_CONSUMED);
        return;
    }

    switch (pending_pair) {
    case CURRENT_PAIR_UV:
        /* ADC1_IN3 = U (OPAMP1 external output), ADC2_IN3 = V. */
        adc1_channel = 3U;
        adc2_channel = 3U;
        LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
        break;

    case CURRENT_PAIR_UW:
        /* ADC1_IN3 = U, ADC2_CH18 = OPAMP3 internal W output. */
        adc1_channel = 3U;
        adc2_channel = 18U;
        LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
        break;

    case CURRENT_PAIR_VW:
        /* ADC1_IN12 = W (OPAMP3 external output), ADC2_IN3 = V. */
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

    /* Falling-edge selection is only a one-cycle request. */
    pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_RISING;
    MODIFY_REG(htim1.Instance->CR2, TIM_CR2_MMS, TIM_TRGO_OC4REF);
}

/*
 * Plan routing and sampling point from the next PWM compare values.
 * The phase with the largest CCR has the shortest low-side window, so it is
 * reconstructed from the other two phases. The sector is retained only for
 * the FOC API; actual routing follows CCR ordering to avoid sector convention
 * ambiguities.
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

    if ((ccr_u == ccr_v) && (ccr_v == ccr_w)) {
        pending_pair = CURRENT_PAIR_UV;
    } else if ((ccr_u >= ccr_v) && (ccr_u >= ccr_w)) {
        pending_pair = CURRENT_PAIR_VW;
    } else if ((ccr_v >= ccr_u) && (ccr_v >= ccr_w)) {
        pending_pair = CURRENT_PAIR_UW;
    } else {
        pending_pair = CURRENT_PAIR_UV;
    }
    g_current_shunt_diag.pending_pair = (uint8_t)pending_pair;

    /* Find the largest and second-largest CCR without sorting all phases. */
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
     * Normal case: trigger near CNT=ARR, the V0 common low-side window.
     * Narrow windows use the same R3_2 placement policy as ST MCSDK.
     */
    pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_RISING;
    if ((PWM_CNT - max_duty) > CURRENT_SAMPLE_TAFTER) {
        sampling_point = PWM_CNT - 1U;
    } else if ((max_duty - mid_duty) > ((PWM_CNT - max_duty) * 2U)) {
        if (max_duty <= CURRENT_SAMPLE_TBEFORE) {
            current_shunt_fail(CURRENT_SHUNT_FAULT_INVALID_WINDOW);
            return;
        }
        sampling_point = max_duty - CURRENT_SAMPLE_TBEFORE;
    } else {
        sampling_point = max_duty + CURRENT_SAMPLE_TAFTER;
        if (sampling_point >= PWM_CNT) {
            /* Mirror onto the down-count half and trigger on OC4REF falling. */
            pending_trigger_edge = LL_ADC_INJ_TRIG_EXT_FALLING;
            sampling_point = (2U * PWM_CNT) - sampling_point - 1U;
        }
    }

    if ((sampling_point == 0U) || (sampling_point >= PWM_CNT)) {
        current_shunt_fail(CURRENT_SHUNT_FAULT_INVALID_WINDOW);
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sampling_point);
}

/* Copy the latest published currents without exposing the internal pair state. */
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
