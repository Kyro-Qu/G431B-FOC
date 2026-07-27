#ifndef CURRENT_SHUNT_H
#define CURRENT_SHUNT_H

#include <stdint.h>

/*
 * Three-low-side-shunt current-sensing state.
 * Calibration is split because only two ADCs convert in each PWM cycle.
 */
typedef enum {
    CURRENT_SHUNT_IDLE = 0,
    CURRENT_SHUNT_CAL_UV,
    CURRENT_SHUNT_CAL_W,
    CURRENT_SHUNT_READY
} current_shunt_state_t;

/*
 * Latched current-sensing failure reason.  The value remains available in
 * g_current_shunt_diag.fault_code after the driver returns to IDLE.
 */
typedef enum {
    CURRENT_SHUNT_FAULT_NONE = 0,
    CURRENT_SHUNT_FAULT_OPAMP1_START,
    CURRENT_SHUNT_FAULT_OPAMP2_START,
    CURRENT_SHUNT_FAULT_OPAMP3_START,
    CURRENT_SHUNT_FAULT_ADC1_CALIBRATION,
    CURRENT_SHUNT_FAULT_ADC2_CALIBRATION,
    CURRENT_SHUNT_FAULT_ADC1_READY_TIMEOUT,
    CURRENT_SHUNT_FAULT_ADC2_READY_TIMEOUT,
    CURRENT_SHUNT_FAULT_ADC1_STOP_TIMEOUT,
    CURRENT_SHUNT_FAULT_ADC2_STOP_TIMEOUT,
    CURRENT_SHUNT_FAULT_ADC1_CONTEXT_FLUSH,
    CURRENT_SHUNT_FAULT_ADC2_CONTEXT_FLUSH,
    CURRENT_SHUNT_FAULT_CALIBRATION_TIMEOUT,
    CURRENT_SHUNT_FAULT_CONTEXT_NOT_ARMED,
    CURRENT_SHUNT_FAULT_ADC1_JEOS_MISSING,
    CURRENT_SHUNT_FAULT_QUEUE_OVERFLOW,
    CURRENT_SHUNT_FAULT_CAL_UV_PAIR,
    CURRENT_SHUNT_FAULT_CAL_W_PAIR,
    CURRENT_SHUNT_FAULT_CONTEXT_NOT_CONSUMED,
    CURRENT_SHUNT_FAULT_INVALID_PAIR,
    CURRENT_SHUNT_FAULT_OFFSET_RANGE,
    CURRENT_SHUNT_FAULT_INVALID_WINDOW,
    /* ADC context started, but ADC2 JEOS was absent for two TIM1 updates. */
    CURRENT_SHUNT_FAULT_ADC_RESULT_TIMEOUT
} current_shunt_fault_t;

/* Last initialization/calibration stage reached by the driver. */
typedef enum {
    CURRENT_SHUNT_STAGE_RESET = 0,
    CURRENT_SHUNT_STAGE_OPAMP1,
    CURRENT_SHUNT_STAGE_OPAMP2,
    CURRENT_SHUNT_STAGE_OPAMP3,
    CURRENT_SHUNT_STAGE_ADC1_CALIBRATION,
    CURRENT_SHUNT_STAGE_ADC2_CALIBRATION,
    CURRENT_SHUNT_STAGE_ADC1_ENABLE,
    CURRENT_SHUNT_STAGE_ADC2_ENABLE,
    CURRENT_SHUNT_STAGE_INITIALIZED,
    CURRENT_SHUNT_STAGE_CAL_UV,
    CURRENT_SHUNT_STAGE_CAL_W,
    CURRENT_SHUNT_STAGE_RUNNING
} current_shunt_stage_t;

/*
 * Read-only diagnostic snapshot for the application and debugger.
 * Offset fields are 12-bit ADC counts. Current fields are amperes.
 * Values are updated from interrupt context.
 */
typedef struct {
    volatile uint16_t offset_u;
    volatile uint16_t offset_v;
    volatile uint16_t offset_w;
    volatile uint16_t adc1_raw;
    volatile uint16_t adc2_raw;
    volatile float current_u;
    volatile float current_v;
    volatile float current_w;
    volatile uint32_t sample_count;
    volatile uint32_t tim_update_count;
    volatile uint32_t adc_irq_count;
    volatile uint32_t adc_deferred_count; /* Lifetime tolerated/failed delays. */
    volatile uint8_t adc_deferred_consecutive; /* 0/1 normal, 2 trips fault 22. */
    volatile uint32_t adc1_isr;
    volatile uint32_t adc2_isr;
    volatile uint32_t adc1_jsqr;
    volatile uint32_t adc2_jsqr;
    volatile uint32_t adc1_cr;
    volatile uint32_t adc2_cr;
    volatile uint32_t tim_cnt;
    volatile uint32_t tim_cr1;
    volatile uint8_t state;
    volatile uint8_t fault_code;
    volatile uint8_t init_stage;
    volatile uint8_t active_pair;
    volatile uint8_t pending_pair;
    volatile uint8_t contexts_armed;
    volatile uint8_t sector;
} current_shunt_diag_t;

extern volatile current_shunt_diag_t g_current_shunt_diag;

/* Start OPAMPs, calibrate/enable ADCs, and arm the injected conversion path. */
uint8_t current_shunt_init(void);

/*
 * Collect zero-current offsets with PWM main outputs disabled.
 * This call blocks until calibration completes or timeout_ms expires.
 */
uint8_t current_shunt_calibrate(uint32_t timeout_ms);

/*
 * Process the completed ADC1/ADC2 injected samples.
 * Called once per PWM period from ADC1_2_IRQHandler after ADC2 JEOS.
 * Returns 1 only when valid running currents were reconstructed.
 */
uint8_t current_shunt_adc_irq(void);

/*
 * Commit the pending ADC injected context at the TIM1 update event.
 * This function owns the safe TRGO-off -> JSQR update -> TRGO-on sequence.
 */
void current_shunt_tim_update_irq(void);

/*
 * Plan the next sampling pair and CH4 trigger point from the PWM compare values.
 * The sector argument is retained for the FOC interface; routing uses CCR ordering.
 */
void current_shunt_prepare_pwm(uint32_t ccr_u,
                               uint32_t ccr_v,
                               uint32_t ccr_w,
                               uint8_t sector);

/* Copy the latest reconstructed U/V/W currents in amperes to non-null outputs. */
void current_shunt_get_currents(float *iu, float *iv, float *iw);

/* Return 1 only after offset calibration completes without a sampling fault. */
uint8_t current_shunt_is_ready(void);

#endif /* CURRENT_SHUNT_H */
