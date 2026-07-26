#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include "foc_config.h"
#include "foc_math.h"

typedef struct {
    float angle_rad;
    float velocity_rpm;
} foc_feedback_t;

extern foc_feedback_t foc_feedback;

typedef enum {
    FOC_SAFETY_FAULT_NONE = 0,
    FOC_SAFETY_FAULT_CURRENT_SENSE,
    FOC_SAFETY_FAULT_CALIB_OVERCURRENT,
    FOC_SAFETY_FAULT_RUN_OVERCURRENT,
    FOC_SAFETY_FAULT_CALIB_TIMEOUT,
    FOC_SAFETY_FAULT_CALIB_STATE
} foc_safety_fault_t;

typedef struct {
    volatile float peak_current_a;
    volatile float max_observed_current_a;
    volatile float current_limit_a;
    volatile float hard_current_limit_a;
    volatile float trip_current_u_a;
    volatile float trip_current_v_a;
    volatile float trip_current_w_a;
    volatile uint32_t trip_tim_cnt;
    volatile uint16_t consecutive_over_limit;
    volatile uint16_t trip_adc1_raw;
    volatile uint16_t trip_adc2_raw;
    volatile uint8_t fault_code;
    volatile uint8_t trip_calib_state;
    volatile uint8_t trip_pwm_stage;
    volatile uint8_t trip_active_pair;
    volatile uint8_t trip_hard_limit;
} foc_safety_diag_t;

extern volatile foc_safety_diag_t g_foc_safety_diag;

typedef struct {
    float electrical_angle_rad;
    float shaft_angle_rad;
    float angle_step_rad;
    float pole_pairs;
    uint8_t svpwm_sector;
    dq_t dq;
    ab_t ab;
} foc_vf_state_t;

extern foc_vf_state_t vf;

typedef enum {
    FOC_ANGLE_OPEN_LOOP = 0,
    FOC_ANGLE_ENCODER_CALIBRATED
} foc_angle_source_t;

void foc_contr_init(float pole_pairs, float init_rpm);
void foc_set_speed_rpm(float rpm);
void foc_vf_set_voltage(float v_q);
void foc_set_dq_voltage(float vd, float vq);
void foc_openloop_hold(float theta_e, float vd, float vq);
void foc_openloop_spin(float rpm, float vd, float vq);
void foc_set_angle_source(foc_angle_source_t source);
foc_angle_source_t foc_get_angle_source(void);
float foc_get_calibrated_electrical_angle(void);
void foc_tim_irq(void);
void foc_safety_latch_fault(foc_safety_fault_t fault);

#endif /* FOC_CONTROLLER_H */
