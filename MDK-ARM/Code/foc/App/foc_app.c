#include "foc_app.h"
#include "../Core/foc_controller.h"
#include "../HAL/foc_config.h"
#include "../Driver/encoder/abz_encoder.h"
#include "../Driver/current/current_shunt.h"

static volatile struct
{
    foc_ctrl_mode_t mode;
    foc_state_t state;
} app_state = {.mode = FOC_CTRL_VF_OPENLOOP, .state = FOC_STATE_IDLE};

volatile uint8_t g_foc_state_diag = (uint8_t)FOC_STATE_IDLE;

void foc_init(void)
{
    foc_motor_init();
    foc_contr_init(foc_motor_info.pole_pairs, 10.0f);
    foc_vf_set_voltage(0.0f);

#if FOC_LOG_MONITOR
    foc_log_monitor_init();
#endif

    abz_encoder_init(&foc_feedback.angle_rad, &foc_feedback.velocity_rpm);
    app_state.state = FOC_STATE_IDLE;
    g_foc_state_diag = (uint8_t)FOC_STATE_IDLE;
    if (current_shunt_init() == 0U) {
        app_state.state = FOC_STATE_FAULT;
        g_foc_state_diag = (uint8_t)FOC_STATE_FAULT;
        foc_safety_latch_fault(FOC_SAFETY_FAULT_CURRENT_SENSE);
        return;
    }

    foc_timer_init();
    if (current_shunt_calibrate(500U) == 0U) {
        foc_pwm_disable();
        app_state.state = FOC_STATE_FAULT;
        g_foc_state_diag = (uint8_t)FOC_STATE_FAULT;
        foc_safety_latch_fault(FOC_SAFETY_FAULT_CURRENT_SENSE);
    }
}

foc_state_t foc_get_state(void)
{
    return app_state.state;
}

void foc_set_state(foc_state_t state)
{
    app_state.state = state;
    g_foc_state_diag = (uint8_t)state;
}

foc_ctrl_mode_t foc_get_ctrl_mode(void)
{
    return app_state.mode;
}

void foc_set_ctrl_mode(foc_ctrl_mode_t mode)
{
    app_state.mode = mode;
}

foc_sensor_t foc_get_sensor_type(void)
{
    return FOC_SENS_NONE;
}

void foc_set_sensor_type(foc_sensor_t type)
{
    (void)type;
}
