#include "vofa.h"
#include "main.h"
#include "foc_config.h"
#include "../foc/Core/foc_math.h"
#include "../foc/Core/foc_controller.h"
#include "../foc/Driver/encoder/abz_encoder.h"
#include "../foc/Driver/current/current_shunt.h"
#include "../foc/App/foc_calib.h"


extern UART_HandleTypeDef huart2;
extern foc_vf_state_t vf;
extern volatile uint32_t g_abz_z_irq_count;

VOFA_Send_Handle_t VOFA_Handle = {
    .tail = {0x00, 0x00, 0x80, 0x7f},
};

void VOFA_Task(void)
{
    if (foc_log_monitor.angle == 0 || foc_log_monitor.dq == 0 || foc_log_monitor.ab == 0 ||
        foc_log_monitor.pwm_a == 0 || foc_log_monitor.pwm_b == 0 || foc_log_monitor.pwm_c == 0)
    {
        return;
    }

    if (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    {
        return;
    }


    VOFA_Handle.fdata[0] = *foc_log_monitor.angle;
  //  VOFA_Handle.fdata[1] = foc_log_monitor.dq->d;
   // VOFA_Handle.fdata[2] = foc_log_monitor.dq->q;

    VOFA_Handle.fdata[1] = foc_feedback.angle_rad;          // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?    VOFA_Handle.fdata[2] = foc_feedback.velocity_rpm;

    VOFA_Handle.fdata[3] = (float)g_abz_z_irq_count;
    VOFA_Handle.fdata[4] = (float)foc_calib_get_state();
    {
        const float pwm_to_voltage = U_DC / PWM_CNT;
        VOFA_Handle.fdata[5] = (*foc_log_monitor.pwm_a) * pwm_to_voltage;
        VOFA_Handle.fdata[6] = (*foc_log_monitor.pwm_b) * pwm_to_voltage;
        VOFA_Handle.fdata[7] = (*foc_log_monitor.pwm_c) * pwm_to_voltage;
    }

    current_shunt_get_currents(&VOFA_Handle.fdata[8],
                               &VOFA_Handle.fdata[9],
                               &VOFA_Handle.fdata[10]);
    VOFA_Handle.fdata[11] = (float)current_shunt_is_ready();

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&VOFA_Handle, sizeof(VOFA_Handle));
}
