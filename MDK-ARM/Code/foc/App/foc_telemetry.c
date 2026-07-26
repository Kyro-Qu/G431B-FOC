/**
 * @file    foc_telemetry.c
 * @brief   VOFA+ JustFloat 遥测实现
 *
 * JustFloat 协议：N 个 float（小端）+ 帧尾 {0x00,0x00,0x80,0x7f}。
 * 帧长 16×4+4 = 68 字节，1 kHz 下占用 68 kB/s，
 * 远小于 6.5 Mbaud（约 650 kB/s）的链路容量。
 */

#include "foc_telemetry.h"
#include "foc_app.h"
#include "foc_calib.h"
#include "main.h"
#include "../Driver/current/current_shunt.h"

extern UART_HandleTypeDef huart2;

typedef struct {
    float ch[FOC_TELEMETRY_CH];
    uint8_t tail[4];
} telemetry_frame_t;

static telemetry_frame_t frame = {
    .tail = {0x00U, 0x00U, 0x80U, 0x7FU},
};

static volatile uint8_t telem_enable = FOC_TELEMETRY_DEFAULT_ON;
static uint16_t decim_cnt = 0U;

void foc_telemetry_init(void)
{
    decim_cnt = 0U;
}

void foc_telemetry_set_enable(uint8_t enable)
{
    telem_enable = (enable != 0U) ? 1U : 0U;
}

uint8_t foc_telemetry_get_enable(void)
{
    return telem_enable;
}

void foc_telemetry_isr_tick(void)
{
    const foc_motor_t *m0 = &g_foc_motors[0];

    if (telem_enable == 0U) {
        return;
    }
    if (++decim_cnt < (uint16_t)FOC_TELEMETRY_DIV) {
        return;
    }
    decim_cnt = 0U;

    /* 上一帧还没发完就跳过本帧，绝不在中断里等待 */
    if (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {
        return;
    }

    frame.ch[0] = m0->theta_e;
    frame.ch[1] = m0->theta_mech;
    frame.ch[2] = m0->velocity_rpm;
    frame.ch[3] = m0->vel_ref_rpm;
    frame.ch[4] = m0->i_dq_filt.d;
    frame.ch[5] = m0->i_dq_filt.q;
    frame.ch[6] = m0->iq_ref;
    frame.ch[7] = m0->v_dq.d;
    frame.ch[8] = m0->v_dq.q;
    frame.ch[9] = m0->i_abc.a;
    frame.ch[10] = m0->i_abc.b;
    frame.ch[11] = m0->i_abc.c;
    frame.ch[12] = ((float)m0->state * 10.0f) + (float)foc_calib_get_state();
    frame.ch[13] = ((float)m0->safety.fault_code * 100.0f) +
                   (float)g_current_shunt_diag.fault_code;
    frame.ch[14] = m0->position_rad;
#if FOC_NUM_AXES >= 2
    frame.ch[15] = g_foc_motors[1].theta_e;
#else
    frame.ch[15] = m0->target;
#endif

    (void)HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&frame, sizeof(frame));
}
