/**
 * @file    foc_telemetry.c
 * @brief   FOC 自解释掩码遥测实现 (FOC-STP v1.0)
 *
 * 采用快慢双缓冲与统一所有权仲裁，支持：
 * 1. 500 Hz 自解释掩码波形流 (WAVE, 快环 ISR 专用 s_wave_buf)
 * 2. 10 Hz 独立状态心跳流 (STATUS, 主循环 s_slow_buf)
 * 3. 故障跳闸瞬态事件流 (EVENT, 锁存快照并可靠异步调度)
 * 4. 配置应答流 (ACK, 掩码与采样率生效回传)
 */

#include "foc_telemetry.h"
#include "foc_stp.h"
#include "foc_app.h"
#include "foc_calib.h"
#include "foc_angle_manager.h"
#include "foc_sensorless_bench.h"
#include "../Core/foc_port.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "../Driver/current/current_shunt.h"
#include "../HAL/foc_board_g431.h"

extern UART_HandleTypeDef huart2;

/* 发送状态枚举与仲裁 */
typedef enum {
    FOC_TX_IDLE = 0,
    FOC_TX_DMA_WAVE,
    FOC_TX_DMA_SLOW
} foc_tx_state_t;

static volatile foc_tx_state_t s_tx_state = FOC_TX_IDLE;

/* 独立物理缓冲：彻底杜绝 ISR 与主循环相互踩踏 */
static uint8_t s_wave_buf[80]; /* 16 通道最大 80 字节 */
static uint8_t s_slow_buf[32]; /* STATUS(23B) / EVENT(19B) / ACK(16B) */

static volatile uint8_t telem_enable = FOC_TELEMETRY_DEFAULT_ON;
static volatile uint8_t telem_suspend = 0U;
static volatile uint32_t s_channel_mask = FOC_TELEMETRY_DEFAULT_MASK;
static volatile uint16_t s_telem_div = FOC_TELEMETRY_DIV;

static uint16_t decim_cnt = 0U;
static uint16_t s_wave_seq = 0U;
static uint16_t s_slow_seq = 0U;
static uint32_t s_last_status_ms = 0U;

/* 慢速流待发状态管理 */
static volatile uint8_t s_status_pending = 0U;
static volatile uint8_t s_event_pending = 0U;
static volatile uint8_t s_ack_pending = 0U;

typedef struct {
    uint8_t event_id;
    uint8_t motor_fault;
    uint8_t shunt_fault;
    uint32_t detail;
} foc_event_snapshot_t;
static foc_event_snapshot_t s_event_snap;

typedef struct {
    uint8_t cmd_code;
    uint8_t status;
    uint32_t effective_mask;
    uint16_t effective_rate_hz;
} foc_ack_snapshot_t;
static foc_ack_snapshot_t s_ack_snap;

void foc_telemetry_init(void)
{
    decim_cnt = 0U;
    s_wave_seq = 0U;
    s_slow_seq = 0U;
    s_last_status_ms = 0U;
    s_channel_mask = FOC_TELEMETRY_DEFAULT_MASK;
    s_telem_div = FOC_TELEMETRY_DIV;
    s_tx_state = FOC_TX_IDLE;
    s_status_pending = 0U;
    s_event_pending = 0U;
    s_ack_pending = 0U;
}

void foc_telemetry_set_enable(uint8_t enable)
{
    telem_enable = (enable != 0U) ? 1U : 0U;
}

uint8_t foc_telemetry_get_enable(void)
{
    return telem_enable;
}

uint16_t foc_telemetry_get_rate_hz(void)
{
    return (uint16_t)((uint32_t)FOC_PWM_FREQ_HZ / (uint32_t)s_telem_div);
}

static void queue_ack(uint8_t cmd_code, uint8_t status)
{
    s_ack_snap.cmd_code = cmd_code;
    s_ack_snap.status = status;
    s_ack_snap.effective_mask = s_channel_mask;
    s_ack_snap.effective_rate_hz = foc_telemetry_get_rate_hz();
    s_ack_pending = 1U;
}

uint8_t foc_telemetry_set_mask(uint32_t mask)
{
    uint8_t count = foc_stp_popcount32(mask);
    uint8_t status;

    if (count > (uint8_t)FOC_STP_MAX_WAVE_CHANNELS) {
        /* 超过硬件 16 通道上限，拒绝修改，保持旧掩码 */
        status = FOC_STP_ACK_LIMITED;
    } else {
        s_channel_mask = mask;
        status = FOC_STP_ACK_OK;
    }

    queue_ack(FOC_STP_ACK_CMD_SET_MASK, status);
    return status;
}

uint32_t foc_telemetry_get_mask(void)
{
    return s_channel_mask;
}

uint8_t foc_telemetry_set_rate_hz(uint16_t rate_hz)
{
    const uint32_t pwm_hz = (uint32_t)FOC_PWM_FREQ_HZ;
    uint32_t div;
    uint8_t status;

    if ((rate_hz < FOC_TELEMETRY_RATE_MIN_HZ) || (rate_hz > FOC_TELEMETRY_RATE_MAX_HZ)) {
        status = FOC_STP_ACK_REJECTED;
    } else {
        /* 分频只能取整数：非整除时向下取整分频（实际速率略高于请求），ACK 回传真实生效值 */
        div = pwm_hz / (uint32_t)rate_hz;
        if (div < (uint32_t)FOC_TELEMETRY_DIV) {
            div = (uint32_t)FOC_TELEMETRY_DIV;
        }
        s_telem_div = (uint16_t)div;
        decim_cnt = 0U;
        status = ((pwm_hz % (uint32_t)rate_hz) == 0U) ? FOC_STP_ACK_OK : FOC_STP_ACK_LIMITED;
    }

    queue_ack(FOC_STP_ACK_CMD_SET_RATE, status);
    return status;
}

void foc_telemetry_suspend(uint8_t on)
{
    telem_suspend = (on != 0U) ? 1U : 0U;
}

void foc_telemetry_reset_tx_state(void)
{
    uint32_t primask = foc_critical_enter();
    s_tx_state = FOC_TX_IDLE;
    foc_critical_exit(primask);
}

static float extract_channel_value(const foc_motor_t *m0, uint8_t ch)
{
    switch (ch) {
    case 0:  return m0->theta_e;
    case 1:  return m0->i_dq.q;
    case 2:  return m0->velocity_filt_rpm;
    case 3:  return m0->vel_ref_rpm;
    case 4:  return m0->i_dq_filt.d;
    case 5:  return m0->i_dq_filt.q;
    case 6:  return m0->iq_ref;
    case 7:  return m0->v_dq.d;
    case 8:  return m0->v_dq.q;
    case 9:  return m0->i_abc.a;
    case 10: return m0->i_abc.b;
    case 11: return m0->i_abc.c;
    case 12: return m0->svm.duty_a;
    case 13: return m0->i_dq.d;
    case 14: return m0->id_ref;
    case 15: return m0->velocity_rpm;
    case 16: return m0->pos_origin_rad + m0->target;
    case 17: return m0->position_rad;
    case 18: return m0->svm.duty_b;
    case 19: return m0->svm.duty_c;
    case 20: return g_angle_mgr.theta_sensorless;
    case 21: return g_angle_mgr.speed_obs_rpm;
    case 22: return g_angle_mgr.angle_error_deg;
    case 23: return g_angle_mgr.conf_window;
    case 24: return g_sensorless_bench.obs2_vesc.flux_mag;
    case 25: return 1.5f * ((m0->v_dq.d * m0->i_dq.d) + (m0->v_dq.q * m0->i_dq.q));
    case 26: return g_foc_vbus_diag.voltage_v;
    case 27: return 1.5f * FOC_M0_POLE_PAIRS * 0.0009f * m0->i_dq.q;
    case 28: return m0->iq_ref - m0->i_dq.q;
    case 29: return m0->id_ref - m0->i_dq.d;
    case 30: return m0->vel_ref_rpm - m0->velocity_filt_rpm;
    case 31: return (float)g_angle_mgr.state;
    default: return 0.0f;
    }
}

/** 快环中调用 (500 Hz)：按掩码提取变量并 DMA 发送 WAVE 帧 */
void foc_telemetry_isr_tick(void)
{
    const foc_motor_t *m0 = &g_foc_motors[0];
    float vals[FOC_STP_MAX_WAVE_CHANNELS];
    uint8_t val_count = 0U;
    uint32_t mask = s_channel_mask;
    uint8_t bit;
    uint16_t tx_len;
    uint32_t primask;

    if ((telem_enable == 0U) || (telem_suspend != 0U) || (mask == 0U)) {
        return;
    }
    if (++decim_cnt < s_telem_div) {
        return;
    }
    decim_cnt = 0U;

    /* 极短原子临界区检查并认领 TX DMA */
    primask = foc_critical_enter();
    if ((s_tx_state != FOC_TX_IDLE) || (huart2.gState != HAL_UART_STATE_READY)) {
        foc_critical_exit(primask);
        return;
    }
    s_tx_state = FOC_TX_DMA_WAVE;
    foc_critical_exit(primask);

    /* 按掩码从低到高提取物理量 */
    for (bit = 0U; (bit < 32U) && (val_count < (uint8_t)FOC_STP_MAX_WAVE_CHANNELS); bit++) {
        if ((mask & (1UL << bit)) != 0U) {
            vals[val_count++] = extract_channel_value(m0, bit);
        }
    }

    /* 打包到独立 s_wave_buf */
    tx_len = foc_stp_pack_wave(s_wave_buf, (uint16_t)sizeof(s_wave_buf), s_wave_seq++,
                               HAL_GetTick(), mask, vals, val_count);
    if (tx_len > 0U) {
        if (HAL_UART_Transmit_DMA(&huart2, s_wave_buf, tx_len) != HAL_OK) {
            s_tx_state = FOC_TX_IDLE;
        }
    } else {
        s_tx_state = FOC_TX_IDLE;
    }
}

/** 故障跳闸/状态改变事件：只锁存快照并排队，绝不阻塞打断 DMA。
 *  单槽快照：尚未发出的 FAULT_TRIP 不允许被后续低优先级事件覆盖。 */
void foc_telemetry_report_event(uint8_t event_id, uint8_t motor_fault, uint8_t shunt_fault, uint32_t detail)
{
    if ((s_event_pending != 0U) &&
        (s_event_snap.event_id == FOC_STP_EVENT_FAULT_TRIP) &&
        (event_id != FOC_STP_EVENT_FAULT_TRIP)) {
        return;
    }
    s_event_snap.event_id = event_id;
    s_event_snap.motor_fault = motor_fault;
    s_event_snap.shunt_fault = shunt_fault;
    s_event_snap.detail = detail;
    s_event_pending = 1U;
}

/** 主循环中调用：调度 10 Hz STATUS、异步 EVENT 及配置 ACK */
void foc_telemetry_slow_tick(void)
{
    uint32_t now = HAL_GetTick();
    const foc_motor_t *m0 = &g_foc_motors[0];
    uint16_t tx_len = 0U;
    uint8_t claimed = 0U;
    uint32_t primask;

    /* 检查 100ms STATUS 周期 */
    if ((now - s_last_status_ms) >= 100U) {
        s_status_pending = 1U;
    }

    if ((s_event_pending == 0U) && (s_ack_pending == 0U) && (s_status_pending == 0U)) {
        return;
    }

    if (telem_suspend != 0U) {
        return;
    }

    /* 极短原子临界区检查并认领 TX DMA */
    primask = foc_critical_enter();
    if ((s_tx_state == FOC_TX_IDLE) && (huart2.gState == HAL_UART_STATE_READY)) {
        s_tx_state = FOC_TX_DMA_SLOW;
        claimed = 1U;
    }
    foc_critical_exit(primask);

    if (claimed == 0U) {
        /* 当前 DMA 忙，保留 pending 标记，下一次循环立即重试 */
        return;
    }

    /* 优先级：EVENT > ACK > STATUS */
    if (s_event_pending != 0U) {
        tx_len = foc_stp_pack_event(
            s_slow_buf, (uint16_t)sizeof(s_slow_buf), s_slow_seq++,
            now, s_event_snap.event_id, s_event_snap.motor_fault,
            s_event_snap.shunt_fault, s_event_snap.detail
        );
        if (tx_len > 0U) {
            if (HAL_UART_Transmit_DMA(&huart2, s_slow_buf, tx_len) == HAL_OK) {
                s_event_pending = 0U;
            } else {
                s_tx_state = FOC_TX_IDLE;
            }
        } else {
            s_tx_state = FOC_TX_IDLE;
        }
    } else if (s_ack_pending != 0U) {
        tx_len = foc_stp_pack_ack(
            s_slow_buf, (uint16_t)sizeof(s_slow_buf), s_slow_seq++,
            s_ack_snap.cmd_code, s_ack_snap.status,
            s_ack_snap.effective_mask, s_ack_snap.effective_rate_hz
        );
        if (tx_len > 0U) {
            if (HAL_UART_Transmit_DMA(&huart2, s_slow_buf, tx_len) == HAL_OK) {
                s_ack_pending = 0U;
            } else {
                s_tx_state = FOC_TX_IDLE;
            }
        } else {
            s_tx_state = FOC_TX_IDLE;
        }
    } else if (s_status_pending != 0U) {
        tx_len = foc_stp_pack_status(
            s_slow_buf, (uint16_t)sizeof(s_slow_buf), s_slow_seq++, now,
            (uint16_t)(g_foc_vbus_diag.voltage_v * 100.0f),
            m0->safety.fault_code, g_current_shunt_diag.fault_code,
            (uint8_t)m0->state, (uint8_t)m0->mode,
            -128, /* 芯片温度暂未开启硬件 ADC 采样通道，标 -128 表示无效 */
            (int16_t)m0->velocity_filt_rpm,
            (int16_t)(m0->i_dq_filt.q * 100.0f)
        );
        if (tx_len > 0U) {
            if (HAL_UART_Transmit_DMA(&huart2, s_slow_buf, tx_len) == HAL_OK) {
                s_status_pending = 0U;
                s_last_status_ms = now;
            } else {
                s_tx_state = FOC_TX_IDLE;
            }
        } else {
            s_tx_state = FOC_TX_IDLE;
        }
    } else {
        s_tx_state = FOC_TX_IDLE;
    }
}

void foc_telemetry_status_tick(void)
{
    foc_telemetry_slow_tick();
}

/* HAL UART 发送完成中断回调：安全释放 TX 所有权 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        s_tx_state = FOC_TX_IDLE;
    }
}
