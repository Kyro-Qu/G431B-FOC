/**
 * @file    abz_encoder.c
 * @brief   ABZ 增量式编码器驱动实现（算法与时序在本板硬件上验证过）
 */

#include "abz_encoder.h"
#include "main.h"

extern TIM_HandleTypeDef ABZ_ENCODER_TIM_HANDLE;

/* ======================== 内部状态 ======================== */

typedef struct {
    uint16_t last_cnt;            /* 上一拍 TIM 计数值 */
    int32_t  position_cnt;        /* 单圈位置计数 [0, CPR) */
    float    angle_rad;           /* 机械角 [0, 2π) */
    float    velocity_rpm;        /* 滤波后速度 */
    float    velocity_filt;       /* IIR 内部状态 */
    int32_t  delta_buf[5];        /* 5 点去极值均值窗口 */
    uint8_t  delta_idx;
    /* Z / index 事件（中断置位，update 消费） */
    volatile uint8_t  zero_request;
    volatile uint8_t  index_request;
    volatile uint16_t index_hw_cnt;
    uint8_t  index_pending;
    int32_t  index_position_cnt;
    uint8_t  zero_on_index;
    uint8_t  is_calibrated;
} abz_encoder_t;

static abz_encoder_t enc;

/* ======================== 内部函数 ======================== */

/**
 * 计算两拍之间的计数增量，处理 16 位计数器回绕。
 * |delta| > 半量程 说明跨越了 0/ARR 边界，需要修正方向。
 * 前提：采样频率下电机不可能一拍转过半圈（物理保证）。
 */
static int32_t abz_calc_delta(uint16_t cur_cnt, uint16_t last_cnt)
{
    int32_t delta = (int32_t)cur_cnt - (int32_t)last_cnt;

    if (delta > ABZ_TIMER_HALF_RANGE) {
        delta -= ABZ_TIMER_COUNTER_RANGE;
    } else if (delta < -ABZ_TIMER_HALF_RANGE) {
        delta += ABZ_TIMER_COUNTER_RANGE;
    }
    return delta;
}

/** 位置计数取模到 [0, CPR) */
static int32_t abz_wrap_position(int32_t position_cnt)
{
    position_cnt %= (int32_t)ABZ_ENCODER_CPR;
    if (position_cnt < 0) {
        position_cnt += (int32_t)ABZ_ENCODER_CPR;
    }
    return position_cnt;
}

/**
 * 5 点去极值均值：去掉窗口内最大和最小，剩余 3 个取均值。
 * 比 3 点中值更平滑（能容忍连续 2 个异常拍），
 * 延迟仅多 2 拍（0.125 ms @16kHz），对速度环无感。
 */
static float abz_trimmed_mean5(const int32_t buf[5])
{
    int32_t min_val = buf[0];
    int32_t max_val = buf[0];
    int32_t sum = buf[0];
    uint8_t i;

    for (i = 1U; i < 5U; i++) {
        sum += buf[i];
        if (buf[i] < min_val) {
            min_val = buf[i];
        }
        if (buf[i] > max_val) {
            max_val = buf[i];
        }
    }
    return (float)(sum - min_val - max_val) * (1.0f / 3.0f);
}

/* ======================== API ======================== */

void abz_encoder_init(void)
{
    uint8_t i;

    enc.last_cnt = 0U;
    enc.position_cnt = 0;
    enc.angle_rad = 0.0f;
    enc.velocity_rpm = 0.0f;
    enc.velocity_filt = 0.0f;
    for (i = 0U; i < 5U; i++) {
        enc.delta_buf[i] = 0;
    }
    enc.delta_idx = 0U;
    enc.zero_request = 0U;
    enc.index_request = 0U;
    enc.index_hw_cnt = 0U;
    enc.index_pending = 0U;
    enc.index_position_cnt = 0;
    enc.zero_on_index = 1U;
    enc.is_calibrated = 0U;

    __HAL_TIM_SET_COUNTER(&ABZ_ENCODER_TIM_HANDLE, 0);
    HAL_TIM_Encoder_Start(&ABZ_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);
}

void abz_encoder_deinit(void)
{
    HAL_TIM_Encoder_Stop(&ABZ_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);
}

void abz_encoder_update(void)
{
    uint16_t cur_cnt;
    uint16_t index_cnt = 0U;
    uint8_t index_request = 0U;
    uint32_t primask;
    int32_t delta;
    float delta_filtered;
    float velocity_raw;

    /* 原子地取走 Z 事件，避免 EXTI 在 index_hw_cnt 读取与请求清除之间
     * 写入一个新事件。临界区只包含三个内存访问。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (enc.index_request != 0U) {
        index_cnt = enc.index_hw_cnt;
        enc.index_request = 0U;
        index_request = 1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    /* 先处理 Z/index 事件。index_hw_cnt 是 Z 边沿时刻，而当前 CNT 已经
     * 继续运行；零位重映射时必须保留这段 post-index 增量。 */
    if (index_request != 0U) {
        uint16_t post_cnt;
        int32_t index_delta = abz_calc_delta(index_cnt, enc.last_cnt);
        int32_t post_index_delta;

        cur_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&ABZ_ENCODER_TIM_HANDLE);
        post_index_delta = abz_calc_delta(cur_cnt, index_cnt);

        enc.index_position_cnt = abz_wrap_position(enc.position_cnt + index_delta);
        enc.index_pending = 1U;
        enc.is_calibrated = 1U;

        if (enc.zero_on_index != 0U) {
            post_cnt = (uint16_t)post_index_delta;
            __HAL_TIM_SET_COUNTER(&ABZ_ENCODER_TIM_HANDLE, post_cnt);
            enc.last_cnt = post_cnt;
            enc.position_cnt = abz_wrap_position(post_index_delta);
            enc.angle_rad = (float)enc.position_cnt * ABZ_RAD_PER_CNT;
            return;
        }
    }

    if (enc.zero_request != 0U) {
        __HAL_TIM_SET_COUNTER(&ABZ_ENCODER_TIM_HANDLE, 0);
        enc.last_cnt = 0U;
        enc.position_cnt = 0;
        enc.zero_request = 0U;
        enc.index_pending = 0U;
        enc.is_calibrated = 1U;
        enc.angle_rad = 0.0f;
        return;
    }

    cur_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&ABZ_ENCODER_TIM_HANDLE);
    delta = abz_calc_delta(cur_cnt, enc.last_cnt);
    enc.last_cnt = cur_cnt;

    /* 干扰保护：单拍超过 1/4 圈判为异常，丢弃 */
    if ((delta > ABZ_MAX_DELTA) || (delta < -ABZ_MAX_DELTA)) {
        delta = 0;
    }

    enc.delta_buf[enc.delta_idx] = delta;
    enc.delta_idx = (uint8_t)((enc.delta_idx + 1U) % 5U);
    delta_filtered = abz_trimmed_mean5(enc.delta_buf);

    enc.position_cnt = abz_wrap_position(enc.position_cnt + delta);
    enc.angle_rad = (float)enc.position_cnt * ABZ_RAD_PER_CNT;

    velocity_raw = delta_filtered * ABZ_RPM_COEFF;
    enc.velocity_filt += ABZ_VELOCITY_LPF_ALPHA * (velocity_raw - enc.velocity_filt);
    enc.velocity_rpm = enc.velocity_filt;
}

float abz_encoder_angle_rad(void)
{
    return enc.angle_rad;
}

float abz_encoder_velocity_rpm(void)
{
    return enc.velocity_rpm;
}

void abz_encoder_on_index(void)
{
    enc.index_hw_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&ABZ_ENCODER_TIM_HANDLE);
    enc.index_request = 1U;
}

void abz_encoder_force_zero(void)
{
    enc.zero_request = 1U;
}

void abz_encoder_set_zero_on_index(uint8_t enable)
{
    enc.zero_on_index = (enable != 0U) ? 1U : 0U;
}

uint8_t abz_encoder_consume_index(int32_t *position_cnt)
{
    if (enc.index_pending == 0U) {
        return 0U;
    }
    if (position_cnt != 0) {
        *position_cnt = enc.index_position_cnt;
    }
    enc.index_pending = 0U;
    return 1U;
}

uint8_t abz_encoder_is_calibrated(void)
{
    return enc.is_calibrated;
}
