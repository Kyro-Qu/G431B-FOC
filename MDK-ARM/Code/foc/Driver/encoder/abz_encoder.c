/**
 * @file    abz_encoder.c
 * @brief   ABZ 增量式编码器驱动（面�?FOC，角度输�?[0, 2π)�? *
 * 核心思想（参�?ABZ编码�?md）：
 *   - ARR 设为最大值，适用任何编码器，无需针对线数调整
 *   - 每次�?PWM 中断里读�?CNT，用 delta = CNT_now - CNT_last 计算速度和方�? *   - 笃定采样频率下电机不可能转半圈以上，�?ABZ_TIMER_HALF_RANGE 判断回绕
 *   - Z 相每圈清零防累积误差
 *   - delta 直接�?ABZ_RPM_COEFF 得速度
 *   - position_cnt 直接�?ABZ_RAD_PER_CNT 得角�? *
 * 典型用法�? *   float motor_angle = 0.0f;
 *   float motor_rpm   = 0.0f;
 *   abz_encoder_init(&motor_angle, &motor_rpm);   // 绑定一�? *
 *   // FOC PWM 中断�? *   abz_encoder_update();                         // 自动刷新变量
 *   float theta_e = motor_angle * pole_pairs;     // 计算电角�? *   theta_e = limit_angle_rad(theta_e);           // 限制�?[0, 2π)
 *
 * Z 相接入示例：
 *   void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 *       if (GPIO_Pin == ABZ_Z_Pin) {
 *           abz_encoder_set_zero();
 *       }
 *   }
 */

#include "abz_encoder.h"
#include <stddef.h>

/* ================================================================
 *  模块内部状�? * ================================================================ */
static ABZ_Encoder_t enc = {0};

/* ================================================================
 *  内部辅助函数
 * ================================================================ */

/**
 * @brief  计算两次采样间的计数差值，处理定时器回�? *
 * 原理�? *   delta = CNT_now - CNT_last
 *   正常运转�?|delta| 远小�?ABZ_TIMER_HALF_RANGE�? *   如果 |delta| > ABZ_TIMER_HALF_RANGE，说明跨越了 0/ARR 边界�? *   需要加�?ABZ_TIMER_COUNTER_RANGE 修正方向�? *
 *   前提：采样频率下电机不可能转半圈以上（物理不可能），
 *   所以半量程判断绝对可靠�? *
 * @param  cur_cnt  当前 CNT �? * @param  last_cnt 上一�?CNT �? * @return 修正后的增量（正 = 正转，负 = 反转�? */
static int32_t abz_calc_delta(uint16_t cur_cnt, uint16_t last_cnt)
{
    int32_t delta = (int32_t)cur_cnt - (int32_t)last_cnt;

    if (delta > ABZ_TIMER_HALF_RANGE) {
        /* 反转跨越 0 边界，修正为负增�?*/
        delta -= ABZ_TIMER_COUNTER_RANGE;
    } else if (delta < -ABZ_TIMER_HALF_RANGE) {
        /* 正转跨越 ARR 边界，修正为正增�?*/
        delta += ABZ_TIMER_COUNTER_RANGE;
    }

    return delta;
}

/**
 * @brief  将累计位置限制到单圈范围 [0, ABZ_ENCODER_CPR)
 *
 * @param  position_cnt 累计位置计数
 * @return 限制后的位置 [0, ABZ_ENCODER_CPR)
 */
static int32_t abz_wrap_position(int32_t position_cnt)
{
    position_cnt %= (int32_t)ABZ_ENCODER_CPR;

    if (position_cnt < 0) {
        position_cnt += (int32_t)ABZ_ENCODER_CPR;
    }

    return position_cnt;
}

/**
 * @brief  5 点去极值均值（Trimmed Mean�? *
 * 取最�?5 �?delta，去掉最大值和最小值，剩余 3 个取均值�? * �?3 点中值更平滑�? *   - 能容忍连�?2 个异常采样（最�?最小同时被剔除�? *   - 均值输出有小数，比纯整数中值更平滑
 *   - 延迟仅多 2 个采样周期（0.3 ms @16kHz），对速度环无�? */
static float abz_trimmed_mean5(const int32_t buf[5])
{
    int32_t min_val = buf[0];
    int32_t max_val = buf[0];
    int32_t sum = buf[0];
                                                                                             
    for (uint8_t i = 1U; i < 5U; i++) {
        sum += buf[i];
        if (buf[i] < min_val) min_val = buf[i];
        if (buf[i] > max_val) max_val = buf[i];
    }

    /* 去掉最大和最小，剩余 3 个取均�?*/
    return (float)(sum - min_val - max_val) * (1.0f / 3.0f);
}

/* ================================================================
 *  公开 API
 * ================================================================ */

/**
 * @brief  初始化编码器，绑定外部输出变量并启动定时器编码器模式
 */
void abz_encoder_init(float *angle_rad, float *velocity_rpm)
{
    enc.last_cnt = 0U;
    enc.position_cnt = 0;
    enc.velocity_filtered = 0.0f;
    enc.delta_buf[0] = 0;
    enc.delta_buf[1] = 0;
    enc.delta_buf[2] = 0;
    enc.delta_buf[3] = 0;
    enc.delta_buf[4] = 0;
    enc.delta_idx = 0U;
    enc.angle_rad = angle_rad;
    enc.velocity_rpm = velocity_rpm;
    enc.zero_request = 0U;
    enc.index_request = 0U;
    enc.index_pending = 0U;
    enc.index_hw_cnt = 0U;
    enc.index_position_cnt = 0;
    enc.zero_on_index = 1U;
    enc.is_calibrated = 0U;

    if (enc.angle_rad != NULL) {
        *enc.angle_rad = 0.0f;
    }
    if (enc.velocity_rpm != NULL) {
        *enc.velocity_rpm = 0.0f;
    }

    __HAL_TIM_SET_COUNTER(&ABZ_ENCODER_TIM_HANDLE, 0);
    HAL_TIM_Encoder_Start(&ABZ_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);
}
/**
 * @brief  停止编码器定时器
 */
void abz_encoder_deinit(void)
{
    HAL_TIM_Encoder_Stop(&ABZ_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);
}

/**
 * @brief  周期更新机械角度和机械速度
 *
 * �?FOC PWM 中断中调用（频率 = PWM_FREQ_HZ），执行流程�? *   1. 处理 Z 相归零请求（避免中断竞争�? *   2. 读取当前 CNT
 *   3. delta = CNT_now - CNT_last（含回绕处理�? *   4. delta 限幅（丢弃干扰导致的异常跳变�? *   5. position_cnt += delta，取模到 [0, ABZ_ENCODER_CPR)
 *   6. angle = position_cnt × ABZ_RAD_PER_CNT
 *   7. velocity = 一�?IIR 低通滤�? */
void abz_encoder_update(void)
{
    if ((enc.angle_rad == NULL) || (enc.velocity_rpm == NULL)) {
        return;
    }

    if (enc.index_request) {
        uint16_t index_cnt = enc.index_hw_cnt;
        int32_t index_delta = abz_calc_delta(index_cnt, enc.last_cnt);

        enc.index_position_cnt = abz_wrap_position(enc.position_cnt + index_delta);
        enc.index_pending = 1U;
        enc.index_request = 0U;
        enc.is_calibrated = 1U;

        if (enc.zero_on_index) {
            __HAL_TIM_SET_COUNTER(&ABZ_ENCODER_TIM_HANDLE, 0);
            enc.last_cnt = 0U;
            enc.position_cnt = 0;
            *enc.angle_rad = 0.0f;
            return;
        }
    }

    if (enc.zero_request) {
        __HAL_TIM_SET_COUNTER(&ABZ_ENCODER_TIM_HANDLE, 0);
        enc.last_cnt = 0U;
        enc.position_cnt = 0;
        enc.zero_request = 0U;
        enc.index_pending = 0U;
        enc.is_calibrated = 1U;
        *enc.angle_rad = 0.0f;
        return;
    }

    uint16_t cur_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&ABZ_ENCODER_TIM_HANDLE);
    int32_t delta = abz_calc_delta(cur_cnt, enc.last_cnt);
    enc.last_cnt = cur_cnt;

    if (delta > ABZ_MAX_DELTA || delta < -ABZ_MAX_DELTA) {
        delta = 0;
    }

    enc.delta_buf[enc.delta_idx] = delta;
    enc.delta_idx = (enc.delta_idx + 1U) % 5U;
    float delta_filtered = abz_trimmed_mean5(enc.delta_buf);

    enc.position_cnt = abz_wrap_position(enc.position_cnt + delta);
    *enc.angle_rad = (float)enc.position_cnt * ABZ_RAD_PER_CNT;

    float velocity_raw = delta_filtered * ABZ_RPM_COEFF;
    enc.velocity_filtered += ABZ_VELOCITY_LPF_ALPHA * (velocity_raw - enc.velocity_filtered);
    *enc.velocity_rpm = enc.velocity_filtered;
}
/**
 * @brief  Z 相归零校准（�?Z 相外部中断回调中调用�? *
 * 电机每转一�?Z 相来一个脉冲，触发中断�? * 为避免与 abz_encoder_update() 的竞争条件，此函数仅设置标志�? * 实际清零操作�?update() 在下一�?PWM 周期统一处理�? *
 * 这样保证 last_cnt、position_cnt、CNT 三者始终在同一上下文中修改�? * 不会出现中断打断 update 导致�?delta 异常尖峰�? */
void abz_encoder_set_zero(void)
{
    enc.zero_request = 1U;
}
 
/**
 * @brief  获取校准状�? * @return 1 = 已通过 Z 相校准，0 = 尚未校准
 */

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

    if (position_cnt != NULL) {
        *position_cnt = enc.index_position_cnt;
    }
    enc.index_pending = 0U;
    return 1U;
}

int32_t abz_encoder_get_position_cnt(void)
{
    return enc.position_cnt;
}
uint8_t abz_encoder_is_calibrated(void)
{
    return enc.is_calibrated;
}
