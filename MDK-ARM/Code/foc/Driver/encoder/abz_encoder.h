/**
 * @file    abz_encoder.h
 * @brief   ABZ 增量式编码器驱动接口（面�?FOC，角度输�?[0, 2π)�? *
 * 核心思想（参�?ABZ编码�?md）：
 *   - ARR 设为最大值（16 位全量程），适用任何编码�? *   - 笃定采样频率下电机不可能转半圈以上，用半量程法判断回�? *   - Z 相每圈清零防累积误差，仅做校准不参与角度计算
 *   - delta 直接乘系数得速度，position 直接乘系数得角度
 *
 * 硬件配置�? *   - TIM4 编码器模式，ARR = ABZ_TIMER_COUNTER_RANGE - 1
 *   - A/B 相接 PB6/PB7，Z 相接 PB8（外部中断，下拉 + 上升沿触发）
 */

#ifndef ABZ_ENCODER_H
#define ABZ_ENCODER_H

#include "foc_math.h"   /* 提供 _2PI, PWM_FREQ_HZ, stm32g4xx_hal */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 用户配置�?========================
 * 更换编码器或定时器时，只需修改这里的宏定义�? */

/** 编码器绑定的定时器句�?*/
#ifndef ABZ_ENCODER_TIM_HANDLE
#define ABZ_ENCODER_TIM_HANDLE  htim4
#endif

/** 编码器每转脉冲数�? 倍频后的�?= 线数 × 4�?*/
#ifndef ABZ_ENCODER_CPR
#define ABZ_ENCODER_CPR         2048U
#endif

#if (ABZ_ENCODER_CPR == 0U)
#error "ABZ_ENCODER_CPR must be greater than 0"
#endif

/* ======================== 派生常量 ========================
 * 以下常量全部由上面的配置宏推导，不使用硬编码数字�? */

/** 定时器计数器总范围（ARR + 1�?6 位定时器 = 65536�? *  ARR 设为最大值，无论编码器线数多少都能自由计�?*/
#ifndef ABZ_TIMER_COUNTER_RANGE
#define ABZ_TIMER_COUNTER_RANGE (65536L)
#endif

/** 回绕判断阈�?= COUNTER_RANGE / 2
 *  |delta| > 此�?�?认为发生了边界回�? *  前提：采样频率下电机不可能转半圈以上 */
#define ABZ_TIMER_HALF_RANGE    (ABZ_TIMER_COUNTER_RANGE / 2L)

/** 每个计数对应的弧度增�?= 2π / ABZ_ENCODER_CPR */
#define ABZ_RAD_PER_CNT         (_2PI / (float)ABZ_ENCODER_CPR)

/** 速度系数：delta × ABZ_RPM_COEFF = RPM
 *  推导：RPM = (delta / ABZ_ENCODER_CPR) × PWM_FREQ_HZ × 60 */
#define ABZ_RPM_COEFF           (60.0f * PWM_FREQ_HZ / (float)ABZ_ENCODER_CPR)

/** 速度低通滤波系�?α（一�?IIR�? *  α �?2π × fc / fs，fc 为截止频率，fs = PWM_FREQ_HZ
 *  0.02 对应�?50 Hz 截止频率，纹波小且速度环响应足�?*/
#ifndef ABZ_VELOCITY_LPF_ALPHA
#define ABZ_VELOCITY_LPF_ALPHA  0.02f
#endif

/** delta 限幅阈值：单个采样周期内最大允许的脉冲增量
 *  超过此值认为是干扰或竞争导致的异常，直接丢�? *  默认 CPR/4 = 最多允许转 1/4 圈（物理上已经极端） */
#ifndef ABZ_MAX_DELTA
#define ABZ_MAX_DELTA           ((int32_t)ABZ_ENCODER_CPR / 4)
#endif

/** 单圈半量程（用于 Z 相丢步检测） */
#define ABZ_HALF_CPR            ((int32_t)ABZ_ENCODER_CPR / 2)

/* ======================== 外部声明 ======================== */

extern TIM_HandleTypeDef ABZ_ENCODER_TIM_HANDLE;

/* ======================== 数据结构 ======================== */

/** ABZ 编码器运行状�?*/
typedef struct {
    uint16_t last_cnt;           /**< 上一�?TIM 计数�?*/
    int32_t  position_cnt;       /**< 单圈累计计数，范�?[0, ABZ_ENCODER_CPR) */
    float    velocity_filtered;  /**< 一�?IIR 滤波后的速度（内部状态） */
    int32_t  delta_buf[5];       /**< 最�?5 �?delta，用于去极值均值滤�?*/
    uint8_t  delta_idx;          /**< delta_buf 环形索引 */
    float    *angle_rad;         /**< 外部机械角度输出指针，范�?[0, 2π) */
    float    *velocity_rpm;      /**< 外部机械速度输出指针，单�?RPM（滤波后�?*/
    volatile uint8_t zero_request; /**< Z 相归零请求标志（中断置位，update 处理�?*/
    volatile uint8_t index_request;
    uint8_t  index_pending;
    uint16_t index_hw_cnt;
    int32_t  index_position_cnt;
    uint8_t  zero_on_index;
    uint8_t  is_calibrated;      /**< Z 相是否已完成首次校准 */
} ABZ_Encoder_t;

/* ======================== API ======================== */

/**
 * @brief  初始化编码器，绑定外部输出变量并启动定时器编码器模式
 * @param  angle_rad    指向机械角度变量的指针（弧度，[0, 2π)�? * @param  velocity_rpm 指向机械转速变量的指针（RPM�? */
void abz_encoder_init(float *angle_rad, float *velocity_rpm);

/**
 * @brief  停止编码器定时器
 */
void abz_encoder_deinit(void);

/**
 * @brief  周期更新（在 FOC PWM 中断中调用，频率 = PWM_FREQ_HZ�? *         读取 CNT �?计算 delta �?更新角度和速度
 */
void abz_encoder_update(void);

/**
 * @brief  Z 相归零校准（�?Z 相外部中断回调中调用�? *         �?CNT 清零，重置单圈位置，标记已校�? *
 * 使用示例�? *   void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 *       if (GPIO_Pin == ABZ_Z_Pin) {
 *           abz_encoder_set_zero();
 *       }
 *   }
 */
void abz_encoder_set_zero(void);

void abz_encoder_on_index(void);
void abz_encoder_force_zero(void);
void abz_encoder_set_zero_on_index(uint8_t enable);
uint8_t abz_encoder_consume_index(int32_t *position_cnt);
int32_t abz_encoder_get_position_cnt(void);
/**
 * @brief  获取校准状�? * @return 1 = 已通过 Z 相校准，0 = 尚未校准
 */
uint8_t abz_encoder_is_calibrated(void);

#ifdef __cplusplus
}
#endif

#endif /* ABZ_ENCODER_H */
