/**
 * @file    foc_board_g431.h
 * @brief   STM32G431 板级绑定层：把本板硬件包装成 Core 层的三大接口表
 *
 * 这一层是"硬件说明书"：
 *   轴 0（真实硬件）：
 *     PWM    → TIM1 三相互补（CH1/PC13N, CH2/PA12N, CH3/PB15N），
 *              中心对齐，CH4 生成 ADC 触发（OC4REF → TRGO）
 *     电流   → OPAMP1/2/3 PGA×16 + ADC1/ADC2 注入组（current_shunt 驱动）
 *     传感器 → TIM4 编码器模式 ABZ（abz_encoder 驱动）
 *   轴 1（虚拟轴，FOC_NUM_AXES >= 2 时存在）：
 *     所有硬件函数为空实现，仅用于演示/验证双轴调度框架。
 *     接入真实第二套功率级时，把 m1_* 函数换成对应外设操作即可，
 *     Core / App 层代码一行都不用改。
 *
 * 移植到其它板卡：只需要重写本文件（和 CubeMX 外设初始化）。
 */

#ifndef FOC_BOARD_G431_H
#define FOC_BOARD_G431_H

#include "foc_types.h"
#include "foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** PWM 输出阶段（Keil Watch 观察 + 断电诊断记录）：
 *  0 = 全关，1 = 低边自举充电，2 = 正常互补 PWM */
extern volatile uint8_t g_foc_pwm_stage;
extern volatile uint8_t g_foc_pwm_enabled;

/** 轴 0 接口表 */
extern const foc_driver_if_t  g_board_m0_driver;
extern const foc_current_if_t g_board_m0_current;
extern const foc_sensor_if_t  g_board_m0_sensor;

#if FOC_NUM_AXES >= 2
/** 轴 1 接口表（虚拟） */
extern const foc_driver_if_t g_board_m1_driver;
#endif

/**
 * @brief 功率级定时器安全上电（原 MCSDK 验证流程）：
 *        MOE 保持关闭、比较值预置中点、计数器起始位置对齐、
 *        启动 CH4 触发通道、装配六路输出但不使能功率。
 *        必须在 current_shunt_init() 之后、零偏校准之前调用。
 */
void foc_board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_BOARD_G431_H */
