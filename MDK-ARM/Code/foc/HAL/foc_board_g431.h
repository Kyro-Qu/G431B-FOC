/**
 * @file    foc_board_g431.h
 * @brief   STM32G431 板级绑定层：把本板硬件包装成 Core 层的三大接口表
 *
 * 这一层是"硬件说明书"：
 *   轴 0（真实硬件）：
 *     PWM    -> TIM1 三相互补（CH1/PC13N, CH2/PA12N, CH3/PB15N），
 *              中心对齐，CH4 生成 ADC 触发（OC4REF -> TRGO）
 *     电流   -> OPAMP1/2/3 PGAx16 + ADC1/ADC2 注入组（current_shunt 驱动）
 *     传感器 -> TIM4 编码器模式 ABZ（abz_encoder 驱动）
 *   轴 1（虚拟轴，FOC_NUM_AXES >= 2 时存在）：
 *     所有硬件函数为空实现，仅用于演示/验证双轴调度框架。
 *     接入真实第二套功率级时，把 m1_* 函数换成对应外设操作即可，
 *     Core / App 层代码一行都不用改。
 *
 * 移植到其它板卡：只需要重写本文件（和 CubeMX 外设初始化）。
 */

#ifndef FOC_BOARD_G431_H
#define FOC_BOARD_G431_H

#include <stdint.h>
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

/* ---- 系统级稳定性设施 ---- */

/** 快环 CPU 占用统计（DWT 周期计数，status 命令显示） */
typedef struct {
    volatile uint32_t last_cycles;  /* 最近一拍快环执行周期数 */
    volatile uint32_t max_cycles;   /* 上电以来最大值 */
    volatile float load_pct;        /* 最近一拍占用率 %（预算 = 一个 PWM 周期） */
} foc_cpu_diag_t;

extern foc_cpu_diag_t g_foc_cpu_diag;

/** DWT 周期计数器使能（CPU 统计用），上电调用一次 */
void foc_board_dwt_init(void);
/** 读当前 DWT 周期计数 */
uint32_t foc_board_cycles(void);
/** 提交一次快环执行周期数（在 ISR 出口调用） */
void foc_board_cpu_sample(uint32_t cycles);

/** 独立看门狗启动（IWDG，LSI 时钟，一旦启动不可关闭；
 *  已配置调试器断点冻结）。在所有阻塞初始化完成后调用 */
void foc_board_watchdog_init(uint32_t timeout_ms);
/** 喂狗（主循环每圈调用） */
void foc_board_watchdog_kick(void);

/* ---- 母线电压实时采样接口（PA0 / ADC1_IN1） ---- */
typedef struct {
    volatile uint16_t raw_adc;       /* 最新 ADC1 原始采样值 0..4095 */
    volatile float    voltage_v;     /* 一阶低通滤波后的母线电压真值 V */
    volatile uint8_t  valid;         /* 1=已完成至少一次有效转换 */
    volatile uint32_t sample_count;  /* 采样累计计数 */
} foc_vbus_diag_t;

extern volatile foc_vbus_diag_t g_foc_vbus_diag;

/** 初始化 PA0 模拟输入及 ADC1 Regular 序列（保持电流注入序列不变） */
void foc_board_vbus_init(void);
/** 周期性触发并更新母线电压（主循环 100Hz 轮询调用） */
void foc_board_vbus_update(void);
/** 读取当前实时测得的母线电压 V（未就绪时回退返回 FOC_UDC_V 标称值） */
float foc_board_get_vbus_v(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_BOARD_G431_H */
