/**
 * @file    foc_app.h
 * @brief   应用层：多轴电机对象管理、初始化时序、任务调度与按键交互
 *
 * 职责边界（参考 README 分层原则）：
 *   - App 负责：拥有电机对象数组、上电初始化顺序、状态机跳转、
 *     按键/串口命令入口、慢速健康监测；
 *   - App 不负责：任何控制算法（在 Core/foc_motor）、
 *     任何寄存器操作（在 HAL/Driver）。
 *
 * 中断链路（轴 0）：
 *   TIM1 更新中断 ──► current_shunt_tim_update_irq()   (提交 ADC 注入上下文)
 *   ADC1_2 中断  ──► current_shunt_adc_irq()           (读取并重构三相电流)
 *                     └─成功─► foc_app_isr_current_loop()
 *                                ├─► foc_motor_fast_loop(轴0)   16 kHz
 *                                ├─► foc_motor_fast_loop(轴1)   (虚拟轴)
 *                                └─► foc_telemetry_isr_tick()   1 kHz 分频
 */

#ifndef FOC_APP_H
#define FOC_APP_H

#include "foc_motor.h"
#include "foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 全部电机轴对象（轴 0 = 本板硬件，轴 1 = 虚拟演示轴） */
extern foc_motor_t g_foc_motors[FOC_NUM_AXES];

/** 轴 0 状态镜像（Keil Watch / 断电诊断记录用） */
extern volatile uint8_t g_foc_state_diag;

/** 开环 V/F 的目标与斜坡后实际给定（编码器机械正方向）。 */
extern volatile float g_m0_openloop_vq;
extern volatile float g_m0_openloop_rpm;
extern volatile float g_m0_openloop_vq_applied;
extern volatile float g_m0_openloop_rpm_applied;
extern volatile float g_m0_vf_slope_v_per_rpm;
extern volatile float g_m0_vf_vq_target;

/** 清除轴0 V/F 目标和斜坡状态；停机、切模式和故障恢复时调用。 */
void foc_app_vf_reset_commands(void);

/** 上电初始化：电机对象 → 编码器 → 电流采样 → PWM 定时器 → 零偏校准 → 通信 */
void foc_app_init(void);

/** 电流采样完成中断入口（16 kHz，由 ADC ISR 调用） */
void foc_app_isr_current_loop(void);

/** 主循环任务：校准状态机、串口命令、健康监测 */
void foc_app_task(void);

/** 按键入口：IDLE→(校准→)RUN→IDLE 循环 */
void foc_app_on_key(void);

/** 取轴对象指针（idx 越界返回轴 0） */
foc_motor_t *foc_app_motor(uint8_t idx);

/** 轴 0 状态（供板级检查点记录） */
uint8_t foc_app_diag_state(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_APP_H */
