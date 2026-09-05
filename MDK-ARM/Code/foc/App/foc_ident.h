/**
 * @file    foc_ident.h
 * @brief   电机参数自动测量（Rs / Ls），串口命令 `ident` 触发
 *
 * 来源：ODrive 的 measure_phase_resistance / measure_phase_inductance，
 * VESC (foc_measure_res/ind) 与 MESC 也用同样思路。
 *
 * 为什么值得要？电流环增益自整定（Kp=Ls·ω，Ki=Rs·ω）依赖准确的
 * Rs/Ls，而多数用户拿到电机根本查不到这两个参数。测一次填进
 * foc_config.h（或 `ident apply` 直接应用），闭环就稳了。
 *
 * 测量原理：
 *   Rs：固定电角度（θe=0）给 d 轴电压，积分控制器缓慢升压直到
 *       d 轴电流达到目标值（默认 1A），稳定后取平均：Rs = V̄d / Īd。
 *       转子被吸住不动，无反电动势干扰。
 *   Ls：继续锁定转子，d 轴注入 ±V 方波（每个快环拍翻转极性），
 *       电流按 di/dt=(V-Rs·i)/L 三角波动。按极性符号累计电压伏秒
 *       与电流增量：Ls = Σ(V-Rs·i)·dt / Σdi。已用测得的 Rs 修正压降。
 *
 * 安全：全程使用校准级低电流阈值（软 1.5A/硬 3.0A），过流/采样失效
 * 立即断 PWM；测量完成或失败都回到 IDLE 并清理测试钩子。
 *
 * 使用（串口）：
 *   ident       开始测量（需 IDLE 且电流采样就绪）
 *   ident apply  把上次测量结果应用到本轴参数并重整定电流环（RAM 内，
 *           重启失效；长期使用请写入 foc_config.h）
 */

#ifndef FOC_IDENT_H
#define FOC_IDENT_H

#include <stdint.h>
#include "foc_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FOC_IDENT_IDLE = 0,
    FOC_IDENT_BOOTSTRAP,   /* 自举电容充电 */
    FOC_IDENT_NEUTRAL,     /* 中点 PWM 稳定 */
    FOC_IDENT_RS_RAMP,     /* 升压至目标电流 */
    FOC_IDENT_RS_MEASURE,  /* 平均 V/I 求 Rs */
    FOC_IDENT_LS,          /* 方波注入求 Ls（快环钩子执行） */
    FOC_IDENT_DONE,
    FOC_IDENT_FAIL
} foc_ident_state_t;

typedef struct {
    uint8_t valid;
    float rs_ohm;
    float ls_henry;
    float test_current_a;  /* Rs 测量时的实际平均电流 */
} foc_ident_result_t;

/** 开始测量（轴需 IDLE 且有电流采样）。结果通过串口打印 */
void foc_ident_start(foc_motor_t *m);

/** 状态机任务：主循环周期调用 */
void foc_ident_task(void);

uint8_t foc_ident_is_active(void);
foc_ident_state_t foc_ident_get_state(void);
const foc_ident_result_t *foc_ident_get_result(void);

/** 把测量结果应用到轴参数并重整定电流环。成功返回 1 */
uint8_t foc_ident_apply(foc_motor_t *m);

#ifdef __cplusplus
}
#endif

#endif /* FOC_IDENT_H */
