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
    FOC_IDENT_LD_MEASURE,  /* 在 d 轴注入方波求 Ld */
    FOC_IDENT_LQ_MEASURE,  /* 在 q 轴注入方波求 Lq */
    FOC_IDENT_PP_ALIGN,    /* 极对数测量：初始电角度吸附对齐 */
    FOC_IDENT_PP_SPIN,     /* 极对数测量：已知电角度开环旋转累积机械位移 */
    FOC_IDENT_PP_SETTLE,   /* 极对数到磁链过渡：撤压平稳缓冲 */
    FOC_IDENT_FLUX_SPIN,   /* 磁链与 Ke 测量：开环稳态旋转采样 Vq/Iq/we */
    FOC_IDENT_DONE,
    FOC_IDENT_FAIL
} foc_ident_state_t;

typedef enum {
    FOC_IDENT_MODE_FULL = 0, /* 全套辨识: Rs -> Ls -> Pp -> Flux -> Done */
    FOC_IDENT_MODE_RS_LS,   /* 仅辨识相电阻与电感 (静止转子) */
    FOC_IDENT_MODE_PP,      /* 仅辨识极对数 (低速微转) */
    FOC_IDENT_MODE_FLUX,    /* 仅辨识磁链与 Ke (开环中速旋转) */
    FOC_IDENT_MODE_LD_LQ    /* 辨识 Ld/Lq 及凸极性差值与比值 */
} foc_ident_mode_t;

typedef struct {
    uint8_t valid;
    uint8_t has_rs_ls;
    uint8_t has_pp;
    uint8_t has_flux;
    uint8_t has_ld_lq;
    float rs_ohm;
    float ls_henry;
    float ld_henry;        /* d 轴高频电感 (H) */
    float lq_henry;        /* q 轴高频电感 (H) */
    float delta_l_henry;   /* 凸极电感差 Lq - Ld (H) */
    float saliency_ratio;  /* 凸极率 (Lq - Ld) / Ld */
    float test_current_a;  /* Rs 测量时的实际平均电流 */
    float pole_pairs;      /* 辨识出的极对数 (取整) */
    float pp_calc_raw;     /* 未取整的原始计算值 */
    float pp_residual;     /* 极对数相对残差 (|raw-round|/round) */
    float flux_linkage_wb; /* 永磁磁链 (Wb) */
    float ke_v_krpm;       /* 反电势常数 (V_peak_line / krpm) */
} foc_ident_result_t;

/** 开始全套测量（轴需 IDLE 且有电流采样）。结果通过串口打印 */
void foc_ident_start(foc_motor_t *m);

/** 启动指定模式的辨识 */
void foc_ident_start_mode(foc_motor_t *m, foc_ident_mode_t mode);

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
