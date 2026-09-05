/**
 * @file    foc_hfi.h
 * @brief   【预留区】HFI 高频注入低速无感（MESC/VESC 思路，教学版）
 *
 * ⚠ 未加入 Keil 工程。接入步骤见《Docs/09_预留特性接入手册.md》第 2 章。
 * ⚠ 五个预留特性里 HFI 是最依赖实验调试的一个，先读完 09 手册的
 *   预期管理再动手。
 *
 * 原理（一段话版）：
 *   磁链观测器靠反电动势工作，静止/低速时反电动势≈0 → 失明。
 *   HFI 换一条路：往 d 轴打高频小电压方波，如果我们猜的 d 轴方向
 *   是对的，高频电流响应只出现在 d 轴；猜偏了角度 Δθ，响应就会
 *   "漏"到 q 轴，漏的量 ∝ (Lq-Ld)·sin(2Δθ)。把 q 轴的高频响应
 *   当误差信号送进 PLL，把角度锁到误差为零 → 静止也能知道转子方向。
 *
 * 先决条件（决定你的电机能不能用 HFI）：
 *   凸极性 Lq/Ld 需要 > 约 1.05。表贴电机（SPM）凸极性天生弱，
 *   但饱和效应通常也能提供一点（本工程 DJI 2312S 未验证）。
 *   测法：`ident` 测 Ls 时分别在 θe=0 和 θe=90° 各测一次（09 手册）。
 *
 *   另外 HFI 只能分辨 d 轴的"轴线"，不知道 N/S 极性（180° 模糊），
 *   教学版用"d 轴正负脉冲电流差"做一次极性判别。
 *
 * 注入频率：每 2 拍翻转一次 → 4kHz 方波（@16kHz 快环），
 *   与 foc_ident 的 Ls 测量共用"每极性 2 拍"的抗预装载时序。
 */

#ifndef FOC_HFI_H
#define FOC_HFI_H

#include "../Core/foc_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FOC_HFI_IDLE = 0,
    FOC_HFI_POLARITY,    /* 极性判别（区分 N/S，消 180° 模糊） */
    FOC_HFI_TRACK        /* 正常跟踪：θ 由 HFI PLL 提供 */
} foc_hfi_state_t;

typedef struct {
    /* 配置 */
    float v_inject;       /* 注入电压幅值 V（0.5~2V 起步实验） */
    float pll_kp;         /* 角度 PLL 增益 */
    float pll_ki;
    /* 状态 */
    foc_hfi_state_t state;
    float theta_e;        /* HFI 估计的电角度 */
    float speed_e_rads;   /* PLL 估计的电角速度 */
    /* 内部 */
    int8_t pol;           /* 当前注入极性 */
    uint8_t phase;        /* 0=过渡拍 1=测量拍（抗 CCR 预装载） */
    float iq_prev;        /* 上一拍 q 轴电流 */
    float demod;          /* 解调出的误差信号（遥测观察用） */
    uint16_t pol_cnt;     /* 极性判别累计拍 */
    float pol_acc;        /* 极性判别累计量 */
} foc_hfi_t;

void foc_hfi_init(foc_hfi_t *h, float v_inject, float pll_kp, float pll_ki);

/** 从静止开始 HFI（先跑极性判别，几十 ms 后进入 TRACK） */
void foc_hfi_start(foc_hfi_t *h, float theta_guess);

/**
 * @brief 快环每拍调用（在电流采样之后、电压命令之前，接入点见手册）。
 *        内部：解调上一拍响应 → PLL 更新角度 → 写入本拍注入电压。
 * @param m 电机对象（读 i_dq，把注入电压叠加到 m->v_dq 上）
 * @return 当前估计电角度
 */
float foc_hfi_update(foc_hfi_t *h, foc_motor_t *m);

#ifdef __cplusplus
}
#endif

#endif /* FOC_HFI_H */
