/**
 * @file    foc_observer.h
 * @brief   【预留模块】无感磁链观测器 + PLL 转速跟踪（纯算法，暂未接入控制链）
 *
 * 来源：VESC (mcpwm_foc) 的非线性磁链观测器（Ortega 型）。
 * 这是 VESC 无感控制的核心，也是开源界公认效果最好的实现之一，
 * 按本工程代码风格重写并加中文注释，留作后续开发无感 FOC 使用。
 *
 * 原理速览：
 *   电机反电动势 e = dλ/dt（λ 是转子磁链在 αβ 系的投影）。
 *   观测器用电压方程积分估计磁链：
 *     x' = v - R·i + 修正项
 *   修正项用"磁链幅值应恒等于 λm"这个物理约束把估计拉回真值：
 *     err = λm² - |x - L·i|²
 *     x' += γ/2 · (x - L·i) · err
 *   转子电角度就是磁链矢量的方向：
 *     θe = atan2(x2 - L·iβ, x1 - L·iα)
 *
 *   PLL 再从 θe 里提取平滑的转速（比直接差分干净得多）：
 *     Δθ = wrap(θe - θpll)
 *     ω += Ki·Δθ·dt;  θpll += (ω + Kp·Δθ)·dt
 *
 * 未来接入方式（无感模式）：
 *   1. 快环里喂入 (vα, vβ)（上一拍的输出电压）和 (iα, iβ)（本拍电流）；
 *   2. 起动阶段用开环拖动（本工程已有 openloop_spin），转速足够后
 *      切换 angle_source 使用观测器角度；
 *   3. λm = 60/(√3·π·pole_pairs·KV·2)，或由 Ke 换算。
 */

#ifndef FOC_OBSERVER_H
#define FOC_OBSERVER_H

#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 配置 */
    float rs_ohm;          /* 相电阻 Ω */
    float ls_henry;        /* 相电感 H */
    float flux_wb;         /* 转子磁链幅值 λm (Wb) */
    float gamma;           /* 观测器增益（VESC 典型 1e5~1e8，与磁链平方成反比） */
    float pll_kp;          /* PLL 比例增益（典型 2000） */
    float pll_ki;          /* PLL 积分增益（典型 30000） */
    /* 状态 */
    float x1;              /* 磁链估计 α 分量 */
    float x2;              /* 磁链估计 β 分量 */
    float theta_e;         /* 估计电角度 [0, 2π) */
    float pll_theta;       /* PLL 跟踪角度 */
    float speed_e_rads;    /* 估计电角速度 rad/s（带符号） */
    float theta_offset_rad; /* 角度偏移补偿（切换实验用） */
} foc_observer_t;

/** 初始化：填参数并复位状态。gamma<=0 时自动取 VESC 推荐值 */
void foc_observer_init(foc_observer_t *obs,
                       float rs_ohm, float ls_henry, float flux_wb,
                       float pll_kp, float pll_ki, float gamma);

/** 复位状态（如重新起动前） */
void foc_observer_reset(foc_observer_t *obs);

/**
 * @brief 每个快环周期调用一次
 * @param v_ab  上一拍施加的 αβ 电压 (V)
 * @param i_ab  本拍测得的 αβ 电流 (A)
 * @param dt    周期 s
 * @return      估计电角度 [0, 2π)
 */
float foc_observer_update(foc_observer_t *obs,
                          const ab_t *v_ab, const ab_t *i_ab, float dt);

/** 估计的电角速度 rad/s（PLL 输出，带符号） */
float foc_observer_speed_e_rads(const foc_observer_t *obs);

#ifdef __cplusplus
}
#endif

#endif /* FOC_OBSERVER_H */
