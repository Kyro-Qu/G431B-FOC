/**
 * @file    foc_sensorless_bench.h
 * @brief   无感观测器多算法影子初测与性能评估平台 (Sensorless Benchmark Arena)
 *
 * 包含四套算法实现/变体：
 *   1. Obs 1: Ortega 非线性磁链观测器 + 二阶临界阻尼 PLL
 *   2. Obs 2: VESC 单向约束鲁棒磁链观测器 (Benjamin Vedder 约束版)
 *   3. Obs 3: 简化状态观测器 (Simplified STO + 软件 PLL 锁相)
 *   4. Obs 4: 简化状态观测器 + STM32G4 真实硬件片上 CORDIC 协处理器求相
 *
 * 每个观测器具备独立的 direction、offset、收敛门槛检测与稳态统计窗口！
 */

#ifndef FOC_SENSORLESS_BENCH_H
#define FOC_SENSORLESS_BENCH_H

#include "foc_types.h"
#include "foc_motor.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float theta_e;          /* 估计电角度 [0, 2pi) */
    float speed_rpm;        /* 估计机械转速 RPM */
    float err_deg;          /* 当前拍与编码器真值的瞬时电角度误差 (-180, 180] */
    float err_abs_deg;      /* 绝对误差 (度) */
    float err_sum_deg;      /* 稳态累积绝对误差，用于求均值 */
    float err_sq_sum;       /* 稳态误差平方和，用于求 RMS 抖动 */
    float err_peak_deg;     /* 统计窗口内最大峰值误差 */
    uint32_t samples;       /* 稳态统计有效采样数 */
    uint32_t exec_cycles;   /* Cortex-M4 单拍执行真实 CPU 周期数 (DWT) */

    /* 磁链圆轨迹与正交性诊断 */
    float flux_mag;         /* 当前转子磁链估算幅值 (Wb) */
    float flux_center_a;    /* 磁链圆中心 Alpha 漂移估算 (Wb) */
    float flux_center_b;    /* 磁链圆中心 Beta 漂移估算 (Wb) */

    /* 独立坐标对齐与收敛状态 */
    int8_t direction;       /* 观测器独立极性对齐 (+1 或 -1) */
    float theta_offset;     /* 观测器独立零点偏置 (rad) */
    uint8_t converged;      /* 是否收敛锁定 (0=失锁/脱锁, 1=已稳定锁定) */
    uint32_t unlock_count;  /* 窗口内失锁次数计数 */
} foc_bench_obs_metrics_t;

typedef struct {
    uint8_t enabled;        /* 评测基准使能标记 */
    uint8_t steady_state;   /* 是否处于稳态采样窗口 (丢弃启停与变速过渡) */
    uint8_t obs_deadtime_comp_enable; /* 观测器输入端电压死区修正使能 (默认 1) */
    float deadtime_comp_v;  /* 死区补偿电压幅值 (默认 0.17V) */
    uint32_t total_samples; /* 总统计点数 */

    /* 5 套观测器指标 */
    foc_bench_obs_metrics_t obs1_ortega;
    foc_bench_obs_metrics_t obs2_vesc;
    foc_bench_obs_metrics_t obs3_sto_pll;
    foc_bench_obs_metrics_t obs4_sto_cordic;
    foc_bench_obs_metrics_t obs5_hfi;

    /* 内部算法状态 */
    /* Obs 1: Ortega */
    float od_flux_a;
    float od_flux_b;
    float od_pll_pos;
    float od_pll_vel;

    /* Obs 2: VESC Constrained */
    float vesc_x1;
    float vesc_x2;
    float vesc_pll_pos;
    float vesc_pll_vel;

    /* Obs 3: Simplified STO */
    float sto_i_est_a;
    float sto_i_est_b;
    float sto_bemf_a;
    float sto_bemf_b;
    float sto_pll_pos;
    float sto_pll_vel;

    /* Obs 4: Hardware CORDIC Result */
    float cordic_theta;
    float cordic_speed_rpm;
    float cordic_pll_pos;
    float cordic_pll_vel;

    /* Obs 5: HFI (High Frequency Injection 连续脉动高频注入影子模块)
     * 状态归档:
     *   Implementation: VERIFIED (代码实现与影子解调框架完备)
     *   Feasibility on DJI 2312S + MatchStick: NOT_FEASIBLE_FOR_DJI2312S_MATCHSTICK
     *   Reason: 表贴弱凸极 (xi ≈ -6.64%) + 采样硬件量化门槛 (1 LSB ≈ 29.3mA > 理论信号 21.4mA)
     *   Rule: 严格保持影子模式，严禁接入实际 FOC 换相控制角！
     */
    uint8_t  hfi_enabled;          /* HFI 注入使能 */
    float    hfi_inj_volt;         /* 注入方波幅值 V (默认 1.0V) */
    int8_t   hfi_inj_pol;          /* 本拍注入极性 (+1 或 -1) */
    float    hfi_v_inj_alpha;      /* 本拍输出到 Alpha 的注入电压 V */
    float    hfi_v_inj_beta;       /* 本拍输出到 Beta 的注入电压 V */
    float    hfi_i_prev_q;         /* 上一拍估计坐标系下的 Iq 电流 */
    float    hfi_demod_err_filt;   /* 滤波后的解调角度误差信号 */
    float    hfi_pll_pos;          /* HFI PLL 估计角度 */
    float    hfi_pll_vel;          /* HFI PLL 估计电角速度 (rad/s) */
    float    hfi_pll_kp;           /* PLL 比例增益 */
    float    hfi_pll_ki;           /* PLL 积分增益 */
    float    hfi_confidence;       /* HFI 置信度 (0.0~1.0) */
    float    hfi_carrier_mag;      /* 载波误差信号模长 (用于信噪比与失锁判定) */
    float    hfi_iq_ripple;        /* 采样周期内的 Iq 高频纹波峰峰值 */

    /* 单独快照记录（供诊断解剖） */
    float diag_v_alpha;
    float diag_v_beta;
    float diag_i_alpha;
    float diag_i_beta;
    float diag_od_eta_a;
    float diag_od_eta_b;
    float diag_od_theta_raw;
    float diag_sto_bemf_a;
    float diag_sto_bemf_b;
    float diag_sto_bemf_mag;
    float diag_enc_theta;
} foc_sensorless_bench_t;

extern foc_sensorless_bench_t g_sensorless_bench;

/** 初始化评测平台（初始化硬件 CORDIC 外设与 DWT 计数器） */
void foc_sensorless_bench_init(foc_motor_t *m);

/** 硬件 CORDIC 极速求解反正切相位 [-PI, PI] (Q1.31) */
float foc_cordic_calc_phase(float y, float x);

/** 复位统计计数与指标累加器 */
void foc_sensorless_bench_reset_metrics(void);

/** 设置是否进入稳态统计区间（由测试脚本在转速平稳后标记） */
void foc_sensorless_bench_set_steady(uint8_t steady);

/** 启停评测 */
void foc_sensorless_bench_enable(uint8_t en);

/** 自动标定当前转速下各算法的相位 offset */
void foc_sensorless_bench_auto_align(foc_motor_t *m);

/** HFI 影子模块启停与注入电压设置 */
void foc_sensorless_bench_hfi_enable(uint8_t en, float inj_volt);

/** 16kHz 快环影子比对更新 */
void foc_sensorless_bench_update(foc_motor_t *m, float v_alpha, float v_beta,
                                 float i_alpha, float i_beta, float theta_enc, float dt);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SENSORLESS_BENCH_H */
