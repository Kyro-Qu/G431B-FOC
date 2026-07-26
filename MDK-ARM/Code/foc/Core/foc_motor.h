/**
 * @file    foc_motor.h
 * @brief   电机轴对象 foc_motor_t 与级联控制环（纯算法层）
 *
 * ============================ 设计说明 ============================
 *
 * 一个 foc_motor_t 就是一个完整独立的电机控制轴（参考 ODrive Axis）：
 *   - 绑定：PWM 驱动 / 电流采样 / 位置传感器三个接口表
 *   - 配置：电机参数 + 控制环参数
 *   - 状态：状态机、目标值、控制器、反馈量、诊断量
 *
 * 双电机 = 两个 foc_motor_t 实例，各自绑定各自的硬件接口表，
 * 各自的电流采样完成中断里调用各自的 foc_motor_fast_loop()。
 * 算法层没有任何全局状态，天然支持任意轴数。
 *
 * ============================ 控制链路 ============================
 *
 *  位置模式:  target(rad)──►[位置P]──►vel_ref──►[速度PI]──►iq_ref─┐
 *  速度模式:  target(rpm)──►[斜坡]───────────►[速度PI]──►iq_ref─┤
 *  力矩模式:  target(A)────────────────────────────────►iq_ref─┤
 *                                                              ▼
 *                 快环 16kHz:  [Id PI]+[Iq PI]+解耦前馈──►vd,vq
 *                                                              ▼
 *  开环模式:  target(V)──────────────────────────────►vd,vq(直接)
 *                                                              ▼
 *                              反Park ──► SVPWM ──► 三相占空比 ──► 硬件
 *
 * 速度/位置环在快环内按 slow_div 分频执行（16kHz/16 = 1kHz），
 * 这是 VESC/ODrive 的调度方式：全部控制都在中断里跑，主循环只做
 * 通信和状态机，任何时刻控制时序都是确定的。
 */

#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H

#include "foc_types.h"
#include "foc_pid.h"
#include "foc_svm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 上电校准结果（由校准状态机写入） */
typedef struct {
    uint8_t valid;                 /* 1 = 本次上电已校准成功 */
    int8_t direction;              /* 编码器正方向 vs 电角度正方向：+1/-1 */
    float electrical_offset_rad;   /* 电角度偏移：θe = dir·pp·θm + offset */
} foc_calib_result_t;

/** 电机轴对象 */
typedef struct foc_motor {
    /* ---- 硬件绑定（初始化后不变） ---- */
    const foc_driver_if_t  *drv;
    const foc_current_if_t *cur;
    const foc_sensor_if_t  *sensor;   /* 可为 NULL（纯开环轴） */

    /* ---- 配置 ---- */
    foc_motor_params_t params;
    foc_ctrl_cfg_t cfg;
    float dt_fast;                 /* 快环周期 s（= 1/PWM频率） */
    uint16_t slow_div;             /* 慢环分频比（16k/16=1kHz） */

    /* ---- 状态机 ---- */
    volatile foc_state_t state;
    volatile foc_mode_t mode;
    volatile foc_angle_source_t angle_source;
    volatile uint8_t pwm_hold;     /* 1 = 快环不写 PWM（自举充电等特殊阶段） */
    foc_safety_diag_t safety;
    foc_calib_result_t calib;

    /* ---- 目标 ---- */
    volatile float target;         /* 语义随 mode：V / A / RPM / rad */
    dq_t v_openloop;               /* 开环&校准直接电压命令 */
    float vel_ref_rpm;             /* 斜坡后的速度给定 */
    float iq_ref;                  /* q 轴电流给定（慢环输出） */

    /* ---- 反馈 ---- */
    float theta_e;                 /* 当前使用的电角度 rad [0,2π) */
    float theta_mech;              /* 机械角 rad [0,2π) */
    float position_rad;            /* 连续多圈机械位置 rad */
    float velocity_rpm;            /* 机械转速（传感器滤波后） */
    float velocity_filt_rpm;       /* 慢环再滤波后的速度反馈 */
    abc_t i_abc;                   /* 三相电流 A */
    dq_t i_dq;                     /* 实测 dq 电流 A */
    dq_t i_dq_filt;                /* 低通后的 dq 电流（遥测/观察用） */

    /* ---- 输出 ---- */
    dq_t v_dq;                     /* 电流环输出电压 V */
    foc_svm_t svm;                 /* 最近一次调制结果 */

    /* ---- 控制器 ---- */
    foc_pid_t pid_id;
    foc_pid_t pid_iq;
    foc_pid_t pid_vel;
    foc_pid_t pid_pos;
    foc_lpf_t lpf_vel;             /* 速度反馈低通（慢环） */
    foc_lpf_t lpf_id;
    foc_lpf_t lpf_iq;

    /* ---- 内部 ---- */
    float ol_angle_step;           /* 开环每拍电角度增量 */
    uint16_t slow_cnt;             /* 慢环分频计数 */
} foc_motor_t;

/* ======================== 生命周期 ======================== */

/**
 * @brief 初始化电机轴对象并根据电机参数自整定电流环
 *
 * 电流环增益按带宽整定（ODrive 方式）：
 *   Kp = Ls · ω_bw    Ki = Rs · ω_bw
 * 电机的电气传函是 1/(Ls·s+Rs)，这样配的 PI 恰好把零点对消到
 * 极点上，闭环变成一阶惯性环节，带宽就是 ω_bw，不会震荡。
 */
void foc_motor_init(foc_motor_t *m,
                    const foc_driver_if_t *drv,
                    const foc_current_if_t *cur,
                    const foc_sensor_if_t *sensor,
                    const foc_motor_params_t *params,
                    const foc_ctrl_cfg_t *cfg,
                    float dt_fast,
                    uint16_t slow_div);

/** 快环入口：在该轴电流采样完成中断（16 kHz）中调用 */
void foc_motor_fast_loop(foc_motor_t *m);

/* ======================== 命令接口 ======================== */

/** 使能输出进入 RUN（IDLE→RUN）。闭环模式要求已校准，否则返回 0 并锁存故障 */
uint8_t foc_motor_arm(foc_motor_t *m);

/** 停止输出回到 IDLE（RUN/CALIB→IDLE） */
void foc_motor_disarm(foc_motor_t *m);

/** 设置控制模式（会复位相关控制器状态，运行中切换是安全的） */
void foc_motor_set_mode(foc_motor_t *m, foc_mode_t mode);

/** 设置目标值，语义随当前模式：V / A / RPM / rad */
void foc_motor_set_target(foc_motor_t *m, float value);

/** 设置电角度来源（开环推进 or 编码器+校准偏移） */
void foc_motor_set_angle_source(foc_motor_t *m, foc_angle_source_t src);

/* ---- 开环电压命令（V/f 模式与校准状态机使用） ---- */

/** 固定电角度 + 固定 vd/vq（"吸住转子"） */
void foc_motor_openloop_hold(foc_motor_t *m, float theta_e, float vd, float vq);

/** 以指定机械转速开环旋转 + 固定 vd/vq */
void foc_motor_openloop_spin(foc_motor_t *m, float rpm, float vd, float vq);

/* ======================== 故障处理 ======================== */

/** 立即关闭功率级并锁存故障（任何上下文可调用，包括中断） */
void foc_motor_fault(foc_motor_t *m, foc_fault_t fault);

/** 清除故障回到 IDLE（只有显式调用才能离开 FAULT） */
void foc_motor_clear_fault(foc_motor_t *m);

/* ======================== 辅助 ======================== */

/** 临时覆盖过流阈值（校准期间用小电流阈值），恢复用 foc_motor_restore_current_limits */
void foc_motor_override_current_limits(foc_motor_t *m, float soft_a, float hard_a);
void foc_motor_restore_current_limits(foc_motor_t *m);

/** 由编码器机械角计算电角度（要求 calib.valid） */
float foc_motor_encoder_theta_e(const foc_motor_t *m);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MOTOR_H */
