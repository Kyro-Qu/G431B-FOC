/**
 * @file    foc_motor.h
 * @brief   电机轴对象 foc_motor_t 与电流/速度/位置控制环（纯算法层）
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
#include "foc_traj.h"
#include "foc_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 上电校准结果（由校准状态机写入） */
typedef struct {
    uint8_t valid;                 /* 1 = 本次上电已校准成功 */
    uint8_t from_store;            /* 1 = direction/offset 来自 Flash 存储，
                                    * 校准可走快速索引搜索（免对齐吸附）。
                                    * 注意 valid 仍需等 Z 脉冲重建零点后才置 1 */
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
    foc_runtime_t runtime;
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
    float vel_track_pos_rad;       /* 速度模式低速位置轨迹参考 */
    float vel_start_boost_a;       /* 仅起步阶段生效的脱槽附加电流 */
    int8_t vel_start_sign;          /* 本次起步方向：-1/0/+1 */
    uint8_t vel_start_active;       /* 已脱槽后清零，防止稳态持续前馈 */
    uint16_t vel_start_release_cnt; /* 同向转动确认计数，过滤测速毛刺 */
    float id_ref;                  /* d 轴弱磁电流给定（快环输出，<=0） */
    float iq_ref;                  /* q 轴转矩电流给定（慢环输出） */
    float fw_integral;             /* 弱磁电压闭环反馈积分 */

    /* ---- 反馈 ---- */
    float theta_e;                 /* 当前使用的电角度 rad [0,2π) */
    float theta_mech;              /* 机械角 rad [0,2π) */
    float position_rad;            /* 连续多圈机械位置 rad */
    float velocity_rpm;            /* 诊断速度：编码器滑动拟合结果 */
    float velocity_observer_rpm;   /* 低延迟速度：角度 PLL 观察器结果 */
    float velocity_filt_rpm;       /* 中值+二阶低通后的控制速度反馈 */
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
    foc_speed_filter_t vel_filter;     /* 高速路径：可配置截止频率 */
    foc_speed_filter_t vel_filter_low; /* 低速路径：固定 15 Hz，兼顾平滑与相位裕量 */
    foc_lpf_t lpf_id;
    foc_lpf_t lpf_iq;
    /* 电流环反馈陷波器：剔除 2.5kHz 采样混叠假影（诊断 2026-09-02） */
    foc_notch_t notch_id;
    foc_notch_t notch_iq;

    /* ---- 无感观测器（在线对比/后续切换用，2026-09-03） ---- */
    foc_observer_t observer;
    ab_t v_ab_last;                /* 上一拍施加的 αβ 电压（观测器输入） */
    uint8_t obs_enabled;           /* 观测器使能（在线对比开关） */
    uint32_t obs_switch_ms;        /* 角度源切换时刻（HAL tick，豁免窗用） */
    float obs_blend;               /* 切换渐变权重 0=编码器 1=观测器
                                    * （离散跳变会 16° 相位阶跃 → BOR 掉电） */
    uint8_t obs_blending;          /* 渐变进行中标志 */

    /* ---- 位置模式轨迹规划 ---- */
    foc_traj_t traj;
    float traj_target_latch;       /* 已规划的目标（变化即触发重规划） */

    /* ---- 内部 ---- */
    float ol_angle_step;           /* 开环每拍电角度增量 */
    uint16_t slow_cnt;             /* 慢环分频计数 */
    uint16_t stall_cnt;            /* 堵转连续计数（慢环拍） */
    uint16_t stall_trip_ticks;     /* 堵转跳闸阈值（由超时 ms 换算） */

    /* 测试信号注入钩子：CALIB 状态且 pwm_hold==0 时快环每拍调用，
     * 供参数辨识（Rs/Ls 方波注入）等测试例程改写 v_openloop。
     * 平时必须为 NULL。 */
    void (*test_hook)(struct foc_motor *m);
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

/**
 * 在线设置速度反馈滤波截止频率。hz=0 完全直通；设置后以当前速度重置
 * 滤波状态，避免参数切换给速度 PI 制造假阶跃。
 */
void foc_motor_set_velocity_filter_hz(foc_motor_t *m, float hz);

/** 沿用旧 `vel lpf` 的时间常数接口；fc=1/(2*pi*tf)，tf=0 为直通。 */
void foc_motor_set_velocity_filter_tf(foc_motor_t *m, float tf);

/** 查询当前二阶低通截止频率；直通时返回 0。 */
float foc_motor_get_velocity_filter_hz(const foc_motor_t *m);

/* ======================== 命令接口 ======================== */

/** 使能输出进入 RUN（IDLE→RUN）。闭环模式要求已校准，否则返回 0 并锁存故障 */
uint8_t foc_motor_arm(foc_motor_t *m);

/** 停止输出回到 IDLE（RUN/CALIB→IDLE） */
void foc_motor_disarm(foc_motor_t *m);

/**
 * @brief 设置控制模式（会复位相关控制器状态）
 * @return 1 成功；0 被拒绝：CALIB 中不许切换，
 *         RUN 中开环→闭环要求已有有效校准（防止绕过 arm 的保护）
 */
uint8_t foc_motor_set_mode(foc_motor_t *m, foc_mode_t mode);

/** 设置目标值，语义随当前模式：V / A / RPM / rad */
void foc_motor_set_target(foc_motor_t *m, float value);

/** 设置电角度来源（开环推进 or 编码器+校准偏移） */
void foc_motor_set_angle_source(foc_motor_t *m, foc_angle_source_t src);

/* ---- 开环电压命令（V/f 模式与校准状态机使用） ---- */

/** 固定电角度 + 固定 vd/vq（"吸住转子"） */
void foc_motor_openloop_hold(foc_motor_t *m, float theta_e, float vd, float vq);

/** 以指定机械转速开环旋转 + 固定 vd/vq */
/** 开环旋转；rpm 使用编码器机械正方向，内部按 calib.direction 换算电角速度。 */
void foc_motor_openloop_spin(foc_motor_t *m, float rpm, float vd, float vq);

/* ======================== 故障处理 ======================== */

/** 立即关闭功率级并锁存故障（任何上下文可调用，包括中断） */
void foc_motor_fault(foc_motor_t *m, foc_fault_t fault);

/** 清除故障回到 IDLE（只有显式调用才能离开 FAULT） */
void foc_motor_clear_fault(foc_motor_t *m);

/* ---- 16kHz 故障黑匣子（纯 RAM 诊断） ---- */
/** 冻结/恢复快环逐拍记录（fault 自动冻结，clear_fault 自动恢复） */
void foc_motor_blackbox_freeze(void);
void foc_motor_blackbox_resume(void);
/** 导出环形缓冲：故障前 256 拍（16ms）的 iu/iw/theta_e/vq */
void foc_motor_blackbox_dump(float *out_u, float *out_w,
                             float *out_th, float *out_vq,
                             float *out_duty);
/** 1 = 已冻结（故障数据有效） */
uint8_t foc_motor_blackbox_active(void);

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
