/**
 * @file    foc_angle_manager.h
 * @brief   反馈角度仲裁与无感后台监控管理器
 *
 * 负责：
 *  - 编码器主控与健康状态评估 (NORMAL / SUSPECT / FAILED)
 *  - 无感观测器后台持续监控 (Lock、Confidence、误差判别)
 *  - 无感算法选型配置 (VESC / Ortega / STO)
 *  - 接管资格判定与平滑过渡 (Blend 加权插值)
 *  - 统一向快环输出换相电角度 theta_control
 */

#ifndef FOC_ANGLE_MANAGER_H
#define FOC_ANGLE_MANAGER_H

#include "foc_types.h"
#include "foc_motor.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 运行模式 */
typedef enum {
    FOC_FEEDBACK_SENSORED_PRIMARY   = 0, /* 默认有感主控，无感后台监控 */
    FOC_FEEDBACK_SENSORLESS_PRIMARY = 1, /* 手动纯无感模式 (低速开环/HFI -> 中高速观测器) */
    FOC_FEEDBACK_AUTO_FALLBACK      = 2  /* 自动接管模式 (编码器故障且无感具备资格时自动切入) */
} foc_feedback_mode_t;

/** 角度仲裁与接管状态机 */
typedef enum {
    FOC_ANGLE_SENSORED              = 0, /* 纯编码器控制中 */
    FOC_ANGLE_SENSORLESS_STARTUP    = 1, /* 纯无感启动中 (开环拖动/HFI) - 兼容保留 */
    FOC_ANGLE_SENSORLESS_CANDIDATE  = 2, /* 无感达到接管资格候选态 (连续稳定达标) */
    FOC_ANGLE_BLEND_TO_SENSORLESS   = 3, /* 平滑过渡切入无感 (blend: 0.0 -> 1.0) */
    FOC_ANGLE_SENSORLESS            = 4, /* 纯无感观测器闭环控制中 (Auto Fallback) */
    FOC_ANGLE_BLEND_TO_SENSORED     = 5, /* 平滑过渡切回编码器 (blend: 1.0 -> 0.0) */
    FOC_ANGLE_REJECTED              = 6, /* 接管请求被拒绝 (无感不满足准入条件) */
    FOC_ANGLE_SAFE_STOP             = 7, /* 故障停机保护 */

    /* 第四阶段：Sensorless Primary 纯无感独立启动状态机 */
    FOC_ANGLE_SENSORLESS_IF_START   = 10, /* I/F 初始直流吸附/对齐 (Align) */
    FOC_ANGLE_SENSORLESS_IF_ACCEL   = 11, /* I/F 虚拟电角度开环加速拖动 (Ramp) */
    FOC_ANGLE_SENSORLESS_OBS_LOCKING= 12, /* 到达目标切入转速，等待 VESC 观测器连续硬锁定 */
    FOC_ANGLE_SENSORLESS_BLEND      = 13, /* 开环角向 VESC 观测角平滑无顿挫过渡 (100~200ms) */
    FOC_ANGLE_SENSORLESS_RUN        = 14, /* 纯无感闭环稳定运行 (编码器仅作后台诊断参考) */
    FOC_ANGLE_SENSORLESS_LOST       = 15  /* 观测器失锁确认态 (受控关断 PWM，记录快照，转 SAFE_STOP) */
} foc_angle_state_t;

/** 编码器健康评估等级 */
typedef enum {
    ENCODER_HEALTH_NORMAL           = 0, /* 编码器正常 */
    ENCODER_HEALTH_SUSPECT          = 1, /* 疑似异常 (单次或短时毛刺/不一致，观察防抖中) */
    ENCODER_HEALTH_FAILED           = 2  /* 确认故障 (持续无信号、严重跳变或速度不匹配) */
} foc_encoder_health_t;

/** 无感观测器主算法选型 */
typedef enum {
    SENSORLESS_ALGO_VESC            = 0, /* VESC 约束磁链观测器 (第一主接管候选) */
    SENSORLESS_ALGO_ORTEGA          = 1, /* Ortega 非线性磁链观测器 (第二主接管候选) */
    SENSORLESS_ALGO_STO             = 2  /* STO 状态观测器 (中高速备选) */
} foc_sensorless_algo_t;

/** 故障与回退原因 */
typedef enum {
    FALLBACK_NONE                   = 0,
    FALLBACK_ENC_TIMEOUT            = 1, /* 编码器计数长时间未更新 */
    FALLBACK_ENC_STEP_FAULT         = 2, /* 编码器角度异常阶跃 */
    FALLBACK_ENC_SPEED_FAULT        = 3, /* 编码器速度超限或方向异常 */
    FALLBACK_OBS_UNLOCK             = 4, /* 无感观测器失锁 (unlock) */
    FALLBACK_OBS_SPEED_ERR          = 5, /* 无感速度估计严重偏差 */
    FALLBACK_SPEED_TOO_LOW          = 6, /* 转速低于无感安全下限 */
    FALLBACK_MANUAL_REQUEST         = 7  /* 用户指令请求回退 */
} foc_fallback_reason_t;

/** 故障注入模拟类型 */
typedef enum {
    INJECT_NONE         = 0, /* 无故障注入 */
    INJECT_FREEZE       = 1, /* 编码器角度冻结，停止更新 */
    INJECT_STEP         = 2, /* 编码器注入角度阶跃跳变 (度) */
    INJECT_SPEED_SPIKE  = 3  /* 编码器速度超限虚假跳变 (RPM) */
} foc_enc_fault_inject_t;

/** 角度管理器综合遥测与状态结构体 */
typedef struct {
    /* 运行配置 */
    foc_feedback_mode_t   mode;             /* 当前配置的反馈模式 */
    foc_sensorless_algo_t active_algo;      /* 当前指定的无感观测器算法 */
    float                 enter_speed_rpm;  /* 无感接管准入切入转速 (默认 800 RPM) */
    float                 exit_speed_rpm;   /* 无感安全切出转速 (默认 500 RPM) */
    float                 blend_time_s;     /* 平滑切换过渡时间 (默认 0.2s) */

    /* 实时仲裁状态 */
    foc_angle_state_t     state;            /* 当前角度状态机 */
    foc_encoder_health_t  enc_health;       /* 编码器健康状态 */
    foc_fallback_reason_t fallback_reason;  /* 最近一次回退或故障原因 */
    float                 handover_blend;   /* 当前平滑过渡加权因子 [0.0, 1.0] (0=编码器, 1=无感) */
    float                 handover_delta_rad;/* 切换瞬间记录的相位差 (rad)，用于平滑衰减，避免把冻结角度卷入加权 */

    /* 电角度与速度实时监控值 */
    float                 theta_encoder;    /* 编码器当前电角度 [0, 2pi) */
    float                 theta_sensorless; /* 选定无感算法估计电角度 [0, 2pi) */
    float                 theta_control;    /* 最终仲裁输出给 Park 变换的控制电角度 [0, 2pi) */
    float                 speed_control;    /* 最终仲裁输出给速度环的控制转速 (RPM) */
    float                 angle_error_deg;  /* 瞬时角度差 (|theta_obs - theta_enc|, wrap 到 180 度内) */
    float                 speed_encoder_rpm;/* 编码器物理转速 */
    float                 speed_obs_rpm;    /* 选定无感算法估计转速 */
    float                 speed_obs_filt_rpm;/* 无感算法估计转速滤波值 (50Hz 带宽，与编码器滤波对齐) */
    float                 speed_error_rpm;  /* 转速估计绝对误差 */

    /* 无感观测器长窗口 (500ms / 8000拍) 统计与品质监控 */
    uint8_t               obs_lock;         /* 观测器瞬时锁定标志 (1=锁定, 0=脱锁) */
    float                 conf_inst;        /* 瞬时保护置信度 [0.0 ~ 1.0] (单拍保护与瞬态监测) */
    float                 conf_window;      /* 窗口准入置信度 [0.0 ~ 1.0] (500ms 滑动窗口综合品质) */
    float                 obs_confidence;   /* 综合置信度评分 (映射为 conf_window 保持向下兼容) */
    uint32_t              qualified_cycles; /* 动态容忍准入资格评分 (支持爬坡轻度纹波容忍，需>=8000拍/500ms) */
    uint32_t              qualified_streak; /* 严格连续无中断满足全部门禁的拍数计数器 (硬与门连续>=8000拍/500ms) */
    uint8_t               streak_fail_reason;/* 最近一次正在累积的 streak 被打断清零的原因码 (1:conf,2:lock,3:rms,4:peak,5:spd_err,6:dir,7:flux,8:err_deg,9:spd_low) */
    uint32_t              streak_fail_count; /* streak 被打断清零累计次数 */
    uint32_t              over_limit_streak;/* 当前在非正常区的持续超限拍数 */
    uint32_t              over_26_streak;   /* 持续处于 >26° 重度超限区的拍数 */
    uint32_t              ramp_ripple_streak;/* 动态爬坡期纹波容忍拍数上限计数器 (<=1600拍/100ms) */
    uint32_t              window_samples;   /* 滑动窗口有效采样数 */
    float                 window_err_sum;   /* 窗口绝对误差和 */
    float                 window_err_sq_sum;/* 窗口误差平方和 */
    float                 window_conf_sum;  /* 窗口瞬时置信度累加和 */
    float                 window_peak_err;  /* 窗口最大误差峰值 (度) */
    float                 window_mean_deg;  /* 窗口平均误差 (度) */
    float                 window_rms_deg;   /* 窗口 RMS 误差 (度) */
    float                 window_min_conf;  /* 窗口内最低置信度 */
    uint32_t              window_unlock_cnt;/* 窗口内脱锁累计次数 */

    /* 内部健康防抖计数器 */
    uint32_t              obs_unlock_cnt;   /* 观测器失锁持续拍数 (防抖用) */
    uint32_t              enc_stagnant_cnt; /* 编码器未更新计数器 */
    uint32_t              enc_err_streak;   /* 编码器异常持续拍数 */
    uint32_t              enc_step_latch;   /* 阶跃异常锁存计数器 */
    uint32_t              enc_skip_streak;  /* 恢复阶段跳过微分阶跃判定拍数 */
    float                 enc_last_raw_rad; /* 上一拍机械角，用于阶跃检测 */
    float                 enc_freeze_rad;   /* 注入冻结时刻记录的机械角 */

    /* 故障注入控制器 */
    foc_enc_fault_inject_t inject_type;     /* 当前激活的故障注入类型 */
    float                  inject_param;    /* 故障注入参数 (如跳变度数或速度) */

    /* 第四阶段：纯无感 I/F 独立启动控制与保护参数 */
    float                 if_current_a;     /* I/F 开环吸附与拖动电流给定 (A，默认 0.50A，限幅 <=0.80A) */
    float                 if_target_rpm;    /* I/F 开环加速目标切入转速 (RPM，默认 500.0 RPM) */
    float                 if_accel_rpm_s;   /* I/F 开环加速度斜坡 (RPM/s，默认 300.0 RPM/s) */
    float                 theta_open;       /* I/F 虚拟开环电角度 [0, 2pi) */
    float                 open_speed_rpm;   /* I/F 当前开环转速 (RPM) */
    uint32_t              state_ticks;      /* 当前纯无感子状态执行拍数 (用于硬超时判定) */
    uint8_t               lost_reason;      /* 纯无感失锁停机诊断原因码 (1:unlock, 2:flux_bad, 3:spd_rev, 4:nan, 5:timeout) */
} foc_angle_manager_t;

extern foc_angle_manager_t g_angle_mgr;

/**
 * @brief 初始化角度管理器
 */
void foc_angle_mgr_init(void);

/**
 * @brief 设置反馈模式 (有感 / 纯无感 / 自动接管)
 */
void foc_angle_mgr_set_mode(foc_feedback_mode_t mode);

/**
 * @brief 设置指定的无感主选算法 (VESC / Ortega / STO)
 */
void foc_angle_mgr_set_algo(foc_sensorless_algo_t algo);

/**
 * @brief 注入编码器模拟故障进行容错与接管验证
 */
void foc_angle_mgr_inject_fault(foc_enc_fault_inject_t type, float param);

/**
 * @brief 核心快环角度仲裁任务 (在快环电流采样后、Park 变换前调用)
 * @param m 电机对象指针
 * @return 最终施加于控制闭环的电角度 theta_control [0, 2pi)
 */
float foc_angle_mgr_update(foc_motor_t *m);

/**
 * @brief 重置管理器状态 (在电机使能或故障清除时调用)
 */
void foc_angle_mgr_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ANGLE_MANAGER_H */
