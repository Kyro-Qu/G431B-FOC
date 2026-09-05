/**
 * @file    foc_types.h
 * @brief   FOC 库通用数据类型、枚举与硬件抽象接口表（纯 C，无硬件依赖）
 *
 * 架构参考：
 *   - ODrive  ：Axis 对象化设计 —— 每个电机一个独立对象，包含配置、
 *               状态机、控制器和硬件绑定，多电机即多个对象实例。
 *   - SimpleFOC：Sensor / CurrentSense / Driver 三大抽象基类 ——
 *               算法层只面向接口编程，换硬件只换接口实现。
 *   - VESC    ：故障码集中管理 + 诊断变量全局可见的调试哲学。
 *
 * 本文件定义"算法层与硬件层之间的合同"：
 *   Core 层（foc_motor.c 等）只允许通过这里的接口表访问硬件；
 *   HAL 层（foc_board_*.c）负责把接口表填上具体实现。
 *   这就是本工程支持双电机的关键 —— 每个轴一份接口表 + 一个电机对象。
 */

#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 坐标系向量 ========== */

/** 三相静止坐标系 (a-b-c) */
typedef struct abc_tag {
    float a;
    float b;
    float c;
} abc_t;

/** 两相静止坐标系 (α-β)，α 与 A 相重合，β 超前 90° */
typedef struct ab_tag {
    float alpha;
    float beta;
} ab_t;

/** 两相旋转坐标系 (d-q)，d 对准转子磁链，q 产生转矩 */
typedef struct dq_tag {
    float d;
    float q;
} dq_t;

/* ========== 运行状态与控制模式 ========== */

/** 轴运行状态（生命周期状态机，参考 ODrive AxisState 简化） */
typedef enum {
    FOC_STATE_IDLE  = 0,  /* 空闲：PWM 关闭 */
    FOC_STATE_RUN   = 1,  /* 运行：PWM 输出，控制环生效 */
    FOC_STATE_CALIB = 2,  /* 校准：校准状态机接管电压命令 */
    FOC_STATE_FAULT = 3   /* 故障：锁定，需显式清除 */
} foc_state_t;

/** 控制模式（参考 SimpleFOC MotionControlType） */
typedef enum {
    FOC_MODE_OPENLOOP_VF = 0, /* 开环 V/f：虚拟角度 + 电压给定，无需任何反馈 */
    FOC_MODE_TORQUE      = 1, /* 力矩模式：Iq 闭环（需要电流采样 + 角度） */
    FOC_MODE_VELOCITY    = 2, /* 速度模式：速度环 → Iq 电流环 */
    FOC_MODE_POSITION    = 3  /* 位置模式：位置 PI + 速度阻尼 → Iq 电流环 */
} foc_mode_t;

/** Park/反 Park 使用的电角度来源 */
typedef enum {
    FOC_ANGLE_OPEN_LOOP = 0,          /* 软件推进的虚拟角度 */
    FOC_ANGLE_ENCODER_CALIBRATED = 1, /* 编码器机械角 × 极对数 + 校准偏移 */
    FOC_ANGLE_OBSERVER = 2            /* 无感磁链观测器角度（高速用） */
} foc_angle_source_t;

/** 轴级故障码（统一入口，参考 VESC mc_fault_code） */
typedef enum {
    FOC_FAULT_NONE = 0,
    FOC_FAULT_CURRENT_SENSE,      /* 电流采样链路失效 */
    FOC_FAULT_CALIB_OVERCURRENT,  /* 校准期间过流 */
    FOC_FAULT_RUN_OVERCURRENT,    /* 运行期间过流 */
    FOC_FAULT_CALIB_TIMEOUT,      /* 校准超时（等不到 Z 脉冲） */
    FOC_FAULT_CALIB_STATE,        /* 校准状态机异常 */
    FOC_FAULT_NOT_CALIBRATED,     /* 闭环模式要求校准但尚未完成 */
    FOC_FAULT_CONTROL_NAN,        /* 控制环输出出现 NaN（参数/数值异常） */
    FOC_FAULT_STALL,              /* 堵转：电流饱和且转速≈0 持续超时 */
    FOC_FAULT_BAD_CONFIG          /* 上电参数自检失败（参数非法） */
} foc_fault_t;

/* ========== 硬件抽象接口表 ========== */

/**
 * 三相 PWM 功率级接口（一轴一份）。
 * Core 层不知道 TIM1/TIM8，也不知道 CCR 寄存器 —— 只会调这些函数。
 */
typedef struct {
    void (*enable)(void);      /* 常规互补 PWM 上电（中点占空比起步） */
    void (*disable)(void);     /* 立即关闭功率级（MOE=0），故障安全出口 */
    void (*bootstrap)(void);   /* 三路低边导通，给高边自举电容充电 */
    /** 写三相比较值。sector 供采样窗口规划参考。实现方内部负责
     *  电流采样触发点规划与越界保护。 */
    void (*set_compare)(uint32_t ccr_a, uint32_t ccr_b,
                        uint32_t ccr_c, uint8_t sector);
    uint16_t full_count;       /* PWM 计数满量程（ARR） */
    float    u_dc;             /* 母线电压 V */
} foc_driver_if_t;

/** 相电流采样接口（一轴一份） */
typedef struct {
    uint8_t (*is_ready)(void);                            /* 零偏校准完成且链路无故障 */
    void (*get)(float *ia, float *ib, float *ic);         /* 最新三相电流 A */
} foc_current_if_t;

/**
 * 位置传感器接口（一轴一份）。
 * update/angle_rad/velocity_rpm 是必选项；velocity_control_rpm 可选，用于
 * 提供低延迟观察器速度。带 index/Z 脉冲的增量编码器再实现后三个，
 * 供上电校准状态机使用（无 Z 相的传感器可置 NULL）。
 */
typedef struct {
    void  (*update)(void);            /* 快环中调用，刷新内部计数 */
    float (*angle_rad)(void);         /* 机械角 [0, 2π) */
    float (*velocity_rpm)(void);      /* 诊断机械转速 RPM（可较强拟合） */
    float (*velocity_control_rpm)(void); /* 可选：低延迟控制速度 RPM */

    /* ---- 可选：增量编码器 index（Z 脉冲）支持 ---- */
    void    (*force_zero)(void);              /* 立即把当前位置设为零点 */
    void    (*set_zero_on_index)(uint8_t en); /* Z 脉冲到来时是否自动清零 */
    uint8_t (*consume_index)(int32_t *cnt);   /* 取出一次 Z 事件的位置计数 */
    float   rad_per_cnt;                      /* 每计数对应的机械弧度 */
} foc_sensor_if_t;

/* ========== 控制配置与电机参数 ========== */

/** 电机铭牌/实测参数（一轴一份） */
typedef struct {
    float pole_pairs;     /* 极对数 */
    float rs_ohm;         /* 相电阻 Ω（星形等效单相） */
    float ls_henry;       /* 相电感 H */
    float ke;             /* 反电动势常数（备用） */
    float max_current_a;  /* 软件电流限制 Apk */
    float hard_current_a; /* 硬件级瞬时电流限制 Apk（单点即断） */
    float max_rpm;        /* 最大机械转速 */
} foc_motor_params_t;

/** 控制环配置（一轴一份） */
typedef struct {
    /* 电流环：按带宽自整定 Kp = Ls·ω，Ki = Rs·ω（ODrive 方式） */
    float current_bw_rads;    /* 电流环带宽 rad/s，典型 500~2000 */
    /* 速度环 PI（输入 RPM 误差，输出 Iq 给定 A） */
    float vel_kp;             /* A / RPM */
    float vel_ki;             /* A / (RPM·s) */
    float vel_ramp_rpm_s;     /* 速度目标斜坡 RPM/s，0 = 不限 */
    float vel_lpf_tf;         /* 速度 median3+BW2 等效时间常数；fc=1/(2πTf) */
    float vel_friction_a;     /* 转动后的库仑摩擦前馈 A */
    float vel_start_a;        /* 静止脱离齿槽所需的起步前馈 A */
    float vel_start_rpm;      /* 起步前馈平滑退出的转速 RPM */
    float vel_track_kp;       /* 低速位置轨迹跟踪增益 A/rad */
    float vel_track_limit_rad;/* 低速轨迹允许的最大位置滞后 rad */
    float vel_track_rpm;      /* 全量跟踪区上限；两倍该值处退出 */
    /* 位置伺服：位置 PI 直接输出 Iq，速度误差提供阻尼/前馈 */
    float pos_kp;             /* A / rad */
    float pos_ki;             /* A / (rad·s) */
    float pos_vel_kp;         /* A / RPM */
    float pos_vel_limit_rpm;  /* 位置模式允许的最大速度给定 */
    /* 位置模式梯形轨迹规划（ODrive trap_traj 方案） */
    uint8_t traj_enable;      /* 1 = 目标位置经轨迹规划器平滑 */
    float traj_accel_rpm_s;   /* 轨迹加/减速度 RPM/s */
    /* dq 轴解耦前馈开关：vd -= ω·Lq·iq, vq += ω·Ld·id */
    uint8_t decouple_enable;
    /* 死区补偿电压 V（0 = 关闭）。死区使每相平均损失约
     * Udc·t_dead·f_pwm 的电压、方向与相电流相反；按电流符号前馈
     * 补回可改善低电流时的过零畸变（VESC/MESC 做法） */
    float deadtime_comp_v;
    /* 堵转保护（仅速度/位置模式；力矩模式堵转是正常工况） */
    uint8_t stall_enable;     /* 1 = 使能堵转检测 */
    float stall_rpm;          /* 转速低于此值视为"不转" */
    uint16_t stall_timeout_ms;/* 电流饱和且不转持续超时 → FOC_FAULT_STALL */
} foc_ctrl_cfg_t;

/** 高速实验参数（仅 RAM，禁止由 Flash 配置持久化） */
typedef struct {
    float angle_delay_cycles;      /* 编码器角度预测的快环周期数 */
    uint8_t fieldweak_enable;      /* 1 = 启用自动弱磁 */
    float fieldweak_enter_rpm;     /* 进入弱磁的速度门槛 */
    float fieldweak_voltage_ratio; /* 弱磁目标电压 / 线性区上限 */
    float fieldweak_gain;          /* 弱磁积分增益 */
    float fieldweak_id_min_a;      /* 最小 Id，负值表示去磁电流 */
} foc_runtime_t;

/* ========== 安全诊断（一轴一份，Keil Watch 友好） ========== */

typedef struct {
    volatile float peak_current_a;          /* 本周期三相最大 |I| */
    volatile float max_observed_current_a;  /* 上电以来的最大 |I| */
    volatile float soft_current_a;          /* 滤波后的 sqrt(Id^2+Iq^2) */
    volatile float current_limit_a;         /* 当前生效的软限制 */
    volatile float hard_current_limit_a;    /* 当前生效的硬限制 */
    volatile float trip_current_u_a;        /* 触发故障时的三相电流快照 */
    volatile float trip_current_v_a;
    volatile float trip_current_w_a;
    volatile float trip_soft_current_a;     /* 跳闸时的软件限流判据 */
    volatile uint8_t trip_was_hard;         /* 1=单拍硬限，0=软件限流 */
    volatile uint16_t consecutive_over_limit; /* 连续超软限计数 */
    volatile uint8_t fault_code;              /* foc_fault_t */
} foc_safety_diag_t;

#ifdef __cplusplus
}
#endif

#endif /* FOC_TYPES_H */
