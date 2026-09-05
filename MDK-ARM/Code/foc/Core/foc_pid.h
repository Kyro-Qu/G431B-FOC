/**
 * @file    foc_pid.h
 * @brief   PID 控制器与通用滤波器（纯 C，无硬件依赖）
 *
 * 实现参考 SimpleFOC PIDController / LowPassFilter：
 *   - 积分项用 Tustin（双线性）离散化：∫ ≈ Σ Ki·Ts/2·(e[k]+e[k-1])，
 *     比前向欧拉在同样采样率下相位误差更小；
 *   - 抗积分饱和：积分项与总输出都钳位到 out_limit（clamping 法）；
 *   - 输出斜坡 out_ramp 限制输出变化率（V/s 或 A/s），
 *     避免阶跃命令直接打满造成电流冲击（SimpleFOC 的 voltage ramp 思想）。
 *
 * 定点采样：本工程控制环频率固定（16 kHz / 1 kHz），dt 作为参数传入，
 * 不做 SimpleFOC 那种 micros() 自适应，快环里少一次时间读取。
 */

#ifndef FOC_PID_H
#define FOC_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** PID 控制器（P/I/D + 输出限幅 + 输出斜坡） */
typedef struct {
    float kp;         /* 比例增益 */
    float ki;         /* 积分增益（每秒），内部乘 dt */
    float kd;         /* 微分增益，电流/速度环通常为 0 */
    float out_limit;  /* 输出绝对值上限（<=0 表示不限） */
    float out_ramp;   /* 输出变化率上限（单位/秒，<=0 表示不限） */
    /* 内部状态 */
    float integral;
    float prev_error;
    float prev_output;
} foc_pid_t;

/** 一阶低通滤波器 y += dt/(Tf+dt)·(x-y) */
typedef struct {
    float tf;      /* 时间常数 s；0 = 直通 */
    float y;       /* 上次输出 */
} foc_lpf_t;

/**
 * 速度反馈专用滤波器：三点中值 + 二阶 Butterworth 低通。
 *
 * 中值级去掉孤立的单样本尖峰，只引入约一个采样周期的延迟；二阶低通
 * 使用 Direct Form II Transposed，float 下比直接保存 x/y 历史更稳健。
 * tf 沿用一阶低通的时间常数定义，截止频率 fc = 1/(2*pi*tf)，这样旧的
 * Flash 参数字段和 `vel lpf` 命令仍可表达同一个物理带宽。
 */
typedef struct {
    float tf;
    float sample_hz;
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float z1;
    float z2;
    float median_z1;
    float median_z2;
    uint8_t enabled;
} foc_speed_filter_t;

/* ---------------- PID ---------------- */

/** 初始化（清状态并设置增益/限幅） */
void foc_pid_init(foc_pid_t *pid, float kp, float ki, float kd,
                  float out_limit, float out_ramp);

/** 执行一步，返回控制输出。dt 为本次调用距上次的时间（秒） */
float foc_pid_update(foc_pid_t *pid, float error, float dt);

/** 清除积分与历史状态（模式切换/重新使能时调用，防止旧积分顶出冲击） */
void foc_pid_reset(foc_pid_t *pid);

/** 运行中修改输出限幅（如动态调整电流限制） */
void foc_pid_set_limit(foc_pid_t *pid, float out_limit);

/**
 * 二阶陷波滤波器（RBJ biquad，Direct Form II Transposed）。
 * 用于剔除电流反馈中固定频率的采样混叠分量（如 2.5 kHz 纹波假影），
 * 陷波点外相位失真极小，不影响电流环带宽。
 */
typedef struct {
    float b0, b1, b2;   /* 分子系数 */
    float a1, a2;       /* 分母反馈系数（a0 归一化为 1） */
    float z1, z2;       /* 状态 */
    uint8_t enabled;
} foc_notch_t;

/* ---------------- LPF ---------------- */

void foc_lpf_init(foc_lpf_t *lpf, float tf);
float foc_lpf_update(foc_lpf_t *lpf, float x, float dt);
void foc_lpf_reset(foc_lpf_t *lpf, float value);

/* ---------------- Notch ---------------- */

/** 初始化陷波器：f0 陷波中心 Hz，bw 陷波带宽 Hz，fs 采样率 Hz */
void foc_notch_init(foc_notch_t *n, float f0, float bw, float fs);

/** 单步滤波；未初始化(enabled=0)时直通 */
float foc_notch_update(foc_notch_t *n, float x);

/** 清状态（保持系数） */
void foc_notch_reset(foc_notch_t *n);

/* ---------------- Speed feedback filter ---------------- */

/** 配置三点中值 + 二阶 Butterworth；tf<=0 时完全直通。 */
void foc_speed_filter_init(foc_speed_filter_t *filter,
                           float tf, float sample_hz);

/** 执行一个等周期样本。 */
float foc_speed_filter_update(foc_speed_filter_t *filter, float input);

/** 把中值历史和二阶状态重置为同一个稳态值，切换参数时不会产生跳变。 */
void foc_speed_filter_reset(foc_speed_filter_t *filter, float value);

/** 返回当前截止频率 Hz；直通时返回 0。 */
float foc_speed_filter_cutoff_hz(const foc_speed_filter_t *filter);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PID_H */
