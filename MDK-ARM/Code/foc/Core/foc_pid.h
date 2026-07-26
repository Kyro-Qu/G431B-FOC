/**
 * @file    foc_pid.h
 * @brief   PID 控制器与一阶低通滤波器（纯 C，无硬件依赖）
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

/* ---------------- LPF ---------------- */

void foc_lpf_init(foc_lpf_t *lpf, float tf);
float foc_lpf_update(foc_lpf_t *lpf, float x, float dt);
void foc_lpf_reset(foc_lpf_t *lpf, float value);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PID_H */
