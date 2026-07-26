/**
 * @file    foc_sensorless.h
 * @brief   【预留区】无感 FOC 绑定层：观测器 + 开环启动 + 切换状态机
 *
 * ⚠ 本文件在 Reserve/ 目录，未加入 Keil 工程，不参与编译。
 *    接入步骤与调试方法见《Docs/09_预留特性接入手册.md》第 1 章。
 *
 * 干什么用：
 *   Core/foc_observer.c 只是"数学"（磁链观测器+PLL），本模块把它
 *   包装成完整的无感方案：
 *     1. 静止时观测器没有反电动势可看，无法工作 → 必须开环启动；
 *     2. 开环把电机 ramp 到足够转速（反电动势足够大）；
 *     3. 比较观测器角度与开环角度，稳定一致后切换到观测器闭环；
 *   这就是 VESC "openloop ramp start" 的做法。
 *
 * 磁链 λ 怎么定（观测器唯一的关键参数）：
 *   λ ≈ 60 / (√3 · 2π · KV · pp)      （VESC 公式，KV 单位 RPM/V）
 *   例：KV=1400、pp=6 → λ ≈ 60/(1.732×6.283×1400×6) ≈ 6.6e-4 Wb
 *   拿不到 KV：空转电机测线反电动势峰值 / 电角速度也可以。
 */

#ifndef FOC_SENSORLESS_H
#define FOC_SENSORLESS_H

#include "../Core/foc_motor.h"
#include "../Core/foc_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FOC_SL_IDLE = 0,     /* 未启动 */
    FOC_SL_RAMP,         /* 开环加速中（观测器后台跟踪） */
    FOC_SL_CLOSED,       /* 已切换：角度来自观测器 */
    FOC_SL_LOST          /* 失锁（角度差发散），需要重启 */
} foc_sensorless_state_t;

typedef struct {
    foc_observer_t obs;
    foc_sensorless_state_t state;

    /* 配置 */
    float ramp_target_rpm;   /* 开环加速目标转速（建议 10~20% 额定） */
    float ramp_rpm_s;        /* 开环加速斜率 */
    float switch_err_rad;    /* 切换判据：|θ_obs - θ_ol| 持续小于此值 */
    uint16_t switch_ticks;   /* 判据需持续的快环拍数（如 1600 = 100ms） */

    /* 内部 */
    float ramp_rpm_now;
    uint16_t agree_cnt;
    float last_err_rad;      /* 遥测观察用 */
} foc_sensorless_t;

/**
 * @brief 初始化。flux_wb 用头注释公式估算；gamma 传 0 用默认值
 */
void foc_sensorless_init(foc_sensorless_t *sl,
                         const foc_motor_params_t *params,
                         float flux_wb, float dt_fast);

/** 开始无感启动流程（电机须已 arm、模式力矩/速度） */
void foc_sensorless_start(foc_sensorless_t *sl);

/**
 * @brief 快环每拍调用（接入点见 09 手册）。
 *        内部驱动观测器 + 状态机，返回本拍应使用的电角度。
 *        RAMP 阶段返回开环角度（同时后台喂观测器），
 *        CLOSED 阶段返回观测器角度。
 * @param m 电机对象（读取 v_dq/i_dq/theta_e 重建 αβ 量）
 */
float foc_sensorless_update(foc_sensorless_t *sl, foc_motor_t *m);

/** 观测器估计的电角速度 rad/s（速度环反馈用：除以 pp 转机械） */
float foc_sensorless_speed_e_rads(const foc_sensorless_t *sl);

foc_sensorless_state_t foc_sensorless_get_state(const foc_sensorless_t *sl);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SENSORLESS_H */
