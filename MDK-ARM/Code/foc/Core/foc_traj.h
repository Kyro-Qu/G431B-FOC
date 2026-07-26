/**
 * @file    foc_traj.h
 * @brief   梯形速度轨迹规划器（纯算法，位置模式使用）
 *
 * 来源：ODrive TrapezoidalTrajectory（trap_traj.cpp）的 C 移植，
 * moteus 的位置模式也是同类思路。
 *
 * 解决什么问题？
 *   位置模式如果直接把目标塞给位置环，P 控制器会立刻输出最大速度
 *   指令"硬拉"过去——起停冲击大、超调、对机械不友好。
 *   轨迹规划器把"跳变的目标位置"翻译成一条平滑的位置-时间曲线：
 *
 *   速度 ▲     ┌────────┐ ← 巡航速度 vmax
 *        │    ╱          ╲
 *        │   ╱ 加速段 amax ╲ 减速段
 *        └──┴──────────────┴────► 时间
 *
 *   每个慢环周期输出（位置参考, 速度前馈），位置环只需要跟踪一条
 *   本来就可行的曲线，速度前馈再把大部分工作直接交给速度环。
 *   短距离到不了巡航速度时自动退化为三角形曲线。
 *   支持非零初速度重规划（运动中改目标不会抽搐）。
 */

#ifndef FOC_TRAJ_H
#define FOC_TRAJ_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 规划结果（plan 时算好，eval 只查询） */
    float xi;        /* 起点位置 */
    float xf;        /* 终点位置 */
    float vi;        /* 起始速度 */
    float ar;        /* 加速段加速度（带符号） */
    float vr;        /* 巡航速度（带符号） */
    float dr;        /* 减速段加速度（带符号） */
    float t_acc;     /* 加速段时长 */
    float t_vel;     /* 巡航段时长 */
    float t_dec;     /* 减速段时长 */
    float t_total;
    float y_acc_end; /* 加速段结束位置（缓存） */
    /* 运行状态 */
    float t;         /* 已运行时间 */
    uint8_t active;  /* 1 = 轨迹进行中 */
} foc_traj_t;

/**
 * @brief 规划一条从当前状态到目标位置的梯形轨迹
 * @param x_target 目标位置（rad）
 * @param x_now    当前位置（rad）
 * @param v_now    当前速度（rad/s，可为 0）
 * @param vmax     巡航速度上限（rad/s，> 0）
 * @param amax     加速度上限（rad/s²，> 0）
 * @param dmax     减速度上限（rad/s²，> 0）
 */
void foc_traj_plan(foc_traj_t *tr, float x_target, float x_now, float v_now,
                   float vmax, float amax, float dmax);

/**
 * @brief 推进 dt 并输出当前设定点
 * @param pos_ref 位置参考输出（rad）
 * @param vel_ff  速度前馈输出（rad/s）
 * @return 1 = 轨迹仍在进行，0 = 已到终点（输出终点值，active 清零）
 */
uint8_t foc_traj_eval(foc_traj_t *tr, float dt, float *pos_ref, float *vel_ff);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRAJ_H */
