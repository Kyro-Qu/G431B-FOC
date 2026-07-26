/**
 * @file    foc_calib.h
 * @brief   上电编码器电角度校准状态机（面向 foc_motor_t 对象）
 *
 * 为什么需要校准？
 *   FOC 要求知道"转子磁极此刻指向哪个电角度"。增量编码器上电时
 *   只知道相对位置，不知道绝对零点，更不知道零点和磁极的关系。
 *   校准做两件事：
 *     1. D 轴对齐：给 d 轴通电压把转子"吸"到已知电角度上；
 *     2. Z 脉冲搜索：慢速开环旋转，记录 Z 脉冲位置与对齐点的
 *        机械角距离，换算出电角度偏移 offset。
 *   之后任意时刻：θe = direction × pole_pairs × θmech + offset
 *
 * 校准流程（与本板验证过的时序一致）：
 *   BOOTSTRAP  低边全通给自举电容充电（10ms）
 *   NEUTRAL    中点 PWM 观察稳定（无电压矢量）
 *   ALIGN      d 轴电压缓升，把转子吸到 θe = 0
 *   SETTLE     把对齐位置临时设为编码器零点，稍等稳定
 *   SEARCH     低速开环旋转找 Z 脉冲，捕获后计算 offset
 *   DONE/FAIL  成功回 IDLE（结果写入 motor->calib）；失败进 FAULT
 *
 * 安全：校准期间电流阈值被压低（软 1.5A / 硬 3.0A），
 * 任何过流、采样失效、超时都立即断 PWM 并锁存故障。
 * 结果只存 RAM，每次上电需重新校准（增量编码器的宿命）。
 */

#ifndef FOC_CALIB_H
#define FOC_CALIB_H

#include <stdint.h>
#include "foc_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 校准状态 */
typedef enum {
    FOC_CALIB_IDLE = 0,      /* 未开始 */
    FOC_CALIB_BOOTSTRAP = 1, /* 低边导通，自举电容充电 */
    FOC_CALIB_NEUTRAL = 2,   /* 互补 PWM 中点观察 */
    FOC_CALIB_ALIGN = 3,     /* D 轴对齐（电压缓升） */
    FOC_CALIB_SETTLE = 4,    /* 对齐点强制清零后等待 */
    FOC_CALIB_SEARCH = 5,    /* 慢速开环旋转找 Z 脉冲 */
    FOC_CALIB_DONE = 6,      /* 成功，motor->calib 有效 */
    FOC_CALIB_FAIL = 7       /* 失败（超时/过流/链路异常） */
} foc_calib_state_t;

/** 全局状态镜像：Keil Watch / 断电诊断记录用 */
extern volatile uint8_t g_foc_calib_state;

/** 运行时可调的校准电压（Keil Watch 里小步试探用） */
extern volatile float g_foc_calib_align_voltage;
extern volatile float g_foc_calib_search_voltage;

/** 启动指定轴的校准（轴必须处于 IDLE 且电流采样就绪） */
void foc_calib_start(foc_motor_t *m);

/** 校准状态机任务：主循环中周期调用 */
void foc_calib_task(void);

/** 1 = 校准状态机正在接管该轴的电压命令 */
uint8_t foc_calib_is_active(void);

foc_calib_state_t foc_calib_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CALIB_H */
