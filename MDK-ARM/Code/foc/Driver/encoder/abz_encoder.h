/**
 * @file    abz_encoder.h
 * @brief   ABZ 增量式编码器驱动（TIM 编码器模式，角度输出 [0, 2π)）
 *
 * 核心思想（本板已验证）：
 *   - TIM ARR 设为 16 位最大值，任何线数的编码器都无需改 ARR；
 *   - 每个快环周期读一次 CNT，delta = CNT_now - CNT_last；
 *     采样频率下电机物理上不可能转过半量程，所以用"半量程法"
 *     判断计数器回绕方向，绝对可靠；
 *   - Z 脉冲每转清零一次，消除累积误差；
 *   - 速度 = delta × 系数，经 5 点去极值均值 + 一阶 IIR 两级滤波。
 *
 * 中断安全：Z 脉冲中断只置标志，实际清零由 update() 在快环上下文
 * 统一执行，避免 CNT/position 在两个中断上下文竞争。
 *
 * 硬件绑定（轴 0）：TIM4 编码器模式，A/B = PB6/PB7，Z = PB8 EXTI。
 */

#ifndef ABZ_ENCODER_H
#define ABZ_ENCODER_H

#include <stdint.h>
#include "foc_config.h"
#include "foc_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 配置 ======================== */

/** 编码器定时器句柄（在板级 MX 初始化中配置为编码器模式） */
#ifndef ABZ_ENCODER_TIM_HANDLE
#define ABZ_ENCODER_TIM_HANDLE  htim4
#endif

/** 4 倍频后的每转计数 */
#ifndef ABZ_ENCODER_CPR
#define ABZ_ENCODER_CPR         FOC_M0_ENCODER_CPR
#endif

/* ======================== 派生常量 ======================== */

#define ABZ_TIMER_COUNTER_RANGE (65536L)
#define ABZ_TIMER_HALF_RANGE    (ABZ_TIMER_COUNTER_RANGE / 2L)

/** 每计数对应的机械弧度 */
#define ABZ_RAD_PER_CNT         (_2PI / (float)ABZ_ENCODER_CPR)

/** delta → RPM：RPM = (delta / CPR) × f_sample × 60 */
#define ABZ_RPM_COEFF           (60.0f * FOC_PWM_FREQ_HZ / (float)ABZ_ENCODER_CPR)

/** 速度一阶 IIR 系数：0.02 ≈ 50 Hz 截止 @16kHz */
#ifndef ABZ_VELOCITY_LPF_ALPHA
#define ABZ_VELOCITY_LPF_ALPHA  0.02f
#endif

/** 单拍最大可信 delta（超过视为干扰丢弃）：1/4 圈 */
#ifndef ABZ_MAX_DELTA
#define ABZ_MAX_DELTA           ((int32_t)ABZ_ENCODER_CPR / 4)
#endif

/* ======================== API ======================== */

/** 初始化：清状态并启动定时器编码器模式 */
void abz_encoder_init(void);

/** 停止定时器 */
void abz_encoder_deinit(void);

/** 周期更新（快环中调用）：读 CNT → 更新角度与速度 */
void abz_encoder_update(void);

/** 机械角 [0, 2π) */
float abz_encoder_angle_rad(void);

/** 机械转速 RPM（两级滤波后） */
float abz_encoder_velocity_rpm(void);

/* ---- Z / index 脉冲支持（供校准状态机使用） ---- */

/** Z 相外部中断回调中调用：锁存计数并置事件标志 */
void abz_encoder_on_index(void);

/** 立即把当前位置设为零点（在 update 上下文生效） */
void abz_encoder_force_zero(void);

/** Z 脉冲到来时是否自动清零位置 */
void abz_encoder_set_zero_on_index(uint8_t enable);

/** 消费一次 Z 事件：返回 1 并输出事件时的位置计数 */
uint8_t abz_encoder_consume_index(int32_t *position_cnt);

/** 是否已经历过至少一次零点校准（Z 或强制清零） */
uint8_t abz_encoder_is_calibrated(void);

#ifdef __cplusplus
}
#endif

#endif /* ABZ_ENCODER_H */
