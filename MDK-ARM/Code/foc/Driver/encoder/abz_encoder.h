/**
 * @file    abz_encoder.h
 * @brief   增量式 ABZ 编码器的 TIM 编码器模式驱动。
 *
 * 轴 0 的硬件绑定：
 *   - A/B: PB6/PB7 -> TIM4 编码器模式
 *   - Z:   PB8 -> EXTI
 *   - 位置与角度在 16 kHz FOC 快环更新
 *   - 速度历史在 1 kHz 慢环更新
 *
 * 速度估计：
 *   - 静止/低速：64 ms 最小二乘位置拟合
 *   - 中速：      32 ms 最小二乘位置拟合
 *   - 高速：      16 ms 最小二乘位置拟合
 *   - 相邻区间交叉淡入淡出，避免窗口切换阶跃
 *   - 64 ms 位置跨度门限用于抑制 AS5047P 静止时
 *     ±1 计数的边界抖动
 *
 * 本板曾验证过短窗口 M/T 测速法，但 AS5047P 的 ABI 输出偶尔出现
 * 密集边沿簇：仅测 4 个计数会把这些边沿簇放大成 300 RPM 量级的尖峰
 * （真实转速仅 20/50 RPM），因此控制版本刻意改用稳健的位置拟合法。
 */

#ifndef ABZ_ENCODER_H
#define ABZ_ENCODER_H

#include <stdint.h>
#include "foc_config.h"
#include "foc_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 硬件绑定 ======================== */

#ifndef ABZ_ENCODER_TIM_HANDLE
#define ABZ_ENCODER_TIM_HANDLE  htim4
#endif

/** 每机械圈的正交解码计数（CPR）。 */
#ifndef ABZ_ENCODER_CPR
#define ABZ_ENCODER_CPR         FOC_M0_ENCODER_CPR
#endif

/* ======================== 位置常量 ======================== */

#define ABZ_TIMER_COUNTER_RANGE (65536L)
#define ABZ_TIMER_HALF_RANGE    (ABZ_TIMER_COUNTER_RANGE / 2L)
#define ABZ_RAD_PER_CNT         (_2PI / (float)ABZ_ENCODER_CPR)

/**
 * 剔除不可能的单个快环跳变。正常电机不可能在一个 62.5 us
 * 控制周期内转过四分之一机械圈。
 */
#ifndef ABZ_MAX_DELTA
#define ABZ_MAX_DELTA           ((int32_t)ABZ_ENCODER_CPR / 4)
#endif

/* ======================== 速度估计器 ======================== */

/** 每个慢环周期存储一个连续位置历史点。 */
#define ABZ_VELOCITY_SAMPLE_DIV         FOC_SLOW_DIV

/** 最长历史与自适应最小二乘拟合窗口长度，此处单位为 ms。 */
#define ABZ_VELOCITY_HISTORY_LEN        64U
#define ABZ_VELOCITY_LOW_WINDOW         64U
#define ABZ_VELOCITY_MID_WINDOW         32U
#define ABZ_VELOCITY_HIGH_WINDOW        16U

/** 用于交叉淡入淡出窗口长度的长窗口稳定转速阈值。 */
#define ABZ_VELOCITY_LOW_BLEND_START_RPM    60.0f
#define ABZ_VELOCITY_LOW_BLEND_END_RPM     120.0f
#define ABZ_VELOCITY_HIGH_BLEND_START_RPM  800.0f
#define ABZ_VELOCITY_HIGH_BLEND_END_RPM   1200.0f

/**
 * 64 ms 窗口的静止判定。
 *
 * 端点位移判据无需在 16 kHz 中断里扫描整段历史即可抑制固定边界的
 * ABI 抖动；拟合速度条件避免把短暂反转误判为静止。
 * 约 2 RPM 以下的运动被刻意报为零；真实的 10 RPM 运动不受影响。
 */
#define ABZ_STATIONARY_ENDPOINT_COUNTS  4LL
#define ABZ_STATIONARY_MAX_RPM          2.0f

#define ABZ_VELOCITY_SAMPLE_DT_S                                      \
    ((float)ABZ_VELOCITY_SAMPLE_DIV / FOC_PWM_FREQ_HZ)
#define ABZ_RPM_PER_COUNT_PER_SAMPLE                                  \
    (60.0f / ((float)ABZ_ENCODER_CPR * ABZ_VELOCITY_SAMPLE_DT_S))

/**
 * FOC 换相路径使用的机械角度跟踪 PLL。
 *
 * 二阶观测器跟踪匀速时稳态相位误差为零，优于简单角度低通。
 * 默认 600 rad/s 带宽可抑制 AS5047P ABI 边沿量化噪声，同时在设定的
 * 速度斜坡下加速度跟踪误差保持在一个编码器计数以内。
 */
#define ABZ_ANGLE_PLL_BW_RAD_S        600.0f
#define ABZ_ANGLE_PLL_MIN_BW_RAD_S    100.0f
#define ABZ_ANGLE_PLL_MAX_BW_RAD_S    2000.0f
#define ABZ_ANGLE_PLL_KP              (2.0f * ABZ_ANGLE_PLL_BW_RAD_S)
#define ABZ_ANGLE_PLL_KI              \
    (ABZ_ANGLE_PLL_BW_RAD_S * ABZ_ANGLE_PLL_BW_RAD_S)
#define ABZ_FAST_DT_S                 (1.0f / FOC_PWM_FREQ_HZ)

/* ======================== 公开 API ======================== */

/** 复位驱动状态，清零 TIM CNT 并启动两个编码器通道。 */
void abz_encoder_init(void);

/** 停止 TIM 编码器的两个通道。 */
void abz_encoder_deinit(void);

/** 读取 TIM CNT 并更新角度/速度；每个 FOC 快环调用一次。 */
void abz_encoder_update(void);

/** 单圈机械角度，范围 [0, 2π)。 */
float abz_encoder_angle_rad(void);

/** 自适应交叠最小二乘估计器给出的机械转速 RPM。 */
float abz_encoder_velocity_rpm(void);

/**
 * 16 kHz 角度 PLL 给出的低延迟机械转速 RPM。用作可配置控制反馈
 * 滤波的输入；最小二乘结果仍单独保留给遥测和诊断使用。
 */
float abz_encoder_pll_velocity_rpm(void);

/** 设置/查询角度 PLL 带宽（仅 RAM；调用方应在 IDLE 时修改）。 */
uint8_t abz_encoder_set_pll_bw_rad_s(float bw_rad_s);
float abz_encoder_get_pll_bw_rad_s(void);

void abz_encoder_on_index(void);

/** 请求位置清零；在 abz_encoder_update() 中消费。 */
void abz_encoder_force_zero(void);

/** 使能/禁止在下一个 Z 边沿到来时重映射硬件计数器。 */
void abz_encoder_set_zero_on_index(uint8_t enable);

/** 消费一次已锁存的 Z 事件，可选返回其单圈计数值。 */
uint8_t abz_encoder_consume_index(int32_t *position_cnt);

/** 发生过 Z 事件或显式清零后返回非零。 */
uint8_t abz_encoder_is_calibrated(void);

#ifdef __cplusplus
}
#endif

#endif /* ABZ_ENCODER_H */
