/**
 * @file    foc_telemetry.h
 * @brief   VOFA+ JustFloat 遥测流（USART2 DMA，默认 1 kHz）
 *
 * 通道表（VOFA+ 里按此顺序命名波形）：
 *   ch0  轴0 电角度 θe (rad)
 *   ch1  轴0 机械角 θm (rad)
 *   ch2  轴0 转速 (RPM)
 *   ch3  轴0 速度给定 (RPM)
 *   ch4  轴0 Id 实测滤波 (A)
 *   ch5  轴0 Iq 实测滤波 (A)
 *   ch6  轴0 Iq 给定 (A)
 *   ch7  轴0 Vd 输出 (V)
 *   ch8  轴0 Vq 输出 (V)
 *   ch9  轴0 Iu (A)
 *   ch10 轴0 Iv (A)
 *   ch11 轴0 Iw (A)
 *   ch12 轴0 state×10 + 校准状态
 *   ch13 轴0 电机故障×100 + 电流采样故障
 *   ch14 轴0 多圈位置 (rad)
 *   ch15 轴0 目标值 / （双轴时为轴1 电角度）
 */

#ifndef FOC_TELEMETRY_H
#define FOC_TELEMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_TELEMETRY_CH 16U

void foc_telemetry_init(void);

/** 快环中调用，内部按 FOC_TELEMETRY_DIV 分频后 DMA 发送一帧 */
void foc_telemetry_isr_tick(void);

void foc_telemetry_set_enable(uint8_t enable);
uint8_t foc_telemetry_get_enable(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TELEMETRY_H */
