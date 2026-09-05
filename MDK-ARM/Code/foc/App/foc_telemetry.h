/**
 * @file    foc_telemetry.h
 * @brief   VOFA+ JustFloat 遥测流（USART2 DMA，默认 1 kHz）
 *
 * 通道表（VOFA+ 里按此顺序命名波形）：
 *   ch0  轴0 FOC 电角度 θe (rad)
 *   ch1  轴0 原始 Iq（Park 后、低通前；诊断 2380RPM 爆发专用）
 *   ch2  轴0 自适应滑动拟合速度 (RPM)
 *   ch3  轴0 斜坡后速度给定 (RPM)
 *   ch4  轴0 Id 遥测滤波值 (A；电流 PI 使用未低通值)
 *   ch5  轴0 Iq 遥测滤波值 (A；电流 PI 使用未低通值)
 *   ch6  轴0 Iq 给定 (A)
 *   ch7  轴0 Vd 控制指令 (V；非 ADC 实测电压)
 *   ch8  轴0 Vq 控制指令 (V；非 ADC 实测电压)
 *   ch9  轴0 Iu (A)
 *   ch10 轴0 Iv (A)
 *   ch11 轴0 Iw (A)
 *   ch12 轴0 A 相占空比 duty_a (0..1)
 *   ch13 轴0 电机故障×100 + 电流采样故障
 *   ch14 轴0 弱磁积分 fw_integral (A)
 *   ch15 轴0 控制实际使用速度 (RPM；PLL + median3 + 自适应5/30Hz BW2)
 *        （双轴时为轴1 电角度）
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

/** 临时挂起遥测（cmd_print 阻塞发送期间独占 UART 用），on=1 挂起 */
void foc_telemetry_suspend(uint8_t on);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TELEMETRY_H */
