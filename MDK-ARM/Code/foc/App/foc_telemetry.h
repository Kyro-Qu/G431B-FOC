/**
 * @file    foc_telemetry.h
 * @brief   FOC 自解释掩码遥测流 (FOC-STP v1.0, USART2 DMA 500 Hz + 10 Hz STATUS)
 */

#ifndef FOC_TELEMETRY_H
#define FOC_TELEMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_TELEMETRY_CH 32U

/* 默认订阅掩码：包含 theta_e, iq_raw, vel_ctrl, vel_ref, id_filt, iq_filt, iq_ref, vd, vq, vbus_fast (0x040001FF) */
#define FOC_TELEMETRY_DEFAULT_MASK  0x040001FFU

void foc_telemetry_init(void);

/** 快环中调用 (500 Hz)：按掩码提取变量并 DMA 发送 WAVE 帧 */
void foc_telemetry_isr_tick(void);

/** 主循环中调用：调度 10 Hz STATUS、异步 EVENT 及配置 ACK */
void foc_telemetry_slow_tick(void);
void foc_telemetry_status_tick(void); /* 兼容别名 */

/** 故障或事件发生时调用：锁存快照并异步可靠发送 EVENT 帧（非阻塞，不打断 DMA） */
void foc_telemetry_report_event(uint8_t event_id, uint8_t motor_fault, uint8_t shunt_fault, uint32_t detail);

void foc_telemetry_set_enable(uint8_t enable);
uint8_t foc_telemetry_get_enable(void);

/** 设置遥测掩码，popcount <= 16 生效并返回 FOC_STP_ACK_OK(0)，否则返回 FOC_STP_ACK_LIMITED(2) */
uint8_t foc_telemetry_set_mask(uint32_t mask);
uint32_t foc_telemetry_get_mask(void);

/** 临时挂起遥测（cmd_print 独占 UART 用），on=1 挂起 */
void foc_telemetry_suspend(uint8_t on);

/** 强制重置 TX 状态机（DMA 发生错误或中止后恢复） */
void foc_telemetry_reset_tx_state(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TELEMETRY_H */
