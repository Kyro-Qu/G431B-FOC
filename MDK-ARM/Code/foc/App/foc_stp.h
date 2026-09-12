/**
 * @file    foc_stp.h
 * @brief   FOC 自解释掩码遥测协议 (FOC-STP v1.0) 纯 C 编解码器与校验模块
 *
 * 帧格式：
 *   A5 5A | VER_TYPE:u8 | LEN:u8 | SEQ:u16 | PAYLOAD | CRC16:u16
 *
 * CRC16 多项式 0x1021 (CCITT-FALSE, Init 0xFFFF),
 * 校验范围：VER_TYPE + LEN + SEQ + PAYLOAD
 */

#ifndef FOC_STP_H
#define FOC_STP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_STP_SYNC0               0xA5U
#define FOC_STP_SYNC1               0x5AU
#define FOC_STP_VERSION             1U

#define FOC_STP_TYPE_WAVE           0x01U
#define FOC_STP_TYPE_STATUS         0x02U
#define FOC_STP_TYPE_EVENT          0x03U
#define FOC_STP_TYPE_TEXT           0x04U
#define FOC_STP_TYPE_ACK            0x05U

#define FOC_STP_MAKE_VER_TYPE(type) ((uint8_t)(((FOC_STP_VERSION & 0x0FU) << 4U) | ((type) & 0x0FU)))
#define FOC_STP_GET_VER(ver_type)   ((uint8_t)(((ver_type) >> 4U) & 0x0FU))
#define FOC_STP_GET_TYPE(ver_type)  ((uint8_t)((ver_type) & 0x0FU))

#define FOC_STP_HEADER_SIZE         6U  /* SYNC(2) + VER_TYPE(1) + LEN(1) + SEQ(2) */
#define FOC_STP_CRC_SIZE            2U
#define FOC_STP_OVERHEAD            8U  /* HEADER_SIZE + CRC_SIZE */

#define FOC_STP_MAX_CHANNELS        32U
#define FOC_STP_MAX_WAVE_CHANNELS   16U /* 单帧波形最多同时发送 16 通道，防止缓冲区超限 */

/* 事件 ID */
#define FOC_STP_EVENT_FAULT_TRIP    0x01U
#define FOC_STP_EVENT_STATE_CHANGE  0x02U
#define FOC_STP_EVENT_CALIB_DONE    0x03U
#define FOC_STP_EVENT_WARN          0x04U

/* ACK 状态 */
#define FOC_STP_ACK_OK              0x00U
#define FOC_STP_ACK_REJECTED        0x01U
#define FOC_STP_ACK_LIMITED         0x02U

/* CRC16-CCITT (poly 0x1021, init 0xFFFF) 计算 */
uint16_t foc_stp_crc16(const uint8_t *data, uint16_t len);
uint16_t foc_stp_crc16_update(uint16_t crc, const uint8_t *data, uint16_t len);

/* 32 位掩码置位计数 (popcount) */
uint8_t foc_stp_popcount32(uint32_t mask);

/* 帧打包函数，返回总帧长（含帧头和 CRC16），0 表示失败/缓冲区不足 */

uint16_t foc_stp_pack_wave(uint8_t *buf, uint16_t buf_size, uint16_t seq,
                           uint32_t sample_tick, uint32_t channel_mask,
                           const float *values, uint8_t val_count);

uint16_t foc_stp_pack_status(uint8_t *buf, uint16_t buf_size, uint16_t seq,
                             uint32_t timestamp_ms, uint16_t vbus_cvolts,
                             uint8_t motor_fault, uint8_t shunt_fault,
                             uint8_t state, uint8_t mode, int8_t temp_c,
                             int16_t rpm_est, int16_t iq_est_ca);

uint16_t foc_stp_pack_event(uint8_t *buf, uint16_t buf_size, uint16_t seq,
                            uint32_t timestamp_ms, uint8_t event_id,
                            uint8_t motor_fault, uint8_t shunt_fault,
                            uint32_t detail);

uint16_t foc_stp_pack_text(uint8_t *buf, uint16_t buf_size, uint16_t seq,
                           const char *text, uint8_t text_len);

uint16_t foc_stp_pack_ack(uint8_t *buf, uint16_t buf_size, uint16_t seq,
                          uint8_t cmd_code, uint8_t status,
                          uint32_t effective_mask, uint16_t effective_rate_hz);

#ifdef __cplusplus
}
#endif

#endif /* FOC_STP_H */
