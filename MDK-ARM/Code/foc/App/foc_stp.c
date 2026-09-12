/**
 * @file    foc_stp.c
 * @brief   FOC 自解释掩码遥测协议 (FOC-STP v1.0) 纯 C 编解码器与校验模块实现
 */

#include "foc_stp.h"
#include <string.h>

/*
 * CRC16-CCITT-FALSE:
 * 多项式: 0x1021, 初始值: 0xFFFF, 结果异或: 0x0000, 不反转
 * 测试向量 "123456789" -> 0x29B1
 */
uint16_t foc_stp_crc16_update(uint16_t crc, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    while (len > 0U) {
        crc ^= (uint16_t)((uint16_t)(*data++) << 8U);
        for (i = 0U; i < 8U; i++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1U);
            }
        }
        len--;
    }
    return crc;
}

uint16_t foc_stp_crc16(const uint8_t *data, uint16_t len)
{
    return foc_stp_crc16_update(0xFFFFU, data, len);
}

static void write_u16_le(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val & 0xFFU);
    buf[1] = (uint8_t)((val >> 8U) & 0xFFU);
}

static void write_u32_le(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val & 0xFFU);
    buf[1] = (uint8_t)((val >> 8U) & 0xFFU);
    buf[2] = (uint8_t)((val >> 16U) & 0xFFU);
    buf[3] = (uint8_t)((val >> 24U) & 0xFFU);
}

static void write_f32_le(uint8_t *buf, float val)
{
    (void)memcpy(buf, &val, 4U);
}

uint8_t foc_stp_popcount32(uint32_t mask)
{
    uint8_t count = 0U;
    while (mask != 0U) {
        count += (uint8_t)(mask & 1U);
        mask >>= 1U;
    }
    return count;
}

uint16_t foc_stp_pack_wave(uint8_t *buf, uint16_t buf_size, uint16_t seq,
                           uint32_t sample_tick, uint32_t channel_mask,
                           const float *values, uint8_t val_count)
{
    uint8_t payload_len;
    uint16_t total_len;
    uint16_t crc;
    uint8_t i;
    uint16_t offset;
    uint8_t expected_count;

    expected_count = foc_stp_popcount32(channel_mask);
    if ((expected_count != val_count) || (val_count > (uint8_t)FOC_STP_MAX_WAVE_CHANNELS)) {
        return 0U; /* 严禁掩码与值数量不一致或超过硬件限制 */
    }

    payload_len = (uint8_t)(8U + (uint8_t)(val_count * 4U)); /* tick(4) + mask(4) + floats(K*4) */
    total_len = (uint16_t)((uint16_t)FOC_STP_OVERHEAD + (uint16_t)payload_len);

    if ((buf == 0) || (buf_size < total_len)) {
        return 0U;
    }

    /* 帧头 */
    buf[0] = FOC_STP_SYNC0;
    buf[1] = FOC_STP_SYNC1;
    buf[2] = FOC_STP_MAKE_VER_TYPE(FOC_STP_TYPE_WAVE);
    buf[3] = payload_len;
    write_u16_le(&buf[4], seq);

    /* Payload */
    write_u32_le(&buf[6], sample_tick);
    write_u32_le(&buf[10], channel_mask);

    offset = 14U;
    for (i = 0U; i < val_count; i++) {
        write_f32_le(&buf[offset], values[i]);
        offset += 4U;
    }

    /* CRC16 (覆盖 VER_TYPE 到 Payload 结尾) */
    crc = foc_stp_crc16(&buf[2], (uint16_t)(4U + payload_len));
    write_u16_le(&buf[offset], crc);

    return total_len;
}

uint16_t foc_stp_pack_status(uint8_t *buf, uint16_t buf_size, uint16_t seq,
                             uint32_t timestamp_ms, uint16_t vbus_cvolts,
                             uint8_t motor_fault, uint8_t shunt_fault,
                             uint8_t state, uint8_t mode, int8_t temp_c,
                             int16_t rpm_est, int16_t iq_est_ca)
{
    const uint8_t payload_len = 15U;
    const uint16_t total_len = (uint16_t)((uint16_t)FOC_STP_OVERHEAD + (uint16_t)payload_len); /* 23 字节 */
    uint16_t crc;

    if ((buf == 0) || (buf_size < total_len)) {
        return 0U;
    }

    /* 帧头 */
    buf[0] = FOC_STP_SYNC0;
    buf[1] = FOC_STP_SYNC1;
    buf[2] = FOC_STP_MAKE_VER_TYPE(FOC_STP_TYPE_STATUS);
    buf[3] = payload_len;
    write_u16_le(&buf[4], seq);

    /* Payload (15 字节) */
    write_u32_le(&buf[6], timestamp_ms);
    write_u16_le(&buf[10], vbus_cvolts);
    buf[12] = motor_fault;
    buf[13] = shunt_fault;
    buf[14] = state;
    buf[15] = mode;
    buf[16] = (uint8_t)temp_c;
    write_u16_le(&buf[17], (uint16_t)rpm_est);
    write_u16_le(&buf[19], (uint16_t)iq_est_ca);

    /* CRC16 */
    crc = foc_stp_crc16(&buf[2], (uint16_t)(4U + payload_len));
    write_u16_le(&buf[21], crc);

    return total_len;
}

uint16_t foc_stp_pack_event(uint8_t *buf, uint16_t buf_size, uint16_t seq,
                            uint32_t timestamp_ms, uint8_t event_id,
                            uint8_t motor_fault, uint8_t shunt_fault,
                            uint32_t detail)
{
    const uint8_t payload_len = 11U;
    const uint16_t total_len = (uint16_t)((uint16_t)FOC_STP_OVERHEAD + (uint16_t)payload_len); /* 19 字节 */
    uint16_t crc;

    if ((buf == 0) || (buf_size < total_len)) {
        return 0U;
    }

    /* 帧头 */
    buf[0] = FOC_STP_SYNC0;
    buf[1] = FOC_STP_SYNC1;
    buf[2] = FOC_STP_MAKE_VER_TYPE(FOC_STP_TYPE_EVENT);
    buf[3] = payload_len;
    write_u16_le(&buf[4], seq);

    /* Payload (11 字节) */
    write_u32_le(&buf[6], timestamp_ms);
    buf[10] = event_id;
    buf[11] = motor_fault;
    buf[12] = shunt_fault;
    write_u32_le(&buf[13], detail);

    /* CRC16 */
    crc = foc_stp_crc16(&buf[2], (uint16_t)(4U + payload_len));
    write_u16_le(&buf[17], crc);

    return total_len;
}

uint16_t foc_stp_pack_text(uint8_t *buf, uint16_t buf_size, uint16_t seq,
                           const char *text, uint8_t text_len)
{
    uint16_t total_len;
    uint16_t crc;

    if (text == 0) {
        text_len = 0U;
    }
    total_len = (uint16_t)((uint16_t)FOC_STP_OVERHEAD + (uint16_t)text_len);

    if ((buf == 0) || (buf_size < total_len)) {
        return 0U;
    }

    /* 帧头 */
    buf[0] = FOC_STP_SYNC0;
    buf[1] = FOC_STP_SYNC1;
    buf[2] = FOC_STP_MAKE_VER_TYPE(FOC_STP_TYPE_TEXT);
    buf[3] = text_len;
    write_u16_le(&buf[4], seq);

    /* Payload (text_len 字节) */
    if (text_len > 0U) {
        (void)memcpy(&buf[6], text, text_len);
    }

    /* CRC16 */
    crc = foc_stp_crc16(&buf[2], (uint16_t)(4U + text_len));
    write_u16_le(&buf[6U + text_len], crc);

    return total_len;
}

uint16_t foc_stp_pack_ack(uint8_t *buf, uint16_t buf_size, uint16_t seq,
                          uint8_t cmd_code, uint8_t status,
                          uint32_t effective_mask, uint16_t effective_rate_hz)
{
    const uint8_t payload_len = 8U;
    const uint16_t total_len = (uint16_t)((uint16_t)FOC_STP_OVERHEAD + (uint16_t)payload_len); /* 16 字节 */
    uint16_t crc;

    if ((buf == 0) || (buf_size < total_len)) {
        return 0U;
    }

    /* 帧头 */
    buf[0] = FOC_STP_SYNC0;
    buf[1] = FOC_STP_SYNC1;
    buf[2] = FOC_STP_MAKE_VER_TYPE(FOC_STP_TYPE_ACK);
    buf[3] = payload_len;
    write_u16_le(&buf[4], seq);

    /* Payload (8 字节) */
    buf[6] = cmd_code;
    buf[7] = status;
    write_u32_le(&buf[8], effective_mask);
    write_u16_le(&buf[12], effective_rate_hz);

    /* CRC16 */
    crc = foc_stp_crc16(&buf[2], (uint16_t)(4U + payload_len));
    write_u16_le(&buf[14], crc);

    return total_len;
}
