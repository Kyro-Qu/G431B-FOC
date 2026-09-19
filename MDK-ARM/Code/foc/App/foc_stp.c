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
 *
 * 查表实现：WAVE 帧在 16 kHz 快环中断内打包，逐位算法 76 字节需 ~600 次
 * 移位循环，查表后每字节仅 1 次查表 + 移位，快环占用降到 1/8。
 */
static const uint16_t s_crc16_table[256] = {
    0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
    0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
    0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52B5U, 0x4294U, 0x72F7U, 0x62D6U,
    0x9339U, 0x8318U, 0xB37BU, 0xA35AU, 0xD3BDU, 0xC39CU, 0xF3FFU, 0xE3DEU,
    0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64E6U, 0x74C7U, 0x44A4U, 0x5485U,
    0xA56AU, 0xB54BU, 0x8528U, 0x9509U, 0xE5EEU, 0xF5CFU, 0xC5ACU, 0xD58DU,
    0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76D7U, 0x66F6U, 0x5695U, 0x46B4U,
    0xB75BU, 0xA77AU, 0x9719U, 0x8738U, 0xF7DFU, 0xE7FEU, 0xD79DU, 0xC7BCU,
    0x48C4U, 0x58E5U, 0x6886U, 0x78A7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
    0xC9CCU, 0xD9EDU, 0xE98EU, 0xF9AFU, 0x8948U, 0x9969U, 0xA90AU, 0xB92BU,
    0x5AF5U, 0x4AD4U, 0x7AB7U, 0x6A96U, 0x1A71U, 0x0A50U, 0x3A33U, 0x2A12U,
    0xDBFDU, 0xCBDCU, 0xFBBFU, 0xEB9EU, 0x9B79U, 0x8B58U, 0xBB3BU, 0xAB1AU,
    0x6CA6U, 0x7C87U, 0x4CE4U, 0x5CC5U, 0x2C22U, 0x3C03U, 0x0C60U, 0x1C41U,
    0xEDAEU, 0xFD8FU, 0xCDECU, 0xDDCDU, 0xAD2AU, 0xBD0BU, 0x8D68U, 0x9D49U,
    0x7E97U, 0x6EB6U, 0x5ED5U, 0x4EF4U, 0x3E13U, 0x2E32U, 0x1E51U, 0x0E70U,
    0xFF9FU, 0xEFBEU, 0xDFDDU, 0xCFFCU, 0xBF1BU, 0xAF3AU, 0x9F59U, 0x8F78U,
    0x9188U, 0x81A9U, 0xB1CAU, 0xA1EBU, 0xD10CU, 0xC12DU, 0xF14EU, 0xE16FU,
    0x1080U, 0x00A1U, 0x30C2U, 0x20E3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
    0x83B9U, 0x9398U, 0xA3FBU, 0xB3DAU, 0xC33DU, 0xD31CU, 0xE37FU, 0xF35EU,
    0x02B1U, 0x1290U, 0x22F3U, 0x32D2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
    0xB5EAU, 0xA5CBU, 0x95A8U, 0x8589U, 0xF56EU, 0xE54FU, 0xD52CU, 0xC50DU,
    0x34E2U, 0x24C3U, 0x14A0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
    0xA7DBU, 0xB7FAU, 0x8799U, 0x97B8U, 0xE75FU, 0xF77EU, 0xC71DU, 0xD73CU,
    0x26D3U, 0x36F2U, 0x0691U, 0x16B0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
    0xD94CU, 0xC96DU, 0xF90EU, 0xE92FU, 0x99C8U, 0x89E9U, 0xB98AU, 0xA9ABU,
    0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18C0U, 0x08E1U, 0x3882U, 0x28A3U,
    0xCB7DU, 0xDB5CU, 0xEB3FU, 0xFB1EU, 0x8BF9U, 0x9BD8U, 0xABBBU, 0xBB9AU,
    0x4A75U, 0x5A54U, 0x6A37U, 0x7A16U, 0x0AF1U, 0x1AD0U, 0x2AB3U, 0x3A92U,
    0xFD2EU, 0xED0FU, 0xDD6CU, 0xCD4DU, 0xBDAAU, 0xAD8BU, 0x9DE8U, 0x8DC9U,
    0x7C26U, 0x6C07U, 0x5C64U, 0x4C45U, 0x3CA2U, 0x2C83U, 0x1CE0U, 0x0CC1U,
    0xEF1FU, 0xFF3EU, 0xCF5DU, 0xDF7CU, 0xAF9BU, 0xBFBAU, 0x8FD9U, 0x9FF8U,
    0x6E17U, 0x7E36U, 0x4E55U, 0x5E74U, 0x2E93U, 0x3EB2U, 0x0ED1U, 0x1EF0U,
};

uint16_t foc_stp_crc16_update(uint16_t crc, const uint8_t *data, uint16_t len)
{
    while (len > 0U) {
        crc = (uint16_t)((uint16_t)(crc << 8U) ^ s_crc16_table[(uint8_t)((crc >> 8U) ^ *data++)]);
        len--;
    }
    return crc;
}

uint16_t foc_stp_crc16(const uint8_t *data, uint16_t len)
{
    return foc_stp_crc16_update(0xFFFFU, data, len);
}

typedef struct { uint16_t v; } __attribute__((packed)) stp_packed_u16_t;
typedef struct { uint32_t v; } __attribute__((packed)) stp_packed_u32_t;
typedef struct { float v; } __attribute__((packed)) stp_packed_f32_t;

static inline void write_u16_le(uint8_t *buf, uint16_t val)
{
    ((stp_packed_u16_t *)(void *)buf)->v = val;
}

static inline void write_u32_le(uint8_t *buf, uint32_t val)
{
    ((stp_packed_u32_t *)(void *)buf)->v = val;
}

static inline void write_f32_le(uint8_t *buf, float val)
{
    ((stp_packed_f32_t *)(void *)buf)->v = val;
}

uint8_t foc_stp_popcount32(uint32_t mask)
{
    /* 汉明权重并行算法，无分支循环，仅需数条指令即可算完 32 位置 1 计数 */
    mask = mask - ((mask >> 1U) & 0x55555555UL);
    mask = (mask & 0x33333333UL) + ((mask >> 2U) & 0x33333333UL);
    return (uint8_t)((((mask + (mask >> 4U)) & 0x0F0F0F0FUL) * 0x01010101UL) >> 24U);
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

    if ((val_count > (uint8_t)FOC_STP_MAX_WAVE_CHANNELS) || (buf == 0)) {
        return 0U;
    }

    payload_len = (uint8_t)(8U + (uint8_t)(val_count * 4U)); /* tick(4) + mask(4) + floats(K*4) */
    total_len = (uint16_t)((uint16_t)FOC_STP_OVERHEAD + (uint16_t)payload_len);

    if (buf_size < total_len) {
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
                             uint8_t fault_code, uint8_t state,
                             int8_t temp_c, uint8_t cpu_load_pct)
{
    const uint8_t payload_len = 10U;
    const uint16_t total_len = (uint16_t)((uint16_t)FOC_STP_OVERHEAD + (uint16_t)payload_len); /* 18 字节 */
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

    /* Payload (10 字节) */
    write_u32_le(&buf[6], timestamp_ms);
    write_u16_le(&buf[10], vbus_cvolts);
    buf[12] = fault_code;
    buf[13] = state;
    buf[14] = (uint8_t)temp_c;
    buf[15] = cpu_load_pct;

    /* CRC16 */
    crc = foc_stp_crc16(&buf[2], (uint16_t)(4U + payload_len));
    write_u16_le(&buf[16], crc);

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
