/**
 * @file test_stp_cross.c
 * @brief 编译运行并输出 C 语言序列化的 FOC-STP v1.0 二进制帧，用于跨语言验证
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../MDK-ARM/Code/foc/App/foc_stp.h"

int main(int argc, char **argv)
{
    const char *out_path = "stp_golden.bin";
    if (argc > 1) {
        out_path = argv[1];
    }

    FILE *fp = fopen(out_path, "wb");
    if (!fp) {
        fprintf(stderr, "Failed to open output file: %s\n", out_path);
        return 1;
    }

    uint8_t buf[256];
    uint16_t len;

    /* 1. 打包一帧 4 通道 Wave 帧 (seq=1, tick=10000, mask=0x0000000F) */
    float vals[4] = { 3.14159f, -12.5f, 0.05f, 1500.0f };
    len = foc_stp_pack_wave(buf, sizeof(buf), 1U, 10000U, 0x0000000FU, vals, 4U);
    if (len > 0) {
        fwrite(buf, 1, len, fp);
    }

    /* 2. 打包一帧 Status 心跳帧 (seq=2, timestamp=10050, vbus=14.40V, rpm=1500, iq=0.52A) */
    len = foc_stp_pack_status(buf, sizeof(buf), 2U, 10050U, 1440U, 0U, 0U, 1U, 2U, 42, 1500, 52);
    if (len > 0) {
        fwrite(buf, 1, len, fp);
    }

    /* 3. 打包一帧 Text 文本帧 (seq=3, text="M0 RUN mode=vel") */
    const char *txt = "M0 RUN mode=vel\r\n";
    len = foc_stp_pack_text(buf, sizeof(buf), 3U, txt, (uint8_t)strlen(txt));
    if (len > 0) {
        fwrite(buf, 1, len, fp);
    }

    /* 4. 打包一帧 Event 事件帧 (seq=4, timestamp=10100, fault=OVERVOLTAGE) */
    len = foc_stp_pack_event(buf, sizeof(buf), 4U, 10100U, FOC_STP_EVENT_FAULT_TRIP, 6U, 0U, 1850U);
    if (len > 0) {
        fwrite(buf, 1, len, fp);
    }

    /* 5. 打包一帧 ACK 应答帧 (seq=5, effective_mask=0x000000FF, rate=500Hz) */
    len = foc_stp_pack_ack(buf, sizeof(buf), 5U, 1U, FOC_STP_ACK_OK, 0x000000FFU, 500U);
    if (len > 0) {
        fwrite(buf, 1, len, fp);
    }

    fclose(fp);
    printf("Successfully wrote FOC-STP golden frames to %s\n", out_path);
    return 0;
}
