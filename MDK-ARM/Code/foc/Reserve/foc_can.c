/**
 * @file    foc_can.c
 * @brief   【预留区】FDCAN 总线控制骨架实现
 *
 * 整个文件被 HAL_FDCAN_MODULE_ENABLED 包裹：CubeMX 没开 FDCAN 时
 * 编译为空文件，可以安全地提前加入工程。
 */

#include "foc_can.h"
#include "main.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

#include "../App/foc_app.h"
#include "../App/foc_calib.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

static uint32_t can_last_cmd_tick = 0U;
static uint32_t can_last_beat_tick = 0U;

/* float ↔ 4 字节（CAN 数据段，小端） */
static float can_get_f32(const uint8_t *d)
{
    float v;

    memcpy(&v, d, 4U);
    return v;
}

static void can_put_f32(uint8_t *d, float v)
{
    memcpy(d, &v, 4U);
}

static void can_send(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef hdr = {0};

    hdr.Identifier = std_id;
    hdr.IdType = FDCAN_STANDARD_ID;
    hdr.TxFrameType = FDCAN_DATA_FRAME;
    hdr.DataLength = (uint32_t)len << 16;   /* FDCAN_DLC_BYTES_x 编码 */
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch = FDCAN_BRS_OFF;
    hdr.FDFormat = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, (uint8_t *)data);
}

void foc_can_init(void)
{
    FDCAN_FilterTypeDef f = {0};

    /* 只收本板地址段：BASE_ID ~ BASE_ID + 轴数×16 */
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = 0U;
    f.FilterType = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = FOC_CAN_BASE_ID;
    f.FilterID2 = FOC_CAN_BASE_ID + ((uint32_t)FOC_NUM_AXES * 16U) - 1U;
    (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
    (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_REJECT_REMOTE,
                                       FDCAN_REJECT_REMOTE);
    (void)HAL_FDCAN_ActivateNotification(&hfdcan1,
                                         FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    (void)HAL_FDCAN_Start(&hfdcan1);
    can_last_cmd_tick = HAL_GetTick();
}

void foc_can_on_rx(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    uint32_t off = std_id - FOC_CAN_BASE_ID;
    uint8_t axis = (uint8_t)(off / 16U);
    uint8_t cmd = (uint8_t)(off % 16U);
    foc_motor_t *m;

    if (axis >= (uint8_t)FOC_NUM_AXES) {
        return;
    }
    m = foc_app_motor(axis);
    can_last_cmd_tick = HAL_GetTick();

    switch (cmd) {
    case 0x1U:
        if (len >= 4U) {
            foc_motor_set_target(m, can_get_f32(data));
        }
        break;
    case 0x2U:
        if (len >= 1U) {
            (void)foc_motor_set_mode(m, (foc_mode_t)data[0]);
        }
        break;
    case 0x3U:
        if (len >= 1U) {
            if (data[0] != 0U) {
                (void)foc_motor_arm(m);
            } else {
                foc_motor_disarm(m);
            }
        }
        break;
    case 0x4U:
        foc_motor_clear_fault(m);
        break;
    default:
        break;
    }
}

void foc_can_task(void)
{
    uint32_t now = HAL_GetTick();
    foc_motor_t *m0 = foc_app_motor(0U);

    /* 心跳 100ms：state + fault + 转速 */
    if ((uint32_t)(now - can_last_beat_tick) >= 100U) {
        uint8_t d[8];

        can_last_beat_tick = now;
        d[0] = (uint8_t)m0->state;
        d[1] = m0->safety.fault_code;
        d[2] = 0U;
        d[3] = 0U;
        can_put_f32(&d[4], m0->velocity_rpm);
        can_send(FOC_CAN_BASE_ID + 0x0U, d, 8U);
    }

    /* 命令超时看护：上位机失联时不许电机继续跑 */
    if ((m0->state == FOC_STATE_RUN) &&
        ((uint32_t)(now - can_last_cmd_tick) >= FOC_CAN_CMD_TIMEOUT_MS)) {
        foc_motor_disarm(m0);
    }
}

/* CubeMX 使能 FDCAN 后，把这个回调放进 main.c 的 USER CODE 区
 * （或直接用本文件的定义——HAL 回调是 weak 的）： */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                      &hdr, data) == HAL_OK) {
            if (hdr.IdType == FDCAN_STANDARD_ID) {
                foc_can_on_rx(hdr.Identifier, data,
                              (uint8_t)(hdr.DataLength >> 16));
            }
        }
    }
}

#endif /* HAL_FDCAN_MODULE_ENABLED */
