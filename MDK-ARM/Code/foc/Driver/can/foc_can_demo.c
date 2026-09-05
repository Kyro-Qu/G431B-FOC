/**
 * @file    foc_can_demo.c
 * @brief   Small CAN FD send/receive demo; intentionally not a full protocol.
 */

#include "foc_can_demo.h"

#include "main.h"
#include "foc_app.h"
#include "foc_calib.h"

#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

volatile foc_can_demo_diag_t g_foc_can_demo_diag = {0};

static uint32_t can_demo_last_tx_tick;
static volatile uint8_t can_demo_bus_recover_pending;

static const uint8_t can_demo_tx_data[8] = {
    'H', 'F', 'O', 'C', '_', 'F', 'D', '!'
};

static const uint8_t can_demo_calib_data[8] = {
    'C', 'A', 'L', 'I', 'B', 'F', 'D', '!'
};

static void can_demo_send_fixed_frame(void)
{
    FDCAN_TxHeaderTypeDef header = {0};

    header.Identifier = FOC_CAN_DEMO_TX_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    /* Keep FDF set, but use the 500 kbit/s nominal timing for the complete
     * frame during first hardware bring-up.  BRS can be enabled after this
     * baseline proves CANH/CANL, termination and adapter operation. */
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_FD_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    g_foc_can_demo_diag.tx_attempt_count++;
    g_foc_can_demo_diag.tx_fifo_status = hfdcan1.Instance->TXFQS;

    /* Do not reject a transmission based on TFFL alone.  On STM32G431 the
     * reduced FDCAN message RAM can report TFFL as zero while TFQF (the
     * authoritative queue-full flag used by the ST HAL) is clear.  Let the
     * HAL check TFQF and either enqueue the frame or return an error. */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header,
                                      (uint8_t *)can_demo_tx_data) == HAL_OK) {
        g_foc_can_demo_diag.tx_count++;
    } else {
        g_foc_can_demo_diag.tx_drop_count++;
    }
}

void foc_can_demo_init(void)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    /* The SIT1042T S/STB input is active high.  Keep it in standby until the
     * controller, filter and interrupt path are all ready. */
    HAL_GPIO_WritePin(CAN_SHD_GPIO_Port, CAN_SHD_Pin, GPIO_PIN_SET);

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = FOC_CAN_DEMO_RX_ID;
    filter.FilterID2 = 0x7FFU;

    status = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
    if (status == HAL_OK) {
        status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                               FDCAN_REJECT,
                                               FDCAN_REJECT,
                                               FDCAN_REJECT_REMOTE,
                                               FDCAN_REJECT_REMOTE);
    }
    if (status == HAL_OK) {
        status = HAL_FDCAN_ActivateNotification(
            &hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
            FDCAN_IT_ERROR_PASSIVE |
            FDCAN_IT_BUS_OFF,
            0U);
    }
    if (status == HAL_OK) {
        status = HAL_FDCAN_Start(&hfdcan1);
    }

    if (status != HAL_OK) {
        g_foc_can_demo_diag.last_error = HAL_FDCAN_GetError(&hfdcan1);
        g_foc_can_demo_diag.error_count++;
        return;
    }

    /* Low selects normal/high-speed mode on SIT1042T. */
    HAL_GPIO_WritePin(CAN_SHD_GPIO_Port, CAN_SHD_Pin, GPIO_PIN_RESET);
    can_demo_last_tx_tick = HAL_GetTick();
    g_foc_can_demo_diag.init_ok = 1U;
}

void foc_can_demo_task(void)
{
    uint32_t now;

    if (g_foc_can_demo_diag.init_ok == 0U) {
        return;
    }

    now = HAL_GetTick();
    g_foc_can_demo_diag.tx_fifo_status = hfdcan1.Instance->TXFQS;
    g_foc_can_demo_diag.protocol_status = hfdcan1.Instance->PSR;
    g_foc_can_demo_diag.error_counter = hfdcan1.Instance->ECR;

    /* A board may be powered before the USB-CAN adapter, so its first frame
     * can eventually drive the controller into bus-off due to missing ACKs.
     * Recover in the main loop; never stop/start FDCAN from its IRQ. */
    if (can_demo_bus_recover_pending != 0U) {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        can_demo_bus_recover_pending = 0U;
        if (primask == 0U) {
            __enable_irq();
        }

        if ((HAL_FDCAN_Stop(&hfdcan1) == HAL_OK) &&
            (HAL_FDCAN_Start(&hfdcan1) == HAL_OK)) {
            g_foc_can_demo_diag.bus_recover_count++;
            can_demo_last_tx_tick = now;
        } else {
            g_foc_can_demo_diag.error_count++;
            g_foc_can_demo_diag.last_error = HAL_FDCAN_GetError(&hfdcan1);
        }
    }

    if ((uint32_t)(now - can_demo_last_tx_tick) >= 1000U) {
        can_demo_last_tx_tick = now;
        can_demo_send_fixed_frame();
    }

    if (g_foc_can_demo_diag.calib_request_pending != 0U) {
        foc_motor_t *m0;
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        g_foc_can_demo_diag.calib_request_pending = 0U;
        if (primask == 0U) {
            __enable_irq();
        }

        m0 = foc_app_motor(0U);
        foc_calib_start(m0);
        if (m0->state == FOC_STATE_CALIB) {
            g_foc_can_demo_diag.calib_start_count++;
        } else {
            g_foc_can_demo_diag.calib_reject_count++;
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];

    if ((hfdcan != &hfdcan1) ||
        ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) {
        return;
    }

    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                  &header, data) == HAL_OK) {
        g_foc_can_demo_diag.rx_count++;
        g_foc_can_demo_diag.last_rx_id = header.Identifier;
        g_foc_can_demo_diag.last_rx_is_fd =
            (header.FDFormat == FDCAN_FD_CAN) ? 1U : 0U;
        g_foc_can_demo_diag.last_rx_brs =
            (header.BitRateSwitch == FDCAN_BRS_ON) ? 1U : 0U;

        if ((header.IdType == FDCAN_STANDARD_ID) &&
            (header.RxFrameType == FDCAN_DATA_FRAME) &&
            (header.Identifier == FOC_CAN_DEMO_RX_ID) &&
            (header.DataLength == FDCAN_DLC_BYTES_8) &&
            (header.FDFormat == FDCAN_FD_CAN) &&
            (memcmp(data, can_demo_calib_data,
                    sizeof(can_demo_calib_data)) == 0)) {
            g_foc_can_demo_diag.rx_match_count++;
            g_foc_can_demo_diag.calib_request_pending = 1U;
        }
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) {
        g_foc_can_demo_diag.error_count++;
        g_foc_can_demo_diag.last_error = HAL_FDCAN_GetError(hfdcan);
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t error_status_its)
{
    if (hfdcan == &hfdcan1) {
        g_foc_can_demo_diag.error_count++;
        g_foc_can_demo_diag.last_error = error_status_its;
        if ((error_status_its & FDCAN_IT_BUS_OFF) != 0U) {
            g_foc_can_demo_diag.bus_off_count++;
            can_demo_bus_recover_pending = 1U;
        }
    }
}
