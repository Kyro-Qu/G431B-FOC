/**
 * @file    foc_can.c
 * @brief   Deferred CAN FD command execution and board status publication.
 *
 * Interrupt boundary:
 *   FDCAN ISR -> validate routing and copy a frame into the RX queue only.
 *   main loop -> parse, validate state/value, operate the motor, send reply.
 *
 * This prevents flash writes, calibration startup and motor state changes from
 * running at FDCAN interrupt priority.  A sequence cache makes retransmitted
 * commands idempotent: an identical repeat is answered again but is not
 * executed twice.
 */

#include "foc_can.h"

#if FOC_CAN_ENABLE

#include "main.h"
#include "foc_app.h"
#include "foc_calib.h"
#include "foc_store.h"
#include "current_shunt.h"

#include <math.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

#define CAN_RX_QUEUE_SIZE       5U  /* Four commands plus one ring sentinel. */
#define CAN_TX_QUEUE_SIZE       5U  /* Four responses plus one ring sentinel. */
#define CAN_REQUEST_BYTES      16U
#define CAN_RESPONSE_BYTES     16U
#define CAN_STATUS_BYTES       32U
#define CAN_FAULT_BYTES        16U
#define CAN_BUS_RECOVER_MS    100U
#define CAN_TX_RETRY_MS         5U

typedef struct {
    uint8_t length;
    uint8_t is_fd;
    uint8_t brs;
    uint8_t reserved;
    uint8_t data[CAN_REQUEST_BYTES];
} can_rx_item_t;

typedef struct {
    uint8_t data[CAN_RESPONSE_BYTES];
} can_tx_item_t;

volatile foc_can_diag_t g_foc_can_diag = {
    .enabled = 1U
};

static can_rx_item_t can_rx_queue[CAN_RX_QUEUE_SIZE];
static volatile uint8_t can_rx_head;
static volatile uint8_t can_rx_tail;

static can_tx_item_t can_tx_queue[CAN_TX_QUEUE_SIZE];
static uint8_t can_tx_head;
static uint8_t can_tx_tail;

static uint8_t can_last_request[CAN_REQUEST_BYTES];
static uint8_t can_last_response[CAN_RESPONSE_BYTES];
static uint8_t can_last_request_valid;

static volatile uint8_t can_bus_recover_pending;
static uint32_t can_last_recover_tick;
static uint32_t can_last_status_tick;
static uint8_t can_fault_pending;
static uint8_t can_fault_sequence;
static uint8_t can_last_motor_fault = 0xFFU;
static uint8_t can_last_current_fault = 0xFFU;
static uint8_t can_tx_retry_wait;
static uint32_t can_last_tx_fail_tick;

static uint8_t can_next_index(uint8_t index, uint8_t size)
{
    index++;
    return (index >= size) ? 0U : index;
}

static uint8_t can_dlc_bytes(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0:  return 0U;
    case FDCAN_DLC_BYTES_1:  return 1U;
    case FDCAN_DLC_BYTES_2:  return 2U;
    case FDCAN_DLC_BYTES_3:  return 3U;
    case FDCAN_DLC_BYTES_4:  return 4U;
    case FDCAN_DLC_BYTES_5:  return 5U;
    case FDCAN_DLC_BYTES_6:  return 6U;
    case FDCAN_DLC_BYTES_7:  return 7U;
    case FDCAN_DLC_BYTES_8:  return 8U;
    case FDCAN_DLC_BYTES_12: return 12U;
    case FDCAN_DLC_BYTES_16: return 16U;
    case FDCAN_DLC_BYTES_20: return 20U;
    case FDCAN_DLC_BYTES_24: return 24U;
    case FDCAN_DLC_BYTES_32: return 32U;
    case FDCAN_DLC_BYTES_48: return 48U;
    case FDCAN_DLC_BYTES_64: return 64U;
    default:                 return 0xFFU;
    }
}

static uint32_t can_get_u32(const uint8_t *data)
{
    uint32_t value;

    memcpy(&value, data, sizeof(value));
    return value;
}

static float can_get_f32(const uint8_t *data)
{
    float value;

    memcpy(&value, data, sizeof(value));
    return value;
}

static void can_put_u32(uint8_t *data, uint32_t value)
{
    memcpy(data, &value, sizeof(value));
}

static void can_put_f32(uint8_t *data, float value)
{
    memcpy(data, &value, sizeof(value));
}

static uint8_t can_float_in_range(float value, float low, float high)
{
    return ((value == value) && (value >= low) && (value <= high)) ? 1U : 0U;
}

static HAL_StatusTypeDef can_try_send(uint32_t identifier,
                                      uint32_t dlc,
                                      uint8_t *data)
{
    FDCAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef status;
    uint32_t now = HAL_GetTick();

    /* A full hardware FIFO or an unavailable bus must not turn the main loop
     * into a tight HAL retry loop.  Responses remain queued in software; a
     * fault event remains pending and is retried after this short backoff. */
    if ((can_tx_retry_wait != 0U) &&
        ((uint32_t)(now - can_last_tx_fail_tick) < CAN_TX_RETRY_MS)) {
        return HAL_BUSY;
    }

    header.Identifier = identifier;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dlc;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
#if FOC_CAN_BRS_ENABLE
    header.BitRateSwitch = FDCAN_BRS_ON;
#else
    header.BitRateSwitch = FDCAN_BRS_OFF;
#endif
    header.FDFormat = FDCAN_FD_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
    if (status != HAL_OK) {
        can_last_tx_fail_tick = now;
        can_tx_retry_wait = 1U;
        g_foc_can_diag.last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
    } else {
        can_tx_retry_wait = 0U;
    }
    return status;
}

static uint8_t can_response_enqueue(const uint8_t *data)
{
    uint8_t next = can_next_index(can_tx_head, CAN_TX_QUEUE_SIZE);

    if (next == can_tx_tail) {
        g_foc_can_diag.response_drop_count++;
        g_foc_can_diag.tx_drop_count++;
        return 0U;
    }

    memcpy(can_tx_queue[can_tx_head].data, data, CAN_RESPONSE_BYTES);
    can_tx_head = next;
    return 1U;
}

static void can_flush_responses(void)
{
    while (can_tx_tail != can_tx_head) {
        if (can_try_send(FOC_CAN_RESPONSE_ID, FDCAN_DLC_BYTES_16,
                         can_tx_queue[can_tx_tail].data) != HAL_OK) {
            return;
        }
        can_tx_tail = can_next_index(can_tx_tail, CAN_TX_QUEUE_SIZE);
        g_foc_can_diag.response_count++;
    }
}

static uint8_t can_rx_pop(can_rx_item_t *item)
{
    uint32_t primask;

    if (can_rx_tail == can_rx_head) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(item, &can_rx_queue[can_rx_tail], sizeof(*item));
    can_rx_tail = can_next_index(can_rx_tail, CAN_RX_QUEUE_SIZE);
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

static float can_status_target(const foc_motor_t *motor, uint8_t axis)
{
    if ((axis == 0U) && (motor->mode == FOC_MODE_OPENLOOP_VF)) {
        return g_m0_openloop_rpm;
    }
    return motor->target;
}

static float can_status_vq(const foc_motor_t *motor, uint8_t axis)
{
    if ((axis == 0U) && (motor->mode == FOC_MODE_OPENLOOP_VF)) {
        return g_m0_openloop_vq_applied;
    }
    return motor->v_dq.q;
}

static uint8_t can_motion_update_allowed(const foc_motor_t *motor)
{
    if (foc_calib_is_active() != 0U) {
        return 0U;
    }
    return ((motor->state == FOC_STATE_IDLE) ||
            (motor->state == FOC_STATE_RUN)) ? 1U : 0U;
}

static void can_build_response(uint8_t *response,
                               uint8_t sequence,
                               uint8_t command,
                               uint8_t axis,
                               foc_can_result_t result,
                               const foc_motor_t *motor,
                               float detail)
{
    memset(response, 0, CAN_RESPONSE_BYTES);
    response[0] = FOC_CAN_RESPONSE_MAGIC;
    response[1] = FOC_CAN_PROTOCOL_VERSION;
    response[2] = sequence;
    response[3] = command;
    response[4] = axis;
    response[5] = (uint8_t)result;
    if (motor != 0) {
        response[6] = (uint8_t)motor->state;
        response[7] = (uint8_t)motor->mode;
        response[8] = motor->safety.fault_code;
        response[10] = motor->calib.valid;
    }
    response[9] = g_current_shunt_diag.fault_code;
    response[11] = (uint8_t)foc_calib_get_state();
    can_put_f32(&response[12], detail);
}

static foc_can_result_t can_execute_command(const uint8_t *request,
                                            foc_motor_t *motor,
                                            float *detail)
{
    uint8_t command = request[3];
    uint8_t axis = request[4];
    uint32_t argument_u32 = can_get_u32(&request[8]);
    float argument_f32 = can_get_f32(&request[8]);

    *detail = 0.0f;

    switch (command) {
    case FOC_CAN_CMD_GET_STATUS:
        *detail = motor->velocity_filt_rpm;
        return FOC_CAN_RESULT_OK;

    case FOC_CAN_CMD_ENABLE:
        if (foc_motor_arm(motor) != 0U) {
            return FOC_CAN_RESULT_OK;
        }
        *detail = (float)motor->safety.fault_code;
        if ((motor->mode != FOC_MODE_OPENLOOP_VF) &&
            (motor->calib.valid == 0U)) {
            return FOC_CAN_RESULT_NOT_CALIBRATED;
        }
        return FOC_CAN_RESULT_STATE_REJECTED;

    case FOC_CAN_CMD_DISABLE:
        foc_motor_disarm(motor);
        if (axis == 0U) {
            foc_app_vf_reset_commands();
        }
        return FOC_CAN_RESULT_OK;

    case FOC_CAN_CMD_CLEAR_FAULT:
        if (current_shunt_is_ready() == 0U) {
            *detail = (float)g_current_shunt_diag.fault_code;
            return FOC_CAN_RESULT_STATE_REJECTED;
        }
        foc_motor_clear_fault(motor);
        if (axis == 0U) {
            foc_app_vf_reset_commands();
        }
        return FOC_CAN_RESULT_OK;

    case FOC_CAN_CMD_CALIBRATE:
        {
            uint8_t old_from_store;

            if (argument_u32 > 1U) {
                return FOC_CAN_RESULT_BAD_ARGUMENT;
            }
            if (motor->state != FOC_STATE_IDLE) {
                return FOC_CAN_RESULT_STATE_REJECTED;
            }
            if (foc_calib_is_active() != 0U) {
                return FOC_CAN_RESULT_BUSY;
            }
            if (current_shunt_is_ready() == 0U) {
                *detail = (float)g_current_shunt_diag.fault_code;
                return FOC_CAN_RESULT_STATE_REJECTED;
            }

            old_from_store = motor->calib.from_store;
            if (axis == 0U) {
                foc_app_vf_reset_commands();
            }
            if (argument_u32 != 0U) {
                motor->calib.from_store = 0U;
            }
            foc_calib_start(motor);
            if (motor->state != FOC_STATE_CALIB) {
                motor->calib.from_store = old_from_store;
                *detail = (float)motor->safety.fault_code;
                return FOC_CAN_RESULT_STATE_REJECTED;
            }
            *detail = (float)foc_calib_get_state();
            return FOC_CAN_RESULT_OK;
        }

    case FOC_CAN_CMD_SET_MODE:
        {
            foc_mode_t old_mode;
            foc_mode_t new_mode;

            if (argument_u32 > (uint32_t)FOC_MODE_POSITION) {
                return FOC_CAN_RESULT_BAD_ARGUMENT;
            }
            if (motor->state != FOC_STATE_IDLE) {
                return FOC_CAN_RESULT_STATE_REJECTED;
            }
            old_mode = motor->mode;
            new_mode = (foc_mode_t)argument_u32;
            if (foc_motor_set_mode(motor, new_mode) == 0U) {
                if ((new_mode != FOC_MODE_OPENLOOP_VF) &&
                    (motor->calib.valid == 0U)) {
                    return FOC_CAN_RESULT_NOT_CALIBRATED;
                }
                return FOC_CAN_RESULT_STATE_REJECTED;
            }
            if ((axis == 0U) &&
                ((old_mode == FOC_MODE_OPENLOOP_VF) ||
                 (new_mode == FOC_MODE_OPENLOOP_VF))) {
                foc_app_vf_reset_commands();
            }
            *detail = (float)motor->mode;
            return FOC_CAN_RESULT_OK;
        }

    case FOC_CAN_CMD_SET_TARGET:
        if (can_motion_update_allowed(motor) == 0U) {
            return FOC_CAN_RESULT_STATE_REJECTED;
        }
        if (motor->mode == FOC_MODE_OPENLOOP_VF) {
            return FOC_CAN_RESULT_BAD_ARGUMENT;
        }
        if (motor->mode == FOC_MODE_TORQUE) {
            if (can_float_in_range(argument_f32,
                                   -motor->params.max_current_a,
                                   motor->params.max_current_a) == 0U) {
                return FOC_CAN_RESULT_BAD_ARGUMENT;
            }
        } else if (motor->mode == FOC_MODE_VELOCITY) {
            if (can_float_in_range(argument_f32,
                                   -motor->params.max_rpm,
                                   motor->params.max_rpm) == 0U) {
                return FOC_CAN_RESULT_BAD_ARGUMENT;
            }
        } else if (can_float_in_range(argument_f32,
                                      -1000000.0f, 1000000.0f) == 0U) {
            return FOC_CAN_RESULT_BAD_ARGUMENT;
        }
        foc_motor_set_target(motor, argument_f32);
        *detail = motor->target;
        return FOC_CAN_RESULT_OK;

    case FOC_CAN_CMD_SET_VF_VQ:
        {
            float voltage_max = motor->drv->u_dc * 0.5773503f;

            if (can_motion_update_allowed(motor) == 0U) {
                return FOC_CAN_RESULT_STATE_REJECTED;
            }
            if ((axis != 0U) ||
                (motor->mode != FOC_MODE_OPENLOOP_VF) ||
                (can_float_in_range(argument_f32, 0.0f,
                                    voltage_max) == 0U)) {
                return FOC_CAN_RESULT_BAD_ARGUMENT;
            }
            g_m0_openloop_vq = argument_f32;
            *detail = g_m0_openloop_vq;
            return FOC_CAN_RESULT_OK;
        }

    case FOC_CAN_CMD_SET_VF_RPM:
        if (can_motion_update_allowed(motor) == 0U) {
            return FOC_CAN_RESULT_STATE_REJECTED;
        }
        if ((axis != 0U) ||
            (motor->mode != FOC_MODE_OPENLOOP_VF) ||
            (can_float_in_range(argument_f32,
                                -motor->params.max_rpm,
                                motor->params.max_rpm) == 0U)) {
            return FOC_CAN_RESULT_BAD_ARGUMENT;
        }
        g_m0_openloop_rpm = argument_f32;
        *detail = g_m0_openloop_rpm;
        return FOC_CAN_RESULT_OK;

    case FOC_CAN_CMD_SET_CURRENT_LIMIT:
        /* Calibration temporarily installs much lower trip thresholds.  Do
         * not let a remote parameter update restore the run thresholds while
         * the calibration state machine owns the power stage. */
        if ((motor->state != FOC_STATE_IDLE) ||
            (foc_calib_is_active() != 0U)) {
            return FOC_CAN_RESULT_STATE_REJECTED;
        }
        if (can_float_in_range(argument_f32, 0.0001f,
                               motor->params.hard_current_a) == 0U) {
            return FOC_CAN_RESULT_BAD_ARGUMENT;
        }
        motor->params.max_current_a = argument_f32;
        foc_motor_restore_current_limits(motor);
        foc_pid_set_limit(&motor->pid_vel, argument_f32);
        foc_pid_set_limit(&motor->pid_pos, argument_f32);
        *detail = motor->params.max_current_a;
        return FOC_CAN_RESULT_OK;

    case FOC_CAN_CMD_SAVE_CONFIG:
        if ((motor->state != FOC_STATE_IDLE) ||
            (foc_calib_is_active() != 0U) ||
            (fabsf(motor->velocity_filt_rpm) > 60.0f)) {
            return FOC_CAN_RESULT_STATE_REJECTED;
        }
        if (current_shunt_suspend() == 0U) {
            return FOC_CAN_RESULT_INTERNAL_ERROR;
        }
        if (foc_store_save(motor) == 0U) {
            (void)current_shunt_resume();
            return FOC_CAN_RESULT_INTERNAL_ERROR;
        }
        if (current_shunt_resume() == 0U) {
            return FOC_CAN_RESULT_INTERNAL_ERROR;
        }
        *detail = 1.0f;
        return FOC_CAN_RESULT_OK;

    case FOC_CAN_CMD_SET_VF_SLOPE:
        if (can_motion_update_allowed(motor) == 0U) {
            return FOC_CAN_RESULT_STATE_REJECTED;
        }
        if ((axis != 0U) ||
            (can_float_in_range(argument_f32, 0.0f, 0.01f) == 0U)) {
            return FOC_CAN_RESULT_BAD_ARGUMENT;
        }
        g_m0_vf_slope_v_per_rpm = argument_f32;
        *detail = g_m0_vf_slope_v_per_rpm;
        return FOC_CAN_RESULT_OK;

    default:
        return FOC_CAN_RESULT_BAD_COMMAND;
    }
}

static void can_process_request(const can_rx_item_t *item)
{
    uint8_t response[CAN_RESPONSE_BYTES];
    uint8_t sequence = item->data[2];
    uint8_t command = item->data[3];
    uint8_t axis = item->data[4];
    foc_motor_t *motor = 0;
    foc_can_result_t result;
    float detail = 0.0f;

    g_foc_can_diag.last_sequence = sequence;
    g_foc_can_diag.last_command = command;
    g_foc_can_diag.last_axis = axis;

    if ((item->length != CAN_REQUEST_BYTES) || (item->is_fd == 0U)) {
        result = FOC_CAN_RESULT_BAD_LENGTH;
        g_foc_can_diag.rx_invalid_count++;
    } else if (item->data[0] != FOC_CAN_REQUEST_MAGIC) {
        result = FOC_CAN_RESULT_BAD_MAGIC;
        g_foc_can_diag.rx_invalid_count++;
    } else if (item->data[1] != FOC_CAN_PROTOCOL_VERSION) {
        result = FOC_CAN_RESULT_BAD_VERSION;
        g_foc_can_diag.rx_invalid_count++;
    } else if (axis >= (uint8_t)FOC_NUM_AXES) {
        result = FOC_CAN_RESULT_BAD_AXIS;
        g_foc_can_diag.rx_invalid_count++;
    } else if ((can_last_request_valid != 0U) &&
               (sequence == can_last_request[2])) {
        if (memcmp(item->data, can_last_request, CAN_REQUEST_BYTES) == 0) {
            g_foc_can_diag.duplicate_count++;
            (void)can_response_enqueue(can_last_response);
            return;
        }
        result = FOC_CAN_RESULT_SEQUENCE_CONFLICT;
        motor = foc_app_motor(axis);
    } else {
        motor = foc_app_motor(axis);
        result = can_execute_command(item->data, motor, &detail);
        g_foc_can_diag.command_count++;
    }

    if ((result != FOC_CAN_RESULT_OK) &&
        (result != FOC_CAN_RESULT_BAD_MAGIC) &&
        (result != FOC_CAN_RESULT_BAD_VERSION) &&
        (result != FOC_CAN_RESULT_BAD_LENGTH) &&
        (result != FOC_CAN_RESULT_BAD_AXIS)) {
        g_foc_can_diag.command_reject_count++;
    }

    can_build_response(response, sequence, command, axis, result,
                       motor, detail);
    g_foc_can_diag.last_result = (uint8_t)result;

    if ((item->length == CAN_REQUEST_BYTES) && (item->is_fd != 0U) &&
        (item->data[0] == FOC_CAN_REQUEST_MAGIC) &&
        (item->data[1] == FOC_CAN_PROTOCOL_VERSION) &&
        (axis < (uint8_t)FOC_NUM_AXES) &&
        (result != FOC_CAN_RESULT_SEQUENCE_CONFLICT)) {
        memcpy(can_last_request, item->data, CAN_REQUEST_BYTES);
        memcpy(can_last_response, response, CAN_RESPONSE_BYTES);
        can_last_request_valid = 1U;
    }

    (void)can_response_enqueue(response);
}

static void can_send_status(void)
{
    uint8_t data[CAN_STATUS_BYTES] = {0};
    foc_motor_t *motor = foc_app_motor(0U);

    data[0] = FOC_CAN_STATUS_MAGIC;
    data[1] = FOC_CAN_PROTOCOL_VERSION;
    data[2] = 0U;
    data[3] = (uint8_t)motor->state;
    data[4] = (uint8_t)motor->mode;
    data[5] = motor->calib.valid;
    data[6] = (uint8_t)foc_calib_get_state();
    data[7] = current_shunt_is_ready();
    can_put_u32(&data[8], (uint32_t)motor->safety.fault_code);
    can_put_u32(&data[12], (uint32_t)g_current_shunt_diag.fault_code);
    can_put_f32(&data[16], motor->velocity_filt_rpm);
    can_put_f32(&data[20], can_status_target(motor, 0U));
    can_put_f32(&data[24], motor->i_dq_filt.q);
    can_put_f32(&data[28], can_status_vq(motor, 0U));

    if (can_try_send(FOC_CAN_STATUS_ID, FDCAN_DLC_BYTES_32, data) == HAL_OK) {
        g_foc_can_diag.status_count++;
    } else {
        g_foc_can_diag.tx_drop_count++;
    }
}

static void can_send_fault_event(void)
{
    uint8_t data[CAN_FAULT_BYTES] = {0};
    foc_motor_t *motor = foc_app_motor(0U);

    data[0] = FOC_CAN_FAULT_MAGIC;
    data[1] = FOC_CAN_PROTOCOL_VERSION;
    data[2] = can_fault_sequence;
    data[3] = 0U;
    data[4] = (uint8_t)motor->state;
    data[5] = (uint8_t)motor->mode;
    data[6] = motor->safety.fault_code;
    data[7] = g_current_shunt_diag.fault_code;
    can_put_f32(&data[8], motor->safety.peak_current_a);
    can_put_u32(&data[12], HAL_GetTick());

    if (can_try_send(FOC_CAN_FAULT_ID, FDCAN_DLC_BYTES_16, data) == HAL_OK) {
        can_fault_pending = 0U;
        can_fault_sequence++;
        g_foc_can_diag.fault_event_count++;
    }
}

void foc_can_init(void)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(CAN_SHD_GPIO_Port, CAN_SHD_Pin, GPIO_PIN_SET);

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = FOC_CAN_COMMAND_ID;
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
        g_foc_can_diag.error_count++;
        g_foc_can_diag.last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
        return;
    }

    HAL_GPIO_WritePin(CAN_SHD_GPIO_Port, CAN_SHD_Pin, GPIO_PIN_RESET);
    can_last_status_tick = HAL_GetTick();
    can_last_recover_tick = can_last_status_tick;
    g_foc_can_diag.init_ok = 1U;
}

void foc_can_task(void)
{
    can_rx_item_t item;
    foc_motor_t *motor;
    uint32_t now;

    if (g_foc_can_diag.init_ok == 0U) {
        return;
    }

    now = HAL_GetTick();
    g_foc_can_diag.protocol_status = hfdcan1.Instance->PSR;
    g_foc_can_diag.error_counter = hfdcan1.Instance->ECR;

    if ((can_bus_recover_pending != 0U) &&
        ((uint32_t)(now - can_last_recover_tick) >= CAN_BUS_RECOVER_MS)) {
        can_last_recover_tick = now;
        if ((HAL_FDCAN_Stop(&hfdcan1) == HAL_OK) &&
            (HAL_FDCAN_Start(&hfdcan1) == HAL_OK)) {
            can_bus_recover_pending = 0U;
            g_foc_can_diag.bus_recover_count++;
        } else {
            g_foc_can_diag.error_count++;
            g_foc_can_diag.last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
        }
    }

    can_flush_responses();
    while (can_rx_pop(&item) != 0U) {
        can_process_request(&item);
        can_flush_responses();
    }

    motor = foc_app_motor(0U);
    if ((can_last_motor_fault != motor->safety.fault_code) ||
        (can_last_current_fault != g_current_shunt_diag.fault_code)) {
        can_last_motor_fault = motor->safety.fault_code;
        can_last_current_fault = g_current_shunt_diag.fault_code;
        can_fault_pending = 1U;
    }

    if ((can_fault_pending != 0U) && (can_tx_tail == can_tx_head)) {
        can_send_fault_event();
    }

    if (((uint32_t)(now - can_last_status_tick) >=
         FOC_CAN_STATUS_PERIOD_MS) &&
        (can_tx_tail == can_tx_head) &&
        (can_fault_pending == 0U)) {
        can_last_status_tick = now;
        can_send_status();
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
        uint8_t next;
        uint8_t length;

        g_foc_can_diag.rx_count++;
        g_foc_can_diag.last_rx_is_fd =
            (header.FDFormat == FDCAN_FD_CAN) ? 1U : 0U;
        g_foc_can_diag.last_rx_brs =
            (header.BitRateSwitch == FDCAN_BRS_ON) ? 1U : 0U;

        if ((header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.Identifier != FOC_CAN_COMMAND_ID)) {
            g_foc_can_diag.rx_invalid_count++;
            continue;
        }

        next = can_next_index(can_rx_head, CAN_RX_QUEUE_SIZE);
        if (next == can_rx_tail) {
            g_foc_can_diag.rx_queue_drop_count++;
            continue;
        }

        length = can_dlc_bytes(header.DataLength);
        can_rx_queue[can_rx_head].length = length;
        can_rx_queue[can_rx_head].is_fd =
            (header.FDFormat == FDCAN_FD_CAN) ? 1U : 0U;
        can_rx_queue[can_rx_head].brs =
            (header.BitRateSwitch == FDCAN_BRS_ON) ? 1U : 0U;
        can_rx_queue[can_rx_head].reserved = 0U;
        memset(can_rx_queue[can_rx_head].data, 0, CAN_REQUEST_BYTES);
        if ((length != 0xFFU) && (length != 0U)) {
            uint8_t copy_length = (length > CAN_REQUEST_BYTES)
                                ? CAN_REQUEST_BYTES : length;
            memcpy(can_rx_queue[can_rx_head].data, data, copy_length);
        }
        can_rx_head = next;
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) {
        g_foc_can_diag.error_count++;
        g_foc_can_diag.last_hal_error = HAL_FDCAN_GetError(hfdcan);
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t error_status_its)
{
    if (hfdcan == &hfdcan1) {
        g_foc_can_diag.error_count++;
        g_foc_can_diag.last_hal_error = error_status_its;
        if ((error_status_its & FDCAN_IT_BUS_OFF) != 0U) {
            g_foc_can_diag.bus_off_count++;
            can_bus_recover_pending = 1U;
        }
    }
}

#else /* FOC_CAN_ENABLE */

volatile foc_can_diag_t g_foc_can_diag = {0};

void foc_can_init(void)
{
    g_foc_can_diag.enabled = 0U;
}

void foc_can_task(void)
{
}

#endif /* FOC_CAN_ENABLE */
