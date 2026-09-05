/**
 * @file    foc_can.h
 * @brief   Matchstick HFOC CAN FD control/status protocol.
 *
 * The protocol is a machine interface.  USART2 CLI remains enabled for
 * manual commissioning and VOFA telemetry.  Both interfaces ultimately call
 * the same foc_motor_t APIs and therefore share the same state/fault guards.
 *
 * Bus baseline:
 *   - ISO CAN FD, 11-bit standard identifiers
 *   - nominal bit rate 500 kbit/s
 *   - BRS off by default (FOC_CAN_BRS_ENABLE=0)
 *   - optional data bit rate 2 Mbit/s when BRS is enabled at both ends
 *
 * IDs:
 *   0x300 board -> host : 32-byte periodic status
 *   0x301 host  -> board: 16-byte command request
 *   0x302 board -> host : 16-byte command response
 *   0x303 board -> host : 16-byte fault-change event
 */

#ifndef FOC_CAN_H
#define FOC_CAN_H

#include <stdint.h>
#include "foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FOC_CAN_ENABLE
#define FOC_CAN_ENABLE 0
#endif

#ifndef FOC_CAN_BRS_ENABLE
#define FOC_CAN_BRS_ENABLE 0
#endif

#ifndef FOC_CAN_AUTO_RETRANSMISSION
#define FOC_CAN_AUTO_RETRANSMISSION 1
#endif

#ifndef FOC_CAN_STATUS_PERIOD_MS
#define FOC_CAN_STATUS_PERIOD_MS 100U
#endif

#define FOC_CAN_STATUS_ID          0x300U
#define FOC_CAN_COMMAND_ID         0x301U
#define FOC_CAN_RESPONSE_ID        0x302U
#define FOC_CAN_FAULT_ID           0x303U

#define FOC_CAN_PROTOCOL_VERSION   1U
#define FOC_CAN_REQUEST_MAGIC      0xA5U
#define FOC_CAN_RESPONSE_MAGIC     0x5AU
#define FOC_CAN_STATUS_MAGIC       0x53U  /* ASCII 'S'. */
#define FOC_CAN_FAULT_MAGIC        0x46U  /* ASCII 'F'. */

/** Command code in command-request byte 3. */
typedef enum {
    FOC_CAN_CMD_GET_STATUS        = 0x01,
    FOC_CAN_CMD_ENABLE            = 0x02,
    FOC_CAN_CMD_DISABLE           = 0x03,
    FOC_CAN_CMD_CLEAR_FAULT       = 0x04,
    FOC_CAN_CMD_CALIBRATE         = 0x05,
    FOC_CAN_CMD_SET_MODE          = 0x06,
    FOC_CAN_CMD_SET_TARGET        = 0x07,
    FOC_CAN_CMD_SET_VF_VQ         = 0x08,
    FOC_CAN_CMD_SET_VF_RPM        = 0x09,
    FOC_CAN_CMD_SET_CURRENT_LIMIT = 0x0A,
    FOC_CAN_CMD_SAVE_CONFIG       = 0x0B,
    FOC_CAN_CMD_SET_VF_SLOPE      = 0x0C
} foc_can_command_t;

/** Result code in command-response byte 5. */
typedef enum {
    FOC_CAN_RESULT_OK                = 0,
    FOC_CAN_RESULT_BAD_MAGIC         = 1,
    FOC_CAN_RESULT_BAD_VERSION       = 2,
    FOC_CAN_RESULT_BAD_LENGTH        = 3,
    FOC_CAN_RESULT_BAD_AXIS          = 4,
    FOC_CAN_RESULT_BAD_COMMAND       = 5,
    FOC_CAN_RESULT_BAD_ARGUMENT      = 6,
    FOC_CAN_RESULT_STATE_REJECTED    = 7,
    FOC_CAN_RESULT_NOT_CALIBRATED    = 8,
    FOC_CAN_RESULT_BUSY              = 9,
    FOC_CAN_RESULT_SEQUENCE_CONFLICT = 10,
    FOC_CAN_RESULT_INTERNAL_ERROR    = 11
} foc_can_result_t;

/** Read-only counters/snapshots for Keil Watch. */
typedef struct {
    volatile uint8_t enabled;
    volatile uint8_t init_ok;
    volatile uint8_t last_command;
    volatile uint8_t last_result;
    volatile uint8_t last_sequence;
    volatile uint8_t last_axis;
    volatile uint8_t last_rx_is_fd;
    volatile uint8_t last_rx_brs;
    volatile uint32_t rx_count;
    volatile uint32_t rx_queue_drop_count;
    volatile uint32_t rx_invalid_count;
    volatile uint32_t command_count;
    volatile uint32_t command_reject_count;
    volatile uint32_t duplicate_count;
    volatile uint32_t response_count;
    volatile uint32_t response_drop_count;
    volatile uint32_t status_count;
    volatile uint32_t fault_event_count;
    volatile uint32_t tx_drop_count;
    volatile uint32_t error_count;
    volatile uint32_t bus_off_count;
    volatile uint32_t bus_recover_count;
    volatile uint32_t last_hal_error;
    volatile uint32_t protocol_status;
    volatile uint32_t error_counter;
} foc_can_diag_t;

extern volatile foc_can_diag_t g_foc_can_diag;

/** Start FDCAN, install the command filter and enable the SIT1042T. */
void foc_can_init(void);

/** Execute queued commands and send response/status/fault frames. */
void foc_can_task(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CAN_H */
