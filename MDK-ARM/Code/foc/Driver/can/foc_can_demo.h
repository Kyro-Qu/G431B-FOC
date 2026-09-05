/**
 * @file    foc_can_demo.h
 * @brief   Minimal CAN FD bring-up demo for the Matchstick HFOC board.
 *
 * Bus settings:
 *   - ISO CAN FD; the initial demo frame does not use BRS
 *   - nominal/arbitration bit rate: 500 kbit/s
 *   - initial data phase: 500 kbit/s (BRS off)
 *   - configured BRS data bit rate: 2 Mbit/s for later use
 *   - 11-bit standard identifiers
 *
 * Demo frames:
 *   - board -> host, ID 0x300, 8 bytes: "HFOC_FD!" once per second
 *   - host -> board, ID 0x301, 8 bytes: "CALIBFD!" (BRS on or off)
 *     The receive interrupt only records the request.  foc_can_demo_task()
 *     starts calibration later from the main loop when axis 0 is IDLE.
 */

#ifndef FOC_CAN_DEMO_H
#define FOC_CAN_DEMO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_CAN_DEMO_TX_ID 0x300U
#define FOC_CAN_DEMO_RX_ID 0x301U

typedef struct {
    volatile uint8_t init_ok;
    volatile uint8_t calib_request_pending;
    volatile uint8_t last_rx_is_fd;
    volatile uint8_t last_rx_brs;
    volatile uint32_t tx_count;
    volatile uint32_t tx_drop_count;
    volatile uint32_t tx_attempt_count;
    volatile uint32_t tx_fifo_status;
    volatile uint32_t rx_count;
    volatile uint32_t rx_match_count;
    volatile uint32_t calib_start_count;
    volatile uint32_t calib_reject_count;
    volatile uint32_t error_count;
    volatile uint32_t bus_off_count;
    volatile uint32_t bus_recover_count;
    volatile uint32_t last_rx_id;
    volatile uint32_t last_error;
    volatile uint32_t protocol_status;
    volatile uint32_t error_counter;
} foc_can_demo_diag_t;

/** Read-only diagnostics for Keil Watch. */
extern volatile foc_can_demo_diag_t g_foc_can_demo_diag;

/** Configure the RX filter, start FDCAN1 and enable the SIT1042T. */
void foc_can_demo_init(void);

/** Main-loop task: periodic TX and deferred calibration request handling. */
void foc_can_demo_task(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CAN_DEMO_H */
