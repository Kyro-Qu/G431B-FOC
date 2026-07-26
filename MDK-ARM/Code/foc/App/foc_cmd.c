/**
 * @file    foc_cmd.c
 * @brief   串口命令行实现
 *
 * 接收链路：HAL_UARTEx_ReceiveToIdle_DMA → RxEvent 回调把新字节搬进
 * 行缓冲 → 主循环 foc_cmd_task() 发现整行后解析执行。
 * 回调里只做搬运，绝不解析、绝不打印 —— 快环优先。
 */

#include "foc_cmd.h"
#include "foc_app.h"
#include "foc_calib.h"
#include "foc_telemetry.h"
#include "main.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart2;

#define CMD_RX_DMA_SIZE  64U
#define CMD_LINE_SIZE    64U

static uint8_t rx_dma_buf[CMD_RX_DMA_SIZE];
static char line_buf[CMD_LINE_SIZE];
static volatile uint16_t line_len = 0U;
static volatile uint8_t line_ready = 0U;

static uint8_t cur_axis = 0U;

/* ---------------- 输出 ---------------- */

/*
 * 阻塞发送一段响应。遥测帧只有 ~70 字节（6.5M 波特率下约 0.1 ms），
 * 简单等待 UART 空闲即可，不会与遥测 DMA 冲突。
 */
static void cmd_print(const char *fmt, ...)
{
    static char out[192];
    va_list ap;
    int n;
    uint32_t t0;

    va_start(ap, fmt);
    n = vsnprintf(out, sizeof(out), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }

    t0 = HAL_GetTick();
    while ((HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) &&
           ((uint32_t)(HAL_GetTick() - t0) < 5U)) {
    }
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)out, (uint16_t)n, 20U);
}

static const char *state_name(foc_state_t s)
{
    switch (s) {
    case FOC_STATE_IDLE:  return "IDLE";
    case FOC_STATE_RUN:   return "RUN";
    case FOC_STATE_CALIB: return "CALIB";
    case FOC_STATE_FAULT: return "FAULT";
    default:              return "?";
    }
}

static const char *mode_name(foc_mode_t m)
{
    switch (m) {
    case FOC_MODE_OPENLOOP_VF: return "vf";
    case FOC_MODE_TORQUE:      return "iq";
    case FOC_MODE_VELOCITY:    return "vel";
    case FOC_MODE_POSITION:    return "pos";
    default:                   return "?";
    }
}

static void cmd_print_status(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)FOC_NUM_AXES; i++) {
        foc_motor_t *m = foc_app_motor(i);

        cmd_print("M%u %s mode=%s tgt=%.3f vel=%.1frpm pos=%.2frad "
                  "iq=%.3fA calib=%u fault=%u%s\r\n",
                  (unsigned)i, state_name(m->state), mode_name(m->mode),
                  (double)m->target, (double)m->velocity_rpm,
                  (double)m->position_rad, (double)m->i_dq_filt.q,
                  (unsigned)m->calib.valid,
                  (unsigned)m->safety.fault_code,
                  (i == cur_axis) ? "  <-" : "");
    }
    cmd_print("calib_state=%u telem=%u\r\n",
              (unsigned)foc_calib_get_state(),
              (unsigned)foc_telemetry_get_enable());
}

static void cmd_print_help(void)
{
    cmd_print(
        "FOC cmd:\r\n"
        " s              status\r\n"
        " a <n>          select axis (now M%u)\r\n"
        " e <0|1>        disarm/arm\r\n"
        " c              start calibration\r\n"
        " f              clear fault\r\n"
        " m <vf|iq|vel|pos>  control mode\r\n"
        " t <val>        target (V/A/RPM/rad by mode)\r\n"
        " vq <v> rpm <v> open-loop voltage/speed\r\n"
        " cb <rad/s>     current loop bandwidth\r\n"
        " vp/vi <v>      velocity PI gains\r\n"
        " pp <v>         position P gain\r\n"
        " lim <A>        soft current limit\r\n"
        " log <0|1>      telemetry stream\r\n",
        (unsigned)cur_axis);
}

/* ---------------- 解析 ---------------- */

static uint8_t cmd_parse_float(const char *s, float *out)
{
    char *end = 0;
    float v;

    if ((s == 0) || (*s == '\0')) {
        return 0U;
    }
    v = strtof(s, &end);
    if (end == s) {
        return 0U;
    }
    *out = v;
    return 1U;
}

static void cmd_execute(char *line)
{
    foc_motor_t *m = foc_app_motor(cur_axis);
    char *cmd;
    char *arg;
    float val = 0.0f;
    uint8_t has_val;

    cmd = strtok(line, " \t");
    if (cmd == 0) {
        return;
    }
    arg = strtok(0, " \t");
    has_val = cmd_parse_float(arg, &val);

    if (strcmp(cmd, "help") == 0) {
        cmd_print_help();

    } else if (strcmp(cmd, "s") == 0) {
        cmd_print_status();

    } else if (strcmp(cmd, "a") == 0) {
        if (has_val && ((uint8_t)val < (uint8_t)FOC_NUM_AXES)) {
            cur_axis = (uint8_t)val;
            cmd_print("axis M%u\r\n", (unsigned)cur_axis);
        } else {
            cmd_print("err: a 0..%u\r\n", (unsigned)(FOC_NUM_AXES - 1U));
        }

    } else if (strcmp(cmd, "e") == 0) {
        if (has_val && (val != 0.0f)) {
            if (foc_motor_arm(m) != 0U) {
                cmd_print("M%u armed (%s)\r\n",
                          (unsigned)cur_axis, mode_name(m->mode));
            } else {
                cmd_print("err: arm rejected, state=%s fault=%u\r\n",
                          state_name(m->state),
                          (unsigned)m->safety.fault_code);
            }
        } else {
            foc_motor_disarm(m);
            cmd_print("M%u idle\r\n", (unsigned)cur_axis);
        }

    } else if (strcmp(cmd, "c") == 0) {
        foc_calib_start(m);
        cmd_print("M%u calib start\r\n", (unsigned)cur_axis);

    } else if (strcmp(cmd, "f") == 0) {
        foc_motor_clear_fault(m);
        cmd_print("M%u fault cleared, state=%s\r\n",
                  (unsigned)cur_axis, state_name(m->state));

    } else if (strcmp(cmd, "m") == 0) {
        if (arg == 0) {
            cmd_print("mode=%s\r\n", mode_name(m->mode));
        } else if (strcmp(arg, "vf") == 0) {
            foc_motor_set_mode(m, FOC_MODE_OPENLOOP_VF);
            cmd_print("M%u mode=vf\r\n", (unsigned)cur_axis);
        } else if (strcmp(arg, "iq") == 0) {
            foc_motor_set_mode(m, FOC_MODE_TORQUE);
            cmd_print("M%u mode=iq\r\n", (unsigned)cur_axis);
        } else if (strcmp(arg, "vel") == 0) {
            foc_motor_set_mode(m, FOC_MODE_VELOCITY);
            cmd_print("M%u mode=vel\r\n", (unsigned)cur_axis);
        } else if (strcmp(arg, "pos") == 0) {
            foc_motor_set_mode(m, FOC_MODE_POSITION);
            cmd_print("M%u mode=pos\r\n", (unsigned)cur_axis);
        } else {
            cmd_print("err: m vf|iq|vel|pos\r\n");
        }

    } else if (strcmp(cmd, "t") == 0) {
        if (has_val) {
            foc_motor_set_target(m, val);
            cmd_print("M%u t=%.3f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "vq") == 0) {
        if (has_val) {
            if (cur_axis == 0U) {
                g_m0_openloop_vq = val;
            }
            m->v_openloop.q = val;
            cmd_print("M%u vq=%.3f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "rpm") == 0) {
        if (has_val) {
            if (cur_axis == 0U) {
                g_m0_openloop_rpm = val;
            }
            foc_motor_openloop_spin(m, val, m->v_openloop.d, m->v_openloop.q);
            cmd_print("M%u rpm=%.1f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "cb") == 0) {
        if (has_val && (val > 0.0f)) {
            float v_max = m->drv->u_dc * 0.5773503f;

            m->cfg.current_bw_rads = val;
            foc_pid_init(&m->pid_id, m->params.ls_henry * val,
                         m->params.rs_ohm * val, 0.0f, v_max, 0.0f);
            foc_pid_init(&m->pid_iq, m->params.ls_henry * val,
                         m->params.rs_ohm * val, 0.0f, v_max, 0.0f);
            cmd_print("M%u cb=%.0f kp=%.4f ki=%.2f\r\n",
                      (unsigned)cur_axis, (double)val,
                      (double)m->pid_id.kp, (double)m->pid_id.ki);
        }

    } else if (strcmp(cmd, "vp") == 0) {
        if (has_val) {
            m->pid_vel.kp = val;
            cmd_print("M%u vp=%.4f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "vi") == 0) {
        if (has_val) {
            m->pid_vel.ki = val;
            cmd_print("M%u vi=%.4f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "pp") == 0) {
        if (has_val) {
            m->pid_pos.kp = val;
            cmd_print("M%u pp=%.2f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "lim") == 0) {
        if (has_val && (val > 0.0f) && (val <= m->params.hard_current_a)) {
            m->params.max_current_a = val;
            m->safety.current_limit_a = val;
            foc_pid_set_limit(&m->pid_vel, val);
            cmd_print("M%u lim=%.2fA\r\n", (unsigned)cur_axis, (double)val);
        } else {
            cmd_print("err: 0 < lim <= %.1f\r\n",
                      (double)m->params.hard_current_a);
        }

    } else if (strcmp(cmd, "log") == 0) {
        foc_telemetry_set_enable((has_val && (val != 0.0f)) ? 1U : 0U);
        cmd_print("telem=%u\r\n", (unsigned)foc_telemetry_get_enable());

    } else {
        cmd_print("err: unknown '%s', try help\r\n", cmd);
    }
}

/* ---------------- 接收 ---------------- */

void foc_cmd_init(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, CMD_RX_DMA_SIZE);
}

void foc_cmd_task(void)
{
    if (line_ready == 0U) {
        return;
    }

    /* 解析期间关闭"行就绪"标志即可，接收回调只追加不解析 */
    line_buf[(line_len < CMD_LINE_SIZE) ? line_len : (CMD_LINE_SIZE - 1U)] = '\0';
    line_len = 0U;
    line_ready = 0U;
    cmd_execute(line_buf);
}

/* DMA 接收事件：把"新增的"字节搬进行缓冲，遇到行尾置标志。
 * 事件可能是半满(HT)/全满(TC)/空闲(IDLE)：HT 时接收仍在进行，
 * 用 rx_last_pos 记录已处理位置，避免同一段字节被处理两次。 */
static uint16_t rx_last_pos = 0U;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t i;

    if (huart->Instance != USART2) {
        return;
    }

    for (i = rx_last_pos; i < Size; i++) {
        char ch = (char)rx_dma_buf[i];

        if ((ch == '\r') || (ch == '\n')) {
            if ((line_len > 0U) && (line_ready == 0U)) {
                line_ready = 1U;
            }
        } else if ((line_len < (CMD_LINE_SIZE - 1U)) && (line_ready == 0U)) {
            line_buf[line_len] = ch;
            line_len = (uint16_t)(line_len + 1U);
        }
    }
    rx_last_pos = Size;

    /* IDLE/TC 事件后 HAL 已停止接收（READY），重新武装并从头开始；
     * HT 事件时仍在接收（BUSY_RX），此调用返回 BUSY，无副作用 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, CMD_RX_DMA_SIZE)
        == HAL_OK) {
        rx_last_pos = 0U;
    }
}

/* 接收出错（溢出/噪声帧）：清错误并重启接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, CMD_RX_DMA_SIZE)
            == HAL_OK) {
            rx_last_pos = 0U;
        }
    }
}
