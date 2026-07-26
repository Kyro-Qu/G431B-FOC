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
#include "foc_ident.h"
#include "foc_telemetry.h"
#include "../Core/foc_port.h"
#include "../HAL/foc_board_g431.h"
#include "../HAL/foc_store.h"
#include "main.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

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
 * 阻塞发送一段响应。先挂起遥测（ISR 里不再启动新的 DMA 发送），
 * 等在途 DMA 帧发完（gState 回 READY，~0.1ms@6.5M），再独占发送。
 * 注意判据只看发送方向 gState：组合状态 HAL_UART_GetState() 因
 * RX 常驻 DMA 空闲接收永远不等于 READY。
 * 公开给其它 App 模块（如 foc_ident）使用，仅限主循环上下文。
 */
void foc_cmd_print(const char *fmt, ...)
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

    foc_telemetry_suspend(1U);
    t0 = HAL_GetTick();
    while ((huart2.gState != HAL_UART_STATE_READY) &&
           ((uint32_t)(HAL_GetTick() - t0) < 5U)) {
    }
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)out, (uint16_t)n, 20U);
    foc_telemetry_suspend(0U);
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

        /* calib 后缀 *：偏移来自 Flash 存储（快速索引搜索可用） */
        foc_cmd_print("M%u %s mode=%s tgt=%.3f vel=%.1frpm pos=%.2frad "
                  "iq=%.3fA calib=%u%s fault=%u%s\r\n",
                  (unsigned)i, state_name(m->state), mode_name(m->mode),
                  (double)m->target, (double)m->velocity_rpm,
                  (double)m->position_rad, (double)m->i_dq_filt.q,
                  (unsigned)m->calib.valid,
                  (m->calib.from_store != 0U) ? "*" : "",
                  (unsigned)m->safety.fault_code,
                  (i == cur_axis) ? "  <-" : "");
    }
    foc_cmd_print("calib_state=%u telem=%u cpu=%.1f%% (max %.1f%%)\r\n",
              (unsigned)foc_calib_get_state(),
              (unsigned)foc_telemetry_get_enable(),
              (double)g_foc_cpu_diag.load_pct,
              (double)(100.0f * (float)g_foc_cpu_diag.max_cycles /
                       (170000000.0f / FOC_PWM_FREQ_HZ)));
}

static void cmd_print_help(void)
{
    foc_cmd_print(
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
        " id / id a      measure Rs+Ls / apply result\r\n"
        " save / save e  save params+calib to flash / erase store\r\n"
        " c full         force full calibration (ignore stored offset)\r\n"
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
    /* 拒收 NaN/Inf："rpm nan" 之类的输入若被放行，NaN 会经开环角度
     * 步进绕过快环的电压 NaN 防护，最终变成 CCR=0 的静默低边刹车 */
    if ((v != v) || ((v - v) != 0.0f)) {
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
            foc_cmd_print("axis M%u\r\n", (unsigned)cur_axis);
        } else {
            foc_cmd_print("err: a 0..%u\r\n", (unsigned)(FOC_NUM_AXES - 1U));
        }

    } else if (strcmp(cmd, "e") == 0) {
        if (has_val && (val != 0.0f)) {
            if (foc_motor_arm(m) != 0U) {
                foc_cmd_print("M%u armed (%s)\r\n",
                          (unsigned)cur_axis, mode_name(m->mode));
            } else {
                foc_cmd_print("err: arm rejected, state=%s fault=%u\r\n",
                          state_name(m->state),
                          (unsigned)m->safety.fault_code);
            }
        } else {
            foc_motor_disarm(m);
            foc_cmd_print("M%u idle\r\n", (unsigned)cur_axis);
        }

    } else if (strcmp(cmd, "c") == 0) {
        if (m->state != FOC_STATE_IDLE) {
            foc_cmd_print("err: calib needs IDLE, state=%s\r\n",
                      state_name(m->state));
        } else if (foc_calib_is_active() != 0U) {
            foc_cmd_print("err: calib busy on another axis\r\n");
        } else {
            /* `c full`：忽略存储偏移，强制完整校准（对齐+找Z 重新测）。
             * 校准若根本没启动（前置检查失败），恢复快速路径资格 */
            uint8_t want_full = ((arg != 0) &&
                                 (strcmp(arg, "full") == 0)) ? 1U : 0U;
            uint8_t was_from_store = m->calib.from_store;

            if (want_full != 0U) {
                m->calib.from_store = 0U;
            }
            foc_calib_start(m);
            if (m->state == FOC_STATE_CALIB) {
                foc_cmd_print("M%u calib start%s\r\n", (unsigned)cur_axis,
                              (want_full != 0U) ? " (full)" : "");
            } else {
                if (want_full != 0U) {
                    m->calib.from_store = was_from_store;
                }
                foc_cmd_print("err: calib rejected, fault=%u\r\n",
                          (unsigned)m->safety.fault_code);
            }
        }

    } else if (strcmp(cmd, "f") == 0) {
        foc_motor_clear_fault(m);
        foc_cmd_print("M%u fault cleared, state=%s\r\n",
                  (unsigned)cur_axis, state_name(m->state));

    } else if (strcmp(cmd, "m") == 0) {
        if (arg == 0) {
            foc_cmd_print("mode=%s\r\n", mode_name(m->mode));
        } else {
            foc_mode_t want;
            uint8_t known = 1U;

            if (strcmp(arg, "vf") == 0) {
                want = FOC_MODE_OPENLOOP_VF;
            } else if (strcmp(arg, "iq") == 0) {
                want = FOC_MODE_TORQUE;
            } else if (strcmp(arg, "vel") == 0) {
                want = FOC_MODE_VELOCITY;
            } else if (strcmp(arg, "pos") == 0) {
                want = FOC_MODE_POSITION;
            } else {
                known = 0U;
                foc_cmd_print("err: m vf|iq|vel|pos\r\n");
            }
            if (known != 0U) {
                if (foc_motor_set_mode(m, want) != 0U) {
                    foc_cmd_print("M%u mode=%s\r\n",
                                  (unsigned)cur_axis, mode_name(m->mode));
                } else {
                    foc_cmd_print("err: mode rejected, state=%s calib=%u"
                                  " (closed loop needs calib)\r\n",
                                  state_name(m->state),
                                  (unsigned)m->calib.valid);
                }
            }
        }

    } else if (strcmp(cmd, "t") == 0) {
        if (has_val) {
            foc_motor_set_target(m, val);
            /* 开环 V/f 模式下 t 的语义是电压（V），直接落到 vq 命令 */
            if (m->mode == FOC_MODE_OPENLOOP_VF) {
                if (cur_axis == 0U) {
                    g_m0_openloop_vq = val;
                }
                m->v_openloop.q = val;
            }
            foc_cmd_print("M%u t=%.3f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "vq") == 0) {
        if (has_val) {
            if (cur_axis == 0U) {
                g_m0_openloop_vq = val;
            }
            m->v_openloop.q = val;
            foc_cmd_print("M%u vq=%.3f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "rpm") == 0) {
        if (has_val) {
            if (cur_axis == 0U) {
                g_m0_openloop_rpm = val;
            }
            foc_motor_openloop_spin(m, val, m->v_openloop.d, m->v_openloop.q);
            foc_cmd_print("M%u rpm=%.1f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "cb") == 0) {
        if (has_val && (val > 0.0f)) {
            float v_max = m->drv->u_dc * 0.5773503f;

            m->cfg.current_bw_rads = val;
            foc_pid_init(&m->pid_id, m->params.ls_henry * val,
                         m->params.rs_ohm * val, 0.0f, v_max, 0.0f);
            foc_pid_init(&m->pid_iq, m->params.ls_henry * val,
                         m->params.rs_ohm * val, 0.0f, v_max, 0.0f);
            foc_cmd_print("M%u cb=%.0f kp=%.4f ki=%.2f\r\n",
                      (unsigned)cur_axis, (double)val,
                      (double)m->pid_id.kp, (double)m->pid_id.ki);
        }

    } else if (strcmp(cmd, "vp") == 0) {
        if (has_val) {
            m->pid_vel.kp = val;
            foc_cmd_print("M%u vp=%.4f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "vi") == 0) {
        if (has_val) {
            m->pid_vel.ki = val;
            foc_cmd_print("M%u vi=%.4f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "pp") == 0) {
        if (has_val) {
            m->pid_pos.kp = val;
            foc_cmd_print("M%u pp=%.2f\r\n", (unsigned)cur_axis, (double)val);
        }

    } else if (strcmp(cmd, "lim") == 0) {
        if (has_val && (val > 0.0f) && (val <= m->params.hard_current_a)) {
            m->params.max_current_a = val;
            m->safety.current_limit_a = val;
            foc_pid_set_limit(&m->pid_vel, val);
            foc_cmd_print("M%u lim=%.2fA\r\n", (unsigned)cur_axis, (double)val);
        } else {
            foc_cmd_print("err: 0 < lim <= %.1f\r\n",
                      (double)m->params.hard_current_a);
        }

    } else if (strcmp(cmd, "id") == 0) {
        if ((arg != 0) && (strcmp(arg, "a") == 0)) {
            if (foc_ident_apply(m) != 0U) {
                foc_cmd_print("M%u applied: Rs=%.4f Ls=%.2fuH, "
                              "current loop retuned\r\n",
                              (unsigned)cur_axis,
                              (double)m->params.rs_ohm,
                              (double)(m->params.ls_henry * 1e6f));
            } else {
                foc_cmd_print("err: no valid ident result (run 'id' first, "
                              "axis must not be running)\r\n");
            }
        } else {
            foc_ident_start(m);
        }

    } else if (strcmp(cmd, "save") == 0) {
        /* 擦除和保存都会 stall 总线 ~22ms（含 16kHz 保护中断），
         * 必须停机；转子高速滑行时 22ms 的编码器计数间隙会被
         * 毛刺滤波丢弃，永久污染电角度——也要等停稳 */
        if ((m->state == FOC_STATE_RUN) ||
            (m->state == FOC_STATE_CALIB)) {
            foc_cmd_print("err: flash op needs IDLE (stalls 22ms)\r\n");
        } else if (fabsf(m->velocity_filt_rpm) > 60.0f) {
            foc_cmd_print("err: rotor still spinning, wait for stop\r\n");
        } else if ((arg != 0) && (strcmp(arg, "e") == 0)) {
            foc_cmd_print(foc_store_erase() != 0U
                              ? "store erased (defaults on next boot)\r\n"
                              : "err: erase failed\r\n");
        } else {
            uint8_t with_calib = ((m->calib.valid != 0U) ||
                                  (m->calib.from_store != 0U)) ? 1U : 0U;

            if (foc_store_save(m) != 0U) {
                foc_cmd_print("saved to flash%s (auto-load on boot)\r\n",
                              (with_calib != 0U) ? "" : ", no calib in store");
            } else {
                foc_cmd_print("err: flash save failed\r\n");
            }
        }

    } else if (strcmp(cmd, "log") == 0) {
        if (has_val) {
            foc_telemetry_set_enable((val != 0.0f) ? 1U : 0U);
        }
        foc_cmd_print("telem=%u\r\n", (unsigned)foc_telemetry_get_enable());

    } else {
        foc_cmd_print("err: unknown '%s', try help\r\n", cmd);
    }
}

/* ---------------- 接收 ---------------- */

void foc_cmd_init(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, CMD_RX_DMA_SIZE);
}

void foc_cmd_task(void)
{
    char local[CMD_LINE_SIZE];
    uint32_t pm;
    uint16_t n;

    if (line_ready == 0U) {
        return;
    }

    /* 拷贝到局部缓冲后再解析：line_buf 可能在解析期间被接收
     * 回调（USART2 中断，优先级 0）改写。临界区只包住拷贝+清标志 */
    pm = foc_critical_enter();
    n = (line_len < CMD_LINE_SIZE) ? line_len : (CMD_LINE_SIZE - 1U);
    memcpy(local, line_buf, n);
    line_len = 0U;
    line_ready = 0U;
    foc_critical_exit(pm);

    local[n] = '\0';
    cmd_execute(local);
}

/* DMA 接收事件：把"新增的"字节搬进行缓冲，遇到行尾置标志。
 * 事件可能是半满(HT)/全满(TC)/空闲(IDLE)：HT 时接收仍在进行，
 * 用 rx_last_pos 记录已处理位置，避免同一段字节被处理两次。
 *
 * 重入防护：HT 回调跑在 DMA1_Ch2 中断（优先级 5），IDLE 回调跑在
 * USART2 中断（优先级 0），后者可打断前者造成 rx_last_pos/line_buf
 * 状态错乱。整个处理序列包进临界区（≤64 字节搬运，约 1µs）。 */
static uint16_t rx_last_pos = 0U;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t i;
    uint32_t pm;

    if (huart->Instance != USART2) {
        return;
    }

    pm = foc_critical_enter();
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
    foc_critical_exit(pm);
}

/* 接收出错（溢出/噪声帧）：清错误并重启接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint32_t pm = foc_critical_enter();

        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, CMD_RX_DMA_SIZE)
            == HAL_OK) {
            rx_last_pos = 0U;
        }
        foc_critical_exit(pm);
    }
}
