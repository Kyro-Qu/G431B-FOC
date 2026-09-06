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
#include "foc_anticog.h"
#include "../Core/foc_port.h"
#include "../Core/foc_utils.h"
#include "../Driver/current/current_shunt.h"
#include "../HAL/foc_board_g431.h"
#include "../HAL/foc_store.h"
#include "../Driver/encoder/abz_encoder.h"
#include "main.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern UART_HandleTypeDef huart2;

#define CMD_RX_DMA_SIZE   64U
#define CMD_LINE_SIZE     64U
#define CMD_RX_QUEUE_SIZE 256U
#define CMD_TX_BUF_SIZE   1536U
#define CMD_TWO_PI       6.28318530717958647692f

#define FOC_FW_NAME       "FOC_G431"
#define FOC_FW_VERSION    "0.3.11"
#define FOC_BOARD_NAME    "Matchstick_HFOC_G431"
#define FOC_CLI_VERSION   "2"

static uint8_t rx_dma_buf[CMD_RX_DMA_SIZE];
static char line_buf[CMD_LINE_SIZE];
static volatile uint16_t line_len = 0U;
static volatile uint8_t line_ready = 0U;
static volatile uint32_t rx_queue_overflow_count = 0U;

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
    static char out[CMD_TX_BUF_SIZE];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(out, sizeof(out), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    if (n >= (int)sizeof(out)) {
        n = (int)sizeof(out) - 1;
        out[n] = '\0';
    }

    foc_telemetry_suspend(1U);
    HAL_UART_AbortTransmit(&huart2);
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)out, (uint16_t)n, 200U);
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
        float track_error =
            ((m->state == FOC_STATE_RUN) &&
             (m->mode == FOC_MODE_VELOCITY))
                ? (m->vel_track_pos_rad - m->position_rad) : 0.0f;
        float pos_error =
            (m->mode == FOC_MODE_POSITION)
                ? (m->target - m->position_rad) : 0.0f;

        /* calib 后缀 *：偏移来自 Flash 存储（快速索引搜索可用） */
        foc_cmd_print(
            "M%u %s mode=%s\r\n"
            "tgt=%.3f\r\n"
            "vel=%.1frpm\r\n"
            "vel_obs=%.1frpm vel_filt=%.1frpm ref=%.1frpm\r\n"
            "track_err=%.3frad\r\n"
            "pos=%.3frad pos_err=%.3frad\r\n"
            "id=%.3fA id_ref=%.3fA iq=%.3fA iq_ref=%.3fA\r\n"
            "iu=%.3fA iv=%.3fA iw=%.3fA\r\n"
            "i_soft=%.3fA peak=%.3fA\r\n"
            "vd=%.3fV vq=%.3fV limit=%.2fA trip=%.2fA\r\n"
            "vbus=%.2fV (raw=%u %s) udc_drv=%.2fV\r\n"
            "trip_i=%.3f/%.3f/%.3fA trip_soft=%.3fA hard=%u\r\n"
            "calib=%u%s\r\n"
            "fault=%u%s\r\n",
            (unsigned)i, state_name(m->state), mode_name(m->mode),
            (double)m->target,
            (double)m->velocity_rpm,
            (double)m->velocity_observer_rpm,
            (double)m->velocity_filt_rpm,
            (double)m->vel_ref_rpm,
            (double)track_error,
            (double)m->position_rad,
            (double)pos_error,
            (double)m->i_dq_filt.d,
            (double)m->id_ref,
            (double)m->i_dq_filt.q,
            (double)m->iq_ref,
            (double)m->i_abc.a,
            (double)m->i_abc.b,
            (double)m->i_abc.c,
            (double)m->safety.soft_current_a,
            (double)m->safety.peak_current_a,
            (double)m->v_dq.d,
            (double)m->v_dq.q,
            (double)m->params.max_current_a,
            (double)m->safety.current_limit_a,
            (double)g_foc_vbus_diag.voltage_v,
            (unsigned)g_foc_vbus_diag.raw_adc,
            (g_foc_vbus_diag.valid != 0U) ? "OK" : "INIT",
            (double)m->drv->u_dc,
            (double)m->safety.trip_current_u_a,
            (double)m->safety.trip_current_v_a,
            (double)m->safety.trip_current_w_a,
            (double)m->safety.trip_soft_current_a,
            (unsigned)m->safety.trip_was_hard,
            (unsigned)m->calib.valid,
            (m->calib.from_store != 0U) ? "*" : "",
            (unsigned)m->safety.fault_code,
            (i == cur_axis) ? "  <-" : "");
    }
    foc_cmd_print(
        "calib_state=%u telem=%u\r\n"
        "rst_flags=0x%08lX (IWDG=%u SFT=%u BOR=%u PIN=%u) chk=%lu\r\n"
        "cli_rx_overflow=%lu\r\n"
        "calib_dir=%d offset=%.4frad\r\n"
        "cs_ready=%u\r\n"
        "cs_fault=%u rejected=%lu consecutive=%u\r\n"
        "vf_boost=%.3fV slope=%.6fV/RPM\r\n"
        "vf_rpm_cmd=%.1f target_vq=%.3fV applied=%.3fV/%.1frpm\r\n"
        "vel_start_boost=%.3fA active=%u sign=%d release=%u\r\n"
        "cpu=%.1f%% (max %.1f%%)\r\n",
        (unsigned)foc_calib_get_state(),
        (unsigned)foc_telemetry_get_enable(),
        (unsigned long)g_system_fault_diag.reset_flags,
        (unsigned)((g_system_fault_diag.reset_flags & RCC_CSR_IWDGRSTF) ? 1U : 0U),
        (unsigned)((g_system_fault_diag.reset_flags & RCC_CSR_SFTRSTF) ? 1U : 0U),
        (unsigned)((g_system_fault_diag.reset_flags & RCC_CSR_BORRSTF) ? 1U : 0U),
        (unsigned)((g_system_fault_diag.reset_flags & RCC_CSR_PINRSTF) ? 1U : 0U),
        (unsigned long)g_system_fault_diag.previous_checkpoint,
        (unsigned long)rx_queue_overflow_count,
        (int)g_foc_motors[cur_axis].calib.direction,
        (double)g_foc_motors[cur_axis].calib.electrical_offset_rad,
        (unsigned)current_shunt_is_ready(),
        (unsigned)g_current_shunt_diag.fault_code,
        (unsigned long)g_current_shunt_diag.rejected_sample_count,
        (unsigned)g_current_shunt_diag.rejected_consecutive,
        (double)g_m0_openloop_vq,
        (double)g_m0_vf_slope_v_per_rpm,
        (double)g_m0_openloop_rpm,
        (double)g_m0_vf_vq_target,
        (double)g_m0_openloop_vq_applied,
        (double)g_m0_openloop_rpm_applied,
        (double)g_foc_motors[cur_axis].vel_start_boost_a,
        (unsigned)g_foc_motors[cur_axis].vel_start_active,
        (int)g_foc_motors[cur_axis].vel_start_sign,
        (unsigned)g_foc_motors[cur_axis].vel_start_release_cnt,
        (double)g_foc_cpu_diag.load_pct,
        (double)(100.0f * (float)g_foc_cpu_diag.max_cycles /
                 (170000000.0f / FOC_PWM_FREQ_HZ)));
}

static void cmd_print_help(void)
{
    foc_cmd_print(
        "FOC CLI v" FOC_CLI_VERSION ":\r\n"
        " System:\r\n"
        "  help / version / status\r\n"
        "  motor [n]           show/select motor (now M%u)\r\n"
        "  enable / disable    arm/disarm selected motor\r\n"
        "  fault [clear]       show/clear fault\r\n"
        "  calib [full]        calibration / force full calibration\r\n"
        "  calib offset [rad]  show/set electrical offset (IDLE)\r\n"
        " Control:\r\n"
        "  mode [vf|iq|vel|pos]\r\n"
        "  target <value>      iq:A, vel:RPM, pos:rad\r\n"
        "  angle [enc|ol]      show/set angle source\r\n"
        "  vq [V]             V/F boost query/set (>=0)\r\n"
        "  rpm [RPM]          V/F speed query/set\r\n"
        "  vf [slope <V/RPM>] show/set V/F curve\r\n"
        "  limit [A]           show/set soft current limit\r\n"
        " Tuning:\r\n"
        "  current [bw <rad/s>]\r\n"
        "  tune [angle_delay|fw|pll ...] (RAM only)\r\n"
        "  vel [kp|ki|filter|lpf|ramp|ff|start|start_rpm <value>]\r\n"
        "  vel [track|track_limit|track_rpm <value>]\r\n"
        "  pos [kp|ki|vkp|accel|vmax <value>]\r\n"
        "  ident               start Rs+Ls measurement\r\n"
        "  ident apply         apply latest result\r\n"
        "  acog                anticogging query/start/finish/enable\r\n"
        " Diagnostics:\r\n"
        "  obs [0|1|2 [off]]   sensorless observer compare/switch\r\n"
        "  blackbox            dump 512-sample fault waveform\r\n"
        " Storage/telemetry:\r\n"
        "  conf <read|write|erase>\r\n"
        "  log [0|1]\r\n",
        (unsigned)cur_axis);
}

static void cmd_print_version(void)
{
    foc_motor_t *m = foc_app_motor(cur_axis);

    foc_cmd_print(
        "firmware=" FOC_FW_NAME " version=" FOC_FW_VERSION
        " board=" FOC_BOARD_NAME " cli=" FOC_CLI_VERSION
        " build=%s %s\r\n"
        "M%u pole_pairs=%.0f encoder_cpr=%u udc=%.2fV"
        " max_rpm=%.0f limit=%.2fA\r\n",
        __DATE__, __TIME__,
        (unsigned)cur_axis, (double)m->params.pole_pairs,
        (unsigned)FOC_M0_ENCODER_CPR,
        (double)m->drv->u_dc,
        (double)m->params.max_rpm,
        (double)m->params.max_current_a);
}

/* ---------------- 解析 ---------------- */

static void cmd_print_tune(const foc_motor_t *m)
{
    foc_cmd_print(
        "M%u tune angle_delay=%.2fcycle (0..1.5) RAM only\r\n"
        "fw=%u enter=%.0fRPM target=%.3f gain=%.1f idmax=%.3fA\r\n"
        "pll_bw=%.0frad/s (100..2000) RAM only\r\n",
        (unsigned)cur_axis,
        (double)m->runtime.angle_delay_cycles,
        (unsigned)m->runtime.fieldweak_enable,
        (double)m->runtime.fieldweak_enter_rpm,
        (double)m->runtime.fieldweak_voltage_ratio,
        (double)m->runtime.fieldweak_gain,
        (double)fabsf(m->runtime.fieldweak_id_min_a),
        (double)((cur_axis == 0U) ? abz_encoder_get_pll_bw_rad_s() : 0.0f));
}

static void cmd_tune_execute(foc_motor_t *m,
                             const char *arg1,
                             const char *arg2,
                             const char *arg3,
                             const char *arg4,
                             float val2,
                             float val3,
                             uint8_t has_val2,
                             uint8_t has_val3)
{
    uint8_t setting = 0U;

    if (arg1 == 0) {
        cmd_print_tune(m);
        return;
    }
    if ((arg4 != 0) || (m->state != FOC_STATE_IDLE)) {
        foc_cmd_print("err: tune settings need IDLE and exact arguments\r\n");
        return;
    }

    if ((strcmp(arg1, "angle_delay") == 0) &&
        (arg3 == 0) && (has_val2 != 0U) &&
        (val2 >= 0.0f) && (val2 <= 3.0f)) {
        m->runtime.angle_delay_cycles = val2;
        setting = 1U;
        foc_cmd_print("M%u tune angle_delay=%.2fcycle (RAM only)\r\n",
                      (unsigned)cur_axis, (double)val2);
    } else if ((strcmp(arg1, "fw") == 0) &&
               (arg2 != 0) && (strcmp(arg2, "enable") == 0) &&
               (has_val3 != 0U) && (arg3 != 0) &&
               ((val3 == 0.0f) || (val3 == 1.0f))) {
        m->runtime.fieldweak_enable = (val3 != 0.0f) ? 1U : 0U;
        setting = 1U;
        foc_cmd_print("M%u tune fw enable=%u (RAM only)\r\n",
                      (unsigned)cur_axis,
                      (unsigned)m->runtime.fieldweak_enable);
    } else if ((strcmp(arg1, "fw") == 0) &&
               (arg2 != 0) && (strcmp(arg2, "enter") == 0) &&
               (arg4 == 0) && (has_val3 != 0U) &&
               (val3 >= 0.0f) && (val3 <= m->params.max_rpm)) {
        m->runtime.fieldweak_enter_rpm = val3;
        setting = 1U;
        foc_cmd_print("M%u tune fw enter=%.0fRPM (RAM only)\r\n",
                      (unsigned)cur_axis, (double)val3);
    } else if ((strcmp(arg1, "fw") == 0) &&
               (arg2 != 0) && (strcmp(arg2, "target") == 0) &&
               (arg4 == 0) && (has_val3 != 0U) &&
               (val3 >= 0.0f) && (val3 <= 0.95f)) {
        m->runtime.fieldweak_voltage_ratio = val3;
        setting = 1U;
        foc_cmd_print("M%u tune fw target=%.3f (RAM only)\r\n",
                      (unsigned)cur_axis, (double)val3);
    } else if ((strcmp(arg1, "fw") == 0) &&
               (arg2 != 0) && (strcmp(arg2, "gain") == 0) &&
               (arg4 == 0) && (has_val3 != 0U) &&
               (val3 >= 0.0f) && (val3 <= 2000.0f)) {
        m->runtime.fieldweak_gain = val3;
        setting = 1U;
        foc_cmd_print("M%u tune fw gain=%.1f (RAM only)\r\n",
                      (unsigned)cur_axis, (double)val3);
    } else if ((strcmp(arg1, "fw") == 0) &&
               (arg2 != 0) && (strcmp(arg2, "idmax") == 0) &&
               (arg4 == 0) && (has_val3 != 0U) &&
               (val3 >= 0.0f) &&
               (val3 <= (0.85f * m->params.max_current_a))) {
        m->runtime.fieldweak_id_min_a = -val3;
        setting = 1U;
        foc_cmd_print("M%u tune fw idmax=%.3fA (RAM only)\r\n",
                      (unsigned)cur_axis, (double)val3);
    } else if ((strcmp(arg1, "pll") == 0) &&
               (cur_axis == 0U) && (arg2 != 0) &&
               (strcmp(arg2, "bw") == 0) && (arg4 == 0) &&
               (has_val3 != 0U) &&
               (val3 >= ABZ_ANGLE_PLL_MIN_BW_RAD_S) &&
               (val3 <= ABZ_ANGLE_PLL_MAX_BW_RAD_S) &&
               (abz_encoder_set_pll_bw_rad_s(val3) != 0U)) {
        setting = 1U;
        foc_cmd_print("M%u tune pll bw=%.0frad/s (RAM only)\r\n",
                      (unsigned)cur_axis, (double)val3);
    }

    if (setting == 0U) {
        foc_cmd_print(
            "err: tune angle_delay <0..1.5> | fw enable <0|1> |\r\n"
            "     fw enter <rpm> | fw target <0..0.95> |\r\n"
            "     fw gain <0..2000> | fw idmax <0..%.2f> |\r\n"
            "     pll bw <%.0f..%.0f> (M0, IDLE)\r\n",
            (double)(0.85f * m->params.max_current_a),
            (double)ABZ_ANGLE_PLL_MIN_BW_RAD_S,
            (double)ABZ_ANGLE_PLL_MAX_BW_RAD_S);
    }
}

static uint8_t cmd_parse_float(const char *s, float *out)
{
    char *end = 0;
    float v;

    if ((s == 0) || (*s == '\0')) {
        return 0U;
    }
    v = strtof(s, &end);
    if ((end == s) || (*end != '\0')) {
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

static void cmd_conf_execute(foc_motor_t *m, const char *action)
{
    uint8_t flash_ok;
    uint8_t resume_ok;
    uint8_t erase;
    uint8_t with_calib;

    if ((action == 0) || (strcmp(action, "read") == 0)) {
        foc_cmd_print(
            "M%u conf params:\r\n"
            "  pp=%.0f Rs=%.4f Ls=%.2fuH max_rpm=%.0f limit=%.2fA\r\n"
            "  bw=%.0f vp=%.4f vi=%.4f ramp=%.0f lpf=%.5fs\r\n"
            "  track_kp=%.3f track_limit=%.3f\r\n"
            "  calib: dir=%d offset=%.4frad (valid=%u from_store=%u)\r\n"
            "  acog: state=%u enable=%u\r\n",
            (unsigned)cur_axis,
            (double)m->params.pole_pairs,
            (double)m->params.rs_ohm,
            (double)(m->params.ls_henry * 1e6f),
            (double)m->params.max_rpm,
            (double)m->params.max_current_a,
            (double)m->cfg.current_bw_rads,
            (double)m->pid_vel.kp,
            (double)m->pid_vel.ki,
            (double)m->cfg.vel_ramp_rpm_s,
            (double)m->cfg.vel_lpf_tf,
            (double)m->cfg.vel_track_kp,
            (double)m->cfg.vel_track_limit_rad,
            (int)m->calib.direction,
            (double)m->calib.electrical_offset_rad,
            (unsigned)m->calib.valid,
            (unsigned)m->calib.from_store,
            (unsigned)g_m0_anticog.state,
            (unsigned)g_m0_anticog.enable);
        return;
    }

    if ((strcmp(action, "write") != 0) &&
        (strcmp(action, "erase") != 0)) {
        foc_cmd_print("err: conf read|write|erase\r\n");
        return;
    }

    if ((m->state == FOC_STATE_RUN) ||
        (m->state == FOC_STATE_CALIB)) {
        foc_cmd_print("err: flash op needs IDLE (stalls 22ms)\r\n");
        return;
    }
    if (fabsf(m->velocity_filt_rpm) > 60.0f) {
        foc_cmd_print("err: rotor still spinning, wait for stop\r\n");
        return;
    }

    erase = (strcmp(action, "erase") == 0) ? 1U : 0U;
    with_calib = ((m->calib.valid != 0U) ||
                  (m->calib.from_store != 0U)) ? 1U : 0U;

    if (current_shunt_suspend() == 0U) {
        foc_cmd_print("err: current sense pause failed\r\n");
        return;
    }

    flash_ok = (erase != 0U) ? foc_store_erase() : foc_store_save(m, (cur_axis == 0U) ? &g_m0_anticog : 0);
    resume_ok = current_shunt_resume();
    if (resume_ok == 0U) {
        foc_cmd_print("err: current sense resume failed\r\n");
    } else if (flash_ok == 0U) {
        foc_cmd_print((erase != 0U)
                          ? "err: config erase failed\r\n"
                          : "err: config write failed\r\n");
    } else if (erase != 0U) {
        foc_cmd_print("config erased (defaults on next boot)\r\n");
    } else {
        foc_cmd_print("config written%s (auto-load on boot)\r\n",
                      (with_calib != 0U) ? "" : ", no calib stored");
    }
}

static void cmd_execute(char *line)
{
    foc_motor_t *m = foc_app_motor(cur_axis);
    char *cmd;
    char *arg1;
    char *arg2;
    char *arg3;
    char *arg4;
    float val1 = 0.0f;
    float val2 = 0.0f;
    float val3 = 0.0f;
    uint8_t has_val1;
    uint8_t has_val2;
    uint8_t has_val3;

    cmd = strtok(line, " \t");
    if (cmd == 0) {
        return;
    }
    arg1 = strtok(0, " \t");
    arg2 = strtok(0, " \t");
    arg3 = strtok(0, " \t");
    arg4 = strtok(0, " \t");
    has_val1 = cmd_parse_float(arg1, &val1);
    has_val2 = cmd_parse_float(arg2, &val2);
    has_val3 = cmd_parse_float(arg3, &val3);

    if (strcmp(cmd, "help") == 0) {
        cmd_print_help();

    } else if (strcmp(cmd, "version") == 0) {
        cmd_print_version();

    } else if (strcmp(cmd, "status") == 0) {
        cmd_print_status();

    } else if (strcmp(cmd, "tune") == 0) {
        cmd_tune_execute(m, arg1, arg2, arg3, arg4,
                         val2, val3, has_val2, has_val3);

    } else if (strcmp(cmd, "motor") == 0) {
        if (arg1 == 0) {
            foc_cmd_print("selected motor M%u\r\n", (unsigned)cur_axis);
        } else if ((arg2 == 0) && (has_val1 != 0U) &&
                   (val1 >= 0.0f) &&
                   (val1 == (float)(uint8_t)val1) &&
                   ((uint8_t)val1 < (uint8_t)FOC_NUM_AXES)) {
            cur_axis = (uint8_t)val1;
            foc_cmd_print("selected motor M%u\r\n", (unsigned)cur_axis);
        } else {
            foc_cmd_print("err: motor 0..%u\r\n",
                          (unsigned)(FOC_NUM_AXES - 1U));
        }

    } else if (strcmp(cmd, "enable") == 0) {
        if (arg1 != 0) {
            foc_cmd_print("err: enable takes no argument\r\n");
        } else if (foc_motor_arm(m) != 0U) {
            foc_cmd_print("M%u enabled (%s)\r\n",
                          (unsigned)cur_axis, mode_name(m->mode));
        } else {
            foc_cmd_print("err: enable rejected, state=%s fault=%u\r\n",
                          state_name(m->state),
                          (unsigned)m->safety.fault_code);
        }

    } else if (strcmp(cmd, "disable") == 0) {
        if (arg1 != 0) {
            foc_cmd_print("err: disable takes no argument\r\n");
        } else {
            foc_motor_disarm(m);
            if (cur_axis == 0U) {
                foc_app_vf_reset_commands();
            }
            foc_cmd_print("M%u %s\r\n",
                          (unsigned)cur_axis, state_name(m->state));
        }

    } else if (strcmp(cmd, "calib") == 0) {
        uint8_t want_full;
        uint8_t was_from_store;

        if ((arg1 != 0) && (strcmp(arg1, "offset") == 0)) {
            if ((arg2 == 0) && (arg3 == 0)) {
                foc_cmd_print("M%u calib offset=%.6frad dir=%d\r\n",
                              (unsigned)cur_axis,
                              (double)m->calib.electrical_offset_rad,
                              (int)m->calib.direction);
            } else if ((arg3 == 0) && (has_val2 != 0U) &&
                       (m->state == FOC_STATE_IDLE) &&
                       (foc_calib_is_active() == 0U)) {
                float offset = fmodf(val2, CMD_TWO_PI);

                if (offset < 0.0f) {
                    offset += CMD_TWO_PI;
                }
                m->calib.electrical_offset_rad = offset;
                foc_cmd_print("M%u calib offset=%.6frad (RAM only)\r\n",
                              (unsigned)cur_axis, (double)offset);
            } else if (m->state != FOC_STATE_IDLE) {
                foc_cmd_print("err: calib offset needs IDLE, state=%s\r\n",
                              state_name(m->state));
            } else {
                foc_cmd_print("err: calib offset [rad]\r\n");
            }
        } else {
            if ((arg1 != 0) && (strcmp(arg1, "full") != 0)) {
                foc_cmd_print("err: calib [full] | calib offset [rad]\r\n");
                return;
            }
            if (arg2 != 0) {
                foc_cmd_print("err: calib [full] | calib offset [rad]\r\n");
                return;
            }
            if (m->state != FOC_STATE_IDLE) {
                foc_cmd_print("err: calib needs IDLE, state=%s\r\n",
                              state_name(m->state));
            } else if (foc_calib_is_active() != 0U) {
                foc_cmd_print("err: calib busy on another axis\r\n");
            } else {
                /* `calib full`: ignore stored offset and run align + Z search. */
                want_full = (arg1 != 0) ? 1U : 0U;
                was_from_store = m->calib.from_store;

                if (cur_axis == 0U) {
                    foc_app_vf_reset_commands();
                }

                if (want_full != 0U) {
                    m->calib.from_store = 0U;
                }
                foc_calib_start(m);
                if (m->state == FOC_STATE_CALIB) {
                    foc_cmd_print("M%u calib start%s\r\n",
                                  (unsigned)cur_axis,
                                  (want_full != 0U) ? " (full)" : "");
                } else {
                    if (want_full != 0U) {
                        m->calib.from_store = was_from_store;
                    }
                    foc_cmd_print("err: calib rejected, fault=%u\r\n",
                                  (unsigned)m->safety.fault_code);
                }
            }
        }

    } else if (strcmp(cmd, "fault") == 0) {
        if (arg1 == 0) {
            foc_cmd_print("M%u fault=%u current_sense=%u\r\n",
                          (unsigned)cur_axis,
                          (unsigned)m->safety.fault_code,
                          (unsigned)g_current_shunt_diag.fault_code);
        } else if ((strcmp(arg1, "clear") == 0) && (arg2 == 0)) {
            current_shunt_reset_discontinuity();
            foc_motor_clear_fault(m);
            if (cur_axis == 0U) {
                foc_app_vf_reset_commands();
            }
            foc_cmd_print("M%u fault cleared, state=%s\r\n",
                          (unsigned)cur_axis, state_name(m->state));
        } else {
            foc_cmd_print("err: fault [clear]\r\n");
        }

    } else if (strcmp(cmd, "mode") == 0) {
        if (arg1 == 0) {
            foc_cmd_print("mode=%s\r\n", mode_name(m->mode));
        } else if (arg2 != 0) {
            foc_cmd_print("err: mode vf|iq|vel|pos\r\n");
        } else {
            foc_mode_t want;
            foc_mode_t old_mode = m->mode;
            uint8_t known = 1U;

            if (strcmp(arg1, "vf") == 0) {
                want = FOC_MODE_OPENLOOP_VF;
            } else if (strcmp(arg1, "iq") == 0) {
                want = FOC_MODE_TORQUE;
            } else if (strcmp(arg1, "vel") == 0) {
                want = FOC_MODE_VELOCITY;
            } else if (strcmp(arg1, "pos") == 0) {
                want = FOC_MODE_POSITION;
            } else {
                known = 0U;
                foc_cmd_print("err: mode vf|iq|vel|pos\r\n");
            }
            if (known != 0U) {
                if (foc_motor_set_mode(m, want) != 0U) {
                    if ((cur_axis == 0U) &&
                        ((old_mode == FOC_MODE_OPENLOOP_VF) ||
                         (want == FOC_MODE_OPENLOOP_VF))) {
                        foc_app_vf_reset_commands();
                    }
                    foc_cmd_print("M%u mode=%s\r\n",
                                  (unsigned)cur_axis, mode_name(m->mode));
                } else {
                    foc_cmd_print("err: mode change needs IDLE; state=%s"
                                  " calib=%u (disable first)\r\n",
                                  state_name(m->state),
                                  (unsigned)m->calib.valid);
                }
            }
        }

    } else if (strcmp(cmd, "angle") == 0) {
        if (arg1 == 0) {
            const char *src_str = (m->angle_source == FOC_ANGLE_OPEN_LOOP) ? "ol" : "enc";
            foc_cmd_print("M%u angle_source=%s\r\n", (unsigned)cur_axis, src_str);
        } else if (strcmp(arg1, "enc") == 0) {
            m->angle_source = FOC_ANGLE_ENCODER_CALIBRATED;
            foc_cmd_print("M%u angle_source=enc (encoder)\r\n", (unsigned)cur_axis);
        } else if (strcmp(arg1, "ol") == 0) {
            m->angle_source = FOC_ANGLE_OPEN_LOOP;
            foc_cmd_print("M%u angle_source=ol (open loop)\r\n", (unsigned)cur_axis);
        } else {
            foc_cmd_print("err: angle enc|ol\r\n");
        }

    } else if (strcmp(cmd, "target") == 0) {
        const char *unit;

        if ((has_val1 == 0U) || (arg2 != 0)) {
            foc_cmd_print("err: target <value>\r\n");
        } else if (m->mode == FOC_MODE_OPENLOOP_VF) {
            foc_cmd_print("err: target is ambiguous in vf; use vq and rpm\r\n");
        } else if ((m->mode == FOC_MODE_TORQUE) &&
                   (fabsf(val1) > m->params.max_current_a)) {
            foc_cmd_print("err: iq target must be within +/-%.2fA\r\n",
                          (double)m->params.max_current_a);
        } else {
            foc_motor_set_target(m, val1);
            unit = (m->mode == FOC_MODE_TORQUE) ? "A"
                 : ((m->mode == FOC_MODE_VELOCITY) ? "RPM" : "rad");
            foc_cmd_print("M%u target=%.3f%s\r\n",
                          (unsigned)cur_axis, (double)val1, unit);
        }

    } else if (strcmp(cmd, "vq") == 0) {
        float v_max = m->drv->u_dc * 0.5773503f;

        if (arg1 == 0) {
            foc_cmd_print("M%u vf boost=%.3fV target=%.3fV applied=%.3fV\r\n",
                          (unsigned)cur_axis,
                          (double)g_m0_openloop_vq,
                          (double)g_m0_vf_vq_target,
                          (double)g_m0_openloop_vq_applied);
        } else if ((m->mode != FOC_MODE_OPENLOOP_VF) ||
                   (cur_axis != 0U)) {
            foc_cmd_print("err: vq is only valid for M0 mode vf\r\n");
        } else if ((has_val1 != 0U) && (arg2 == 0) &&
                   (val1 >= 0.0f) && (val1 <= v_max)) {
            g_m0_openloop_vq = val1;
            foc_cmd_print("M%u vf boost=%.3fV\r\n",
                          (unsigned)cur_axis, (double)val1);
        } else {
            foc_cmd_print("err: vq [V], 0 <= V <= %.2f\r\n", (double)v_max);
        }

    } else if (strcmp(cmd, "rpm") == 0) {
        if (arg1 == 0) {
            foc_cmd_print("M%u rpm cmd=%.1f applied=%.1f\r\n",
                          (unsigned)cur_axis,
                          (double)g_m0_openloop_rpm,
                          (double)g_m0_openloop_rpm_applied);
        } else if ((m->mode != FOC_MODE_OPENLOOP_VF) ||
                   (cur_axis != 0U)) {
            foc_cmd_print("err: rpm is only valid for M0 mode vf\r\n");
        } else if ((has_val1 != 0U) && (arg2 == 0) &&
                   (fabsf(val1) <= m->params.max_rpm)) {
            g_m0_openloop_rpm = val1;
            foc_cmd_print("M%u rpm cmd=%.1f\r\n",
                          (unsigned)cur_axis, (double)val1);
        } else {
            foc_cmd_print("err: rpm [RPM], |RPM| <= %.0f\r\n",
                          (double)m->params.max_rpm);
        }

    } else if (strcmp(cmd, "vf") == 0) {
        if (arg1 == 0) {
            foc_cmd_print(
                "M%u vf Vq=boost+slope*|rpm| (rpm=0 -> Vq=0)\r\n"
                "boost=%.3fV slope=%.6fV/RPM rpm_ramp=%.0fRPM/s"
                " vq_ramp=%.1fV/s\r\n",
                (unsigned)cur_axis,
                (double)g_m0_openloop_vq,
                (double)g_m0_vf_slope_v_per_rpm,
                (double)FOC_M0_VF_RPM_RAMP_RPM_S,
                (double)FOC_M0_VF_VQ_RAMP_V_S);
        } else if ((strcmp(arg1, "slope") == 0) &&
                   (has_val2 != 0U) && (arg3 == 0) &&
                   (val2 >= 0.0f) && (val2 <= 0.01f) &&
                   (cur_axis == 0U)) {
            g_m0_vf_slope_v_per_rpm = val2;
            foc_cmd_print("M%u vf slope=%.6fV/RPM\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else {
            foc_cmd_print("err: vf [slope <0..0.01 V/RPM>]\r\n");
        }

    } else if (strcmp(cmd, "current") == 0) {
        if (arg1 == 0) {
            foc_cmd_print("M%u current bw=%.0frad/s\r\n",
                          (unsigned)cur_axis,
                          (double)m->cfg.current_bw_rads);
        } else if (m->state != FOC_STATE_IDLE) {
            foc_cmd_print("err: current bw change needs IDLE; state=%s\r\n",
                          state_name(m->state));
        } else if ((strcmp(arg1, "bw") == 0) &&
                    (has_val2 != 0U) && (val2 > 0.0f) &&
                    (val2 >= FOC_CURRENT_BW_MIN_RADS) &&
                    (val2 <= FOC_CURRENT_BW_MAX_RADS) && (arg3 == 0)) {
            float v_max = m->drv->u_dc * 0.5773503f;

            m->cfg.current_bw_rads = val2;
            foc_pid_init(&m->pid_id, m->params.ls_henry * val2,
                         m->params.rs_ohm * val2, 0.0f, v_max, 0.0f);
            foc_pid_init(&m->pid_iq, m->params.ls_henry * val2,
                         m->params.rs_ohm * val2, 0.0f, v_max, 0.0f);
            foc_cmd_print("M%u current bw=%.0f kp=%.4f ki=%.2f\r\n",
                          (unsigned)cur_axis, (double)val2,
                          (double)m->pid_id.kp, (double)m->pid_id.ki);
        } else {
            foc_cmd_print("err: current bw [%.0f..%.0f rad/s], IDLE only\r\n",
                          (double)FOC_CURRENT_BW_MIN_RADS,
                          (double)FOC_CURRENT_BW_MAX_RADS);
        }

    } else if (strcmp(cmd, "vel") == 0) {
        if (arg1 == 0) {
            foc_cmd_print(
                          "M%u vel kp=%.4f ki=%.4f\r\n"
                          "filter_high=%.1fHz low=%.1fHz blend=50..100RPM (Tf=%.5fs)\r\n"
                          "ramp=%.0fRPM/s\r\n"
                          "ff=%.3fA start=%.3fA start_rpm=%.1fRPM\r\n"
                          "track=%.3fA/rad limit=%.3frad track_rpm=%.1fRPM\r\n",
                          (unsigned)cur_axis,
                          (double)m->pid_vel.kp,
                          (double)m->pid_vel.ki,
                          (double)foc_motor_get_velocity_filter_hz(m),
                          (double)FOC_VEL_LOW_FILTER_HZ,
                          (double)m->cfg.vel_lpf_tf,
                          (double)m->cfg.vel_ramp_rpm_s,
                          (double)m->cfg.vel_friction_a,
                          (double)m->cfg.vel_start_a,
                          (double)m->cfg.vel_start_rpm,
                          (double)m->cfg.vel_track_kp,
                          (double)m->cfg.vel_track_limit_rad,
                          (double)m->cfg.vel_track_rpm);
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "kp") == 0)) {
            m->pid_vel.kp = val2;
            foc_cmd_print("M%u vel kp=%.4f\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "ki") == 0)) {
            m->pid_vel.ki = val2;
            foc_cmd_print("M%u vel ki=%.4f\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "filter") == 0) &&
                   (val2 >= 0.0f) && (val2 <= 200.0f)) {
            foc_motor_set_velocity_filter_hz(m, val2);
            foc_cmd_print("M%u vel filter=%.1fHz median=3\r\n",
                          (unsigned)cur_axis,
                          (double)foc_motor_get_velocity_filter_hz(m));
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "lpf") == 0) &&
                   (val2 >= 0.0f) && (val2 <= 1.0f)) {
            foc_motor_set_velocity_filter_tf(m, val2);
            foc_cmd_print("M%u vel lpf=%.5fs filter=%.1fHz\r\n",
                          (unsigned)cur_axis, (double)m->cfg.vel_lpf_tf,
                          (double)foc_motor_get_velocity_filter_hz(m));
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "ramp") == 0) &&
                   (val2 >= 0.0f) && (val2 <= 100000.0f)) {
            m->cfg.vel_ramp_rpm_s = val2;
            foc_cmd_print("M%u vel ramp=%.0fRPM/s\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "ff") == 0) &&
                   (val2 >= 0.0f) &&
                   (val2 <= m->cfg.vel_start_a)) {
            m->cfg.vel_friction_a = val2;
            foc_cmd_print("M%u vel ff=%.3fA\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "start") == 0) &&
                   (val2 >= m->cfg.vel_friction_a) &&
                   (val2 <= m->params.max_current_a)) {
            m->cfg.vel_start_a = val2;
            foc_cmd_print("M%u vel start=%.3fA\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "start_rpm") == 0) &&
                   (val2 > 0.0f) && (val2 <= 10000.0f)) {
            m->cfg.vel_start_rpm = val2;
            foc_cmd_print("M%u vel start_rpm=%.1fRPM\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "track") == 0) &&
                   (val2 >= 0.0f) && (val2 <= 1000.0f)) {
            m->cfg.vel_track_kp = val2;
            foc_cmd_print("M%u vel track=%.3fA/rad\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "track_limit") == 0) &&
                   (val2 > 0.0f) && (val2 <= 1000.0f)) {
            m->cfg.vel_track_limit_rad = val2;
            foc_cmd_print("M%u vel track_limit=%.3frad\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((has_val2 != 0U) && (arg3 == 0) &&
                   (strcmp(arg1, "track_rpm") == 0) &&
                   (val2 > 0.0f) && (val2 <= 10000.0f)) {
            m->cfg.vel_track_rpm = val2;
            foc_cmd_print("M%u vel track_rpm=%.1fRPM\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else {
            foc_cmd_print(
                "err: vel tuning name/value (run 'vel' for names)\r\n");
        }

    } else if (strcmp(cmd, "pos") == 0) {
        if (arg1 == 0) {
            foc_cmd_print(
                "M%u pos kp=%.3fA/rad ki=%.3fA/(rad*s)\r\n"
                "vkp=%.4fA/RPM "
                "accel=%.0fRPM/s vmax=%.0fRPM\r\n",
                (unsigned)cur_axis, (double)m->pid_pos.kp,
                (double)m->pid_pos.ki,
                (double)m->cfg.pos_vel_kp,
                (double)m->cfg.traj_accel_rpm_s,
                (double)m->cfg.pos_vel_limit_rpm);
        } else if ((strcmp(arg1, "kp") == 0) &&
                   (has_val2 != 0U) && (arg3 == 0)) {
            m->pid_pos.kp = val2;
            foc_cmd_print("M%u pos kp=%.3fA/rad\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((strcmp(arg1, "ki") == 0) &&
                   (has_val2 != 0U) && (arg3 == 0) &&
                   (val2 >= 0.0f) && (val2 <= 10000.0f)) {
            m->cfg.pos_ki = val2;
            m->pid_pos.ki = val2;
            foc_cmd_print("M%u pos ki=%.3fA/(rad*s)\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((strcmp(arg1, "vkp") == 0) &&
                   (has_val2 != 0U) && (arg3 == 0) &&
                   (val2 >= 0.0f) && (val2 <= 10000.0f)) {
            m->cfg.pos_vel_kp = val2;
            foc_cmd_print("M%u pos vkp=%.4fA/RPM\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((strcmp(arg1, "accel") == 0) &&
                   (has_val2 != 0U) && (arg3 == 0) &&
                   (val2 > 0.0f) && (val2 <= 100000.0f)) {
            m->cfg.traj_accel_rpm_s = val2;
            foc_cmd_print("M%u pos accel=%.0fRPM/s\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else if ((strcmp(arg1, "vmax") == 0) &&
                   (has_val2 != 0U) && (arg3 == 0) &&
                   (val2 > 0.0f) && (val2 <= m->params.max_rpm)) {
            m->cfg.pos_vel_limit_rpm = val2;
            foc_cmd_print("M%u pos vmax=%.0fRPM\r\n",
                          (unsigned)cur_axis, (double)val2);
        } else {
            foc_cmd_print(
                "err: pos [kp|ki|vkp|accel|vmax <value>]\r\n");
        }

    } else if (strcmp(cmd, "limit") == 0) {
        if (arg1 == 0) {
            foc_cmd_print("M%u limit=%.2fA trip=%.2fA hard=%.2fA\r\n",
                          (unsigned)cur_axis,
                          (double)m->params.max_current_a,
                          (double)m->safety.current_limit_a,
                          (double)m->params.hard_current_a);
        } else if ((has_val1 != 0U) && (arg2 == 0) &&
                   (val1 > 0.0f) &&
                   (val1 <= m->params.hard_current_a)) {
            m->params.max_current_a = val1;
            foc_motor_restore_current_limits(m);
            foc_pid_set_limit(&m->pid_vel, val1);
            foc_pid_set_limit(&m->pid_pos, val1);
            foc_cmd_print("M%u limit=%.2fA trip=%.2fA\r\n",
                          (unsigned)cur_axis, (double)val1,
                          (double)m->safety.current_limit_a);
        } else {
            foc_cmd_print("err: 0 < limit <= %.1fA\r\n",
                          (double)m->params.hard_current_a);
        }

    } else if (strcmp(cmd, "ident") == 0) {
        if ((arg1 != 0) && (strcmp(arg1, "apply") == 0) &&
            (arg2 == 0)) {
            if (foc_ident_apply(m) != 0U) {
                foc_cmd_print("M%u applied: Rs=%.4f Ls=%.2fuH, "
                              "current loop retuned\r\n",
                              (unsigned)cur_axis,
                              (double)m->params.rs_ohm,
                              (double)(m->params.ls_henry * 1e6f));
            } else {
                foc_cmd_print("err: no valid result (run 'ident' first, "
                              "axis must not be running)\r\n");
            }
        } else if (arg1 == 0) {
            foc_ident_start(m);
        } else {
            foc_cmd_print("err: ident [apply]\r\n");
        }

    } else if (strcmp(cmd, "conf") == 0) {
        if (arg2 != 0) {
            foc_cmd_print("err: conf read|write|erase\r\n");
        } else {
            cmd_conf_execute(m, arg1);
        }

    } else if (strcmp(cmd, "blackbox") == 0) {
        /* 导出故障黑匣子：512 拍 x (iu iw th iq duty_a)，十六进制浮点 */
        if (foc_motor_blackbox_active() != 0U) {
            static float bb_u[512], bb_w[512], bb_th[512], bb_vq[512];
            static float bb_duty[512];
            uint16_t k;
            foc_motor_blackbox_dump(bb_u, bb_w, bb_th, bb_vq, bb_duty);
            for (k = 0U; k < 512U; k++) {
                foc_cmd_print("%d %08x %08x %08x %08x %08x\r\n",
                              (int)k,
                              (unsigned)*(uint32_t *)&bb_u[k],
                              (unsigned)*(uint32_t *)&bb_w[k],
                              (unsigned)*(uint32_t *)&bb_th[k],
                              (unsigned)*(uint32_t *)&bb_vq[k],
                              (unsigned)*(uint32_t *)&bb_duty[k]);
            }
        } else {
            foc_cmd_print("blackbox inactive (no fault since boot/clear)\r\n");
        }

    } else if (strcmp(cmd, "obs") == 0) {
        /* 无感观测器在线对比：obs [0|1]，ch14 输出 theta_obs-θe 差 */
        if (arg1 == 0) {
            foc_cmd_print("M%u obs=%u theta_obs=%.3f speed_obs=%.1fHz\r\n",
                          (unsigned)cur_axis,
                          (unsigned)m->obs_enabled,
                          (double)m->observer.theta_e,
                          (double)(m->observer.speed_e_rads / 6.2831853f));
        } else if ((has_val1 != 0U) && ((val1 == 0.0f) || (val1 == 1.0f) ||
                   (val1 == 2.0f))) {
            if (val1 == 2.0f) {
                /* 切换到无感角度（RUN 中实时切换；对比模式必须先开） */
                if (m->obs_enabled == 0U) {
                    foc_observer_reset(&m->observer);
                    m->obs_enabled = 1U;
                }
                if ((m->state == FOC_STATE_RUN) &&
                    (m->mode == FOC_MODE_VELOCITY) &&
                    (fabsf(m->velocity_filt_rpm) > 800.0f)) {
                    /* obs 2 [offset_rad]：切换并应用偏移补偿。
                     * 判据用快环换相角同源的 raw atan2 角差（含超前
                     * 补偿项）。raw 角纹波 std~0.3rad，0.5rad 内由
                     * 200ms 渐变吸收；PLL 角不用于判据（欠锁 ±0.4rad
                     * 纹波恒在，且渐变对 ±π 内任意差都连续）。 */
                    float we_chk = m->observer.speed_e_rads;
                    float delta = foc_wrap_pm_pi(
                        m->observer.theta_e +
                        m->observer.theta_offset_rad +
                        (m->runtime.angle_delay_cycles * we_chk *
                         m->dt_fast) -
                        m->theta_e);
                    if (fabsf(delta) > 0.5f) {
                        foc_cmd_print("err: obs 2 rejected, raw=%.3frad "
                                      "(obs=%.3f enc=%.3f) - tune offset\r\n",
                                      (double)delta,
                                      (double)m->observer.theta_e,
                                      (double)m->theta_e);
                    } else {
                        if (arg2 != 0) {
                            float off;
                            if (cmd_parse_float(arg2, &off) &&
                                (off > -1.5f) && (off < 1.5f)) {
                                m->observer.theta_offset_rad = off;
                            }
                        }
                        m->angle_source = FOC_ANGLE_OBSERVER;
                        m->obs_switch_ms = foc_board_cycles();  /* 豁免窗起点 */
                        /* 渐变切换：角度从编码器角 200ms 连续过渡到观测角，
                         * 消除相位阶跃（离散跳变→电流爆发→BOR 掉电 2026-09-04）。
                         * 宽限期作为兜底：渐变期间残余暂态若触发采样判据。 */
                        m->obs_blend = 0.0f;
                        m->obs_blending = 1U;
                        current_shunt_allow_transient(8.0f);
                        foc_cmd_print("M%u angle->OBSERVER (%.0frpm, off=%.3f, "
                                      "raw_err=%.3f)\r\n",
                                      (unsigned)cur_axis,
                                      (double)m->velocity_filt_rpm,
                                      (double)m->observer.theta_offset_rad,
                                      (double)delta);
                    }
                } else {
                    foc_cmd_print("err: obs 2 needs vel mode & >800rpm\r\n");
                }
            } else {
                m->obs_enabled = (val1 != 0.0f) ? 1U : 0U;
                if (val1 == 0.0f) {
                    /* 关闭对比：若还在无感角度，反向渐变切回编码器。
                     * obs_enabled 保持 1，渐变完成后由快环收尾清零
                     * （提前清零会冻结观测角、wrap 后乱跳）。 */
                    if (m->angle_source == FOC_ANGLE_OBSERVER) {
                        m->obs_enabled = 1U;
                        m->obs_blending = 2U;
                    }
                }
                if (m->obs_enabled != 0U) {
                    foc_observer_reset(&m->observer);
                }
                foc_cmd_print("M%u obs=%u angle=%u\r\n",
                              (unsigned)cur_axis,
                              (unsigned)m->obs_enabled,
                              (unsigned)m->angle_source);
            }
        } else {
            foc_cmd_print("err: obs [0|1]\r\n");
        }

    } else if (strcmp(cmd, "acog") == 0) {
        /* 抗齿槽力矩补偿命令：
         *   acog              - 查询状态、使能、样本数
         *   acog start        - 清表并开启采样（需要在速度模式例如 20 RPM 运行 ≥3 圈）
         *   acog finish       - 结算齿槽表并自动启用
         *   acog enable [0|1] - 手动开关前馈补偿
         */
        if (arg1 == 0) {
            const char *st_str = "IDLE";
            if (g_m0_anticog.state == FOC_ACOG_CALIB) {
                st_str = "CALIB";
            } else if (g_m0_anticog.state == FOC_ACOG_READY) {
                st_str = "READY";
            }
            foc_cmd_print("M0 acog state=%s enable=%u samples=%lu pts=%u\r\n",
                          st_str,
                          (unsigned)g_m0_anticog.enable,
                          (unsigned long)g_m0_anticog.total_samples,
                          (unsigned)FOC_ANTICOG_POINTS);
        } else if (strcmp(arg1, "start") == 0) {
            foc_anticog_calib_begin(&g_m0_anticog);
            foc_cmd_print("M0 acog calib started. Run vel mode at 20-30 RPM for >=3 turns\r\n");
        } else if (strcmp(arg1, "finish") == 0) {
            if (foc_anticog_calib_finish(&g_m0_anticog) != 0U) {
                foc_cmd_print("M0 acog calib SUCCESS, table ready & enabled\r\n");
            } else {
                foc_cmd_print("err: acog calib FAILED (empty bins, need more turns at low speed)\r\n");
            }
        } else if ((strcmp(arg1, "enable") == 0) && (has_val2 != 0U)) {
            g_m0_anticog.enable = (val2 != 0.0f) ? 1U : 0U;
            foc_cmd_print("M0 acog enable=%u\r\n", (unsigned)g_m0_anticog.enable);
        } else {
            foc_cmd_print("err: acog [start|finish|enable <0|1>]\r\n");
        }

    } else if (strcmp(cmd, "log") == 0) {
        if (arg1 == 0) {
            foc_cmd_print("telem=%u\r\n",
                          (unsigned)foc_telemetry_get_enable());
        } else if ((arg2 == 0) && (has_val1 != 0U) &&
                   ((val1 == 0.0f) || (val1 == 1.0f))) {
            foc_telemetry_set_enable((val1 != 0.0f) ? 1U : 0U);
            foc_cmd_print("telem=%u\r\n",
                          (unsigned)foc_telemetry_get_enable());
        } else {
            foc_cmd_print("err: log [0|1]\r\n");
        }

    } else {
        foc_cmd_print("err: unknown '%s', try help\r\n", cmd);
    }
}

/* ---------------- 接收 ---------------- */

void foc_cmd_init(void)
{
    line_len = 0U;
    line_ready = 0U;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, CMD_RX_DMA_SIZE);
}

void foc_cmd_task(void)
{
    char exec_buf[CMD_LINE_SIZE];
    uint32_t primask;

    /* 如果 DMA 出错或处于空闲状态，重新使能接收 */
    if (huart2.RxState == HAL_UART_STATE_READY) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        __HAL_UART_CLEAR_NEFLAG(&huart2);
        __HAL_UART_CLEAR_FEFLAG(&huart2);
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, CMD_RX_DMA_SIZE);
    }

    if (line_ready == 0U) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(exec_buf, line_buf, sizeof(exec_buf));
    line_len = 0U;
    line_ready = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    cmd_execute(exec_buf);
}

/* DMA 接收事件：把"新增的"字节搬进行缓冲，遇到行尾置标志。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t i;

    if (huart->Instance != USART2) {
        return;
    }

    for (i = 0; i < Size; i++) {
        char ch = (char)rx_dma_buf[i];

        if ((ch == '\r') || (ch == '\n')) {
            if ((line_len > 0U) && (line_ready == 0U)) {
                line_buf[line_len] = '\0';
                line_ready = 1U;
            }
        } else if (line_ready == 0U) {
            if (line_len < (CMD_LINE_SIZE - 1U)) {
                line_buf[line_len] = ch;
                line_len = (uint16_t)(line_len + 1U);
            } else {
                /* 单行超长被截断：真正自增溢出计数器 */
                ++rx_queue_overflow_count;
            }
        }
    }
}

/* 接收出错（溢出/噪声帧）：清错误并重启接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, CMD_RX_DMA_SIZE)
            == HAL_OK) {
            /* restarted */
        }
    }
}
