/**
 * @file    foc_app.c
 * @brief   应用层实现：初始化时序、多轴调度、按键状态机
 */

#include "foc_app.h"
#include "foc_calib.h"
#include "foc_cmd.h"
#include "foc_telemetry.h"
#include "main.h"
#include "../HAL/foc_board_g431.h"
#include "../Driver/current/current_shunt.h"
#include "../Driver/encoder/abz_encoder.h"

foc_motor_t g_foc_motors[FOC_NUM_AXES];
volatile uint8_t g_foc_state_diag = (uint8_t)FOC_STATE_IDLE;

/* 开环 V/f 模式默认给定：0.3 V / 100 RPM，与旧版验证条件一致 */
volatile float g_m0_openloop_vq = 0.30f;
volatile float g_m0_openloop_rpm = 100.0f;

/* ---------------- 轴 0 配置 ---------------- */

static const foc_motor_params_t m0_params = {
    .pole_pairs     = FOC_M0_POLE_PAIRS,
    .rs_ohm         = FOC_M0_RS_OHM,
    .ls_henry       = FOC_M0_LS_H,
    .ke             = FOC_M0_KE,
    .max_current_a  = FOC_M0_MAX_CURRENT_A,
    .hard_current_a = FOC_M0_HARD_CURRENT_A,
    .max_rpm        = FOC_M0_MAX_RPM,
};

static const foc_ctrl_cfg_t m0_cfg = {
    .current_bw_rads   = FOC_M0_CURRENT_BW_RADS,
    .vel_kp            = FOC_M0_VEL_KP,
    .vel_ki            = FOC_M0_VEL_KI,
    .vel_ramp_rpm_s    = FOC_M0_VEL_RAMP_RPM_S,
    .vel_lpf_tf        = FOC_M0_VEL_LPF_TF,
    .pos_kp            = FOC_M0_POS_KP,
    .pos_vel_limit_rpm = FOC_M0_POS_VEL_LIMIT,
    .decouple_enable   = FOC_M0_DECOUPLE,
};

#if FOC_NUM_AXES >= 2
/* 轴 1（虚拟轴）：参数随意但合法，仅用于演示双轴调度 */
static const foc_motor_params_t m1_params = {
    .pole_pairs     = 7.0f,
    .rs_ohm         = 0.05f,
    .ls_henry       = 0.00003f,
    .ke             = 0.5f,
    .max_current_a  = 10.0f,
    .hard_current_a = 15.0f,
    .max_rpm        = 8000.0f,
};

static const foc_ctrl_cfg_t m1_cfg = {
    .current_bw_rads   = 1000.0f,
    .vel_kp            = 0.005f,
    .vel_ki            = 0.02f,
    .vel_ramp_rpm_s    = 2000.0f,
    .vel_lpf_tf        = 0.005f,
    .pos_kp            = 60.0f,
    .pos_vel_limit_rpm = 1000.0f,
    .decouple_enable   = 0U,
};
#endif

/* ---------------- API ---------------- */

void foc_app_init(void)
{
    /* 1. 电机对象初始化（含电流环带宽自整定） */
    foc_motor_init(&g_foc_motors[0],
                   &g_board_m0_driver,
                   &g_board_m0_current,
                   &g_board_m0_sensor,
                   &m0_params, &m0_cfg,
                   FOC_DT_FAST, FOC_SLOW_DIV);

#if FOC_NUM_AXES >= 2
    foc_motor_init(&g_foc_motors[1],
                   &g_board_m1_driver,
                   0,        /* 虚拟轴：无电流采样 */
                   0,        /* 虚拟轴：无传感器 */
                   &m1_params, &m1_cfg,
                   FOC_DT_FAST, FOC_SLOW_DIV);
#endif

    /* 2. 编码器启动 */
    abz_encoder_init();

    /* 3. 电流采样链路：OPAMP → ADC 校准/使能 → 注入触发路径 */
    if (current_shunt_init() == 0U) {
        foc_motor_fault(&g_foc_motors[0], FOC_FAULT_CURRENT_SENSE);
        g_foc_state_diag = (uint8_t)g_foc_motors[0].state;
        return;
    }

    /* 4. PWM 定时器安全上电（MOE 保持关闭） */
    foc_board_init();

    /* 5. 三相零偏校准（阻塞，约 130ms @16kHz×2×1024 采样） */
    if (current_shunt_calibrate(500U) == 0U) {
        foc_motor_fault(&g_foc_motors[0], FOC_FAULT_CURRENT_SENSE);
    }

    g_foc_state_diag = (uint8_t)g_foc_motors[0].state;

    /* 6. 通信：串口命令行 + VOFA 遥测 */
    foc_cmd_init();
    foc_telemetry_init();
}

void foc_app_isr_current_loop(void)
{
    foc_motor_fast_loop(&g_foc_motors[0]);

#if FOC_NUM_AXES >= 2
    /* 虚拟轴与轴 0 共享控制节拍。
     * 接入真实硬件后，把这行移到轴 1 自己的电流采样完成中断里。 */
    foc_motor_fast_loop(&g_foc_motors[1]);
#endif

    foc_telemetry_isr_tick();
}

void foc_app_task(void)
{
    foc_motor_t *m0 = &g_foc_motors[0];

    /* 校准状态机 */
    foc_calib_task();

    /* 串口命令 */
    foc_cmd_task();

    /* 健康监测：运行中电流采样链路失效 → 立即故障停机 */
    if ((m0->state == FOC_STATE_RUN) &&
        (current_shunt_is_ready() == 0U)) {
        foc_motor_fault(m0, FOC_FAULT_CURRENT_SENSE);
    }

    /* 开环 V/f 模式：把调试给定推入电机对象（校准接管时除外） */
    if ((foc_calib_is_active() == 0U) &&
        (m0->state == FOC_STATE_RUN) &&
        (m0->mode == FOC_MODE_OPENLOOP_VF)) {
        foc_motor_openloop_spin(m0, g_m0_openloop_rpm, 0.0f, g_m0_openloop_vq);
    }

    g_foc_state_diag = (uint8_t)m0->state;
}

void foc_app_on_key(void)
{
    foc_motor_t *m0 = &g_foc_motors[0];

    switch (m0->state) {
    case FOC_STATE_IDLE:
#if FOC_KEY_STARTS_CALIB
        /* 闭环模式且未校准：先启动校准；校准完成后再按进入 RUN */
        if ((m0->mode != FOC_MODE_OPENLOOP_VF) && (m0->calib.valid == 0U)) {
            foc_calib_start(m0);
            break;
        }
#endif
        (void)foc_motor_arm(m0);
        break;

    case FOC_STATE_RUN:
        foc_motor_disarm(m0);
        system_power_checkpoint(SYSTEM_CHECKPOINT_RUN_STOP,
                                (uint32_t)g_foc_pwm_stage,
                                (uint32_t)g_foc_calib_state,
                                (uint32_t)m0->state);
        break;

    case FOC_STATE_CALIB:
    case FOC_STATE_FAULT:
    default:
        /* 校准中不响应；故障需通过串口 'f' 显式清除 */
        break;
    }
}

foc_motor_t *foc_app_motor(uint8_t idx)
{
    if (idx >= (uint8_t)FOC_NUM_AXES) {
        idx = 0U;
    }
    return &g_foc_motors[idx];
}

uint8_t foc_app_diag_state(void)
{
    return (uint8_t)g_foc_motors[0].state;
}
