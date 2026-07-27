/**
 * @file    foc_app.c
 * @brief   应用层实现：初始化时序、多轴调度、按键状态机
 */

#include "foc_app.h"
#include "foc_calib.h"
#include "foc_cmd.h"
#include "foc_ident.h"
#include "foc_telemetry.h"
#include "main.h"
#include "../HAL/foc_board_g431.h"
#include "../HAL/foc_store.h"
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
    .traj_enable       = FOC_M0_TRAJ_ENABLE,
    .traj_accel_rpm_s  = FOC_M0_TRAJ_ACC_RPM_S,
    .decouple_enable   = FOC_M0_DECOUPLE,
    .deadtime_comp_v   = FOC_M0_DEADTIME_COMP_V,
    .stall_enable      = FOC_M0_STALL_ENABLE,
    .stall_rpm         = FOC_M0_STALL_RPM,
    .stall_timeout_ms  = FOC_M0_STALL_TIMEOUT_MS,
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
    .traj_enable       = 1U,
    .traj_accel_rpm_s  = 4000.0f,
    .decouple_enable   = 0U,
    .deadtime_comp_v   = 0.0f,
    .stall_enable      = 0U,     /* 虚拟轴没有真实电流，不查堵转 */
    .stall_rpm         = 30.0f,
    .stall_timeout_ms  = 1000U,
};
#endif

/* ---------------- API ---------------- */

/* 上电参数自检：非法参数直接 FAULT，好过带病闭环 */
static uint8_t app_params_sane(const foc_motor_params_t *p,
                               const foc_ctrl_cfg_t *c)
{
    if ((p->pole_pairs <= 0.0f) || (p->rs_ohm <= 0.0f) ||
        (p->ls_henry <= 0.0f) || (p->max_current_a <= 0.0f) ||
        (p->hard_current_a < p->max_current_a) ||
        (p->max_rpm <= 0.0f) || (c->current_bw_rads <= 0.0f) ||
        (c->vel_lpf_tf < 0.0f) || (c->pos_vel_limit_rpm <= 0.0f) ||
        ((c->traj_enable != 0U) && (c->traj_accel_rpm_s <= 0.0f))) {
        return 0U;
    }
    return 1U;
}

void foc_app_init(void)
{
    foc_motor_params_t m0_p = m0_params;
    foc_ctrl_cfg_t m0_c = m0_cfg;
    int8_t stored_dir = 0;
    float stored_offset = 0.0f;
    foc_store_status_t store_st;

    /* 0. Flash 参数加载：有有效存储则覆盖 foc_config.h 默认值 */
    store_st = foc_store_load(&m0_p, &m0_c, &stored_dir, &stored_offset);

    /* 1. 电机对象初始化（含电流环带宽自整定） */
    foc_motor_init(&g_foc_motors[0],
                   &g_board_m0_driver,
                   &g_board_m0_current,
                   &g_board_m0_sensor,
                   &m0_p, &m0_c,
                   FOC_DT_FAST, FOC_SLOW_DIV);

    /* 存储的校准偏移：标记 from_store，等 Z 重建零点后即可闭环
     * （按键/`c` 触发的校准会走快速索引搜索） */
    if (store_st == FOC_STORE_LOADED_CALIB) {
        g_foc_motors[0].calib.from_store = 1U;
        g_foc_motors[0].calib.direction = stored_dir;
        g_foc_motors[0].calib.electrical_offset_rad = stored_offset;
    }

    /* 参数自检（存储数据损坏/配置手滑的最后防线） */
    if (app_params_sane(&m0_p, &m0_c) == 0U) {
        foc_motor_fault(&g_foc_motors[0], FOC_FAULT_BAD_CONFIG);
    }

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

    /* 3. 电流采样链路：OPAMP → ADC 校准/使能 → 注入触发路径。
     *    失败也不能提前返回——串口/看门狗必须照常启动，
     *    否则板子最需要诊断的时候反而既没有 CLI 也没有看门狗 */
    if (current_shunt_init() == 0U) {
        foc_motor_fault(&g_foc_motors[0], FOC_FAULT_CURRENT_SENSE);
    } else {
        /* 4. PWM 定时器安全上电（MOE 保持关闭） */
        foc_board_init();

        /* 5. 三相零偏校准（阻塞，约 130ms @16kHz×2×1024 采样） */
        if (current_shunt_calibrate(500U) == 0U) {
            foc_motor_fault(&g_foc_motors[0], FOC_FAULT_CURRENT_SENSE);
        }
    }

    g_foc_state_diag = (uint8_t)g_foc_motors[0].state;

    /* 6. 通信：串口命令行 + VOFA 遥测 */
    foc_cmd_init();
    foc_telemetry_init();

    /* 7. 稳定性设施：CPU 统计 + 独立看门狗。
     *    看门狗必须放在所有阻塞初始化（含 130ms 零偏校准）之后 */
    foc_board_dwt_init();
#if FOC_WATCHDOG_ENABLE
    foc_board_watchdog_init(FOC_WATCHDOG_TIMEOUT_MS);
#endif

    if (store_st != FOC_STORE_EMPTY) {
        foc_cmd_print("config loaded from flash%s "
                      "(pp=%.0f Rs=%.4f Ls=%.2fuH)\r\n",
                      (store_st == FOC_STORE_LOADED_CALIB)
                          ? " with calib offset" : "",
                      (double)m0_p.pole_pairs,
                      (double)m0_p.rs_ohm, (double)(m0_p.ls_henry * 1e6f));
    }
}

void foc_app_isr_current_loop(void)
{
    uint32_t t0 = foc_board_cycles();

    foc_motor_fast_loop(&g_foc_motors[0]);

#if FOC_NUM_AXES >= 2
    /* 虚拟轴与轴 0 共享控制节拍。
     * 接入真实硬件后，把这行移到轴 1 自己的电流采样完成中断里。 */
    foc_motor_fast_loop(&g_foc_motors[1]);
#endif

    foc_telemetry_isr_tick();
    foc_board_cpu_sample(foc_board_cycles() - t0);
}

void foc_app_task(void)
{
    foc_motor_t *m0 = &g_foc_motors[0];

#if FOC_WATCHDOG_ENABLE
    foc_board_watchdog_kick();
#endif

    /* 校准与参数辨识状态机 */
    foc_calib_task();
    foc_ident_task();

    /* 串口命令 */
    foc_cmd_task();

    /* 采样故障不可在 IDLE/CALIB 中被隐藏。驱动已经关闭 MOE，这里把
     * 轴状态统一锁存为 FAULT，防止下一次按键再次尝试使能功率级。 */
    if ((m0->state != FOC_STATE_FAULT) &&
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
