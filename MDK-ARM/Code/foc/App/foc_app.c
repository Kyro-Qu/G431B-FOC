/**
 * @file    foc_app.c
 * @brief   应用层实现：初始化时序、多轴调度、按键状态机
 */

#include "foc_app.h"
#include "foc_calib.h"
#include "foc_cmd.h"
#include "foc_ident.h"
#include "foc_sensorless_bench.h"
#include "foc_angle_manager.h"
#include "foc_telemetry.h"
#include "foc_stp.h"
#include "main.h"
#include "../HAL/foc_board_g431.h"
#include "../HAL/foc_store.h"
#include "../Driver/current/current_shunt.h"
#include "../Driver/encoder/abz_encoder.h"

#include <math.h>

foc_motor_t g_foc_motors[FOC_NUM_AXES];
volatile uint8_t g_foc_state_diag = (uint8_t)FOC_STATE_IDLE;
foc_anticog_t g_m0_anticog;

/*
 * V/F 上电必须是零给定。旧版默认 0.3 V / 100 RPM，且 RUN 时从这里反复
 * 回写，导致单独执行 enable 也可能恢复上次给定并突然起转。
 */
/* V/F boost voltage. It is a tuning parameter, not a fixed output voltage. */
volatile float g_m0_openloop_vq = FOC_M0_VF_BOOST_V;
volatile float g_m0_openloop_rpm = 0.0f;
volatile float g_m0_openloop_vq_applied = 0.0f;
volatile float g_m0_openloop_rpm_applied = 0.0f;
volatile float g_m0_vf_slope_v_per_rpm = FOC_M0_VF_SLOPE_V_PER_RPM;
volatile float g_m0_vf_vq_target = 0.0f;

static uint32_t vf_ramp_tick_ms = 0U;
static uint8_t vf_ramp_active = 0U;

static float app_approach(float value, float target, float max_step)
{
    if (target > value + max_step) {
        return value + max_step;
    }
    if (target < value - max_step) {
        return value - max_step;
    }
    return target;
}

void foc_app_vf_reset_commands(void)
{
    /* Keep boost/slope tuning across stop; clear every motion/output state. */
    g_m0_openloop_rpm = 0.0f;
    g_m0_openloop_vq_applied = 0.0f;
    g_m0_openloop_rpm_applied = 0.0f;
    g_m0_vf_vq_target = 0.0f;
    vf_ramp_active = 0U;
    vf_ramp_tick_ms = HAL_GetTick();

    /* 不在校准/辨识期间覆盖测试状态机拥有的开环电压。 */
    if (g_foc_motors[0].state != FOC_STATE_CALIB) {
        foc_motor_openloop_spin(&g_foc_motors[0], 0.0f, 0.0f, 0.0f);
        g_foc_motors[0].vel_ref_rpm = 0.0f;
        g_foc_motors[0].iq_ref = 0.0f;
        g_foc_motors[0].id_ref = 0.0f;
    }
}

/* ---------------- 轴 0 抗齿槽力矩适配 ---------------- */

static float foc_anticog_feedforward_m0(float mech_angle)
{
    return foc_anticog_feedforward(&g_m0_anticog, mech_angle);
}

static void foc_anticog_sample_m0(float mech_angle, float iq_ref)
{
    foc_anticog_calib_sample(&g_m0_anticog, mech_angle, iq_ref);
}

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
    .vel_friction_a    = FOC_M0_VEL_FRICTION_A,
    .vel_start_a       = FOC_M0_VEL_START_A,
    .vel_start_rpm     = FOC_M0_VEL_START_RPM,
    .vel_track_kp      = FOC_M0_VEL_TRACK_KP,
    .vel_track_limit_rad = FOC_M0_VEL_TRACK_LIMIT,
    .vel_track_rpm     = FOC_M0_VEL_TRACK_RPM,
    .pos_kp            = FOC_M0_POS_KP,
    .pos_ki            = FOC_M0_POS_KI,
    .pos_vel_kp        = FOC_M0_POS_VEL_KP,
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
    .vel_friction_a    = 0.0f,
    .vel_start_a       = 0.0f,
    .vel_start_rpm     = 30.0f,
    .vel_track_kp      = 0.0f,
    .vel_track_limit_rad = 0.4f,
    .vel_track_rpm     = 100.0f,
    .pos_kp            = 1.0f,
    .pos_ki            = 1.25f,
    .pos_vel_kp        = 0.005f,
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

/* Reject NaN, infinity and values outside the supported control range. */
static uint8_t app_float_sane(float value, float lo, float hi)
{
    return ((value == value) && (value >= lo) && (value <= hi)) ? 1U : 0U;
}

/* 上电参数自检：非法参数直接 FAULT，好过带病闭环 */
static uint8_t app_params_sane(const foc_motor_params_t *p,
                               const foc_ctrl_cfg_t *c)
{
    if (!app_float_sane(p->pole_pairs, 0.01f, 100.0f) ||
        !app_float_sane(p->rs_ohm, 0.000001f, 100.0f) ||
        !app_float_sane(p->ls_henry, 0.000000001f, 1.0f) ||
        !app_float_sane(p->max_current_a, 0.001f, 1000.0f) ||
        !app_float_sane(p->hard_current_a, 0.001f, 1000.0f) ||
        (p->hard_current_a < p->max_current_a) ||
        !app_float_sane(p->max_rpm, 1.0f, 100000.0f) ||
        !app_float_sane(c->current_bw_rads,
                         FOC_CURRENT_BW_MIN_RADS,
                         FOC_CURRENT_BW_MAX_RADS) ||
        !app_float_sane(c->vel_kp, 0.0f, 10000.0f) ||
        !app_float_sane(c->vel_ki, 0.0f, 10000.0f) ||
        !app_float_sane(c->vel_ramp_rpm_s, 0.0f, 1000000.0f) ||
        !app_float_sane(c->vel_lpf_tf, 0.0f, 10.0f) ||
        !app_float_sane(c->vel_friction_a, 0.0f, 1000.0f) ||
        !app_float_sane(c->vel_start_a, 0.0f, 1000.0f) ||
        (c->vel_start_a < c->vel_friction_a) ||
        !app_float_sane(c->vel_start_rpm, 0.1f, 100000.0f) ||
        !app_float_sane(c->vel_track_kp, 0.0f, 1000.0f) ||
        !app_float_sane(c->vel_track_limit_rad, 0.001f, 1000.0f) ||
        !app_float_sane(c->vel_track_rpm, 0.1f, 100000.0f) ||
        !app_float_sane(c->pos_kp, 0.0f, 100000.0f) ||
        !app_float_sane(c->pos_ki, 0.0f, 10000.0f) ||
        !app_float_sane(c->pos_vel_kp, 0.0f, 10000.0f) ||
        !app_float_sane(c->pos_vel_limit_rpm, 0.1f, 100000.0f) ||
        ((c->traj_enable != 0U) && (c->traj_accel_rpm_s <= 0.0f))) {
        return 0U;
    }
    if (!app_float_sane(c->traj_accel_rpm_s, 0.0f, 1000000.0f)) {
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

    /* 0. 结构初始化：先清空齿槽表再读 Flash */
    foc_anticog_init(&g_m0_anticog);

    /* Flash 参数加载：有有效存储则覆盖 foc_config.h 默认值并还原齿槽表 */
    store_st = foc_store_load(&m0_p, &m0_c, &stored_dir, &stored_offset, &g_m0_anticog);

    /* 1. 电机对象初始化（含电流环带宽自整定） */
    foc_motor_init(&g_foc_motors[0],
                   &g_board_m0_driver,
                   &g_board_m0_current,
                   &g_board_m0_sensor,
                   &m0_p, &m0_c,
                   FOC_DT_FAST, FOC_SLOW_DIV);

    /* direction 是板级相序/编码器方向约定，不是运行时辨识量。即使尚无
     * Flash 校准，也让 V/F 的正 RPM 与编码器机械正方向保持一致。 */
    g_foc_motors[0].calib.direction = (int8_t)FOC_CALIB_DIRECTION;

    /* 挂载抗齿槽补偿器钩子 */
    g_foc_motors[0].anticog_hook = foc_anticog_feedforward_m0;
    g_foc_motors[0].anticog_sample_hook = foc_anticog_sample_m0;

    /* 存储的校准偏移：标记 from_store，等 Z 重建零点后即可闭环
     * （按键/`calib` 触发的校准会走快速索引搜索） */
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

    /* 7. 稳定性设施：CPU 统计 + 母线采样 + 独立看门狗。
     *    看门狗必须放在所有阻塞初始化（含 130ms 零偏校准）之后 */
    foc_board_dwt_init();
    foc_board_vbus_init();
#if FOC_WATCHDOG_ENABLE
    foc_board_watchdog_init(FOC_WATCHDOG_TIMEOUT_MS);
#endif

    /* 8. 无感影子评测平台与角度仲裁管理器初始化 */
    foc_sensorless_bench_init(&g_foc_motors[0]);
    foc_angle_mgr_init();

    if (store_st != FOC_STORE_EMPTY) {
        foc_cmd_print("config loaded from flash%s%s "
                      "(pp=%.0f Rs=%.4f Ls=%.2fuH)\r\n",
                      (store_st == FOC_STORE_LOADED_CALIB)
                          ? " with calib offset" : "",
                      (g_m0_anticog.state == FOC_ACOG_READY)
                          ? " + acog" : "",
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

    /* 无感观测器更新已移入快环中断（角度必须逐拍更新，主循环频率
     * 跟不上换相需求——切换实验失败根因 2026-09-03） */

#if FOC_WATCHDOG_ENABLE
    foc_board_watchdog_kick();
#endif

    /* 母线电压 100Hz 周期更新 */
    foc_board_vbus_update();
#if (FOC_VBUS_ENABLE && FOC_VBUS_AUTO_UPDATE_DRV)
    /* 动态将实时测得的母线电压注入功率级接口：
     * 1. 快环 SVPWM、电压圆限幅直接基于真实供电计算，消除电源电压波动带来的占空比误差；
     * 2. 弱磁控制环电压目标 v_target 自动按实时母线动态伸缩，高压充分利用、低压提前深去磁。 */
    foc_board_update_driver_vbus(foc_board_get_vbus_v());
#endif

#if (FOC_VBUS_ENABLE && FOC_VBUS_PROTECT_ENABLE)
    /* 母线欠压 (UVLO) / 过压 (OVLO) 安全保护（防锂电过放与制动反压击穿）。
     * 只有在采样稳定生效（完成至少 10 次采样建立稳态）后才参与判定，杜绝启动初期误触发。 */
    if ((g_foc_vbus_diag.valid != 0U) && (g_foc_vbus_diag.sample_count >= 10U)) {
        static uint32_t s_uv_start_tick = 0U;
        static uint32_t s_ov_start_tick = 0U;
        uint32_t now = HAL_GetTick();
        float vbus = g_foc_vbus_diag.voltage_v;

        if (vbus < g_foc_vbus_uv_threshold_v) {
            if (s_uv_start_tick == 0U) {
                s_uv_start_tick = now;
            } else if ((now - s_uv_start_tick) >= FOC_VBUS_FAULT_TIMEOUT_MS) {
                foc_motor_fault(m0, FOC_FAULT_UNDERVOLTAGE);
            }
        } else {
            s_uv_start_tick = 0U;
        }

        if (vbus > g_foc_vbus_ov_threshold_v) {
            if (s_ov_start_tick == 0U) {
                s_ov_start_tick = now;
            } else if ((now - s_ov_start_tick) >= FOC_VBUS_FAULT_TIMEOUT_MS) {
                foc_motor_fault(m0, FOC_FAULT_OVERVOLTAGE);
            }
        } else {
            s_ov_start_tick = 0U;
        }
    }
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

    /* 采样链 DISCONTINUITY 自动恢复（2026-09-03，用户授权）：cs=23
     * 停机后快环死（遥测停、CLI 假死），原本必须断电重启。延迟 2s
     * 自动重启采样链并清 FAULT——偶发单路假偏差从"停机"降级为
     * "瞬态扰动"。init 只建链路，ready 要靠 calibrate 置位，缺了
     * 它恢复循环每拍重新锁 fault=1（2026-09-04 切换实验定位）。 */
    {
        static uint32_t recov_last_ms = 0xFFFFFFFFU;
        if ((m0->state == FOC_STATE_FAULT) &&
            (m0->safety.fault_code == (uint8_t)FOC_FAULT_CURRENT_SENSE) &&
            (current_shunt_is_ready() == 0U)) {
            uint32_t now = HAL_GetTick();
            if ((recov_last_ms == 0xFFFFFFFFU) ||
                ((now - recov_last_ms) > 10000U)) {
                recov_last_ms = now;
                if ((current_shunt_init() != 0U) &&
                    (current_shunt_calibrate(500U) != 0U)) {
                    foc_motor_clear_fault(m0);
                    foc_cmd_print("sampling chain auto-recovered, re-calib");
                }
            }
        }
    }

    /*
     * 开环 V/F：目标先经过电压和机械转速斜坡，再提交给快环。
     * HAL tick 使斜坡不依赖主循环执行速度；一次最多补20ms，避免调试打印
     * 或短暂阻塞后产生大阶跃。
     */
    if ((foc_calib_is_active() == 0U) &&
        (m0->state == FOC_STATE_RUN) &&
        (m0->mode == FOC_MODE_OPENLOOP_VF)) {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed_ms;
        float dt;

        if (vf_ramp_active == 0U) {
            g_m0_openloop_vq_applied = 0.0f;
            g_m0_openloop_rpm_applied = 0.0f;
            vf_ramp_tick_ms = now;
            vf_ramp_active = 1U;
        }

        elapsed_ms = now - vf_ramp_tick_ms;
        if (elapsed_ms != 0U) {
            if (elapsed_ms > 20U) {
                elapsed_ms = 20U;
            }
            vf_ramp_tick_ms = now;
            dt = (float)elapsed_ms * 0.001f;
            g_m0_openloop_rpm_applied = app_approach(
                g_m0_openloop_rpm_applied, g_m0_openloop_rpm,
                FOC_M0_VF_RPM_RAMP_RPM_S * dt);

            /* Real V/F law: Vq = boost + slope * |rpm|.  At zero speed the
             * voltage is removed, so enable alone cannot heat or shake the
             * motor.  Guard globals as they may also be edited in Watch. */
            if (fabsf(g_m0_openloop_rpm_applied) < FOC_M0_VF_ZERO_RPM) {
                g_m0_vf_vq_target = 0.0f;
            } else {
                float boost = g_m0_openloop_vq;
                float slope = g_m0_vf_slope_v_per_rpm;
                float v_max = m0->drv->u_dc * 0.5773503f;

                if (!((boost >= 0.0f) && (boost <= v_max))) {
                    boost = 0.0f;
                }
                if (!((slope >= 0.0f) && (slope <= 0.01f))) {
                    slope = 0.0f;
                }
                g_m0_vf_vq_target = boost +
                    slope * fabsf(g_m0_openloop_rpm_applied);
                if (g_m0_vf_vq_target > v_max) {
                    g_m0_vf_vq_target = v_max;
                }
            }
            g_m0_openloop_vq_applied = app_approach(
                g_m0_openloop_vq_applied, g_m0_vf_vq_target,
                FOC_M0_VF_VQ_RAMP_V_S * dt);
        }

        foc_motor_openloop_spin(m0, g_m0_openloop_rpm_applied, 0.0f,
                                g_m0_openloop_vq_applied);
        m0->vel_ref_rpm = g_m0_openloop_rpm_applied;
    } else if (m0->state != FOC_STATE_CALIB) {
        g_m0_openloop_vq_applied = 0.0f;
        g_m0_openloop_rpm_applied = 0.0f;
        g_m0_vf_vq_target = 0.0f;
        vf_ramp_active = 0U;
        vf_ramp_tick_ms = HAL_GetTick();
    }

    g_foc_state_diag = (uint8_t)m0->state;

    /* 状态跳变与故障单触发事件监控 (FOC-STP EVENT 闭环) */
    {
        static uint8_t s_last_reported_state = 0xFFU;
        static uint8_t s_last_reported_fault = 0xFFU;

        if (m0->state != s_last_reported_state) {
            if (m0->state == FOC_STATE_FAULT) {
                foc_telemetry_report_event(FOC_STP_EVENT_FAULT_TRIP, m0->safety.fault_code,
                                           g_current_shunt_diag.fault_code,
                                           (uint32_t)(m0->safety.soft_current_a * 100.0f));
            } else if (s_last_reported_state != 0xFFU) {
                foc_telemetry_report_event(FOC_STP_EVENT_STATE_CHANGE, m0->safety.fault_code,
                                           g_current_shunt_diag.fault_code,
                                           (uint32_t)m0->state);
            }
            s_last_reported_state = (uint8_t)m0->state;
            s_last_reported_fault = m0->safety.fault_code;
        } else if (m0->safety.fault_code != s_last_reported_fault) {
            if (m0->safety.fault_code != 0U) {
                foc_telemetry_report_event(FOC_STP_EVENT_FAULT_TRIP, m0->safety.fault_code,
                                           g_current_shunt_diag.fault_code,
                                           (uint32_t)(m0->safety.soft_current_a * 100.0f));
            }
            s_last_reported_fault = m0->safety.fault_code;
        }
    }

    foc_telemetry_slow_tick();
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
        foc_app_vf_reset_commands();
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
