/**
 * @file    foc_calib.c
 * @brief   上电编码器电角度校准状态机实现
 */

#include "foc_calib.h"
#include "foc_config.h"
#include "foc_utils.h"
#include "main.h"
#include "../Core/foc_port.h"
#include "../HAL/foc_board_g431.h"

volatile uint8_t g_foc_calib_state = (uint8_t)FOC_CALIB_IDLE;
volatile float g_foc_calib_align_voltage = FOC_CALIB_ALIGN_VOLTAGE;
volatile float g_foc_calib_search_voltage = FOC_CALIB_SEARCH_VOLTAGE;

static foc_motor_t *calib_motor = 0;
static foc_calib_state_t calib_state = FOC_CALIB_IDLE;
static uint32_t calib_tick = 0U;

/* ---------------- 内部工具 ---------------- */

static void calib_set_state(foc_calib_state_t s)
{
    calib_state = s;
    g_foc_calib_state = (uint8_t)s;
}

/* tick 回绕安全的超时判断 */
static uint8_t calib_elapsed(uint32_t now, uint32_t start, uint32_t delay_ms)
{
    return ((uint32_t)(now - start) >= delay_ms) ? 1U : 0U;
}

/* 阶段起始的电压线性缓升，避免在静止低阻电机上产生电流阶跃 */
static float calib_ramp_voltage(uint32_t now, float target_voltage)
{
    uint32_t elapsed = (uint32_t)(now - calib_tick);

    if ((FOC_CALIB_VOLTAGE_RAMP_MS == 0U) ||
        (elapsed >= FOC_CALIB_VOLTAGE_RAMP_MS)) {
        return target_voltage;
    }
    return target_voltage * ((float)elapsed / (float)FOC_CALIB_VOLTAGE_RAMP_MS);
}

/*
 * Full calibration alignment trajectory.
 *
 * Building the field directly at theta_e=0 can leave the rotor trapped near
 * the opposite electrical equilibrium when the alignment voltage is small.
 * Build the field at -90 electrical degrees first, then sweep it to zero.
 * The moving field supplies a deterministic pull-out torque before the final
 * encoder zero is captured.
 */
static float calib_align_theta(uint32_t now)
{
    uint32_t elapsed = (uint32_t)(now - calib_tick);
    uint32_t sweep_ms;
    float progress;

    if (elapsed <= FOC_CALIB_VOLTAGE_RAMP_MS) {
        return -_PI_2;
    }
    if (FOC_CALIB_ALIGN_MS <= FOC_CALIB_VOLTAGE_RAMP_MS) {
        return FOC_CALIB_ALIGN_THETA_E;
    }

    sweep_ms = FOC_CALIB_ALIGN_MS - FOC_CALIB_VOLTAGE_RAMP_MS;
    progress = (float)(elapsed - FOC_CALIB_VOLTAGE_RAMP_MS) /
               (float)sweep_ms;
    progress = foc_clampf(progress, 0.0f, 1.0f);
    return -_PI_2 * (1.0f - progress);
}

/* 清掉历史 Z 事件，防止旧事件污染本阶段 */
static void calib_clear_pending_index(foc_motor_t *m)
{
    int32_t dummy;

    while (m->sensor->consume_index(&dummy) != 0U) {
    }
}

static void calib_checkpoint(uint32_t event)
{
    system_power_checkpoint(event,
                            (uint32_t)g_foc_pwm_stage,
                            (uint32_t)g_foc_calib_state,
                            (uint32_t)calib_motor->state);
}

/* 校准失败按真实故障处理：断 PWM、锁存故障、恢复电流阈值 */
static void calib_fail(foc_fault_t fault)
{
    foc_motor_t *m = calib_motor;

    if (m != 0) {
        foc_motor_openloop_hold(m, FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);
        m->pwm_hold = 0U;
        foc_motor_restore_current_limits(m);
        /* 失败也要关掉"每圈 Z 清零"，否则之后开环运行时
         * 每圈 Z 硬清零都会丢计数（位置遥测每圈漂移） */
        if ((m->sensor != 0) && (m->sensor->set_zero_on_index != 0)) {
            m->sensor->set_zero_on_index(0U);
        }
        foc_motor_fault(m, fault);
    }
    calib_set_state(FOC_CALIB_FAIL);
    if (m != 0) {
        calib_checkpoint(SYSTEM_CHECKPOINT_CALIB_FAIL);
    }
}

/* ---------------- API ---------------- */

void foc_calib_start(foc_motor_t *m)
{
    if ((m == 0) || (m->state != FOC_STATE_IDLE)) {
        return;
    }

    /* 单例 FSM 占用防护：另一根轴正在校准时拒绝新的 start。
     * 没有这个防护，对虚拟轴发 'c' 会把 calib_motor 改写掉，
     * 正在校准的轴从此无人监管（永远带电旋转、超时永不触发）。 */
    if (foc_calib_is_active() != 0U) {
        return;
    }

    /* 前置依赖检查：失败只故障目标轴，不碰全局 FSM
     * （calib_motor/calib_state 只属于真正开始了的校准会话） */
    if ((m->sensor == 0) || (m->sensor->consume_index == 0)) {
        foc_motor_fault(m, FOC_FAULT_CALIB_STATE);
        return;
    }
    if ((m->cur == 0) || (m->cur->is_ready() == 0U)) {
        foc_motor_fault(m, FOC_FAULT_CURRENT_SENSE);
        return;
    }

    calib_motor = m;
    m->calib.valid = 0U;
    /* from_store：direction/offset 来自 Flash 存储，保留不动，
     * 走"快速索引搜索"（跳过对齐吸附，只需慢转找 Z 重建零点） */
    if (m->calib.from_store == 0U) {
        m->calib.direction = (int8_t)FOC_CALIB_DIRECTION;
        m->calib.electrical_offset_rad = 0.0f;
    }

    calib_clear_pending_index(m);
    m->sensor->set_zero_on_index(1U);

    /* 校准期间使用压低的电流阈值 */
    foc_motor_override_current_limits(m,
                                      FOC_CALIB_CURRENT_LIMIT_A,
                                      FOC_CALIB_HARD_LIMIT_A);

    calib_tick = HAL_GetTick();
    calib_set_state(FOC_CALIB_BOOTSTRAP);

    /* 进入 CALIB：先给高边驱动的自举电容充电。
     * pwm_hold=1 让快环不要用中点占空比覆盖"低边全通" */
    foc_motor_openloop_hold(m, FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);
    m->pwm_hold = 1U;
    m->state = FOC_STATE_CALIB;
    calib_checkpoint(SYSTEM_CHECKPOINT_CALIB_START);
    m->drv->bootstrap();
}

void foc_calib_task(void)
{
    foc_motor_t *m = calib_motor;
    uint32_t now = HAL_GetTick();
    int32_t index_cnt;

    if ((m == 0) || (foc_calib_is_active() == 0U)) {
        return;
    }

    /* 电流采样链路失效：立即终止 */
    if (m->cur->is_ready() == 0U) {
        calib_fail(FOC_FAULT_CURRENT_SENSE);
        return;
    }

    /* 快环里的过流保护把轴打进了 FAULT：收尾并标记失败 */
    if (m->state == FOC_STATE_FAULT) {
        foc_motor_restore_current_limits(m);
        m->pwm_hold = 0U;
        m->sensor->set_zero_on_index(0U);
        calib_set_state(FOC_CALIB_FAIL);
        calib_checkpoint(SYSTEM_CHECKPOINT_CALIB_FAIL);
        return;
    }

    /* 用户在校准中途 disable：安静收尾，不锁存故障 */
    if (m->state != FOC_STATE_CALIB) {
        foc_motor_restore_current_limits(m);
        m->pwm_hold = 0U;
        m->sensor->set_zero_on_index(0U);
        foc_motor_openloop_hold(m, FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);
        calib_set_state(FOC_CALIB_FAIL);
        return;
    }

    switch (calib_state) {
    case FOC_CALIB_BOOTSTRAP:
        if (calib_elapsed(now, calib_tick, FOC_CALIB_BOOTSTRAP_MS)) {
            m->drv->disable();
            foc_motor_openloop_hold(m, FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);
            m->pwm_hold = 0U;
            calib_tick = now;
            calib_set_state(FOC_CALIB_NEUTRAL);
            calib_checkpoint(SYSTEM_CHECKPOINT_CALIB_NEUTRAL);
            m->drv->enable();
        }
        break;

    case FOC_CALIB_NEUTRAL:
        foc_motor_openloop_hold(m, FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);
        if (calib_elapsed(now, calib_tick, FOC_CALIB_NEUTRAL_MS)) {
            calib_tick = now;
            if (m->calib.from_store != 0U) {
                /* 快速索引搜索（ODrive index_search 语义）：
                 * 偏移已知，无需对齐吸附，直接慢转找 Z 重建编码器零点 */
                calib_clear_pending_index(m);
                m->sensor->set_zero_on_index(1U);
                foc_motor_openloop_spin(m, FOC_CALIB_SEARCH_RPM, 0.0f, 0.0f);
                calib_set_state(FOC_CALIB_SEARCH);
                calib_checkpoint(SYSTEM_CHECKPOINT_CALIB_SEARCH);
            } else {
                calib_set_state(FOC_CALIB_ALIGN);
                calib_checkpoint(SYSTEM_CHECKPOINT_CALIB_ALIGN);
            }
        }
        break;

    case FOC_CALIB_ALIGN:
        /* -90° 建场后缓慢扫到 0°，避免卡在反向电角度平衡点。 */
        foc_motor_openloop_hold(m, calib_align_theta(now),
                                calib_ramp_voltage(now, g_foc_calib_align_voltage),
                                0.0f);
        if (calib_elapsed(now, calib_tick, FOC_CALIB_ALIGN_MS)) {
            calib_clear_pending_index(m);
            /* 对齐后的转子位置作为增量编码器的临时零点 */
            m->sensor->force_zero();
            calib_tick = now;
            calib_set_state(FOC_CALIB_SETTLE);
            calib_checkpoint(SYSTEM_CHECKPOINT_CALIB_CLEAR);
        }
        break;

    case FOC_CALIB_SETTLE:
        /* 强制清零后稍等，让编码器驱动在快环上下文完成清零 */
        foc_motor_openloop_hold(m, FOC_CALIB_ALIGN_THETA_E,
                                g_foc_calib_align_voltage, 0.0f);
        if (calib_elapsed(now, calib_tick, FOC_CALIB_SETTLE_MS)) {
            calib_clear_pending_index(m);
            m->sensor->set_zero_on_index(1U);
            /* 低速开环旋转找 Z，q 轴电压重新从 0 缓升 */
            foc_motor_openloop_spin(m, FOC_CALIB_SEARCH_RPM, 0.0f, 0.0f);
            calib_tick = now;
            calib_set_state(FOC_CALIB_SEARCH);
            calib_checkpoint(SYSTEM_CHECKPOINT_CALIB_SEARCH);
        }
        break;

    case FOC_CALIB_SEARCH:
        foc_motor_openloop_spin(m, FOC_CALIB_SEARCH_RPM, 0.0f,
                                calib_ramp_voltage(now, g_foc_calib_search_voltage));
        if (m->sensor->consume_index(&index_cnt) != 0U) {
            /* Z 脉冲到来：对齐点 → Z 点的机械角距离换算电角度偏移。
             * 之后 θm 从 Z 点起算，所以 offset = θe(Z)：
             *   θe(Z) = θe(对齐) + dir·pp·Δθm */
            /* 存储偏移：Z 点的电角度是电机的固有属性，直接沿用；
             * 全新校准：由对齐点到 Z 点的机械角距离计算 */
            if (m->calib.from_store == 0U) {
                float index_rad = foc_wrap_0_2pi(
                    (float)index_cnt * m->sensor->rad_per_cnt);

                m->calib.electrical_offset_rad = foc_wrap_0_2pi(
                    FOC_CALIB_ALIGN_THETA_E +
                    ((float)m->calib.direction * m->params.pole_pairs *
                     index_rad));
            }
            m->calib.valid = 1U;

            /* 校准已建立零点，此后关闭"每圈 Z 清零"：Z 中断锁存的
             * 计数到快环处理之间有最多 62.5µs 延迟，运行中每圈硬清零
             * 会把这段时间转过的计数丢掉（角度倒跳，误差正比转速，
             * 12450 RPM 时可达 0.49 rad 电角度）。增量计数的半量程
             * 回绕法本身没有累积误差来源，Z 只在校准时用。 */
            m->sensor->set_zero_on_index(0U);

            foc_motor_openloop_hold(m, FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);
            foc_motor_set_angle_source(m, FOC_ANGLE_ENCODER_CALIBRATED);
            foc_motor_restore_current_limits(m);
            foc_motor_disarm(m);

            /*
             * 统一走标准停机路径：除关闭 PWM 外，还会清除速度/位置 PI、
             * 低速轨迹以及 dq 电流遥测残值。disarm() 内部保护收尾瞬间
             * 由 ISR 锁存的 FAULT，不会把它误写回 IDLE。
             */
            if (m->state == FOC_STATE_IDLE) {
                calib_set_state(FOC_CALIB_DONE);
                calib_checkpoint(SYSTEM_CHECKPOINT_CALIB_DONE);
            } else {
                /* 收尾瞬间被打进 FAULT：角度结果仍有效（valid=1），
                 * 但本次会话按失败收场，故障码留给用户 fault clear 清除 */
                calib_set_state(FOC_CALIB_FAIL);
                calib_checkpoint(SYSTEM_CHECKPOINT_CALIB_FAIL);
            }
        } else if (calib_elapsed(now, calib_tick, FOC_CALIB_SEARCH_TIMEOUT_MS)) {
            calib_fail(FOC_FAULT_CALIB_TIMEOUT);
        }
        break;

    default:
        calib_fail(FOC_FAULT_CALIB_STATE);
        break;
    }
}

uint8_t foc_calib_is_active(void)
{
    return ((calib_state == FOC_CALIB_BOOTSTRAP) ||
            (calib_state == FOC_CALIB_NEUTRAL) ||
            (calib_state == FOC_CALIB_ALIGN) ||
            (calib_state == FOC_CALIB_SETTLE) ||
            (calib_state == FOC_CALIB_SEARCH)) ? 1U : 0U;
}

foc_calib_state_t foc_calib_get_state(void)
{
    return calib_state;
}
