#include "foc_calib.h"
#include "foc_app.h"
#include "../Core/foc_controller.h"
#include "../Driver/encoder/abz_encoder.h"
#include "../Driver/current/current_shunt.h"
#include "../HAL/foc_config.h"

/*
 * FOC 编码器电角度校准。
 *
 * 当前模块不保存 Flash。每次上电后执行一次校准，
 * 校准结果只保存在 RAM 中，仅对本次运行有效。
 *
 * 校准流程：
 *   1. 在已知电角度上施加 D 轴电压，把转子吸到参考位置。
 *   2. 暂时把此时的转子位置当作增量编码器零点。
 *   3. 以很低速度开环旋转，直到 ABZ 的 Z/index 脉冲到来。
 *   4. Z/index 到来时，先捕获编码器计数，再允许自动清零。
 *   5. 把这个机械计数偏移换算成电角度偏移。
 *
 * 运行时电角度：
 *   theta_e = wrap(direction * pole_pairs * theta_mech + electrical_offset_rad)
 *
 * 有了这个角度，反 Park 变换里的 D 轴就能对准转子磁链，
 * Q 轴则用于产生转矩。
 */
volatile foc_calib_state_t g_foc_calib_state = FOC_CALIB_IDLE;
volatile float g_foc_calib_align_voltage = FOC_CALIB_ALIGN_VOLTAGE;
volatile float g_foc_calib_search_voltage = FOC_CALIB_SEARCH_VOLTAGE;
#define calib_state g_foc_calib_state
static foc_calib_result_t calib_result = {0};
static uint32_t calib_state_tick = 0U;

/* 带 tick 回绕保护的延时判断。 */
static uint8_t foc_calib_elapsed(uint32_t now, uint32_t start, uint32_t delay_ms)
{
    return ((uint32_t)(now - start) >= delay_ms) ? 1U : 0U;
}

/* Linear voltage ramp used at the beginning of each energized stage. */
static float foc_calib_ramp_voltage(uint32_t now, float target_voltage)
{
    uint32_t elapsed = (uint32_t)(now - calib_state_tick);

    if ((FOC_CALIB_VOLTAGE_RAMP_MS == 0U) ||
        (elapsed >= FOC_CALIB_VOLTAGE_RAMP_MS)) {
        return target_voltage;
    }

    return target_voltage * ((float)elapsed / (float)FOC_CALIB_VOLTAGE_RAMP_MS);
}

/* 进入新的校准阶段前，清掉旧的 Z/index 事件。 */
static void foc_calib_clear_pending_index(void)
{
    int32_t dummy;

    while (abz_encoder_consume_index(&dummy)) {
    }
}

/* 校准失败按真实故障处理。 */
static void foc_calib_finish_fail(void)
{
    foc_openloop_hold(FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);
    foc_pwm_disable();
    foc_set_state(FOC_STATE_FAULT);
    calib_result.valid = 0U;
    calib_state = FOC_CALIB_FAIL;
    system_power_checkpoint(SYSTEM_CHECKPOINT_CALIB_FAIL,
                            (uint32_t)g_foc_pwm_stage,
                            (uint32_t)calib_state,
                            (uint32_t)g_foc_state_diag);
}

void foc_calib_abort(void)
{
    foc_calib_finish_fail();
}

void foc_calib_start(void)
{
    if (current_shunt_is_ready() == 0U) {
        foc_safety_latch_fault(FOC_SAFETY_FAULT_CURRENT_SENSE);
        foc_calib_finish_fail();
        return;
    }

    /* 清空上一次运行留下的校准结果。 */
    calib_result.valid = 0U;
    calib_result.direction = (int8_t)FOC_CALIB_DIRECTION;
    calib_result.index_offset_cnt = 0;
    calib_result.index_offset_rad = 0.0f;
    calib_result.electrical_offset_rad = 0.0f;

    foc_calib_clear_pending_index();

    /* 保持当前 ABZ 行为：允许在下一个 Z/index 到来后自动清零。 */
    abz_encoder_set_zero_on_index(1U);

    calib_state_tick = HAL_GetTick();
    calib_state = FOC_CALIB_CHARGE_BOOTSTRAP;

    /* Charge the gate-driver bootstrap supply before normal high-side PWM. */
    foc_openloop_hold(FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);
    foc_set_state(FOC_STATE_CALIB);
    system_power_checkpoint(SYSTEM_CHECKPOINT_CALIB_START,
                            (uint32_t)g_foc_pwm_stage,
                            (uint32_t)calib_state,
                            (uint32_t)g_foc_state_diag);
    foc_pwm_bootstrap_start();
}

void foc_calib_task(void)
{
    uint32_t now = HAL_GetTick();
    int32_t index_cnt;

    if ((current_shunt_is_ready() == 0U) &&
        (foc_calib_is_active() != 0U)) {
        foc_safety_latch_fault(FOC_SAFETY_FAULT_CURRENT_SENSE);
        foc_calib_finish_fail();
        return;
    }

    switch (calib_state) {
    case FOC_CALIB_IDLE:
    case FOC_CALIB_DONE:
    case FOC_CALIB_FAIL:
        return;

    case FOC_CALIB_CHARGE_BOOTSTRAP:
        if (foc_calib_elapsed(now, calib_state_tick, FOC_CALIB_BOOTSTRAP_MS)) {
            foc_pwm_disable();
            foc_openloop_hold(FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);
            calib_state_tick = now;
            calib_state = FOC_CALIB_PWM_NEUTRAL;
            system_power_checkpoint(SYSTEM_CHECKPOINT_CALIB_NEUTRAL,
                                    (uint32_t)g_foc_pwm_stage,
                                    (uint32_t)calib_state,
                                    (uint32_t)g_foc_state_diag);
            foc_pwm_enable();
        }
        break;

    case FOC_CALIB_PWM_NEUTRAL:
        foc_openloop_hold(FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);
        if (foc_calib_elapsed(now, calib_state_tick, FOC_CALIB_NEUTRAL_MS)) {
            calib_state_tick = now;
            calib_state = FOC_CALIB_ALIGN_D;
            system_power_checkpoint(SYSTEM_CHECKPOINT_CALIB_ALIGN,
                                    (uint32_t)g_foc_pwm_stage,
                                    (uint32_t)calib_state,
                                    (uint32_t)g_foc_state_diag);
        }
        break;

    case FOC_CALIB_ALIGN_D:
        /* 持续把转子吸在已知电角度，等待机械位置稳定。 */
        foc_openloop_hold(FOC_CALIB_ALIGN_THETA_E,
                          foc_calib_ramp_voltage(now, g_foc_calib_align_voltage),
                          0.0f);
        if (foc_calib_elapsed(now, calib_state_tick, FOC_CALIB_ALIGN_MS)) {
            foc_calib_clear_pending_index();

            /* 把对齐后的转子位置作为增量编码器的临时零点。 */
            abz_encoder_force_zero();
            calib_state_tick = now;
            calib_state = FOC_CALIB_CLEAR_AT_ALIGN;
            system_power_checkpoint(SYSTEM_CHECKPOINT_CALIB_CLEAR,
                                    (uint32_t)g_foc_pwm_stage,
                                    (uint32_t)calib_state,
                                    (uint32_t)g_foc_state_diag);
        }
        break;

    case FOC_CALIB_CLEAR_AT_ALIGN:
        /* 强制清零后稍等一下，让编码器驱动和控制周期完成处理。 */
        foc_openloop_hold(FOC_CALIB_ALIGN_THETA_E, g_foc_calib_align_voltage, 0.0f);
        if (foc_calib_elapsed(now, calib_state_tick, FOC_CALIB_ZERO_SETTLE_MS)) {
            foc_calib_clear_pending_index();
            abz_encoder_set_zero_on_index(1U);

            /* 低速开环旋转，搜索 Z/index。电压从 0 重新缓升。 */
            foc_openloop_spin(FOC_CALIB_SEARCH_RPM, 0.0f, 0.0f);
            calib_state_tick = now;
            calib_state = FOC_CALIB_SEARCH_INDEX;
            system_power_checkpoint(SYSTEM_CHECKPOINT_CALIB_SEARCH,
                                    (uint32_t)g_foc_pwm_stage,
                                    (uint32_t)calib_state,
                                    (uint32_t)g_foc_state_diag);
        }
        break;

    case FOC_CALIB_SEARCH_INDEX:
        /* 继续旋转，直到消费到新的 Z/index 事件。 */
        foc_openloop_spin(FOC_CALIB_SEARCH_RPM,
                          0.0f,
                          foc_calib_ramp_voltage(now, g_foc_calib_search_voltage));
        if (abz_encoder_consume_index(&index_cnt)) {
            calib_result.index_offset_cnt = index_cnt;
            calib_result.index_offset_rad = limit_angle_rad((float)index_cnt * ABZ_RAD_PER_CNT);

            /* 把对齐位置到 Z/index 的机械角距离换算成电角度偏移。 */
            calib_result.electrical_offset_rad = limit_angle_rad(
                FOC_CALIB_ALIGN_THETA_E +
                ((float)FOC_CALIB_DIRECTION * foc_motor_info.pole_pairs * calib_result.index_offset_rad));
            calib_result.valid = 1U;

            foc_openloop_hold(FOC_CALIB_ALIGN_THETA_E, 0.0f, 0.0f);

            /* 从这里开始，RUN 模式可以使用校准后的编码器角度。 */
            foc_set_angle_source(FOC_ANGLE_ENCODER_CALIBRATED);
            foc_pwm_disable();
            foc_set_state(FOC_STATE_IDLE);
            calib_state = FOC_CALIB_DONE;
            system_power_checkpoint(SYSTEM_CHECKPOINT_CALIB_DONE,
                                    (uint32_t)g_foc_pwm_stage,
                                    (uint32_t)calib_state,
                                    (uint32_t)g_foc_state_diag);
        } else if (foc_calib_elapsed(now, calib_state_tick, FOC_CALIB_SEARCH_TIMEOUT_MS)) {
            foc_safety_latch_fault(FOC_SAFETY_FAULT_CALIB_TIMEOUT);
            foc_calib_finish_fail();
        }
        break;

    default:
        foc_safety_latch_fault(FOC_SAFETY_FAULT_CALIB_STATE);
        foc_calib_finish_fail();
        break;
    }
}

uint8_t foc_calib_is_active(void)
{
    /* active 表示校准模块正在接管 PWM 电压命令。 */
    return ((calib_state == FOC_CALIB_ALIGN_D) ||
            (calib_state == FOC_CALIB_CHARGE_BOOTSTRAP) ||
            (calib_state == FOC_CALIB_PWM_NEUTRAL) ||
            (calib_state == FOC_CALIB_CLEAR_AT_ALIGN) ||
            (calib_state == FOC_CALIB_SEARCH_INDEX)) ? 1U : 0U;
}

uint8_t foc_calib_is_valid(void)
{
    return calib_result.valid;
}

foc_calib_state_t foc_calib_get_state(void)
{
    return calib_state;
}

const foc_calib_result_t *foc_calib_get_result(void)
{
    return &calib_result;
}

float foc_calib_get_electrical_offset(void)
{
    return calib_result.electrical_offset_rad;
}
