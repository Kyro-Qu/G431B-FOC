/**
 * @file    foc_ident.c
 * @brief   电机参数自动测量实现（Rs 升压平均法 / Ls 方波注入法）
 */

#include "foc_ident.h"
#include "foc_cmd.h"
#include "foc_config.h"
#include "foc_utils.h"
#include "main.h"

/* ---- 测量参数（保守默认值，必要时在此调整） ---- */
#define IDENT_TEST_CURRENT_A   1.0f    /* Rs 目标电流 */
#define IDENT_MAX_VOLTAGE_V    2.0f    /* Rs 电压上限（2V/1A → 最大可测 2Ω） */
#define IDENT_MIN_CURRENT_A    0.1f    /* 平均电流低于此值判失败 */
#define IDENT_RAMP_GAIN        0.003f  /* 升压积分增益 V/(A·ms) */
#define IDENT_BOOTSTRAP_MS     10U
#define IDENT_NEUTRAL_MS       200U
#define IDENT_RAMP_MS          1000U   /* 升压+稳定总时长 */
#define IDENT_MEASURE_MS       400U    /* Rs 平均窗口 */
#define IDENT_LS_VOLTAGE_V     0.15f   /* Ls 方波幅值（20µH 时纹波约 ±0.23A） */
#define IDENT_LS_CYCLES        4096U   /* Ls 记账拍数（每极性2拍，共约512ms） */
#define IDENT_LS_SKIP          8U      /* 起始丢弃拍数（等电流进入稳态三角波） */
#define IDENT_LS_MIN_H         0.0000005f /* 0.5 µH，排除噪声/数值异常 */
#define IDENT_LS_MAX_H         0.02f      /* 20 mH，超出本驱动适用范围 */

static foc_motor_t *ident_motor = 0;
static foc_ident_state_t ident_state = FOC_IDENT_IDLE;
static foc_ident_result_t ident_result = {0};
static uint32_t ident_tick = 0U;
static uint32_t ident_last_ms = 0U;

/* Rs 阶段累计 */
static float rs_voltage = 0.0f;
static float rs_v_sum = 0.0f;
static float rs_i_sum = 0.0f;
static uint32_t rs_samples = 0U;

/* Ls 阶段累计（快环钩子上下文写，任务上下文只在结束后读） */
static volatile float ls_vs_sum = 0.0f;   /* Σ 极性·(Vapplied - Rs·i)·dt */
static volatile float ls_di_sum = 0.0f;   /* Σ 极性·Δi */
static volatile uint32_t ls_count = 0U;
static float ls_i_prev = 0.0f;
static int8_t ls_pol_applied = 0;         /* 当前施加电压的极性 */
static uint8_t ls_phase = 0U;             /* 0=过渡拍（刚翻转），1=稳定拍（记账） */

static void ident_set_state(foc_ident_state_t s)
{
    ident_state = s;
}

static void ident_cleanup(foc_motor_t *m)
{
    m->test_hook = 0;
    foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
    foc_motor_restore_current_limits(m);
    m->pwm_hold = 0U;
    m->id_ref = 0.0f;
    m->iq_ref = 0.0f;
    m->v_dq.d = 0.0f;
    m->v_dq.q = 0.0f;
    if (m->state == FOC_STATE_CALIB) {
        m->drv->disable();
        m->state = FOC_STATE_IDLE;
    }
}

static void ident_fail(const char *reason)
{
    if (ident_motor != 0) {
        ident_cleanup(ident_motor);
    }
    ident_set_state(FOC_IDENT_FAIL);
    foc_cmd_print("ident FAIL: %s\r\n", reason);
}

/*
 * Ls 方波注入钩子：CALIB 状态下由快环每拍调用（电流已采样、电压未选择）。
 *
 * 时序要点：写入的比较值经过预装载，在下一次 update 事件才进入 PWM，
 * "写电压"到"电压完整作用于一个采样间隔"之间隔着一个过渡拍。
 * 若每拍翻转极性，采样间隔内新旧电压各占一段，Δi 被系统性衰减。
 * 因此每个极性保持 2 拍：第 1 拍是过渡拍（丢弃），第 2 拍整个采样
 * 间隔内电压恒定，只把这一拍的 Δi 记账——不依赖装载相位的细节。
 */
static void ident_ls_hook(foc_motor_t *m)
{
    float i_now = m->i_dq.d;

    /* 达到拍数后停注入（置 0V）并停止记账，任务层负责收尾计算 */
    if (ls_count >= (IDENT_LS_CYCLES + IDENT_LS_SKIP)) {
        m->v_openloop.d = 0.0f;
        m->v_openloop.q = 0.0f;
        return;
    }

    if ((ls_pol_applied != 0) && (ls_phase == 1U)) {
        /* 稳定拍：这个采样间隔内 PWM 输出恒为 pol·V */
        if (ls_count >= IDENT_LS_SKIP) {
            float pol = (float)ls_pol_applied;
            float applied_v = pol * IDENT_LS_VOLTAGE_V;

            ls_vs_sum += pol *
                (applied_v - (ident_result.rs_ohm * ls_i_prev)) *
                m->dt_fast;
            ls_di_sum += pol * (i_now - ls_i_prev);
        }
        ++ls_count;
    }
    ls_i_prev = i_now;

    if ((ls_pol_applied != 0) && (ls_phase == 0U)) {
        ls_phase = 1U;                    /* 保持极性，下一拍是稳定拍 */
        return;
    }

    ls_pol_applied = (ls_pol_applied == 1) ? -1 : 1;
    ls_phase = 0U;
    m->v_openloop.d = (float)ls_pol_applied * IDENT_LS_VOLTAGE_V;
    m->v_openloop.q = 0.0f;
}

void foc_ident_start(foc_motor_t *m)
{
    if ((m == 0) || (m->state != FOC_STATE_IDLE)) {
        foc_cmd_print("err: ident needs IDLE\r\n");
        return;
    }
    if ((m->cur == 0) || (m->cur->is_ready() == 0U)) {
        foc_cmd_print("err: ident needs current sensing\r\n");
        return;
    }

    ident_motor = m;
    ident_result.valid = 0U;
    rs_voltage = 0.0f;
    rs_v_sum = 0.0f;
    rs_i_sum = 0.0f;
    rs_samples = 0U;

    foc_motor_override_current_limits(m,
                                      FOC_CALIB_CURRENT_LIMIT_A,
                                      FOC_CALIB_HARD_LIMIT_A);

    ident_tick = HAL_GetTick();
    ident_last_ms = ident_tick;
    ident_set_state(FOC_IDENT_BOOTSTRAP);

    /* 与校准相同的安全上电序列：先充自举电容 */
    foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
    m->pwm_hold = 1U;
    m->state = FOC_STATE_CALIB;
    m->drv->bootstrap();
    foc_cmd_print("ident start (Rs then Ls, ~2s, rotor will lock)\r\n");
}

void foc_ident_task(void)
{
    foc_motor_t *m = ident_motor;
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms;

    if ((m == 0) || (foc_ident_is_active() == 0U)) {
        return;
    }

    /* 过流等异常已把轴打进 FAULT：收尾 */
    if (m->state == FOC_STATE_FAULT) {
        m->test_hook = 0;
        foc_motor_restore_current_limits(m);
        m->pwm_hold = 0U;
        ident_set_state(FOC_IDENT_FAIL);
        foc_cmd_print("ident FAIL: fault=%u\r\n",
                      (unsigned)m->safety.fault_code);
        return;
    }
    /* 用户中途 disable：安静退出 */
    if (m->state != FOC_STATE_CALIB) {
        ident_cleanup(m);
        ident_set_state(FOC_IDENT_FAIL);
        return;
    }
    if (m->cur->is_ready() == 0U) {
        ident_fail("current sense lost");
        return;
    }

    dt_ms = now - ident_last_ms;

    switch (ident_state) {
    case FOC_IDENT_BOOTSTRAP:
        if ((uint32_t)(now - ident_tick) >= IDENT_BOOTSTRAP_MS) {
            m->drv->disable();
            m->pwm_hold = 0U;
            foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
            ident_tick = now;
            ident_set_state(FOC_IDENT_NEUTRAL);
            m->drv->enable();
        }
        break;

    case FOC_IDENT_NEUTRAL:
        if ((uint32_t)(now - ident_tick) >= IDENT_NEUTRAL_MS) {
            ident_tick = now;
            ident_set_state(FOC_IDENT_RS_RAMP);
        }
        break;

    case FOC_IDENT_RS_RAMP:
        /* 积分升压逼近目标电流；电压到顶则以到顶电压测量 */
        if (dt_ms > 0U) {
            rs_voltage += IDENT_RAMP_GAIN * (float)dt_ms *
                          (IDENT_TEST_CURRENT_A - m->i_dq_filt.d);
            rs_voltage = foc_clampf(rs_voltage, 0.0f, IDENT_MAX_VOLTAGE_V);
            foc_motor_openloop_hold(m, 0.0f, rs_voltage, 0.0f);
        }
        if ((uint32_t)(now - ident_tick) >= IDENT_RAMP_MS) {
            ident_tick = now;
            ident_set_state(FOC_IDENT_RS_MEASURE);
        }
        break;

    case FOC_IDENT_RS_MEASURE:
        /* 保持电压不变，按毫秒节拍累计平均。
         * 不能每次主循环都累加：主循环远快于 1kHz，float 累加到
         * 千万级样本后精度耗尽（2^24 问题），毫秒节流后约 400 个样本 */
        if (dt_ms > 0U) {
            rs_v_sum += rs_voltage;
            rs_i_sum += m->i_dq_filt.d;
            ++rs_samples;
        }
        if ((uint32_t)(now - ident_tick) >= IDENT_MEASURE_MS) {
            float i_avg;

            if (rs_samples == 0U) {
                ident_fail("no samples");
                break;
            }
            i_avg = rs_i_sum / (float)rs_samples;

            if (i_avg < IDENT_MIN_CURRENT_A) {
                ident_fail("no measurable current (check motor wiring)");
                break;
            }
            ident_result.rs_ohm = (rs_v_sum / (float)rs_samples) / i_avg;
            ident_result.test_current_a = i_avg;

            /* 进入 Ls：先撤压，由钩子接管 d 轴电压 */
            foc_motor_openloop_hold(m, 0.0f, 0.0f, 0.0f);
            ls_vs_sum = 0.0f;
            ls_di_sum = 0.0f;
            ls_count = 0U;
            ls_i_prev = 0.0f;
            ls_pol_applied = 0;
            ls_phase = 0U;
            m->test_hook = ident_ls_hook;
            ident_tick = now;
            ident_set_state(FOC_IDENT_LS);
        }
        break;

    case FOC_IDENT_LS:
        if (ls_count >= (IDENT_LS_CYCLES + IDENT_LS_SKIP)) {
            float di = ls_di_sum;
            float ls;

            m->test_hook = 0;
            if ((di < 1e-3f) && (di > -1e-3f)) {
                ident_fail("no current response (Ls too big or wiring)");
                break;
            }
            ls = ls_vs_sum / di;
            if (!((ls >= IDENT_LS_MIN_H) && (ls <= IDENT_LS_MAX_H))) {
                ident_fail("Ls out of range or noisy");
                break;
            }
            ident_result.ls_henry = ls;
            ident_result.valid = 1U;
            ident_cleanup(m);
            ident_set_state(FOC_IDENT_DONE);
            foc_cmd_print("ident OK: Rs=%.4f ohm  Ls=%.2f uH  (I=%.2fA)\r\n"
                          "apply with 'ident apply', then 'conf write'\r\n",
                          (double)ident_result.rs_ohm,
                          (double)(ident_result.ls_henry * 1e6f),
                          (double)ident_result.test_current_a);
        } else if ((uint32_t)(now - ident_tick) >= 2000U) {
            ident_fail("Ls timeout");
        }
        break;

    default:
        ident_fail("bad state");
        break;
    }

    ident_last_ms = now;
}

uint8_t foc_ident_is_active(void)
{
    return ((ident_state == FOC_IDENT_BOOTSTRAP) ||
            (ident_state == FOC_IDENT_NEUTRAL) ||
            (ident_state == FOC_IDENT_RS_RAMP) ||
            (ident_state == FOC_IDENT_RS_MEASURE) ||
            (ident_state == FOC_IDENT_LS)) ? 1U : 0U;
}

foc_ident_state_t foc_ident_get_state(void)
{
    return ident_state;
}

const foc_ident_result_t *foc_ident_get_result(void)
{
    return &ident_result;
}

uint8_t foc_ident_apply(foc_motor_t *m)
{
    float v_max;
    float bw;

    if ((m == 0) || (ident_result.valid == 0U) ||
        (m->state == FOC_STATE_RUN) || (m->state == FOC_STATE_CALIB)) {
        return 0U;
    }

    m->params.rs_ohm = ident_result.rs_ohm;
    m->params.ls_henry = ident_result.ls_henry;

    /* 用新参数按当前带宽重整定电流环 */
    bw = m->cfg.current_bw_rads;
    v_max = m->drv->u_dc * INV_SQRT_3;
    foc_pid_init(&m->pid_id, m->params.ls_henry * bw,
                 m->params.rs_ohm * bw, 0.0f, v_max, 0.0f);
    foc_pid_init(&m->pid_iq, m->params.ls_henry * bw,
                 m->params.rs_ohm * bw, 0.0f, v_max, 0.0f);
    return 1U;
}
