/**
 * @file    foc_motor.c
 * @brief   电机轴对象实现：快环（电流环+调制）与慢环（速度/位置环）
 *
 * 本文件是纯算法层：不 include 任何 STM32 头文件，
 * 所有硬件访问都通过 foc_types.h 里的接口表。
 */

#include "foc_motor.h"
#include "foc_transform.h"
#include "foc_utils.h"
#include "foc_port.h"
#include "../HAL/foc_config.h"
#include "../HAL/foc_board_g431.h"   /* 仅用于快环分段 CPU 剖析 */
#include "../App/foc_sensorless_bench.h"
#include "../App/foc_angle_manager.h"
#include "../Driver/encoder/abz_encoder.h"

/* dq 电流遥测低通时间常数（仅用于观察，不进控制环） */
#define FOC_IDQ_TELEM_LPF_TF 0.002f
/* 256 点标准正弦查找表 (Q1.31 插值基准, 覆盖 [0, 2pi]) */
const float g_foc_sin_tab[257] = {
     0.00000000f,  0.02454123f,  0.04906767f,  0.07356456f,  0.09801714f,  0.12241068f,  0.14673047f,  0.17096189f,
     0.19509032f,  0.21910124f,  0.24298018f,  0.26671276f,  0.29028468f,  0.31368174f,  0.33688985f,  0.35989504f,
     0.38268343f,  0.40524131f,  0.42755509f,  0.44961133f,  0.47139674f,  0.49289819f,  0.51410274f,  0.53499762f,
     0.55557023f,  0.57580819f,  0.59569930f,  0.61523159f,  0.63439328f,  0.65317284f,  0.67155895f,  0.68954054f,
     0.70710678f,  0.72424708f,  0.74095113f,  0.75720885f,  0.77301045f,  0.78834643f,  0.80320753f,  0.81758481f,
     0.83146961f,  0.84485357f,  0.85772861f,  0.87008699f,  0.88192126f,  0.89322430f,  0.90398929f,  0.91420976f,
     0.92387953f,  0.93299280f,  0.94154407f,  0.94952818f,  0.95694034f,  0.96377607f,  0.97003125f,  0.97570213f,
     0.98078528f,  0.98527764f,  0.98917651f,  0.99247953f,  0.99518473f,  0.99729046f,  0.99879546f,  0.99969882f,
     1.00000000f,  0.99969882f,  0.99879546f,  0.99729046f,  0.99518473f,  0.99247953f,  0.98917651f,  0.98527764f,
     0.98078528f,  0.97570213f,  0.97003125f,  0.96377607f,  0.95694034f,  0.94952818f,  0.94154407f,  0.93299280f,
     0.92387953f,  0.91420976f,  0.90398929f,  0.89322430f,  0.88192126f,  0.87008699f,  0.85772861f,  0.84485357f,
     0.83146961f,  0.81758481f,  0.80320753f,  0.78834643f,  0.77301045f,  0.75720885f,  0.74095113f,  0.72424708f,
     0.70710678f,  0.68954054f,  0.67155895f,  0.65317284f,  0.63439328f,  0.61523159f,  0.59569930f,  0.57580819f,
     0.55557023f,  0.53499762f,  0.51410274f,  0.49289819f,  0.47139674f,  0.44961133f,  0.42755509f,  0.40524131f,
     0.38268343f,  0.35989504f,  0.33688985f,  0.31368174f,  0.29028468f,  0.26671276f,  0.24298018f,  0.21910124f,
     0.19509032f,  0.17096189f,  0.14673047f,  0.12241068f,  0.09801714f,  0.07356456f,  0.04906767f,  0.02454123f,
     0.00000000f, -0.02454123f, -0.04906767f, -0.07356456f, -0.09801714f, -0.12241068f, -0.14673047f, -0.17096189f,
    -0.19509032f, -0.21910124f, -0.24298018f, -0.26671276f, -0.29028468f, -0.31368174f, -0.33688985f, -0.35989504f,
    -0.38268343f, -0.40524131f, -0.42755509f, -0.44961133f, -0.47139674f, -0.49289819f, -0.51410274f, -0.53499762f,
    -0.55557023f, -0.57580819f, -0.59569930f, -0.61523159f, -0.63439328f, -0.65317284f, -0.67155895f, -0.68954054f,
    -0.70710678f, -0.72424708f, -0.74095113f, -0.75720885f, -0.77301045f, -0.78834643f, -0.80320753f, -0.81758481f,
    -0.83146961f, -0.84485357f, -0.85772861f, -0.87008699f, -0.88192126f, -0.89322430f, -0.90398929f, -0.91420976f,
    -0.92387953f, -0.93299280f, -0.94154407f, -0.94952818f, -0.95694034f, -0.96377607f, -0.97003125f, -0.97570213f,
    -0.98078528f, -0.98527764f, -0.98917651f, -0.99247953f, -0.99518473f, -0.99729046f, -0.99879546f, -0.99969882f,
    -1.00000000f, -0.99969882f, -0.99879546f, -0.99729046f, -0.99518473f, -0.99247953f, -0.98917651f, -0.98527764f,
    -0.98078528f, -0.97570213f, -0.97003125f, -0.96377607f, -0.95694034f, -0.94952818f, -0.94154407f, -0.93299280f,
    -0.92387953f, -0.91420976f, -0.90398929f, -0.89322430f, -0.88192126f, -0.87008699f, -0.85772861f, -0.84485357f,
    -0.83146961f, -0.81758481f, -0.80320753f, -0.78834643f, -0.77301045f, -0.75720885f, -0.74095113f, -0.72424708f,
    -0.70710678f, -0.68954054f, -0.67155895f, -0.65317284f, -0.63439328f, -0.61523159f, -0.59569930f, -0.57580819f,
    -0.55557023f, -0.53499762f, -0.51410274f, -0.49289819f, -0.47139674f, -0.44961133f, -0.42755509f, -0.40524131f,
    -0.38268343f, -0.35989504f, -0.33688985f, -0.31368174f, -0.29028468f, -0.26671276f, -0.24298018f, -0.21910124f,
    -0.19509032f, -0.17096189f, -0.14673047f, -0.12241068f, -0.09801714f, -0.07356456f, -0.04906767f, -0.02454123f,
     0.00000000f,
};

#define FOC_VEL_ZERO_RESET_RPM 5.0f
#define FOC_RUN_TRIP_MARGIN_SCALE 1.25f
#define FOC_RUN_TRIP_MARGIN_A     0.10f
#define FOC_VEL_START_BOOST_RAMP_A_S  2.0f
#define FOC_VEL_START_BOOST_DECAY_A_S 4.0f
#define FOC_VEL_IQ_RAMP_A_S           4.0f
#define FOC_VEL_START_RELEASE_TICKS   50U
/* Low speed needs strong rejection of AS5047P ABI edge bursts, while using
 * the same low-speed feedback above roughly 100 RPM adds enough phase delay to
 * excite the speed loop.  Both filters run continuously and are cross-faded
 * so acceleration does not reset a filter or create a feedback step. */
#define FOC_VEL_FILTER_BLEND_START_RPM  50.0f
#define FOC_VEL_FILTER_BLEND_END_RPM   100.0f
#define FOC_VEL_FILTER_MIN_HZ           15.0f
#define FOC_VEL_LOW_FILTER_HZ           15.0f
/* 位置环速度阻尼只用于耗散动能，不允许单独打满 Iq。 */
#define FOC_POS_DAMP_CURRENT_RATIO 0.30f

/* ======================== 内部函数 ======================== */

/**
 * 电压圆限幅：保证 |v_dq| ≤ v_max（SVPWM 线性区上限 U_dc/√3）。
 * d 轴优先（MCSDK/VESC 惯例）：d 轴电压维持磁场定向，被裁剪的
 * 应该是产生转矩的 q 轴，这样过载时牺牲加速度而不是失去定向。
 */
static void foc_voltage_circle_limit(dq_t *v, float v_max)
{
    float vd = foc_clampf(v->d, -v_max, v_max);
    float vq_max_sq = (v_max * v_max) - (vd * vd);
    float vq = v->q;

    if ((vq * vq) > vq_max_sq) {
        float vq_max = sqrtf(vq_max_sq);
        vq = foc_clampf(vq, -vq_max, vq_max);
    }
    v->d = vd;
    v->q = vq;
}

/* 16kHz 故障黑匣子：快环每拍记录 iu/iw/theta_e/iq，故障时冻结。
 * 512 拍 = 32ms 历史。2026-09-02 高速诊断结论：2380 RPM 附近的电流
 * 爆发是事件型（28ms 稳态纹波后 1ms 内从 ±1A 冲到 5 A+），512 拍
 * 窗口确保完整覆盖起振时刻。 */
#ifndef FOC_BLACKBOX_LEN
#define FOC_BLACKBOX_LEN 512U   /* 正常由 foc_motor.h 提供，此处仅兜底 */
#endif

static float s_blackbox_u[FOC_BLACKBOX_LEN];
static float s_blackbox_w[FOC_BLACKBOX_LEN];
static float s_blackbox_th[FOC_BLACKBOX_LEN];
static float s_blackbox_iq[FOC_BLACKBOX_LEN];
static float s_blackbox_id[FOC_BLACKBOX_LEN];
static uint16_t s_blackbox_head = 0U;
static uint8_t s_blackbox_frozen = 0U;

static void foc_motor_blackbox_record(const foc_motor_t *m)
{
    uint32_t head = s_blackbox_head;
    s_blackbox_u[head] = m->i_abc.a;
    s_blackbox_w[head] = m->i_abc.c;
    s_blackbox_th[head] = m->theta_e;
    s_blackbox_iq[head] = m->i_dq.q;
    /* 诊断：记录低通前 id——解耦项 we*Ls*id 直通 vq，是爆发
     * 放大器的核心输入，验证 id/iq 摆动的相对幅度与相位。 */
    s_blackbox_id[head] = m->i_dq.d;
    s_blackbox_head = (uint16_t)((head + 1U) & (FOC_BLACKBOX_LEN - 1U));
}

void foc_motor_blackbox_freeze(void)
{
    s_blackbox_frozen = 1U;
}

void foc_motor_blackbox_resume(void)
{
    s_blackbox_frozen = 0U;
}

/* 按时间序（最旧→最新）读取第 i 拍。不再整表拷贝：调用方逐拍打印即可，
 * 省掉 foc_cmd 里 10 KB 的镜像缓冲（RAM 从 640B 余量回到 ~10.9KB）。 */
void foc_motor_blackbox_get(uint16_t i, foc_blackbox_sample_t *out)
{
    uint16_t idx = (uint16_t)((s_blackbox_head + i) % FOC_BLACKBOX_LEN);
    out->iu = s_blackbox_u[idx];
    out->iw = s_blackbox_w[idx];
    out->theta_e = s_blackbox_th[idx];
    out->iq = s_blackbox_iq[idx];
    out->id = s_blackbox_id[idx];
}

uint8_t foc_motor_blackbox_active(void)
{
    return s_blackbox_frozen;
}

/* Hard protection uses raw phase current; command-limit protection uses a
 * short filtered dq-vector magnitude.  This keeps the 6.5 A one-sample short
 * circuit response while preventing normal PWM ripple from being interpreted
 * as sustained torque-current overload. */
#define FOC_OVERCURRENT_TRIP_SAMPLES 8U

static void foc_motor_latch_current_trip(foc_motor_t *m, uint8_t hard_trip)
{
    m->safety.trip_current_u_a = m->i_abc.a;
    m->safety.trip_current_v_a = m->i_abc.b;
    m->safety.trip_current_w_a = m->i_abc.c;
    m->safety.trip_soft_current_a = m->safety.soft_current_a;
    m->safety.trip_was_hard = hard_trip;

    foc_motor_fault(m, (m->state == FOC_STATE_CALIB)
                           ? FOC_FAULT_CALIB_OVERCURRENT
                           : FOC_FAULT_RUN_OVERCURRENT);
}

static uint8_t foc_motor_check_hard_current(foc_motor_t *m)
{
    float ia = m->i_abc.a;
    float ib = m->i_abc.b;
    float ic = m->i_abc.c;
    float peak = fabsf(ia);
    float hard = m->safety.hard_current_limit_a;

    if (fabsf(ib) > peak) {
        peak = fabsf(ib);
    }
    if (fabsf(ic) > peak) {
        peak = fabsf(ic);
    }

    m->safety.peak_current_a = peak;
    if (peak > m->safety.max_observed_current_a) {
        m->safety.max_observed_current_a = peak;
    }

    if (peak <= hard) {
        return 1U;
    }

    foc_motor_latch_current_trip(m, 1U);
    return 0U;
}

static uint8_t foc_motor_check_soft_current(foc_motor_t *m)
{
    float sample;

    if (m->state == FOC_STATE_CALIB) {
        /* Calibration deliberately builds a stationary d-axis current.  Its
         * historical 1.5 A limit is a phase-peak limit, so keep that meaning
         * instead of comparing the larger dq-vector magnitude. */
        sample = m->safety.peak_current_a;
        m->safety.soft_current_a = sample;
    } else {
        /* Filter signed dq components before taking the magnitude.  Taking
         * abs/magnitude first rectifies zero-mean PWM ripple into a false DC
         * current and caused repeatable soft trips near 2350 RPM. */
        float mag_sq = (m->i_dq_filt.d * m->i_dq_filt.d) +
                       (m->i_dq_filt.q * m->i_dq_filt.q);
        float limit = m->safety.current_limit_a;
        float limit_sq = limit * limit;

        if (mag_sq > limit_sq) {
            sample = sqrtf(mag_sq);
            m->safety.soft_current_a = sample;
            if (m->safety.consecutive_over_limit < 0xFFFFU) {
                ++m->safety.consecutive_over_limit;
            }
        } else {
            /* 绝大多数正常工况下平方未超限，彻底规避 sqrtf 开方库调用与分支 */
            m->safety.soft_current_a = 0.0f;
            m->safety.consecutive_over_limit = 0U;
        }
    }

    /* 角度源切换后 1s 豁免窗：16° 角度跳变的暂态电流天然超软限，
     * 与预触发停机共用 obs_switch_ms 起点（硬过流 12A 仍生效）。
     * obs_switch_ms=0（未切换过）时 cycles-0 上电 1s 后恒大于窗口，
     * 自动失去豁免。 */
    if ((m->obs_switch_ms != 0U) &&
        ((foc_port_cycles() - m->obs_switch_ms) <= 170000000U)) {
        m->safety.consecutive_over_limit = 0U;
        return 1U;
    }

    if (m->safety.consecutive_over_limit < FOC_OVERCURRENT_TRIP_SAMPLES) {
        return 1U;
    }

    foc_motor_latch_current_trip(m, 0U);
    return 0U;
}

/* Iq command limit and measured phase-current trip threshold need margin. */
static float foc_motor_run_trip_limit(float command_limit_a, float hard_limit_a)
{
    return foc_clampf((command_limit_a * FOC_RUN_TRIP_MARGIN_SCALE) +
                      FOC_RUN_TRIP_MARGIN_A,
                      command_limit_a, hard_limit_a);
}

static void foc_velocity_start_reset(foc_motor_t *m)
{
    m->vel_start_boost_a = 0.0f;
    m->vel_start_sign = 0;
    m->vel_start_active = 0U;
    m->vel_start_release_cnt = 0U;
}

/**
 * 慢环（默认 1 kHz）：控制速度滤波 → 速度 PI / 位置 PI。
 * 输出写入 m->iq_ref，由快环的电流环去执行。
 *
 * 方向约定：用户帧 = 编码器帧（速度/位置反馈的正方向）。
 * θe = dir·pp·θm + offset 意味着正 iq 产生的转矩沿"电角度增大"方向，
 * 即编码器帧里的 dir 方向。所以外环输出跨进电角度帧时必须乘 dir，
 * 否则 dir=-1（本板实测值）时速度/位置环是正反馈，直接飞车。
 */
static void foc_motor_slow_loop(foc_motor_t *m)
{
    const float dt = m->dt_fast * (float)m->slow_div;
    const float dir = ((m->angle_source == FOC_ANGLE_ENCODER_CALIBRATED) &&
                      (m->calib.valid != 0U) &&
                      (g_angle_mgr.mode != FOC_FEEDBACK_SENSORLESS_PRIMARY))
                          ? (float)m->calib.direction : 1.0f;
    float velocity_input = m->velocity_observer_rpm;
    float velocity_fast;
    float velocity_low;
    float velocity_blend;

    /* 若角度管理器已处于过渡、接管、纯无感主控或编码器疑似/确认故障状态，
     * 速度环输入无缝切换至仲裁平滑速度，彻底避免编码器故障将速度拉到 0 导致电流冲击或堵转保护误判 */
    uint8_t cur_spd_is_obs = 0U;
    if ((g_angle_mgr.handover_blend > 0.001f) ||
        (g_angle_mgr.enc_health != ENCODER_HEALTH_NORMAL) ||
        (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY) ||
        (g_angle_mgr.state >= FOC_ANGLE_BLEND_TO_SENSORLESS)) {
        velocity_input = g_angle_mgr.speed_control;
        cur_spd_is_obs = 1U;
    }

    /* 当处于纯无感主控模式且未进入稳定的闭环运行态 (RUN) 之前:
     * 电流给定完全由角度管理器快环精准生成（包含对齐斜坡与平滑衰减），慢环速度 PI 不写入覆盖 */
    if ((g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY) &&
        (g_angle_mgr.state < FOC_ANGLE_SENSORLESS_RUN)) {
        m->vel_ref_rpm = g_angle_mgr.open_speed_rpm;
        velocity_input = g_angle_mgr.speed_control;
        m->velocity_filt_rpm = velocity_input;
        foc_speed_filter_reset(&m->vel_filter, velocity_input);
        foc_speed_filter_reset(&m->vel_filter_low, velocity_input);
        m->vel_track_pos_rad = m->position_rad;
        return;
    }

    /* 速度源切换检测：当在编码器速度与无感估计速度之间切换时，执行无扰平滑切换 (Bumpless Transfer)，
     * 重置滤波器并对速度环 PI 积分项做前向平衡补偿，消除首拍输入阶跃引起的电流冲击！
     * 注意：纯无感主控模式在 BLEND 结束切入 RUN 时已由角度管理器完成无扰重平衡，此处不重复触发 */
    if ((cur_spd_is_obs != m->last_spd_is_obs) &&
        (g_angle_mgr.mode != FOC_FEEDBACK_SENSORLESS_PRIMARY)) {
        m->last_spd_is_obs = cur_spd_is_obs;
        foc_speed_filter_reset(&m->vel_filter, velocity_input);
        foc_speed_filter_reset(&m->vel_filter_low, velocity_input);
        /* 对速度 PI 积分项无扰重平衡:保持切换前后瞬时比例+积分总输出平滑一致 */
        if (m->mode == FOC_MODE_VELOCITY) {
            float err_new = m->vel_ref_rpm - velocity_input;
            float p_new = m->pid_vel.kp * err_new;
            float i_target = m->pid_vel.prev_output - p_new;
            if (m->pid_vel.out_limit > 0.0f) {
                i_target = foc_clampf(i_target, -m->pid_vel.out_limit, m->pid_vel.out_limit);
            }
            m->pid_vel.integral = i_target;
            m->pid_vel.prev_error = err_new;
        }
    } else {
        m->last_spd_is_obs = cur_spd_is_obs;
    }

    /* 只有零速命令且长窗测速也确认静止时才把观察器残余抖动归零。
     * 运动中不能用诊断速度的瞬时 0 去门控 PLL，否则齿槽停顿会在控制
     * 反馈里制造 0 RPM 台阶，反而触发更大的补偿转矩。 */
    if ((fabsf(m->vel_ref_rpm) < 0.001f) &&
        (m->velocity_rpm == 0.0f)) {
        velocity_input = 0.0f;
    }
    velocity_fast = foc_speed_filter_update(&m->vel_filter, velocity_input);
    velocity_low = foc_speed_filter_update(
        &m->vel_filter_low, velocity_input);
    velocity_blend = foc_clampf(
        (fabsf(m->vel_ref_rpm) - FOC_VEL_FILTER_BLEND_START_RPM) /
        (FOC_VEL_FILTER_BLEND_END_RPM -
         FOC_VEL_FILTER_BLEND_START_RPM),
        0.0f, 1.0f);
    m->velocity_filt_rpm = velocity_low +
        velocity_blend * (velocity_fast - velocity_low);

    switch (m->mode) {
    case FOC_MODE_VELOCITY: {
        /* 目标速度斜坡：限制加速度，避免阶跃目标产生电流冲击 */
        float tgt = m->target;
        float track_error = 0.0f;
        float track_blend = 0.0f;
        if (m->cfg.vel_ramp_rpm_s > 0.0f) {
            float step = m->cfg.vel_ramp_rpm_s * dt;
            m->vel_ref_rpm = foc_clampf(tgt,
                                        m->vel_ref_rpm - step,
                                        m->vel_ref_rpm + step);
        } else {
            m->vel_ref_rpm = tgt;
        }
        m->vel_ref_rpm = foc_clampf(m->vel_ref_rpm,
                                    -m->params.max_rpm, m->params.max_rpm);

        /*
         * 低速位置轨迹：用斜坡后的速度积分出连续位置参考。相较只看瞬时
         * 速度，位置误差会在进入下一个齿槽前就建立转矩，因此不会等到
         * 转速掉为零后才补电流。轨迹滞后被钳位，避免堵转时无限积累。
         * 注意：纯无感主控模式严禁使用编码器 position_rad，track 必须强制为 0！
         */
        if ((fabsf(tgt) < 0.001f) || (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY)) {
            m->vel_track_pos_rad = m->position_rad;
        } else {
            m->vel_track_pos_rad +=
                m->vel_ref_rpm * FOC_RPM_TO_RADS * dt;
            track_error = foc_clampf(
                m->vel_track_pos_rad - m->position_rad,
                -m->cfg.vel_track_limit_rad,
                m->cfg.vel_track_limit_rad);
            m->vel_track_pos_rad = m->position_rad + track_error;

            /*
             * With tracking enabled, use the moving-position controller at
             * very low speed and cross-fade to the speed PI by 2x the
             * threshold.  Gating on Kp keeps `vel track 0` as an exact
             * fallback to the original pure speed-loop behaviour.
             */
            if (m->cfg.vel_track_kp > 0.0f) {
                /* Keep tracking active while the rotor is actually slow.
                 * Using vel_ref here released the breakaway controller as
                 * soon as the command ramp completed, even when the rotor
                 * was still stalled. */
                track_blend = foc_clampf(
                    ((2.0f * m->cfg.vel_track_rpm) -
                     fabsf(m->velocity_filt_rpm)) /
                        m->cfg.vel_track_rpm,
                    0.0f, 1.0f);
            }
        }

        /*
         * 弱磁控制与动态电流圆限制（与 ST MCSDK FW_CalcCurrRef 架构一致）：
         * 1. 监测当前端电压利用率模长 |Vdq|；
         * 2. 当电压逼近或超过设定利用率（如 88% Udc/sqrt(3)）且转速高于 enter_rpm 时，
         *    弱磁积分器输出负向去磁电流 id_ref；
         * 3. 动态计算矢量圆剩余可用 Iq 限幅: Iq_sat = sqrt(I_max^2 - Id^2)；
         * 4. 联动速度 PI 输出限幅与积分限幅，彻底消除电压饱和时的积分发散（Anti-windup）。
         */
        float id_ref_cmd = 0.0f;
        float max_iq_sat = m->params.max_current_a;

        if ((m->drv != 0) &&
            (m->runtime.fieldweak_enable != 0U) &&
            (fabsf(m->velocity_filt_rpm) > m->runtime.fieldweak_enter_rpm)) {
            float v_max = m->drv->u_dc * INV_SQRT_3;
            float v_target = m->runtime.fieldweak_voltage_ratio * v_max;
            float v_mag = sqrtf((m->v_dq.d * m->v_dq.d) + (m->v_dq.q * m->v_dq.q));
            float v_err = v_target - v_mag; /* 当端电压超限时 v_err < 0 */
            float id_min = m->runtime.fieldweak_id_min_a; /* 负数 */

            m->fw_integral += m->runtime.fieldweak_gain * v_err * dt;
            m->fw_integral = foc_clampf(m->fw_integral, id_min, 0.0f);
            id_ref_cmd = m->fw_integral;

            /* 矢量圆限幅：仅当实际产生弱磁负 Id 时才需裁剪可用 Iq，否则保持标称最大电流 */
            if (id_ref_cmd < -0.01f) {
                float iq_sat_sq = (m->params.max_current_a * m->params.max_current_a) -
                                  (id_ref_cmd * id_ref_cmd);
                max_iq_sat = (iq_sat_sq > 0.0f) ? sqrtf(iq_sat_sq) : 0.05f;
            } else {
                max_iq_sat = m->params.max_current_a;
            }
        } else {
            m->fw_integral = 0.0f;
            id_ref_cmd = 0.0f;
            max_iq_sat = m->params.max_current_a;
        }

        m->id_ref = id_ref_cmd;
        if (max_iq_sat != m->pid_vel.out_limit) {
            foc_pid_set_limit(&m->pid_vel, max_iq_sat);
        }

        if ((fabsf(tgt) < 0.001f) &&
            (fabsf(m->vel_ref_rpm) < 0.001f) &&
            (fabsf(m->velocity_filt_rpm) < FOC_VEL_ZERO_RESET_RPM)) {
            /*
             * 零速命令且转子已基本停止时清除历史积分。否则静摩擦会把残留
             * 积分“冻住”，表现为 target=0 后电机仍持续带电顶住转子。
             */
            foc_pid_reset(&m->pid_vel);
            m->iq_ref = 0.0f;
        } else {
            float iq_speed;
            float iq_track_damp;
            float iq_user;

            if (track_blend >= 0.999f) {
                /* Do not leave a hidden speed integrator charged while the
                 * moving-position controller has full authority. */
                foc_pid_reset(&m->pid_vel);
                iq_speed = 0.0f;
            } else {
                iq_speed = foc_pid_update(
                    &m->pid_vel,
                    m->vel_ref_rpm - m->velocity_filt_rpm,
                    dt);
            }
            /*
             * The moving-position loop needs velocity damping.  Reuse the
             * configured speed Kp as the D term, but never its integrator in
             * the full low-speed tracking region.  This is a conventional
             * position-P / velocity-PD servo and avoids the undamped spring
             * oscillation seen with position tracking alone.
             */
            iq_track_damp = track_blend * m->pid_vel.kp *
                (m->vel_ref_rpm - m->velocity_filt_rpm);
            iq_user = (1.0f - track_blend) * iq_speed;
            iq_user += iq_track_damp;

            /*
             * 低速摩擦前馈：
             * 静止时使用 vel_start_a 克服齿槽/静摩擦；转速上升后在
             * vel_start_rpm 内平滑过渡到 vel_friction_a，避免普通 PI
             * 先积累很大电流、脱离齿槽后再突然冲转。
             *
             * target=0 时立即撤掉前馈，让 PI 负责制动；转子停止后上面的
             * 零速分支会清空积分并把 Iq 置零。
             * 注意：纯无感主控模式在开环拖动到 500 RPM 切入闭环时已处于高速运动区，
             * 严禁触发起步 boost 前馈，避免无感闭环瞬间被大前馈电流冲击冲垮！
             */
            if ((fabsf(tgt) >= 0.001f) &&
                (g_angle_mgr.mode != FOC_FEEDBACK_SENSORLESS_PRIMARY)) {
                int8_t cmd_sign = (tgt > 0.0f) ? 1 : -1;
                float boost_max = m->cfg.vel_start_a -
                                  m->cfg.vel_friction_a;
                float ff_mag;

                if (boost_max < 0.0f) {
                    boost_max = 0.0f;
                }
                /* A new start or reversal arms one breakaway pulse.  It is
                 * released only after motion is observed in the requested
                 * direction, then decays monotonically.  Therefore encoder
                 * ripple cannot repeatedly re-trigger the large start
                 * current and create the previous 300-RPM limit cycle. */
                if (m->vel_start_sign != cmd_sign) {
                    m->vel_start_sign = cmd_sign;
                    m->vel_start_active = 1U;
                    m->vel_start_boost_a = 0.0f;
                    m->vel_start_release_cnt = 0U;
                }
                if (m->vel_start_active != 0U) {
                    m->vel_start_boost_a +=
                        FOC_VEL_START_BOOST_RAMP_A_S * dt;
                    if (m->vel_start_boost_a > boost_max) {
                        m->vel_start_boost_a = boost_max;
                    }
                    if (((float)cmd_sign * m->velocity_filt_rpm) >=
                        m->cfg.vel_start_rpm) {
                        if (m->vel_start_release_cnt <
                            FOC_VEL_START_RELEASE_TICKS) {
                            ++m->vel_start_release_cnt;
                        }
                        if (m->vel_start_release_cnt >=
                            FOC_VEL_START_RELEASE_TICKS) {
                            m->vel_start_active = 0U;
                        }
                    } else {
                        m->vel_start_release_cnt = 0U;
                    }
                } else {
                    m->vel_start_boost_a -=
                        FOC_VEL_START_BOOST_DECAY_A_S * dt;
                    if (m->vel_start_boost_a < 0.0f) {
                        m->vel_start_boost_a = 0.0f;
                    }
                }
                ff_mag = m->cfg.vel_friction_a +
                         m->vel_start_boost_a;
                ff_mag = foc_clampf(ff_mag, 0.0f,
                                    m->params.max_current_a);
                iq_user += (cmd_sign > 0) ? ff_mag : -ff_mag;
            } else {
                foc_velocity_start_reset(m);
            }
            iq_user += m->cfg.vel_track_kp * track_error * track_blend;
            m->iq_ref = dir * iq_user;
        }
        break;
    }

    case FOC_MODE_POSITION: {
        /* target 语义 = 相对使能原点的偏移 rad；换算成多圈绝对位置参考。 */
        float pos_ref = m->pos_origin_rad + m->target;
        float vel_ff_rpm = 0.0f;
        float pos_error;
        float iq_user;
        float iq_damp;
        float iq_damp_limit;

        /* 梯形轨迹：目标变化时从当前状态重规划，之后每拍输出
         * 平滑的位置参考 + 速度前馈（ODrive trap_traj 方案） */
        if (m->target != m->traj_target_latch) {
            float pos_goal = m->pos_origin_rad + m->target;

            /* 新位置目标不继承上一个目标的保持转矩积分。 */
            foc_pid_reset(&m->pid_pos);
            foc_traj_plan(
                &m->traj, pos_goal,
                m->position_rad,
                m->velocity_filt_rpm * FOC_RPM_TO_RADS,
                m->cfg.pos_vel_limit_rpm * FOC_RPM_TO_RADS,
                m->cfg.traj_accel_rpm_s * FOC_RPM_TO_RADS,
                m->cfg.traj_accel_rpm_s * FOC_RPM_TO_RADS);
            m->traj_target_latch = m->target;
        }

        if (m->traj.active != 0U) {
            float pos_r;
            float vel_ff_rads;
            (void)foc_traj_eval(&m->traj, dt, &pos_r, &vel_ff_rads);
            pos_ref = pos_r;
            vel_ff_rpm = vel_ff_rads * FOC_RADS_TO_RPM;
        }

        /*
         * 位置 PI 直接输出用户坐标系 Iq；轨迹速度与实际速度之差
         * 通过 pos_vel_kp 形成前馈/阻尼。相比“位置 P -> 速度 PI”，
         * 位置积分直接补偿静摩擦，不会把制动阶段的速度积分带到
         * 下一次脱槽；相比固定 0.38 A 前馈，也不会形成继电极限环。
        */
        pos_error = pos_ref - m->position_rad;
        /*
         * 低限流、小惯量外转子存在明显齿槽效应。位置误差越过零点时，
         * 上一方向积累的保持电流会继续推动转子并形成低频极限环；
         * 清除该积分后由速度阻尼接管制动，再从新方向平滑建立保持力。
         */
        if ((m->pid_pos.prev_error * pos_error) < 0.0f) {
            foc_pid_reset(&m->pid_pos);
        }
        iq_user = foc_pid_update(&m->pid_pos, pos_error, dt);
        iq_damp_limit =
            FOC_POS_DAMP_CURRENT_RATIO * m->params.max_current_a;
        iq_damp = m->cfg.pos_vel_kp *
                  (vel_ff_rpm - m->velocity_observer_rpm);
        iq_user += foc_clampf(iq_damp, -iq_damp_limit, iq_damp_limit);
        m->vel_ref_rpm = vel_ff_rpm; /* status 中显示轨迹速度前馈 */
        m->iq_ref = dir * foc_clampf(
            iq_user, -m->params.max_current_a, m->params.max_current_a);
        break;
    }

    case FOC_MODE_TORQUE:
        /* 用户给的力矩正方向 = 编码器正方向 */
        m->iq_ref = dir * m->target;
        break;

    case FOC_MODE_OPENLOOP_VF:
    default:
        break;
    }

    /* 抗齿槽标定采样（在注入前采样原始速度环/位置环 iq_ref，解耦方向） */
    if ((m->anticog_sample_hook != 0) && (m->state == FOC_STATE_RUN)) {
        m->anticog_sample_hook(m->theta_mech, dir * m->iq_ref);
    }

    /* 抗齿槽力矩前馈补偿（在限幅前注入）：
     * 必须严格限定在 RUN 状态且非开环 V/F 模式，杜绝在 IDLE/CALIB/VF 状态下
     * 因前馈常量逐拍累加导致 iq_ref 饱和冲至 -5.2A 限幅（2026-09-06 评估定位）。 */
    if ((m->anticog_hook != 0) && (m->state == FOC_STATE_RUN) &&
        (m->mode != FOC_MODE_OPENLOOP_VF)) {
        m->iq_ref += dir * m->anticog_hook(m->theta_mech);
    }

    m->iq_ref = foc_clampf(m->iq_ref,
                           -m->params.max_current_a, m->params.max_current_a);

    /* 堵转保护（VESC 思路）：速度/位置模式下电流给定顶到限幅、
     * 转子却几乎不动，持续超时说明卡死/负载超能力/编码器失效——
     * 继续满电流灌注只会烧电机，跳闸。力矩模式的堵转是正常工况不查。 */
    if ((m->cfg.stall_enable != 0U) &&
        ((m->mode == FOC_MODE_VELOCITY) || (m->mode == FOC_MODE_POSITION)) &&
        (m->state == FOC_STATE_RUN)) {
        float spd_check = ((g_angle_mgr.handover_blend > 0.001f) ||
                           (g_angle_mgr.enc_health != ENCODER_HEALTH_NORMAL))
                          ? fabsf(velocity_input)
                          : fabsf(m->velocity_filt_rpm);
        if ((fabsf(m->iq_ref) >= (0.95f * m->params.max_current_a)) &&
            (spd_check < m->cfg.stall_rpm)) {
            if (m->stall_cnt < 0xFFFFU) {
                ++m->stall_cnt;
            }
            if (m->stall_cnt >= m->stall_trip_ticks) {
                foc_motor_fault(m, FOC_FAULT_STALL);
            }
        } else {
            m->stall_cnt = 0U;
        }
    }
}

/* ======================== 生命周期 ======================== */

void foc_motor_init(foc_motor_t *m,
                    const foc_driver_if_t *drv,
                    const foc_current_if_t *cur,
                    const foc_sensor_if_t *sensor,
                    const foc_motor_params_t *params,
                    const foc_ctrl_cfg_t *cfg,
                    float dt_fast,
                    uint16_t slow_div)
{
    float v_max;
    float kp_i;
    float ki_i;

    m->drv = drv;
    m->cur = cur;
    m->sensor = sensor;
    m->params = *params;
    m->cfg = *cfg;
    m->runtime.angle_delay_cycles = FOC_M0_ANGLE_DELAY_CYCLES;
    m->runtime.fieldweak_enable = FOC_M0_FIELDWEAK_ENABLE;
    m->runtime.fieldweak_enter_rpm = FOC_M0_FIELDWEAK_ENTER_RPM;
    m->runtime.fieldweak_voltage_ratio = FOC_M0_FIELDWEAK_VOLTAGE_RATIO;
    m->runtime.fieldweak_gain = FOC_M0_FIELDWEAK_GAIN;
    m->runtime.fieldweak_id_min_a =
        -m->params.max_current_a * FOC_M0_FIELDWEAK_ID_MIN_RATIO;
    m->dt_fast = dt_fast;
    m->slow_div = (slow_div == 0U) ? 1U : slow_div;

    /* Flash may contain an older, overly slow filter setting.  Normalize it
     * before constructing the feedback filter so boot and CLI behave alike. */
    if ((m->cfg.vel_lpf_tf > 0.0f) &&
        ((1.0f / (_2PI * m->cfg.vel_lpf_tf)) < FOC_VEL_FILTER_MIN_HZ)) {
        m->cfg.vel_lpf_tf = 1.0f / (_2PI * FOC_VEL_FILTER_MIN_HZ);
    }

    m->state = FOC_STATE_IDLE;
    m->mode = FOC_MODE_OPENLOOP_VF;
    m->angle_source = FOC_ANGLE_OPEN_LOOP;
    m->pwm_hold = 0U;

    m->calib.valid = 0U;
    m->calib.from_store = 0U;
    m->calib.direction = 1;
    m->calib.electrical_offset_rad = 0.0f;

    m->target = 0.0f;
    m->v_openloop.d = 0.0f;
    m->v_openloop.q = 0.0f;
    m->vel_ref_rpm = 0.0f;
    m->vel_track_pos_rad = 0.0f;
    foc_velocity_start_reset(m);
    m->id_ref = 0.0f;
    m->iq_ref = 0.0f;

    m->theta_e = 0.0f;
    m->theta_mech = 0.0f;
    m->position_rad = 0.0f;
    m->velocity_rpm = 0.0f;
    m->velocity_observer_rpm = 0.0f;
    m->velocity_filt_rpm = 0.0f;
    m->i_abc.a = m->i_abc.b = m->i_abc.c = 0.0f;
    m->i_dq.d = m->i_dq.q = 0.0f;
    m->i_dq_filt.d = m->i_dq_filt.q = 0.0f;
    m->v_dq.d = m->v_dq.q = 0.0f;
    m->svm.duty_a = m->svm.duty_b = m->svm.duty_c = 0.5f;
    m->svm.sector = 0U;
    m->ol_angle_step = 0.0f;
    m->slow_cnt = m->slow_div / 2U;   /* 慢环与速度估计拍错开半个周期，消除同拍峰值叠加 */
    m->last_spd_is_obs = 0U;
    m->traj.active = 0U;
    m->traj.xf = 0.0f;
    m->traj_target_latch = 0.0f;
    m->pos_origin_rad = 0.0f;
    m->test_hook = 0;
    m->anticog_hook = 0;
    m->anticog_sample_hook = 0;
    m->stall_cnt = 0U;
    /* 慢环节拍 = dt_fast·slow_div，把超时 ms 换算成慢环拍数 */
    m->stall_trip_ticks = (uint16_t)(((float)cfg->stall_timeout_ms * 1e-3f) /
                                     (dt_fast * (float)m->slow_div));

    /* 安全限制默认取电机参数 */
    m->safety.peak_current_a = 0.0f;
    m->safety.max_observed_current_a = 0.0f;
    m->safety.soft_current_a = 0.0f;
    m->safety.current_limit_a =
        foc_motor_run_trip_limit(params->max_current_a,
                                 params->hard_current_a);
    m->safety.hard_current_limit_a = params->hard_current_a;
    m->safety.trip_current_u_a = 0.0f;
    m->safety.trip_current_v_a = 0.0f;
    m->safety.trip_current_w_a = 0.0f;
    m->safety.trip_soft_current_a = 0.0f;
    m->safety.trip_was_hard = 0U;
    m->safety.consecutive_over_limit = 0U;
    m->safety.fault_code = (uint8_t)FOC_FAULT_NONE;

    /* ---- 电流环带宽整定：Kp = Ls·ω, Ki = Rs·ω ---- */
    v_max = drv->u_dc * INV_SQRT_3;
    kp_i = params->ls_henry * cfg->current_bw_rads;
    ki_i = params->rs_ohm * cfg->current_bw_rads;
    foc_pid_init(&m->pid_id, kp_i, ki_i, 0.0f, v_max, 0.0f);
    foc_pid_init(&m->pid_iq, kp_i, ki_i, 0.0f, v_max, 0.0f);

    /* 速度环：以最大电流参数为初始输出限幅与斜坡 */
    foc_pid_init(&m->pid_vel, cfg->vel_kp, cfg->vel_ki, 0.0f,
                 params->max_current_a, FOC_VEL_IQ_RAMP_A_S);

    /* 位置 PI 直接输出 Iq；速度阻尼在慢环中单独叠加。 */
    foc_pid_init(&m->pid_pos, cfg->pos_kp, cfg->pos_ki, 0.0f,
                 params->max_current_a, 0.0f);

    foc_speed_filter_init(&m->vel_filter, cfg->vel_lpf_tf,
                          1.0f / (dt_fast * (float)slow_div));
    foc_speed_filter_init(
        &m->vel_filter_low,
        1.0f / (_2PI * FOC_VEL_LOW_FILTER_HZ),
        1.0f / (dt_fast * (float)slow_div));
    foc_lpf_init(&m->lpf_id, FOC_IDQ_TELEM_LPF_TF);
    foc_lpf_init(&m->lpf_iq, FOC_IDQ_TELEM_LPF_TF);
    /* 电流环反馈 2.5kHz 陷波：该频率为开关纹波与谷底采样的混叠假影，
     * 稳态幅值 |i_dq|≈1.4A，经电流环/解耦项放大后在 2250+ RPM 失稳。
     * 混叠频率随转速缓变（2507→2551Hz @2200→2350rpm），带宽 600Hz 覆盖。 */
    /* 2026-09-03 实测：陷波堵住 PID+解耦两条路径后爆发反而提前
     * （2300 档即爆），说明 2.5kHz 分量并非可切除的寄生种子，
     * 而是纹波-磁饱和交互的内在动态。陷波器保留但默认直通。 */
    foc_notch_init(&m->notch_id, 2500.0f, 600.0f,
                   1.0f / (dt_fast * (float)slow_div));
    foc_notch_init(&m->notch_iq, 2500.0f, 600.0f,
                   1.0f / (dt_fast * (float)slow_div));
    m->notch_id.enabled = 0U;   /* 直通（发现见上） */
    m->notch_iq.enabled = 0U;

    /* 无感磁链观测器（在线对比模式）：参数用实测值——
     * Rs=0.395(辨识), Ls=301µH(方波注入辨识的视在电感——观测器
     * 磁链提取 x−L·i 用的就是视在电感；此前用 80µH 纹波增量电感，
     * iq 1.5A 时 ΔL·i≈0.46V 直接淹没 0.28V 反电动势 → 角度失锁、
     * 无感运行持续 2A 失步对抗（2026-09-04 黑匣子 col1=iw 定论）),
     * λ=1.18mWb(2200rpm 稳态 vq/we 反推)。gamma 用 VESC 推荐公式。
     * PLL 400/6000 带宽 64Hz < 2000rpm 电频率 200Hz → 锁不住、
     * pll_theta 永久滞后/打摆（try0 差 2.9rad 实证）。提到
     * 1500/10000（约 240Hz）才能在 2000rpm 上锁定。 */
    #define DIAG_OBS_FLUX_WB 1.18e-3f
    foc_observer_init(&m->observer,
                      m->params.rs_ohm, m->params.ls_henry, DIAG_OBS_FLUX_WB,
                      1500.0f, 10000.0f,
                      100.0f / (DIAG_OBS_FLUX_WB * DIAG_OBS_FLUX_WB));
    m->v_ab_last.alpha = 0.0f;
    m->v_ab_last.beta = 0.0f;
    m->obs_enabled = 0U;
    m->observer.theta_offset_rad = 0.0f;
}

void foc_motor_set_velocity_filter_tf(foc_motor_t *m, float tf)
{
    float sample_hz;
    float cutoff_hz;

    if (tf < 0.0f) {
        tf = 0.0f;
    }
    /* Keep tf=0 as an explicit diagnostic bypass, but do not allow a
     * positive setting slower than the speed-loop stability margin. */
    cutoff_hz = (tf > 0.0f) ? (1.0f / (_2PI * tf)) : 0.0f;
    if ((cutoff_hz > 0.0f) && (cutoff_hz < FOC_VEL_FILTER_MIN_HZ)) {
        tf = 1.0f / (_2PI * FOC_VEL_FILTER_MIN_HZ);
    }
    sample_hz = 1.0f / (m->dt_fast * (float)m->slow_div);
    m->cfg.vel_lpf_tf = tf;
    foc_speed_filter_init(&m->vel_filter, tf, sample_hz);
    foc_speed_filter_reset(&m->vel_filter, m->velocity_observer_rpm);
    foc_speed_filter_reset(&m->vel_filter_low, m->velocity_observer_rpm);
    m->velocity_filt_rpm = m->velocity_observer_rpm;
}

void foc_motor_set_velocity_filter_hz(foc_motor_t *m, float hz)
{
    float tf = 0.0f;

    if (hz > 0.0f) {
        tf = 1.0f / (_2PI * hz);
    }
    foc_motor_set_velocity_filter_tf(m, tf);
}

float foc_motor_get_velocity_filter_hz(const foc_motor_t *m)
{
    return foc_speed_filter_cutoff_hz(&m->vel_filter);
}

/* ======================== 快环 ======================== */

void foc_motor_fast_loop(foc_motor_t *m)
{
    foc_state_t st;
    float sin_th;
    float cos_th;
    ab_t i_ab;
    ab_t v_ab;
    uint32_t tp = foc_board_cycles();   /* 分段剖析起点 */

    /* 1. 传感器更新：任何状态都执行，保证角度/速度随时可观测 */
    if (m->sensor != 0) {
        float th;
        float dth;

        if (m->sensor->update == abz_encoder_update) {
            /* 针对 ABZ 增量编码器直接函数调用，消除 4 次虚表函数指针流水线冲刷 */
            abz_encoder_update();
            th = abz_encoder_angle_rad();
            dth = th - m->theta_mech;
            if (dth >= _PI) {
                dth -= _2PI;
            } else if (dth < -_PI) {
                dth += _2PI;
            }
            m->position_rad += dth;
            m->theta_mech = th;
            m->velocity_rpm = abz_encoder_velocity_rpm();
            m->velocity_observer_rpm = abz_encoder_pll_velocity_rpm();
        } else {
            m->sensor->update();
            th = m->sensor->angle_rad();
            dth = th - m->theta_mech;
            if (dth >= _PI) {
                dth -= _2PI;
            } else if (dth < -_PI) {
                dth += _2PI;
            }
            m->position_rad += dth;
            m->theta_mech = th;
            m->velocity_rpm = m->sensor->velocity_rpm();
            m->velocity_observer_rpm =
                (m->sensor->velocity_control_rpm != 0)
                    ? m->sensor->velocity_control_rpm()
                    : m->velocity_rpm;
        }
    }

    st = m->state;
    if ((st != FOC_STATE_RUN) && (st != FOC_STATE_CALIB)) {
        m->safety.consecutive_over_limit = 0U;
        if (st == FOC_STATE_IDLE) {
            m->safety.peak_current_a = 0.0f;
            m->safety.soft_current_a = 0.0f;
        }
        /* 停机不运行速度/位置控制，直接显示低延迟观察器速度。重新 arm
         * 时以当时速度复位中值和二阶状态，不会继承停机前的滤波历史。 */
        m->velocity_filt_rpm = m->velocity_observer_rpm;
        return;
    }

    /* 2. 读取电流并做过流保护（先保护后控制） */
    if (m->cur != 0) {
        m->cur->get(&m->i_abc.a, &m->i_abc.b, &m->i_abc.c);
        if (s_blackbox_frozen == 0U) {
            foc_motor_blackbox_record(m);
        }
        if (foc_motor_check_hard_current(m) == 0U) {
            return;
        }
        /* （已移除 iq>0.8A×32 拍诊断陷阱：起步段 vel_start_boost 0.7A +
         * 摩擦前馈 + 加速转矩本来就超 0.8A 持续 2ms 以上，2026-09-04
         * 在 240rpm 起步段误触发假 CURRENT_SENSE 停机。黑匣子冻结
         * 由 foc_motor_fault() 内部机制负责，无需此处抢跑。） */
    }
    foc_board_cpu_sect(0U, tp);   /* 0 = 传感器读取 + 电流读取与黑匣子 */
    tp = foc_board_cycles();

    /* 3. 电角度选择
     *    CALIB 状态与开环 V/f 模式一律用开环角度；
     *    纯无感主控模式或闭环控制下统一交由角度仲裁管理器 foc_angle_mgr_update(m) 决定控制角度 */
    if ((st == FOC_STATE_CALIB) || (m->mode == FOC_MODE_OPENLOOP_VF)) {
        m->theta_e = foc_wrap_0_2pi(m->theta_e + m->ol_angle_step);
    } else if (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY) {
        m->theta_e = foc_angle_mgr_update(m);
    } else if ((m->angle_source == FOC_ANGLE_OPEN_LOOP) || (m->calib.valid == 0U)) {
        m->theta_e = foc_wrap_0_2pi(m->theta_e + m->ol_angle_step);
    } else {
        m->theta_e = foc_angle_mgr_update(m);
    }

    foc_sincos(m->theta_e, &sin_th, &cos_th);
    foc_board_cpu_sect(6U, tp);   /* 6 = 角度仲裁 + sin/cos */
    tp = foc_board_cycles();

    /* 4. Clarke + Park：三相电流 → dq 电流 */
    foc_clarke(&m->i_abc, &i_ab);
    foc_park(&i_ab, sin_th, cos_th, &m->i_dq);

    /* 无感观测器逐拍更新（角度必须以快环频率更新，否则切到无感后
     * 角度冻结、电机失步）。16k/62.5µs 周期内约 5µs，余量充足；
     * 32k 配置时改用隔拍执行。渐变期间（含反向）也必须保持更新，
     * 否则观测角冻结、wrap 后乱跳。 */
    if ((m->obs_enabled != 0U) || (m->obs_blending != 0U)) {
        (void)foc_observer_update(&m->observer, &m->v_ab_last, &i_ab,
                                  m->dt_fast);
    }
    m->i_dq_filt.d = foc_lpf_update(&m->lpf_id, m->i_dq.d, m->dt_fast);
    m->i_dq_filt.q = foc_lpf_update(&m->lpf_iq, m->i_dq.q, m->dt_fast);

    if (foc_motor_check_soft_current(m) == 0U) {
        return;
    }

    /* Bootstrap charging still runs both protection paths, but must not let
     * the control loops overwrite the held PWM pattern. */
    if (m->pwm_hold != 0U) {
        return;
    }

    /* 5. 慢环分频：速度/位置环（与编码器长窗速度估计 0 拍严格锁相在 8 拍，绝对永不同拍重叠） */
    foc_board_cpu_sect(1U, tp);   /* 1 = Clarke/Park + 观测器 + 电流滤波/软限流 */
    tp = foc_board_cycles();
    if (abz_encoder_get_sample_div() == (ABZ_VELOCITY_SAMPLE_DIV / 2U)) {
        foc_motor_slow_loop(m);
        foc_board_cpu_sect(4U, tp);   /* 4 = 慢环（速度/位置） */
        tp = foc_board_cycles();
    }

    /* 6. 电压命令 */
    if ((st == FOC_STATE_CALIB) || (m->mode == FOC_MODE_OPENLOOP_VF)) {
        /* 测试信号注入（参数辨识等）：允许测试例程按拍改写 v_openloop */
        if ((st == FOC_STATE_CALIB) && (m->test_hook != 0)) {
            m->test_hook(m);
        }
        /* 开环/校准：直接使用外部给的 vd/vq */
        m->v_dq = m->v_openloop;
    } else {
        /* 电流闭环：Id 跟踪弱磁给定 id_ref（<=0），Iq 跟随速度/位置环给定。
         * 反馈先过 2.5kHz 陷波（混叠假影剔除），保护/遥测仍用原始值。 */
        float id_fb = foc_notch_update(&m->notch_id, m->i_dq.d);
        float iq_fb = foc_notch_update(&m->notch_iq, m->i_dq.q);

        /* 电流环 PID 输出上限随母线更新（仅在母线变动时重新写入限幅，消除每拍函数调用开销） */
        float v_limit = m->drv->u_dc * INV_SQRT_3;
        if (v_limit != m->pid_id.out_limit) {
            foc_pid_set_limit(&m->pid_id, v_limit);
            foc_pid_set_limit(&m->pid_iq, v_limit);
        }

        float vd = foc_pid_update(&m->pid_id, m->id_ref - id_fb, m->dt_fast);
        float vq = foc_pid_update(&m->pid_iq, m->iq_ref - iq_fb, m->dt_fast);

        /* dq 解耦前馈补偿：
         * 根据表贴式永磁同步电机（SPM, Ld = Lq = Ls）稳态 dq 电压方程：
         *   u_d = R_s * i_d + L_d * (di_d/dt) - ω_e * L_q * i_q
         *   u_q = R_s * i_q + L_q * (di_q/dt) + ω_e * L_d * i_d + ω_e * ψ_f
         * 交叉耦合动态使得 d 轴电压受 q 轴电流牵连，q 轴电压受 d 轴电流牵连。
         * 为了实现 d 轴与 q 轴的完全解耦独立线性控制，在电流环 PI 输出基础上引入前馈补偿项：
         *   v_d_decouple = - ω_e * L_s * i_q   (补偿 d 轴由 q 轴电流耦合产生的电压降)
         *   v_q_decouple = + ω_e * L_s * i_d   (补偿 q 轴由 d 轴弱磁电流耦合产生的电压升)
         * 反电动势项 ω_e * ψ_f 则由速度环前馈或 PI 积分项自适应吸收。
         */
        if (m->cfg.decouple_enable != 0U) {
            float spd_mech;
            float dir_scale;
            if (g_angle_mgr.mode == FOC_FEEDBACK_SENSORED_PRIMARY) {
                spd_mech = m->velocity_observer_rpm;
                dir_scale = (float)m->calib.direction;
            } else {
                spd_mech = ((g_angle_mgr.handover_blend > 0.001f) ||
                            (g_angle_mgr.enc_health != ENCODER_HEALTH_NORMAL) ||
                            (g_angle_mgr.state >= FOC_ANGLE_BLEND_TO_SENSORLESS))
                           ? g_angle_mgr.speed_control
                           : m->velocity_observer_rpm;
                dir_scale = 1.0f;
            }
            float we = dir_scale * spd_mech * FOC_RPM_TO_RADS * (float)m->params.pole_pairs;

            /* 解耦项与 PID 反馈同源：使用陷波后反馈值，精确抵消交叉电抗电压 */
            vd -= we * m->params.ls_henry * iq_fb;
            vq += we * m->params.ls_henry * id_fb;
        }

        m->v_dq.d = vd;
        m->v_dq.q = vq;
    }

    /* NaN 防护 */
    if ((m->v_dq.d != m->v_dq.d) || (m->v_dq.q != m->v_dq.q)) {
        foc_motor_fault(m, FOC_FAULT_CONTROL_NAN);
        return;
    }

    foc_voltage_circle_limit(&m->v_dq, m->drv->u_dc * INV_SQRT_3);

    /* 7. 反 Park + 死区补偿 + SVPWM 输出 */
    foc_inv_park(&m->v_dq, sin_th, cos_th, &v_ab);

    /* 死区补偿（VESC/MESC）：按相电流符号把死区损失的平均电压
     * 前馈补回。±0.05A 死区防电流过零处抖振；共模分量会被 SVM
     * 的中点注入吸收，只有差模起作用 */
    if ((m->cfg.deadtime_comp_v > 0.0f) && (st == FOC_STATE_RUN) &&
        (m->mode != FOC_MODE_OPENLOOP_VF)) {
        const float vc = m->cfg.deadtime_comp_v;
        const float th_i = 0.05f;
        abc_t comp;
        ab_t comp_ab;

        comp.a = (m->i_abc.a > th_i) ? vc
                     : ((m->i_abc.a < -th_i) ? -vc : 0.0f);
        comp.b = (m->i_abc.b > th_i) ? vc
                     : ((m->i_abc.b < -th_i) ? -vc : 0.0f);
        comp.c = (m->i_abc.c > th_i) ? vc
                     : ((m->i_abc.c < -th_i) ? -vc : 0.0f);
        foc_clarke(&comp, &comp_ab);
        v_ab.alpha += comp_ab.alpha;
        v_ab.beta += comp_ab.beta;
    }

    /* 无感影子评测平台更新：
     * 仅在纯无感模式、自动接管模式或显式开启影子评测 (bench/obs) 时更新；
     * 有感主控常态直接跳过，消除函数调用与角度计算开销 */
    foc_board_cpu_sect(2U, tp);   /* 2 = 电流环 PI + 解耦 + 死区补偿（含慢环外的全部） */
    tp = foc_board_cycles();
    if ((g_angle_mgr.mode != FOC_FEEDBACK_SENSORED_PRIMARY) || (g_sensorless_bench.shadow_enabled != 0U)) {
        foc_sensorless_bench_update(m, m->v_ab_last.alpha, m->v_ab_last.beta,
                                    i_ab.alpha, i_ab.beta,
                                    m->theta_e, m->dt_fast);
    }
    foc_board_cpu_sect(3U, tp);   /* 3 = 无感评测平台 */
    tp = foc_board_cycles();

    /* HFI 影子模块高频注入电压安全叠加：
     * 仅在影子 HFI 使能且非 CALIB 状态下，将估计 d 轴的高频方波电压叠加至输出定子电压 v_ab。
     * 注意：此处仅叠加微小探测电压（<=1.5V），绝对不改写 FOC 换相角控制权！ */
    if ((g_sensorless_bench.hfi_enabled != 0U) && (st == FOC_STATE_RUN)) {
        v_ab.alpha += g_sensorless_bench.hfi_v_inj_alpha;
        v_ab.beta  += g_sensorless_bench.hfi_v_inj_beta;
    }

    foc_svm_calc(&v_ab, m->drv->u_dc, &m->svm);
    m->drv->set_compare(
        (uint32_t)(m->svm.duty_a * (float)m->drv->full_count),
        (uint32_t)(m->svm.duty_b * (float)m->drv->full_count),
        (uint32_t)(m->svm.duty_c * (float)m->drv->full_count),
        m->svm.sector);

    /* 根据本拍实际占空比与母线电压重构端电压，供下一拍观测器使用（仅在无感观测器工作时需要） */
    if ((g_angle_mgr.mode != FOC_FEEDBACK_SENSORED_PRIMARY) || (g_sensorless_bench.shadow_enabled != 0U)) {
        float udc_third = m->drv->u_dc * 0.33333333f;
        float da = m->svm.duty_a;
        float db = m->svm.duty_b;
        float dc = m->svm.duty_c;
        m->v_ab_last.alpha = udc_third * ((2.0f * da) - db - dc);
        m->v_ab_last.beta  = m->drv->u_dc * 0.57735027f * (db - dc);
    }
    foc_board_cpu_sect(7U, tp);   /* 7 = SVM + 占空比写入 + 端电压重构 */
}

/* ======================== 命令接口 ======================== */

uint8_t foc_motor_arm(foc_motor_t *m)
{
    if (m->state != FOC_STATE_IDLE) {
        return 0U;
    }

    /* 闭环模式必须先有有效校准（或明确切到开环角度源，或处于纯无感主控模式） */
    if ((m->mode != FOC_MODE_OPENLOOP_VF) &&
        (g_angle_mgr.mode != FOC_FEEDBACK_SENSORLESS_PRIMARY) &&
        ((m->calib.valid == 0U) ||
         (m->angle_source != FOC_ANGLE_ENCODER_CALIBRATED))) {
        foc_motor_fault(m, FOC_FAULT_NOT_CALIBRATED);
        return 0U;
    }

    /* 清干净旧状态，从零起步 */
    foc_angle_mgr_reset();
    foc_pid_reset(&m->pid_id);
    foc_pid_reset(&m->pid_iq);
    foc_pid_reset(&m->pid_vel);
    foc_pid_reset(&m->pid_pos);
    /* 弱磁积分器必须同 PID 一起清零：IDLE 期间慢环不跑（快环 L767
     * 提前 return），失控时积到深负的 fw_integral 不会被 L348 的
     * 未激活分支清掉；若不清，enable 后转子惯性 >enter_rpm 立即
     * 激活弱磁分支，id_ref 带着残留深负值起步 = 重新使能即再失控
     * （2026-09-05 8500rpm 反转失控"状态污染不可恢复"的根因）。 */
    m->fw_integral = 0.0f;
    m->vel_ref_rpm = 0.0f;
    m->vel_track_pos_rad = m->position_rad;
    foc_velocity_start_reset(m);
    m->id_ref = 0.0f;
    m->iq_ref = 0.0f;
    m->velocity_filt_rpm = m->velocity_observer_rpm;
    foc_speed_filter_reset(&m->vel_filter, m->velocity_observer_rpm);
    foc_speed_filter_reset(&m->vel_filter_low, m->velocity_observer_rpm);
    m->slow_cnt = m->slow_div / 2U;   /* 慢环与速度估计拍错开半个周期 */
    m->stall_cnt = 0U;
    m->safety.consecutive_over_limit = 0U;
    m->safety.soft_current_a = 0.0f;
    m->safety.trip_soft_current_a = 0.0f;
    m->safety.trip_was_hard = 0U;

    if (g_angle_mgr.mode == FOC_FEEDBACK_SENSORLESS_PRIMARY) {
        m->angle_source = FOC_ANGLE_OBSERVER;
    }

    if (m->mode == FOC_MODE_OPENLOOP_VF) {
        /* 已校准时从当前转子电角度起步，避免沿用任意旧开环相位。电压和
         * 转速仍从0由应用层斜坡升起，因此 enable 本身不会产生转矩阶跃。 */
        if (m->calib.valid != 0U) {
            m->theta_e = foc_motor_encoder_theta_e(m);
        }
        m->ol_angle_step = 0.0f;
        m->v_openloop.d = 0.0f;
        m->v_openloop.q = 0.0f;
    }

    /* 位置模式使能即锁定当前位置为"原点"：target 语义 = 相对该原点的
     * 偏移 rad（target 0 = 原点静止，target 6.283 = 正转一圈）。
     * 杜绝把多圈累计绝对位置当目标导致"永远在转"的语义缺陷。 */
    if (m->mode == FOC_MODE_POSITION) {
        m->pos_origin_rad = m->position_rad;
        m->target = 0.0f;
    }
    m->traj.active = 0U;
    m->traj.xf = m->position_rad;
    m->traj_target_latch = m->target;

    m->state = FOC_STATE_RUN;
    m->drv->enable();
    return 1U;
}

void foc_motor_disarm(foc_motor_t *m)
{
    uint32_t pm;

    if (m->state == FOC_STATE_FAULT) {
        return; /* FAULT 只能通过 clear_fault 离开 */
    }
    m->drv->disable();
    m->v_dq.d = 0.0f;
    m->v_dq.q = 0.0f;
    m->v_openloop.d = 0.0f;
    m->v_openloop.q = 0.0f;
    m->ol_angle_step = 0.0f;
    foc_pid_reset(&m->pid_vel);
    foc_pid_reset(&m->pid_pos);
    m->fw_integral = 0.0f;   /* 防御式：IDLE 下慢环不跑，不能留残留 */
    m->vel_ref_rpm = 0.0f;
    m->vel_track_pos_rad = m->position_rad;
    foc_velocity_start_reset(m);
    m->id_ref = 0.0f;
    m->iq_ref = 0.0f;
    m->velocity_filt_rpm = m->velocity_observer_rpm;
    foc_speed_filter_reset(&m->vel_filter, m->velocity_observer_rpm);
    foc_speed_filter_reset(&m->vel_filter_low, m->velocity_observer_rpm);
    m->i_abc.a = 0.0f;
    m->i_abc.b = 0.0f;
    m->i_abc.c = 0.0f;
    m->i_dq.d = 0.0f;
    m->i_dq.q = 0.0f;
    m->i_dq_filt.d = 0.0f;
    m->i_dq_filt.q = 0.0f;
    foc_lpf_reset(&m->lpf_id, 0.0f);
    foc_lpf_reset(&m->lpf_iq, 0.0f);

    /* 临界区回写：drv->disable() 期间 ISR 可能刚锁存 FAULT，
     * 无保护的 state=IDLE 会把故障吞掉 */
    pm = foc_critical_enter();
    if (m->state != FOC_STATE_FAULT) {
        m->state = FOC_STATE_IDLE;
    }
    foc_critical_exit(pm);
}

uint8_t foc_motor_set_mode(foc_motor_t *m, foc_mode_t mode)
{
    foc_mode_t old_mode = m->mode;

    /* 模式切换会改变角度源和目标单位，只允许在功率输出关闭时执行。 */
    if (m->state != FOC_STATE_IDLE) {
        return 0U;
    }
    if (m->mode == mode) {
        return 1U;
    }

    m->mode = mode;
    m->target = 0.0f;
    if ((old_mode == FOC_MODE_OPENLOOP_VF) ||
        (mode == FOC_MODE_OPENLOOP_VF)) {
        foc_pid_reset(&m->pid_id);
        foc_pid_reset(&m->pid_iq);
    }
    foc_pid_reset(&m->pid_vel);
    foc_pid_reset(&m->pid_pos);
    m->vel_ref_rpm = 0.0f;
    m->vel_track_pos_rad = m->position_rad;
    foc_velocity_start_reset(m);
    m->iq_ref = 0.0f;

    if (mode == FOC_MODE_OPENLOOP_VF) {
        /* 切入开环：清掉旧的开环电压/步进，电机安全滑行，
         * 等用户重新给 vq / rpm */
        m->v_openloop.d = 0.0f;
        m->v_openloop.q = 0.0f;
        m->ol_angle_step = 0.0f;
    }
    if (mode == FOC_MODE_POSITION) {
        /* 切入位置模式：原点暂记当前位置，target 归零（arm 时再刷新原点） */
        m->pos_origin_rad = m->position_rad;
        m->target = 0.0f;
        m->traj.active = 0U;
        m->traj.xf = m->position_rad;
        m->traj_target_latch = m->target;
    }
    if ((mode != FOC_MODE_OPENLOOP_VF) &&
        (m->calib.valid != 0U)) {
        m->angle_source = FOC_ANGLE_ENCODER_CALIBRATED;
    }
    return 1U;
}

void foc_motor_set_target(foc_motor_t *m, float value)
{
    m->target = value;
}

void foc_motor_set_angle_source(foc_motor_t *m, foc_angle_source_t src)
{
    m->angle_source = src;
}

void foc_motor_openloop_hold(foc_motor_t *m, float theta_e, float vd, float vq)
{
    m->theta_e = foc_wrap_0_2pi(theta_e);
    m->ol_angle_step = 0.0f;
    m->v_openloop.d = vd;
    m->v_openloop.q = vq;
}

void foc_motor_openloop_spin(foc_motor_t *m, float rpm, float vd, float vq)
{
    float dir = (m->calib.direction < 0) ? -1.0f : 1.0f;

    /* rpm 使用编码器机械正方向。由 θe=dir·pp·θm+offset 可知，机械
     * 速度换算到开环电角速度时也必须乘 direction。 */
    m->ol_angle_step = dir * _2PI * (rpm / 60.0f) *
                       m->params.pole_pairs * m->dt_fast;
    m->v_openloop.d = vd;
    m->v_openloop.q = vq;
}

/* ======================== 故障处理 ======================== */

void foc_motor_fault(foc_motor_t *m, foc_fault_t fault)
{
    /* 黑匣子冻结：保留故障前最后 256 拍（16ms）的逐拍数据 */
    if (s_blackbox_frozen == 0U) {
        s_blackbox_frozen = 1U;
    }
    m->drv->disable();
    m->v_dq.d = 0.0f;
    m->v_dq.q = 0.0f;
    m->v_openloop.d = 0.0f;
    m->v_openloop.q = 0.0f;
    m->ol_angle_step = 0.0f;
    m->state = FOC_STATE_FAULT;
    if ((fault != FOC_FAULT_NONE) &&
        (m->safety.fault_code == (uint8_t)FOC_FAULT_NONE)) {
        m->safety.fault_code = (uint8_t)fault; /* 只锁存第一个故障 */
    }
}

void foc_motor_clear_fault(foc_motor_t *m)
{
    if (m->state != FOC_STATE_FAULT) {
        return;
    }
    s_blackbox_frozen = 0U; /* 恢复逐拍记录 */
    m->safety.fault_code = (uint8_t)FOC_FAULT_NONE;
    m->safety.consecutive_over_limit = 0U;
    m->safety.soft_current_a = 0.0f;
    m->safety.trip_current_u_a = 0.0f;
    m->safety.trip_current_v_a = 0.0f;
    m->safety.trip_current_w_a = 0.0f;
    m->safety.trip_soft_current_a = 0.0f;
    m->safety.trip_was_hard = 0U;
    foc_pid_reset(&m->pid_vel);
    foc_pid_reset(&m->pid_pos);
    m->vel_ref_rpm = 0.0f;
    m->vel_track_pos_rad = m->position_rad;
    foc_velocity_start_reset(m);
    m->id_ref = 0.0f;
    m->iq_ref = 0.0f;
    m->velocity_filt_rpm = m->velocity_observer_rpm;
    foc_speed_filter_reset(&m->vel_filter, m->velocity_observer_rpm);
    foc_speed_filter_reset(&m->vel_filter_low, m->velocity_observer_rpm);
    m->i_abc.a = 0.0f;
    m->i_abc.b = 0.0f;
    m->i_abc.c = 0.0f;
    m->i_dq.d = 0.0f;
    m->i_dq.q = 0.0f;
    m->i_dq_filt.d = 0.0f;
    m->i_dq_filt.q = 0.0f;
    foc_lpf_reset(&m->lpf_id, 0.0f);
    foc_lpf_reset(&m->lpf_iq, 0.0f);
    m->state = FOC_STATE_IDLE;
}

/* ======================== 辅助 ======================== */

void foc_motor_override_current_limits(foc_motor_t *m, float soft_a, float hard_a)
{
    m->safety.current_limit_a = soft_a;
    m->safety.hard_current_limit_a = hard_a;
}

void foc_motor_restore_current_limits(foc_motor_t *m)
{
    m->safety.current_limit_a =
        foc_motor_run_trip_limit(m->params.max_current_a,
                                 m->params.hard_current_a);
    m->safety.hard_current_limit_a = m->params.hard_current_a;
}

float foc_motor_encoder_theta_e(const foc_motor_t *m)
{
    /* θe = dir·pp·θm + offset：
     * direction 校正编码器计数方向与电角度方向的关系，
     * offset 来自 D 轴对齐 + Z 脉冲搜索，保证 d 轴对准转子磁链 */
    return foc_wrap_0_2pi(
        ((float)m->calib.direction * m->params.pole_pairs * m->theta_mech) +
        m->calib.electrical_offset_rad);
}
