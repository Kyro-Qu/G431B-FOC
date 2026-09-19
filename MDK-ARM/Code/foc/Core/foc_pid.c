#include "foc_pid.h"
#include "foc_utils.h"

void foc_pid_init(foc_pid_t *pid, float kp, float ki, float kd,
                  float out_limit, float out_ramp)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_limit = out_limit;
    pid->out_ramp = out_ramp;
    foc_pid_reset(pid);
}

float foc_pid_update(foc_pid_t *pid, float error, float dt)
{
    float integral_prev = pid->integral;
    float output_unclamped;
    float output;

    /* 比例 */
    float proportional = pid->kp * error;

    /* Tustin 积分：integral += Ki·dt/2·(e[k] + e[k-1]) */
    float integral = pid->integral +
                     (pid->ki * dt * 0.5f * (error + pid->prev_error));

    /* 抗饱和：积分项单独钳位，防止长期饱和后"退不出来" */
    if (pid->out_limit > 0.0f) {
        integral = foc_clampf(integral, -pid->out_limit, pid->out_limit);
    }

    /* 微分（电流/速度环一般 kd=0，跳过除法） */
    float derivative = 0.0f;
    if (pid->kd != 0.0f) {
        derivative = pid->kd * (error - pid->prev_error) / dt;
    }

    output_unclamped = proportional + integral + derivative;

    /*
     * 条件积分抗饱和：
     * 输出已经超过正限幅且误差仍为正（或超过负限幅且误差仍为负）时，
     * 本拍积分只会把控制器推得更深，因此撤销本拍积分。反向误差仍允许
     * 积分，使控制器能迅速退出饱和。
     */
    if ((pid->out_limit > 0.0f) &&
        (((output_unclamped > pid->out_limit) && (error > 0.0f)) ||
         ((output_unclamped < -pid->out_limit) && (error < 0.0f)))) {
        integral = integral_prev;
        output_unclamped = proportional + integral + derivative;
    }

    output = output_unclamped;
    if (pid->out_limit > 0.0f) {
        output = foc_clampf(output, -pid->out_limit, pid->out_limit);
    }

    /* 输出斜坡限制（限制 d(output)/dt） */
    if (pid->out_ramp > 0.0f) {
        float max_step = pid->out_ramp * dt;
        output = foc_clampf(output,
                            pid->prev_output - max_step,
                            pid->prev_output + max_step);
    }

    pid->integral = integral;
    pid->prev_error = error;
    pid->prev_output = output;
    return output;
}

void foc_pid_reset(foc_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
}

void foc_pid_set_limit(foc_pid_t *pid, float out_limit)
{
    pid->out_limit = out_limit;
}

void foc_lpf_init(foc_lpf_t *lpf, float tf)
{
    lpf->tf = tf;
    lpf->alpha = 0.0f;
    lpf->y = 0.0f;
}

float foc_lpf_update(foc_lpf_t *lpf, float x, float dt)
{
    if (lpf->tf <= 0.0f) {
        lpf->y = x;
        return x;
    }
    if (lpf->alpha <= 0.0f) {
        lpf->alpha = dt / (lpf->tf + dt);
    }
    lpf->y += lpf->alpha * (x - lpf->y);
    return lpf->y;
}

void foc_lpf_reset(foc_lpf_t *lpf, float value)
{
    lpf->y = value;
}

/* ---------------- Notch ---------------- */

void foc_notch_init(foc_notch_t *n, float f0, float bw, float fs)
{
    /* RBJ cookbook biquad notch：alpha = sin(w0)/(2Q)，Q = f0/bw */
    float w0 = 6.2831853f * f0 / fs;
    float sinw = sinf(w0);
    float cosw = cosf(w0);
    float q = (bw > 0.0f) ? (f0 / bw) : 5.0f;
    float alpha = sinw / (2.0f * q);
    float a0 = 1.0f + alpha;

    n->b0 = 1.0f / a0;
    n->b1 = (-2.0f * cosw) / a0;
    n->b2 = 1.0f / a0;
    n->a1 = (-2.0f * cosw) / a0;
    n->a2 = (1.0f - alpha) / a0;
    n->z1 = 0.0f;
    n->z2 = 0.0f;
    n->enabled = 1U;
}

float foc_notch_update(foc_notch_t *n, float x)
{
    float y;
    if (n->enabled == 0U) {
        return x;
    }
    /* Direct Form II Transposed */
    y = (n->b0 * x) + n->z1;
    n->z1 = (n->b1 * x) - n->a1 * y + n->z2;
    n->z2 = (n->b2 * x) - n->a2 * y;
    return y;
}

void foc_notch_reset(foc_notch_t *n)
{
    n->z1 = 0.0f;
    n->z2 = 0.0f;
}

static inline float foc_median3(float a, float b, float c)
{
    float max_ab = (a > b) ? a : b;
    float min_ab = (a > b) ? b : a;
    return (c > max_ab) ? max_ab : ((c < min_ab) ? min_ab : c);
}

void foc_speed_filter_init(foc_speed_filter_t *filter,
                           float tf, float sample_hz)
{
    const float sqrt2 = 1.414213562373095f;
    float cutoff_hz;
    float k;
    float norm;

    filter->tf = tf;
    filter->sample_hz = sample_hz;
    filter->enabled = 0U;
    filter->b0 = 1.0f;
    filter->b1 = 0.0f;
    filter->b2 = 0.0f;
    filter->a1 = 0.0f;
    filter->a2 = 0.0f;

    if ((tf > 0.0f) && (sample_hz > 0.0f)) {
        cutoff_hz = 1.0f / (_2PI * tf);

        /* tan(pi*fc/fs) 在 Nyquist 附近趋于无穷；控制反馈不允许配置到
         * 0.45*fs 以上。CLI 当前进一步限制为 200 Hz。 */
        if (cutoff_hz > (0.45f * sample_hz)) {
            cutoff_hz = 0.45f * sample_hz;
            filter->tf = 1.0f / (_2PI * cutoff_hz);
        }

        k = tanf(_PI * cutoff_hz / sample_hz);
        norm = 1.0f / (1.0f + (sqrt2 * k) + (k * k));
        filter->b0 = k * k * norm;
        filter->b1 = 2.0f * filter->b0;
        filter->b2 = filter->b0;
        filter->a1 = 2.0f * ((k * k) - 1.0f) * norm;
        filter->a2 = (1.0f - (sqrt2 * k) + (k * k)) * norm;
        filter->enabled = 1U;
    }

    foc_speed_filter_reset(filter, 0.0f);
}

float foc_speed_filter_update(foc_speed_filter_t *filter, float input)
{
    float median;
    float output;

    if (filter->enabled == 0U) {
        filter->median_z2 = filter->median_z1;
        filter->median_z1 = input;
        return input;
    }

    median = foc_median3(input, filter->median_z1, filter->median_z2);
    filter->median_z2 = filter->median_z1;
    filter->median_z1 = input;

    /* 二阶 Butterworth，Direct Form II Transposed。 */
    output = (filter->b0 * median) + filter->z1;
    filter->z1 = (filter->b1 * median) - (filter->a1 * output) +
                 filter->z2;
    filter->z2 = (filter->b2 * median) - (filter->a2 * output);
    return output;
}

void foc_speed_filter_reset(foc_speed_filter_t *filter, float value)
{
    filter->median_z1 = value;
    filter->median_z2 = value;

    /* 初始化成直流稳态，重新使能/在线改截止频率时输出从 value 起步。 */
    filter->z1 = (1.0f - filter->b0) * value;
    filter->z2 = (filter->b2 - filter->a2) * value;
}

float foc_speed_filter_cutoff_hz(const foc_speed_filter_t *filter)
{
    if ((filter->enabled == 0U) || (filter->tf <= 0.0f)) {
        return 0.0f;
    }
    return 1.0f / (_2PI * filter->tf);
}
