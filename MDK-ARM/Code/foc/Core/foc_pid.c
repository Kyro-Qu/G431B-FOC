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

    float output = proportional + integral + derivative;
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
    lpf->y = 0.0f;
}

float foc_lpf_update(foc_lpf_t *lpf, float x, float dt)
{
    if (lpf->tf <= 0.0f) {
        lpf->y = x;
        return x;
    }
    lpf->y += (dt / (lpf->tf + dt)) * (x - lpf->y);
    return lpf->y;
}

void foc_lpf_reset(foc_lpf_t *lpf, float value)
{
    lpf->y = value;
}
