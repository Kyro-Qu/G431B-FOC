#include "foc_math.h"
#include "../HAL/foc_config.h"

inline void clarke_transform(const abc_t *abc, ab_t *ab)
{
    ab->alpha = abc->a;
    ab->beta = (abc->a + (2.0f * abc->b)) * INV_SQRT_3;
}

inline void inverse_clarke_transform(const ab_t *ab, abc_t *abc)
{
    const float half_alpha = -0.5f * ab->alpha;
    const float sqrt3_beta = SQRT_3_DIV_2 * ab->beta;

    abc->a = ab->alpha;
    abc->b = half_alpha + sqrt3_beta;
    abc->c = half_alpha - sqrt3_beta;
}

inline void park_transform(const ab_t *ab, float theta, dq_t *dq)
{
    const float sin_theta = arm_sin_f32(theta);
    const float cos_theta = arm_cos_f32(theta);
    const float alpha = ab->alpha;
    const float beta = ab->beta;

    dq->d = (alpha * cos_theta) + (beta * sin_theta);
    dq->q = (-alpha * sin_theta) + (beta * cos_theta);
}

inline void inverse_park_transform(const dq_t *dq, float theta, ab_t *ab)
{
    const float sin_theta = arm_sin_f32(theta);
    const float cos_theta = arm_cos_f32(theta);
    const float d = dq->d;
    const float q = dq->q;

    ab->alpha = (d * cos_theta) - (q * sin_theta);
    ab->beta = (d * sin_theta) + (q * cos_theta);
}

inline float limit_angle_rad(float angle)
{
    angle = fmodf(angle, _2PI);
    if (angle < 0.0f)
    {
        angle += _2PI;
    }
    return angle;
}

inline float limit_angle_deg(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/* Return the conventional SVPWM sector number for diagnostics. */
static uint8_t foc_svpwm_sector(const ab_t *ab)
{
    const float u1 = ab->beta;
    const float u2 = (SQRT_3 * ab->alpha) - ab->beta;
    const float u3 = (-SQRT_3 * ab->alpha) - ab->beta;
    const uint8_t weight = (uint8_t)((u1 > 0.0f) |
                                     ((u2 > 0.0f) << 1) |
                                     ((u3 > 0.0f) << 2));

    switch (weight)
    {
    case 3U:
        return 1U;
    case 1U:
        return 2U;
    case 5U:
        return 3U;
    case 4U:
        return 4U;
    case 6U:
        return 5U;
    case 2U:
        return 6U;
    default:
        return 0U;
    }
}

inline uint8_t svpwm_calc(const ab_t *ab)
{
    const float pwm_count_per_volt = (float)PWM_CNT / U_DC;
    const uint8_t sector = foc_svpwm_sector(ab);
    float phase_a = ab->alpha;
    float phase_b = (-0.5f * ab->alpha) + (SQRT_3_DIV_2 * ab->beta);
    float phase_c = (-0.5f * ab->alpha) - (SQRT_3_DIV_2 * ab->beta);
    float phase_max = phase_a;
    float phase_min = phase_a;
    float phase_span;
    float common_mode;
    float chan_a;
    float chan_b;
    float chan_c;

    if (phase_b > phase_max)
    {
        phase_max = phase_b;
    }
    if (phase_c > phase_max)
    {
        phase_max = phase_c;
    }
    if (phase_b < phase_min)
    {
        phase_min = phase_b;
    }
    if (phase_c < phase_min)
    {
        phase_min = phase_c;
    }

    /*
     * Seven-segment SVPWM is inverse Clarke followed by common-mode injection.
     * A positive alpha command must therefore produce phase-A's largest CCR.
     * The former sector table reversed all three duties and generated -Vref.
     */
    phase_span = phase_max - phase_min;
    if (phase_span > U_DC)
    {
        const float scale = U_DC / phase_span;

        phase_a *= scale;
        phase_b *= scale;
        phase_c *= scale;
        phase_max *= scale;
        phase_min *= scale;
    }

    common_mode = -0.5f * (phase_max + phase_min);
    chan_a = (float)PWM_NEUTRAL_CNT + ((phase_a + common_mode) * pwm_count_per_volt);
    chan_b = (float)PWM_NEUTRAL_CNT + ((phase_b + common_mode) * pwm_count_per_volt);
    chan_c = (float)PWM_NEUTRAL_CNT + ((phase_c + common_mode) * pwm_count_per_volt);

    if (chan_a < 0.0f)
    {
        chan_a = 0.0f;
    }
    else if (chan_a > (float)PWM_CNT)
    {
        chan_a = (float)PWM_CNT;
    }
    if (chan_b < 0.0f)
    {
        chan_b = 0.0f;
    }
    else if (chan_b > (float)PWM_CNT)
    {
        chan_b = (float)PWM_CNT;
    }
    if (chan_c < 0.0f)
    {
        chan_c = 0.0f;
    }
    else if (chan_c > (float)PWM_CNT)
    {
        chan_c = (float)PWM_CNT;
    }

    foc_set_pwm((uint32_t)chan_a,
                (uint32_t)chan_b,
                (uint32_t)chan_c,
                sector);
    return sector;
}

inline void spwm_calc(const ab_t *ab)
{
    const float mid = 0.5f * (float)PWM_CNT;
    const float count_per_volt = 0.5f * (float)PWM_CNT / U_DC;
    abc_t phase_voltage;
    float chan_a;
    float chan_b;
    float chan_c;

    inverse_clarke_transform(ab, &phase_voltage);
    chan_a = mid + (phase_voltage.a * count_per_volt);
    chan_b = mid + (phase_voltage.b * count_per_volt);
    chan_c = mid + (phase_voltage.c * count_per_volt);

    if (chan_a < 0.0f)
    {
        chan_a = 0.0f;
    }
    else if (chan_a > (float)PWM_CNT)
    {
        chan_a = (float)PWM_CNT;
    }
    if (chan_b < 0.0f)
    {
        chan_b = 0.0f;
    }
    else if (chan_b > (float)PWM_CNT)
    {
        chan_b = (float)PWM_CNT;
    }
    if (chan_c < 0.0f)
    {
        chan_c = 0.0f;
    }
    else if (chan_c > (float)PWM_CNT)
    {
        chan_c = (float)PWM_CNT;
    }

    foc_set_pwm((uint32_t)chan_a,
                (uint32_t)chan_b,
                (uint32_t)chan_c,
                0U);
}
