/**
 * @file    foc_anticog.c
 * @brief   【预留区】抗齿槽标定与前馈实现
 */

#include "foc_anticog.h"
#include "../Core/foc_utils.h"
#include <string.h>

void foc_anticog_init(foc_anticog_t *ac)
{
    memset(ac, 0, sizeof(*ac));
    ac->state = FOC_ACOG_IDLE;
}

void foc_anticog_calib_begin(foc_anticog_t *ac)
{
    memset(ac->table, 0, sizeof(ac->table));
    memset(ac->bin_count, 0, sizeof(ac->bin_count));
    ac->total_samples = 0U;
    ac->enable = 0U;
    ac->state = FOC_ACOG_CALIB;
}

/* 机械角 → 桶号 */
static uint32_t acog_bin(float mech_angle)
{
    float pos = foc_wrap_0_2pi(mech_angle) * ((float)FOC_ANTICOG_POINTS / _2PI);
    uint32_t bin = (uint32_t)pos;

    if (bin >= FOC_ANTICOG_POINTS) {
        bin = FOC_ANTICOG_POINTS - 1U;
    }
    return bin;
}

void foc_anticog_calib_sample(foc_anticog_t *ac,
                              float mech_angle, float iq_ref)
{
    uint32_t bin;

    if (ac->state != FOC_ACOG_CALIB) {
        return;
    }
    bin = acog_bin(mech_angle);
    /* 标定期先累加，finish 时统一除样本数（表暂存"和"） */
    ac->table[bin] += iq_ref;
    ++ac->bin_count[bin];
    ++ac->total_samples;
}

uint8_t foc_anticog_calib_finish(foc_anticog_t *ac)
{
    uint32_t i;
    float mean = 0.0f;

    if (ac->state != FOC_ACOG_CALIB) {
        return 0U;
    }

    /* 有空桶说明没转够整圈或转速太快跳桶 → 标定失败 */
    for (i = 0U; i < FOC_ANTICOG_POINTS; i++) {
        if (ac->bin_count[i] == 0U) {
            ac->state = FOC_ACOG_IDLE;
            return 0U;
        }
        ac->table[i] /= (float)ac->bin_count[i];
        mean += ac->table[i];
    }
    mean /= (float)FOC_ANTICOG_POINTS;

    /* 减去均值：均值是摩擦/负载力矩，属于速度环的正常工作，
     * 只有随角度起伏的部分才是齿槽 */
    for (i = 0U; i < FOC_ANTICOG_POINTS; i++) {
        ac->table[i] -= mean;
    }

    ac->state = FOC_ACOG_READY;
    ac->enable = 1U;
    return 1U;
}

float foc_anticog_feedforward(const foc_anticog_t *ac, float mech_angle)
{
    float pos;
    uint32_t i0;
    uint32_t i1;
    float frac;

    if ((ac->state != FOC_ACOG_READY) || (ac->enable == 0U)) {
        return 0.0f;
    }

    pos = foc_wrap_0_2pi(mech_angle) * ((float)FOC_ANTICOG_POINTS / _2PI);
    i0 = (uint32_t)pos;
    if (i0 >= FOC_ANTICOG_POINTS) {
        i0 = FOC_ANTICOG_POINTS - 1U;
    }
    i1 = (i0 + 1U) % FOC_ANTICOG_POINTS;
    frac = pos - (float)i0;

    return ac->table[i0] + ((ac->table[i1] - ac->table[i0]) * frac);
}
