/**
 * @file    foc_fieldweaken.c
 * @brief   【预留区】弱磁扩速实现（电压余量闭环）
 */

#include "foc_fieldweaken.h"
#include "../Core/foc_utils.h"

void foc_fw_init(foc_fw_t *fw, float v_util_target, float ki, float id_max_a)
{
    fw->v_util_target = v_util_target;
    fw->ki = ki;
    fw->id_max_a = id_max_a;
    fw->id_ref = 0.0f;
}

float foc_fw_update(foc_fw_t *fw, const dq_t *v_dq, float v_max, float dt)
{
    float v_mag = sqrtf((v_dq->d * v_dq->d) + (v_dq->q * v_dq->q));
    float err = (fw->v_util_target * v_max) - v_mag;

    /* err<0：电压不够 → id_ref 往负推；err>0：有余量 → 退回 0。
     * 积分器天然带回退，无需额外逻辑 */
    fw->id_ref += fw->ki * err * dt;
    fw->id_ref = foc_clampf(fw->id_ref, -fw->id_max_a, 0.0f);
    return fw->id_ref;
}

void foc_fw_reset(foc_fw_t *fw)
{
    fw->id_ref = 0.0f;
}
