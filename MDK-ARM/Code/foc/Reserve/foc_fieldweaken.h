/**
 * @file    foc_fieldweaken.h
 * @brief   【预留区】弱磁扩速（电压余量闭环法，VESC/MESC 思路）
 *
 * ⚠ 未加入 Keil 工程。接入步骤见《Docs/09_预留特性接入手册.md》第 4 章。
 *
 * 什么时候需要：
 *   转速升高 → 反电动势升高 → 电流环需要的电压逼近母线上限
 *   （Udc/√3）→ 电压饱和，Iq 控不住，转速到顶。
 *   注入负 Id（削弱转子磁场）可以降低反电动势，换取更高转速——
 *   代价是这部分电流不产生转矩、增加发热。
 *
 * 实现（电压余量闭环，最稳健的一种）：
 *   每慢环拍：误差 = 目标电压利用率(如 92% Vmax) - 实际 |V_dq|
 *   误差为负（电压不够用）→ 积分器把 id_ref 往负推；
 *   误差为正（电压有余量）→ id_ref 自动退回 0。
 *   id_ref 限幅在 [-id_max, 0]，永不为正。
 *
 * 表贴电机（SPM）注意：
 *   - Id=0 就是 SPM 的 MTPA（最大转矩/电流比），平时不需要本模块；
 *   - SPM 弱磁效率差（磁阻小），只有确实要超反电动势限速时才用；
 *   - id_max 必须考虑退磁风险：|id_max| < 电机退磁电流（查规格书，
 *     保守取额定电流的 30~50%）。
 */

#ifndef FOC_FIELDWEAKEN_H
#define FOC_FIELDWEAKEN_H

#include <stdint.h>
#include "../Core/foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 配置 */
    float v_util_target;  /* 目标电压利用率 0~1（建议 0.92） */
    float ki;             /* 积分增益 A/(V·s)，建议 50~200 起步 */
    float id_max_a;       /* 最大弱磁电流（正数，内部取负用） */
    /* 状态 */
    float id_ref;         /* 输出：负的 d 轴给定（0 ~ -id_max） */
} foc_fw_t;

void foc_fw_init(foc_fw_t *fw, float v_util_target, float ki, float id_max_a);

/**
 * @brief 慢环每拍调用，返回弱磁 Id 给定（≤0）
 * @param v_dq  当前电流环输出电压
 * @param v_max 电压上限（u_dc/√3）
 * @param dt    慢环周期 s
 */
float foc_fw_update(foc_fw_t *fw, const dq_t *v_dq, float v_max, float dt);

/** 停止弱磁（disarm 时调用，清积分） */
void foc_fw_reset(foc_fw_t *fw);

#ifdef __cplusplus
}
#endif

#endif /* FOC_FIELDWEAKEN_H */
