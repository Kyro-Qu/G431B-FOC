/**
 * @file    foc_anticog.h
 * @brief   【预留区】抗齿槽力矩标定与前馈补偿（ODrive anticogging 思路）
 *
 * ⚠ 未加入 Keil 工程。接入步骤见《Docs/09_预留特性接入手册.md》第 3 章。
 *
 * 原理：
 *   永磁电机的磁铁与定子齿槽相互作用产生周期性的"齿槽力矩"，
 *   表现为低速转动时一顿一顿（cogging）。它只与机械角有关且可复现
 *   → 提前测出每个角度需要多少电流去抵消，运行时按角度查表把这个
 *   电流加到 Iq 给定上（前馈），低速平顺度立竿见影。
 *
 * 标定方法（速度法，实现简单效果够用）：
 *   速度环极低速（如 20 RPM）恒速转 N 圈，把速度环输出的 iq_ref
 *   按机械角分桶平均——恒速下 iq_ref 恰好等于"该角度维持转动所需
 *   的力矩"，减去全表均值（摩擦分量）后剩下的就是齿槽分量。
 *
 * 表大小：512 点/圈 × 4 字节 = 2KB RAM。想掉电保存可扩展 foc_store
 *   （版本号 +1，表放独立一页更稳妥）。
 */

#ifndef FOC_ANTICOG_H
#define FOC_ANTICOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_ANTICOG_POINTS 512U

typedef enum {
    FOC_ACOG_IDLE = 0,
    FOC_ACOG_CALIB,      /* 标定采集中 */
    FOC_ACOG_READY       /* 表已生成，可用于前馈 */
} foc_anticog_state_t;

typedef struct {
    float table[FOC_ANTICOG_POINTS];  /* 齿槽补偿电流 A（已去均值） */
    uint32_t bin_count[FOC_ANTICOG_POINTS]; /* 标定期各桶样本数 */
    foc_anticog_state_t state;
    uint8_t enable;                   /* 1 = 运行时启用前馈 */
    uint32_t total_samples;           /* 标定进度观察 */
} foc_anticog_t;

void foc_anticog_init(foc_anticog_t *ac);

/** 开始标定：先清表。之后让电机速度模式 20 RPM 恒速转 ≥3 圈 */
void foc_anticog_calib_begin(foc_anticog_t *ac);

/**
 * @brief 标定采样：慢环每拍调用（速度模式恒速运行期间）
 * @param mech_angle 当前机械角 [0,2π)
 * @param iq_ref     当前速度环输出的 Iq 给定
 */
void foc_anticog_calib_sample(foc_anticog_t *ac,
                              float mech_angle, float iq_ref);

/** 结束标定：分桶平均 + 去均值。返回 1 成功（所有桶都有样本） */
uint8_t foc_anticog_calib_finish(foc_anticog_t *ac);

/**
 * @brief 运行时前馈：返回当前角度的补偿电流（加到 iq_ref 上）。
 *        线性插值相邻两点，表未就绪或未使能返回 0
 */
float foc_anticog_feedforward(const foc_anticog_t *ac, float mech_angle);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ANTICOG_H */
