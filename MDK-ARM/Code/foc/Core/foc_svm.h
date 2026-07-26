/**
 * @file    foc_svm.h
 * @brief   空间矢量调制 SVPWM（纯函数：电压矢量 → 三相占空比）
 *
 * 实现方式：反 Clarke + 中点电压（零序分量）注入，数学上与七段式
 * SVPWM 完全等价，但不需要查扇区表 —— 这是 VESC(mcpwm_foc) 与
 * SimpleFOC(space vector modulation) 共同采用的简洁写法：
 *
 *   v_common = -(max(va,vb,vc) + min(va,vb,vc)) / 2
 *   duty_x   = 0.5 + (v_x + v_common) / U_dc
 *
 * 注入零序后，相电压幅值可以到 U_dc/√3（比纯正弦调制多 15.5%），
 * 这就是 SVPWM 相比 SPWM 提高母线利用率的全部秘密。
 *
 * 输出为占空比（0.0~1.0），由板级层换算成定时器比较值 ——
 * 因此本文件可以在 PC 上单元测试，与硬件完全解耦。
 */

#ifndef FOC_SVM_H
#define FOC_SVM_H

#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** SVPWM 计算结果 */
typedef struct {
    float duty_a;     /* A 相占空比 0.0 ~ 1.0 */
    float duty_b;
    float duty_c;
    uint8_t sector;   /* 电压矢量所在扇区 1~6（0=无效），供采样窗口规划 */
} foc_svm_t;

/**
 * @brief 由 αβ 电压矢量计算三相占空比
 * @param v_ab  目标电压矢量（V）
 * @param u_dc  母线电压（V）
 * @param out   输出占空比与扇区
 *
 * 超出线性区（|v| > u_dc/√3）时按比例缩放（过调制保护），
 * 输出恒保证在 [0,1] 内。
 */
void foc_svm_calc(const ab_t *v_ab, float u_dc, foc_svm_t *out);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SVM_H */
