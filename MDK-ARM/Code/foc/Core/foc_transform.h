/**
 * @file    foc_transform.h
 * @brief   Clarke / Park 坐标变换（纯数学，头文件内联实现）
 *
 * FOC 的两步核心变换：
 *   Clarke：三相静止 (a,b,c) → 两相静止 (α,β)
 *           物理上就是把 120° 分布的三个绕组电流投影到直角坐标系。
 *   Park  ：两相静止 (α,β) → 两相旋转 (d,q)
 *           乘一个旋转矩阵，让坐标系跟着转子转。转子转多快坐标系
 *           就转多快，于是交流量在 dq 系里变成了直流量，PI 才好控。
 *
 * sin/cos 默认使用 CMSIS-DSP 的查表插值版（arm_sin_f32 / arm_cos_f32，
 * M4 上比标准库 sinf 快数倍）；在非 ARM 平台（PC 单元测试）自动退回
 * 标准库实现，因此本文件可以脱离硬件编译验证。
 */

#ifndef FOC_TRANSFORM_H
#define FOC_TRANSFORM_H

#include "foc_types.h"
#include "foc_utils.h"

#if defined(ARM_MATH_CM4) || defined(ARM_MATH_CM7) || defined(ARM_MATH_CM33)
#include "arm_math.h"
#define foc_sin(x) arm_sin_f32(x)
#define foc_cos(x) arm_cos_f32(x)
#else
#define foc_sin(x) sinf(x)
#define foc_cos(x) cosf(x)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Clarke 变换（等幅值形式）
 *        α = a
 *        β = (a + 2b)/√3     （利用 a+b+c=0 消去 c）
 */
static inline void foc_clarke(const abc_t *abc, ab_t *ab)
{
    ab->alpha = abc->a;
    ab->beta  = (abc->a + (2.0f * abc->b)) * INV_SQRT_3;
}

/**
 * @brief 反 Clarke 变换
 *        a = α
 *        b = -α/2 + √3β/2
 *        c = -α/2 - √3β/2
 */
static inline void foc_inv_clarke(const ab_t *ab, abc_t *abc)
{
    const float half_alpha = -0.5f * ab->alpha;
    const float sqrt3_beta = SQRT_3_DIV_2 * ab->beta;

    abc->a = ab->alpha;
    abc->b = half_alpha + sqrt3_beta;
    abc->c = half_alpha - sqrt3_beta;
}

/**
 * @brief Park 变换（θ 为电角度）
 *        d =  α·cosθ + β·sinθ
 *        q = -α·sinθ + β·cosθ
 */
static inline void foc_park(const ab_t *ab, float sin_th, float cos_th, dq_t *dq)
{
    dq->d = (ab->alpha * cos_th) + (ab->beta * sin_th);
    dq->q = (-ab->alpha * sin_th) + (ab->beta * cos_th);
}

/**
 * @brief 反 Park 变换
 *        α = d·cosθ - q·sinθ
 *        β = d·sinθ + q·cosθ
 */
static inline void foc_inv_park(const dq_t *dq, float sin_th, float cos_th, ab_t *ab)
{
    ab->alpha = (dq->d * cos_th) - (dq->q * sin_th);
    ab->beta  = (dq->d * sin_th) + (dq->q * cos_th);
}

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRANSFORM_H */
