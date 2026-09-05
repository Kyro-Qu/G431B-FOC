/**
 * @file    foc_utils.h
 * @brief   数学常量与内联小工具（纯 C，不依赖任何硬件头文件）
 *
 * 设计参考：
 *   - SimpleFOC foc_utils.h：常量宏 + 轻量内联函数的组织方式
 *   - VESC utils_math.h：限幅/归一化工具集中存放
 *
 * 本文件是整个 FOC 库依赖链的最底层，任何模块都可以放心包含。
 */

#ifndef FOC_UTILS_H
#define FOC_UTILS_H

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 数学常量 ========== */
#define _PI          3.141592653589793f  /* π */
#define _2PI         6.283185307179586f  /* 2π */
#define _PI_2        1.570796326794897f  /* π/2 */
#define SQRT_3       1.732050807568877f  /* √3 */
#define SQRT_3_DIV_2 0.866025403784438f  /* √3/2 */
#define INV_SQRT_3   0.577350269189625f  /* 1/√3 */

/* RPM 与 rad/s（机械）互换系数 */
#define FOC_RPM_TO_RADS (_2PI / 60.0f)
#define FOC_RADS_TO_RPM (60.0f / _2PI)

/* ========== 内联工具 ========== */

/** 上下限幅 */
static inline float foc_clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

/** 角度归一化到 [0, 2π)（无 fmodf 开销，极速内联） */
static inline float foc_wrap_0_2pi(float angle)
{
    while (angle >= _2PI) {
        angle -= _2PI;
    }
    while (angle < 0.0f) {
        angle += _2PI;
    }
    return angle;
}

/** 角度差归一化到 [-π, π)，用于位置误差与角度增量解算（极速内联） */
static inline float foc_wrap_pm_pi(float angle)
{
    while (angle >= _PI) {
        angle -= _2PI;
    }
    while (angle < -_PI) {
        angle += _2PI;
    }
    return angle;
}

/** 取三者最大 */
static inline float foc_max3(float a, float b, float c)
{
    float m = (a > b) ? a : b;
    return (c > m) ? c : m;
}

/** 取三者最小 */
static inline float foc_min3(float a, float b, float c)
{
    float m = (a < b) ? a : b;
    return (c < m) ? c : m;
}

#ifdef __cplusplus
}
#endif

#endif /* FOC_UTILS_H */
