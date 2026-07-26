#ifndef __FOC_MATH_H__
#define __FOC_MATH_H__

#include "foc_config.h"
#include "arm_math.h"  // CMSIS-DSP库，提供arm_sin_f32等硬件加速函数
#include "stm32g4xx.h" // STM32G4系列HAL库
#include <math.h>      // 标准数学库，提供fmodf等


/* ========== 数学常量 ========== */
#define _PI          3.141592653589793f  // π，
#define _2PI         6.283185307179586f  // 2π
#define SQRT_3       1.732050807568877f  // √3
#define SQRT_3_DIV_2 0.866025403784438f  // √3/2
#define INV_SQRT_3   0.577350269189625f  // 1/√3

/* ========== 坐标系结构体 ========== */

/**
 * @brief 三相静止坐标系 (a-b-c)
 */
typedef struct abc_tag{
    float a;  // A相分量
    float b;  // B相分量
    float c;  // C相分量
} abc_t;

/**
 * @brief 两相静止坐标系 (α-β)
 */
typedef struct ab_tag {
    float alpha;  // α轴分量（与A相重合）
    float beta;   // β轴分量（超前α轴90°）
} ab_t;

/**
 * @brief 两相旋转坐标系 (d-q)
 */
typedef struct dq_tag {
    float d;  // 直轴分量（励磁分量）
    float q;  // 交轴分量（转矩分量）
} dq_t;

/* ========== 函数声明 ========== */
void clarke_transform(const abc_t *abc, ab_t *ab);
void inverse_clarke_transform(const ab_t *ab, abc_t *abc);
void park_transform(const ab_t *ab, float theta, dq_t *dq);
void inverse_park_transform(const dq_t *dq, float theta, ab_t *ab);
float limit_angle_rad(float angle);
float limit_angle_deg(float angle);
uint8_t svpwm_calc(const ab_t *ab);
void spwm_calc(const ab_t *ab);

#endif /* __FOC_MATH_H__ */
