/**
 * @file    foc_port.h
 * @brief   硬件/平台隔离适配层：短临界区与系统微秒/时钟周期计时
 *
 * 核心设计原则（Core 纯度保证）：
 *   1. Core 层严禁直接包含 HAL 驱动层具体头文件（如 foc_board_g431.h）；
 *   2. 本文件作为 Core 层与平台相关设施的统一定义接口：
 *      - 短临界区（关中断/恢复中断）
 *      - DWT 系统运行周期计数查询 foc_port_cycles()
 */

#ifndef FOC_PORT_H
#define FOC_PORT_H

#include <stdint.h>

#if defined(__ARMCC_VERSION) || defined(__GNUC__)
#include "cmsis_compiler.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** 进入临界区：返回进入前的 PRIMASK，配对传给 foc_critical_exit */
__STATIC_INLINE uint32_t foc_critical_enter(void)
{
#if defined(__ARMCC_VERSION) || defined(__GNUC__)
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#else
    return 0U;
#endif
}

/** 退出临界区：恢复进入前的中断状态（支持嵌套调用） */
__STATIC_INLINE void foc_critical_exit(uint32_t primask)
{
#if defined(__ARMCC_VERSION) || defined(__GNUC__)
    __set_PRIMASK(primask);
#else
    (void)primask;
#endif
}

/** 平台高精度系统时钟周期获取（解耦 HAL 层） */
uint32_t foc_port_cycles(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PORT_H */
