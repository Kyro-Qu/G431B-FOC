/**
 * @file    foc_port.h
 * @brief   可移植性小工具：短临界区（关中断 → 恢复）
 *
 * 为什么需要：主循环里 "检查 state 再写 state" 的序列可能被 16kHz
 * 中断里的故障保护（写 FAULT）打断，无保护的回写会把 FAULT 吞掉。
 * 用法（保持极短，只包住检查+赋值，绝不包寄存器序列）：
 *   uint32_t pm = foc_critical_enter();
 *   if (m->state != FOC_STATE_FAULT) { m->state = FOC_STATE_IDLE; }
 *   foc_critical_exit(pm);
 *
 * 只依赖 CMSIS 编译器抽象层（cmsis_compiler.h），与具体芯片无关。
 */

#ifndef FOC_PORT_H
#define FOC_PORT_H

#include <stdint.h>
#include "cmsis_compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 进入临界区：返回进入前的 PRIMASK，配对传给 foc_critical_exit */
__STATIC_INLINE uint32_t foc_critical_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/** 退出临界区：恢复进入前的中断状态（支持嵌套调用） */
__STATIC_INLINE void foc_critical_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

#ifdef __cplusplus
}
#endif

#endif /* FOC_PORT_H */
