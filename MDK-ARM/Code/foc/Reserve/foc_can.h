/**
 * @file    foc_can.h
 * @brief   【预留区】FDCAN 总线控制骨架（ODrive 式简化命令集）
 *
 * ⚠ 未加入 Keil 工程。接入步骤见《Docs/09_预留特性接入手册.md》第 5 章。
 * ⚠ 需要硬件：G431 有 FDCAN1 外设，但本板需确认 CAN 收发器
 *   （如 TJA1051）是否焊接/引出；没有收发器芯片就接不上总线。
 *
 * 前置软件条件（缺一编译不过/不工作）：
 *   1. CubeMX：开 FDCAN1（经典 CAN 模式即可，500k/1M），生成初始化；
 *   2. stm32g4xx_hal_conf.h：取消注释 HAL_FDCAN_MODULE_ENABLED；
 *   3. NVIC：FDCAN1_IT0 中断使能，优先级 ≥5（不许打扰控制）。
 *   本文件用 #ifdef HAL_FDCAN_MODULE_ENABLED 包裹，未启用时编译为空。
 *
 * 命令集（标准帧，ID = FOC_CAN_BASE_ID + 轴号×16 + 命令号）：
 *
 *   | 命令号 | 方向 | 数据 | 含义 |
 *   | 0x0 | 板→总线 | state,fault,vel(float) | 心跳 100ms（状态广播） |
 *   | 0x1 | 总线→板 | float target | 设定目标（按当前模式解释） |
 *   | 0x2 | 总线→板 | u8 mode | 设模式 0=vf 1=iq 2=vel 3=pos |
 *   | 0x3 | 总线→板 | u8 enable | 1=arm 0=disarm |
 *   | 0x4 | 总线→板 | - | 清故障 |
 *
 * 安全设计（照抄 ODrive 的教训）：
 *   - 心跳超时：FOC_CAN_CMD_TIMEOUT_MS 内没收到任何命令帧且电机在
 *     RUN → 自动 disarm（上位机死了电机不能继续跑）；
 *   - 所有命令经 foc_app_motor() 的公开接口执行，享受与串口命令
 *     完全相同的状态机保护（未校准拒绝闭环等）。
 */

#ifndef FOC_CAN_H
#define FOC_CAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_CAN_BASE_ID        0x200U
#define FOC_CAN_CMD_TIMEOUT_MS 500U

/** 初始化过滤器并启动 FDCAN（在 MX_FDCAN1_Init 之后调用） */
void foc_can_init(void);

/** 主循环任务：心跳发送 + 命令超时看护（放进 foc_app_task） */
void foc_can_task(void);

/** 收帧处理（在 HAL_FDCAN_RxFifo0Callback 里调用） */
void foc_can_on_rx(uint32_t std_id, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CAN_H */
