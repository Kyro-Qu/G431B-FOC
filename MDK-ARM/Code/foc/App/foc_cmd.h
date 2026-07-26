/**
 * @file    foc_cmd.h
 * @brief   串口命令行（SimpleFOC Commander / VESC terminal 风格）
 *
 * 物理层：USART2 @ 6.5 Mbaud，RX 走 DMA + 空闲中断，TX 阻塞发送。
 * 每行一条命令，以 \r 或 \n 结束，全部小写，参数用空格分隔。
 *
 * 命令一览（详见 help 输出 / Docs/04_上手指南.md）：
 *   help          帮助
 *   s             打印全部轴状态
 *   a <n>         选择当前轴（默认 0）
 *   e <0|1>       停止 / 使能当前轴
 *   c             启动校准
 *   f             清除故障
 *   m <vf|iq|vel|pos>  切换控制模式
 *   t <val>       设目标（随模式：V / A / RPM / rad）
 *   vq <v>        开环 q 轴电压
 *   rpm <v>       开环转速
 *   cb <rad/s>    电流环带宽重整定
 *   vp <v> vi <v> 速度环 Kp / Ki
 *   pp <v>        位置环 Kp
 *   lim <A>       软件电流限制
 *   log <0|1>     VOFA 遥测流开关
 */

#ifndef FOC_CMD_H
#define FOC_CMD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 启动 DMA 空闲接收 */
void foc_cmd_init(void);

/** 主循环任务：解析并执行收到的整行命令 */
void foc_cmd_task(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CMD_H */
