/**
 * @file    foc_cmd.h
 * @brief   串口命令行（SimpleFOC Commander / VESC terminal 风格）
 *
 * 物理层：USART2 @ 6.5 Mbaud，RX 走 DMA + 空闲中断，TX 阻塞发送。
 * 每行一条命令，以 \r 或 \n 结束，全部小写，参数用空格分隔。
 *
 * 命令一览（详见 help 输出 / Docs/04_上手指南.md）：
 *   help / version / status
 *   motor [n]                    查看/选择当前电机
 *   enable / disable             使能/停止
 *   fault [clear]                查看/清除故障
 *   calib [full]                 快速/完整校准
 *   mode [vf|iq|vel|pos]         查看/切换控制模式
 *   target <value>               iq:A、vel:RPM、pos:rad
 *   vq <V> / vf slope <V/RPM>    VF 零速提升/斜率
 *   rpm <RPM>                    VF 开环速度
 *   limit [A]                    查看/设置软件电流限制
 *   current [bw <rad/s>]         电流环带宽
 *   tune [angle_delay|fw|pll ...] 高速实验参数（仅 RAM）
 *   vel [kp|ki <value>]          速度环 PI
 *   pos [kp <value>]             位置环 P
 *   ident / ident apply          启动测量/应用最近的 Rs+Ls
 *   conf <write|erase>           保存/擦除 Flash 配置
 *   log [0|1]                    VOFA 遥测流状态/开关
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

/** 格式化打印到命令串口（阻塞，仅限主循环上下文调用） */
void foc_cmd_print(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CMD_H */
