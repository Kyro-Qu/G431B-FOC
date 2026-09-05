# CAN FD 最小收发 Demo

> 本页记录最初的物理链路联调方法。当前 Keil 工程已改用正式协议，
> 不再编译 `foc_can_demo.c`；正式 ID、数据格式和操作示例见
> `Docs/10_FDCAN通信协议.md`。

这个 Demo 只验证 FDCAN 引脚、SIT1042T 收发器、总线位时序、发送和接收中断，不是最终电机控制协议。

## 1. 硬件与位时序

| 信号 | MCU 引脚 | 功能 |
|---|---|---|
| CAN_TX | PB9 | FDCAN1_TX |
| CAN_RX | PA11 | FDCAN1_RX |
| CAN_SHD | PC11 | SIT1042T 的 S/STB；高电平待机，低电平正常工作 |

总线控制器支持 ISO CAN FD 和 BRS；当前最小联调帧先关闭 BRS：

- 仲裁/标称波特率：500 kbit/s；
- 当前联调数据段：500 kbit/s（BRS 关闭，整帧使用标称速率）；
- 后续 BRS 数据波特率：2 Mbit/s；
- 11 位标准 ID；
- 标称段和数据段采样点均为 82.35%；
- FDCAN 内核时钟为 170 MHz。

板上已经有一个 120 Ω 终端电阻。板子与 USB-CAN 组成一条点对点总线时，USB-CAN 端也应接入 120 Ω。断电测量 CANH 与 CANL，两个终端并联时应接近 60 Ω。

## 2. 板子周期发送

板子每 1 秒发送一帧：

| 设置 | 数值 |
|---|---|
| ID | `0x300` |
| 帧格式 | CAN FD、非 BRS、标准数据帧 |
| DLC | 8 |
| ASCII | `HFOC_FD!` |
| 十六进制数据 | `48 46 4F 43 5F 46 44 21` |

USB-CAN 必须工作在正常模式，不能使用只听模式，否则板子收不到 ACK，最终会进入 Bus-Off。Demo 会在主循环尝试恢复 Bus-Off，但正常联调仍建议先连接并启动 USB-CAN，再给板子上电。

## 3. 发送校准请求

上位机发送下面一帧：

| 设置 | 数值 |
|---|---|
| ID | `0x301` |
| 帧格式 | CAN FD 标准数据帧，BRS 可开或关（首次测试建议关闭） |
| DLC | 8 |
| ASCII | `CALIBFD!` |
| 十六进制数据 | `43 41 4C 49 42 46 44 21` |

只有 ID、DLC、CAN FD 和 8 字节内容全部匹配时才产生校准请求；BRS 开关不参与命令匹配。FDCAN 中断只保存请求，`foc_can_demo_task()` 在主循环调用 `foc_calib_start()`。

轴 0 必须处于 IDLE、电流采样必须就绪且不能已有校准任务；否则请求会被安全拒绝。校准过程会给电机通电并可能转动，测试前应固定电机、限制电流并保证可以立即断电。

## 4. Keil Watch 诊断

查看全局变量：

```text
g_foc_can_demo_diag
```

常用成员：

| 成员 | 含义 |
|---|---|
| `init_ok` | FDCAN 过滤器、中断和外设启动成功 |
| `tx_count` | 成功放入发送队列的帧数 |
| `tx_attempt_count` | 周期发送尝试次数 |
| `tx_drop_count` | HAL 拒绝入队的次数 |
| `tx_fifo_status` | 最近一次 FDCAN `TXFQS` 寄存器快照 |
| `rx_count` | RX FIFO0 收到的总帧数 |
| `rx_match_count` | 完全匹配 `CALIBFD!` 的帧数 |
| `calib_start_count` | 实际进入校准的次数 |
| `calib_reject_count` | 因状态或前置条件不满足而拒绝的次数 |
| `last_rx_is_fd` | 最近一帧是否为 CAN FD |
| `last_rx_brs` | 最近一帧是否开启 BRS |
| `bus_off_count` | Bus-Off 发生次数 |
| `bus_recover_count` | 主循环恢复 FDCAN 的次数 |
| `last_error` | 最近一次 HAL FDCAN 错误或错误状态标志 |
| `protocol_status` | FDCAN `PSR` 协议状态寄存器快照 |
| `error_counter` | FDCAN `ECR` 错误计数器快照，低 8 位为 TEC |

## 5. 收不到数据时

依次检查：

1. 首次测试设置为 ISO CAN FD、500 kbit/s，并关闭 BRS；链路正常后再测试 500 kbit/s / 2 Mbit/s、BRS；
2. USB-CAN 不是 Classical CAN，也不是只听模式；
3. CANH 对 CANH、CANL 对 CANL，并连接公共 GND；
4. 断电测量 CANH-CANL 约为 60 Ω；
5. Keil Watch 中 `init_ok=1`；
6. PC11 为低电平，表示 SIT1042T 已退出待机；
7. 检查 `bus_off_count` 和 `last_error`。

## 6. 当前联调操作

1. Cangaroo 接口选择支持 CAN FD 的 CANable SLCAN，标称波特率设为 500 kbit/s，并使用正常模式而不是只听模式。
2. 当前板端心跳是 CAN FD 非 BRS 帧；接收窗口应每秒出现 `0x300 / HFOC_FD!`。
3. 发送校准命令时勾选 FD、取消 BRS，ID 为 `0x301`，DLC 为 8，数据为 `43 41 4C 49 42 46 44 21`。只单次发送，不要使用 10 ms 循环发送。
4. 如果仍然完全收不到 `0x300`，先不要检查电机控制逻辑。断电测量 CANH-CANL：板端和 USB-CAN 两个 120 Ω 终端同时接入时应约为 60 Ω；同时确认 CANH 对 CANH、CANL 对 CANL，并连接公共 GND。

当前最小 Demo 关闭了自动重发。这样没有 ACK 时发送槽不会长期占满，便于插拔 USB-CAN 和排查接线；正式控制协议上线后再根据故障恢复策略决定是否开启自动重发。

## 7. 双向通信、仲裁与 Cangaroo 显示

CAN 在物理线上使用同一对 CANH/CANL 收发，可以称为半双工，但它不是需要软件切换收发方向的普通半双工串口。所有节点发送时也同时监听总线；多个节点同时开始发送时，由标准 ID 逐位仲裁，数值更小的 ID 优先。失败节点不会破坏获胜帧；自动重发启用且不是 One-Shot 模式时，会在总线空闲后重试。

本 Demo 的板端心跳 ID 为 `0x300`，上位机命令 ID 为 `0x301`。两帧恰好同时开始时 `0x300` 先发送，`0x301` 随后重试，不会造成单向通信。每秒一帧的总线占用率极低，不需要错开定时。

Cangaroo 的 Trace 主要显示适配器从总线接收的帧。部分 SLCAN 固件不会把本机成功发送的帧作为 TX echo 再放入 Trace，因此发送帧可能只出现在下方 Log。Log 中 `to 769` 表示十进制 769，即十六进制 `0x301`；`to 768` 则是 `0x300`。

校准命令必须单次发送。不要使用 Send Repeat：重复命令可能在第一次校准结束后再次启动校准，或在 RUN/CALIB/FAULT 状态下不断产生拒绝记录。

调试电流采样固件时不要长时间暂停 CPU。暂停会切断 16 kHz ADC 节拍，采样看门狗会锁存 `FOC_FAULT_CURRENT_SENSE`；出现 `cs_ready=0`、`cs_fault!=0` 时需要复位后重新检查。
