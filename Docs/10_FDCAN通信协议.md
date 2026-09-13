# 10 · FDCAN 正式控制协议

本工程把两种通信入口同时保留：

- USART2（6.5 Mbaud）继续用于人工输入命令，并发送 FOC-STP 遥测（[11 篇](11_FOC-STP遥测协议.md)）；
- FDCAN1 用于上位机程序或另一块 MCU 的二进制控制、应答、状态和故障上报。

两套入口最终都操作同一个 `foc_motor_t`，所以共用校准、状态机和故障保护。若串口与 CAN 同时修改同一目标，后执行的命令生效。正式运行时应由上位机约定唯一控制源；现场调试仍可用串口发送 `disable` 紧急停机。

## 1. 启用宏与硬件配置

配置位于 `MDK-ARM/Code/foc/HAL/foc_config.h`：

```c
#define FOC_CAN_ENABLE                 1
#define FOC_CAN_BRS_ENABLE             0
#define FOC_CAN_AUTO_RETRANSMISSION    1
#define FOC_CAN_STATUS_PERIOD_MS       100U
```

- `FOC_CAN_ENABLE=0`：CubeMX 仍初始化 FDCAN 句柄和引脚，但正式协议不会配置过滤器、启动控制器或拉低 PC11；SIT1042T 始终处于待机，串口 CLI 和 VOFA 不受影响。保留无条件 `MX_FDCAN1_Init()` 是为了再次从 `.ioc` 生成代码后，关闭宏仍不会被生成器悄悄绕过。
- `FOC_CAN_BRS_ENABLE=0`：当前实机验证基线，CAN FD 整帧使用 500 kbit/s。
- `FOC_CAN_BRS_ENABLE=1`：仲裁段 500 kbit/s、数据段 2 Mbit/s；只有上位机也设置为 2 Mbit/s 时才能开启。
- `FOC_CAN_AUTO_RETRANSMISSION=1`：仲裁丢失或发送错误后由控制器自动重发。

板级连接：PB9=FDCAN1_TX、PA11=FDCAN1_RX、PC11=CAN_SHD。PC11 高电平待机、低电平正常工作。SIT1042T、USB-CAN 和两端 120 Ω 终端组成点到点总线时，断电测量 CANH-CANL 应接近 60 Ω。

Cangaroo 使用：ISO CAN FD、标准帧、500000 bit/s、关闭 Listen Only；当前默认帧勾选 FD、取消 BRS。USB-CAN 本机发出的帧不一定在 Trace 中显示为 TX，但下方 Log 会记录发送。

## 2. 报文 ID

| ID | 方向 | 周期/DLC | 用途 |
|---|---|---|---|
| `0x300` | 板 → 主机 | 100 ms / 32 字节 | 周期状态心跳 |
| `0x301` | 主机 → 板 | 按需 / 16 字节 | 控制命令请求 |
| `0x302` | 板 → 主机 | 每条命令 / 16 字节 | 命令执行应答 |
| `0x303` | 板 → 主机 | 故障变化 / 16 字节 | 故障产生或清除事件 |

所有多字节整数和 IEEE-754 `float` 都是小端序。旧 Demo 的 8 字节 `CALIBFD!` 不再作为正式命令；收到长度不对的 `0x301` 后会返回 `BAD_LENGTH`。

## 3. 控制请求 `0x301`

固定 16 字节：

| 字节 | 字段 | 说明 |
|---|---|---|
| 0 | magic | 固定 `0xA5` |
| 1 | version | 当前为 `0x01` |
| 2 | sequence | 主机每条新命令递增，0～255 回绕 |
| 3 | command | 命令码，见下表 |
| 4 | axis | 当前硬件轴为 0 |
| 5 | flags | 当前保留，发 0 |
| 6..7 | reserved | 发 0 |
| 8..11 | argument0 | `uint32_t` 或 `float`，由命令决定 |
| 12..15 | argument1 | 保留，发 0 |

同一个 sequence 的完全相同报文被视为重发：固件只重发缓存应答，不会再次使能、校准或写 Flash。同一个 sequence 但内容不同会返回 `SEQUENCE_CONFLICT`。

| command | 名称 | argument0 | 状态约束 |
|---:|---|---|---|
| `0x01` | GET_STATUS | 无 | 随时 |
| `0x02` | ENABLE | 无 | 状态机允许；闭环要求已校准 |
| `0x03` | DISABLE | 无 | 随时，清除轴 0 的 V/F 运动给定 |
| `0x04` | CLEAR_FAULT | 无 | 电流采样链必须仍然 ready |
| `0x05` | CALIBRATE | `u32`：0=快速/普通，1=full | IDLE、无其他校准 |
| `0x06` | SET_MODE | `u32`：0=vf、1=iq、2=vel、3=pos | 仅 IDLE |
| `0x07` | SET_TARGET | `float`：A/RPM/rad 随模式变化 | 仅 IDLE/RUN；V/F 模式拒绝 |
| `0x08` | SET_VF_VQ | `float`，V/F boost 电压 V | 仅 IDLE/RUN、M0、mode=vf |
| `0x09` | SET_VF_RPM | `float`，机械 RPM | 仅 IDLE/RUN、M0、mode=vf |
| `0x0A` | SET_CURRENT_LIMIT | `float`，A | 仅 IDLE、无校准；0 < limit ≤ hard limit |
| `0x0B` | SAVE_CONFIG | 无 | IDLE、转子低于 60 RPM |
| `0x0C` | SET_VF_SLOPE | `float`，V/RPM | 仅 IDLE/RUN、M0，范围 0～0.01 |

CAN 没有设置“命令超时自动停机”。原因是本工程保留串口控制：若 CAN 超时看护无条件停机，会把串口启动的电机误停。需要通信失联保护时，应再增加“控制权/租约”机制，只让当前 CAN 控制者的租约超时触发停机。

## 4. 命令应答 `0x302`

固定 16 字节：

| 字节 | 字段 | 说明 |
|---|---|---|
| 0 | magic | 固定 `0x5A` |
| 1 | version | `0x01` |
| 2 | sequence | 原样回显请求序号 |
| 3 | command | 原样回显命令码 |
| 4 | axis | 原样回显轴号 |
| 5 | result | 执行结果 |
| 6 | state | 0=IDLE、1=RUN、2=CALIB、3=FAULT |
| 7 | mode | 0=vf、1=iq、2=vel、3=pos |
| 8 | motor_fault | `foc_fault_t` |
| 9 | current_fault | `current_shunt_fault_t` |
| 10 | calib_valid | 1=闭环角度校准有效 |
| 11 | calib_state | 0～7，见 `foc_calib_state_t` |
| 12..15 | detail | `float`；成功时通常回显实际写入值，失败时辅助说明 |

结果码：

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | OK | 成功 |
| 1 | BAD_MAGIC | magic 错 |
| 2 | BAD_VERSION | 协议版本不支持 |
| 3 | BAD_LENGTH | 不是 16 字节 CAN FD 请求 |
| 4 | BAD_AXIS | 轴号越界 |
| 5 | BAD_COMMAND | 未定义命令 |
| 6 | BAD_ARGUMENT | 参数数值、范围或模式不匹配 |
| 7 | STATE_REJECTED | 当前 IDLE/RUN/CALIB/FAULT 状态不允许 |
| 8 | NOT_CALIBRATED | 闭环操作要求校准 |
| 9 | BUSY | 校准等任务正忙 |
| 10 | SEQUENCE_CONFLICT | 序号与上一条相同但报文内容不同 |
| 11 | INTERNAL_ERROR | Flash/电流采样暂停恢复等内部操作失败 |

## 5. 状态心跳 `0x300`

固定 32 字节：

| 字节 | 字段 | 类型/单位 |
|---|---|---|
| 0 | magic | `0x53`（ASCII `S`） |
| 1 | version | `0x01` |
| 2 | axis | 当前 0 |
| 3 | state | `uint8_t` |
| 4 | mode | `uint8_t` |
| 5 | calib_valid | `uint8_t` |
| 6 | calib_state | `uint8_t` |
| 7 | current_ready | `uint8_t` |
| 8..11 | motor_fault | `uint32_t` |
| 12..15 | current_fault | `uint32_t` |
| 16..19 | velocity | `float`，控制使用的滤波机械 RPM |
| 20..23 | target | `float`；V/F 为 RPM 命令，其余按模式为 A/RPM/rad |
| 24..27 | iq | `float`，滤波 Iq，A |
| 28..31 | vq | `float`；V/F 为斜坡后 Vq，其余为电流环 Vq，V |

## 6. 故障事件 `0x303`

当 motor fault 或 current-shunt fault 发生变化时发送，包括故障清零：

| 字节 | 字段 | 说明 |
|---|---|---|
| 0..3 | 头 | `0x46`、version、事件序号、axis |
| 4..7 | 状态 | state、mode、motor_fault、current_fault |
| 8..11 | peak_current | `float`，当前周期峰值 A |
| 12..15 | tick | `uint32_t`，HAL 毫秒计数 |

`0x303` 是低延迟事件，`0x300` 仍会周期重复完整状态。主机不应只依赖事件帧，因为主机可能在故障发生后才接入总线。

## 7. Cangaroo 手工示例

选择 ID `301`、DLC 16、勾选 FD、取消 BRS。每条命令修改 sequence，不要用 10 ms 循环发送使能/校准/写 Flash。

```text
# 查询状态，seq=1
A5 01 01 01 00 00 00 00  00 00 00 00  00 00 00 00

# 停机，seq=2
A5 01 02 03 00 00 00 00  00 00 00 00  00 00 00 00

# full 校准，seq=3，argument0(u32)=1
A5 01 03 05 00 00 00 00  01 00 00 00  00 00 00 00

# mode=vf，seq=4，argument0(u32)=0
A5 01 04 06 00 00 00 00  00 00 00 00  00 00 00 00

# V/F boost=0.30 V，seq=5，0.30f=9A 99 99 3E
A5 01 05 08 00 00 00 00  9A 99 99 3E  00 00 00 00

# V/F rpm=300，seq=6，300.0f=00 00 96 43
A5 01 06 09 00 00 00 00  00 00 96 43  00 00 00 00

# 使能，seq=7
A5 01 07 02 00 00 00 00  00 00 00 00  00 00 00 00
```

闭环速度模式建议顺序：`DISABLE` → `CALIBRATE` → 等待状态帧 `calib_valid=1` 且 `state=IDLE` → `SET_MODE(vel=2)` → `SET_TARGET(0 RPM)` → `ENABLE` → 小步增加 `SET_TARGET`。人工调试仍推荐先用串口命令，因为 `mode vel`、`target 100` 更直观；CAN 正式协议用于软件界面封装后自动生成这些字节。

## 8. 固件实现与诊断

实现文件：

- `MDK-ARM/Code/foc/Driver/can/foc_can.c/.h`
- 初始化和任务入口：`Core/Src/main.c`
- 位时序和引脚：`FOC_G431.ioc`

FDCAN 中断只把 `0x301` 放入四深度软件队列，主循环才执行命令。四深度应答队列优先于周期状态；故障变化也优先于下一次状态心跳。硬件发送 FIFO 满或总线暂时不可用时，待发应答/故障事件保留并以 5 ms 间隔重试，避免持续占用主循环。CALIB/FAULT 状态拒绝写入运动给定，校准期间也拒绝修改电流限制。

Keil Watch 可查看 `g_foc_can_diag`，重点字段是 `init_ok`、`rx_count`、`command_count`、`last_command`、`last_result`、`response_count`、`rx_queue_drop_count`、`tx_drop_count`、`bus_off_count` 和 `last_hal_error`。其中 `response_count`、`status_count`、`fault_event_count` 表示报文已成功放入 FDCAN 硬件发送 FIFO，不等同于已经在总线上收到 ACK；实际链路状态还需结合分析仪接收结果、`protocol_status` 和 `error_counter` 判断。
