# 11 · FOC-STP v1.0 遥测协议（自解释掩码波形流）

> 固件 v0.4.0 起 USART2 遥测由 VOFA+ JustFloat 切换为本协议，上位机为
> [foc-studio](https://kyroqu.xyz/foc-studio/)（Web Serial）。VOFA+ 不再直接兼容；
> 需要 VOFA 的旧流程请回退 `≤ v0.3.11` 固件。

## 0. 为什么换协议

| JustFloat（旧） | FOC-STP（新） |
| --- | --- |
| 固定 16 通道 68 B/帧，看 2 个通道也要发满 16 个 | 32 位通道字典，按掩码只发订阅通道：`16 + 4K` B/帧 |
| Vbus/故障码/状态这类慢变量塞进 500 Hz 波形重复发 | 10 Hz 独立 `STATUS` 心跳，波形关闭时仪表盘照样刷新 |
| `+Inf` 字节与帧尾 `00 00 80 7F` 相同，混流易失步 | 同步字 + 长度 + CRC16，任意切片/粘包/假头自恢复 |
| 无序号、无时间戳 | 16 位帧序号（丢帧检测）+ MCU 毫秒时间戳 |
| 故障只能靠人眼看 CLI | `EVENT` 帧单触发上报跳闸/状态跳变，`ACK` 帧回传配置生效值 |

## 1. 物理帧格式

```
+--------+--------+----------+-----+--------+---------------+---------+
| SYNC_0 | SYNC_1 | VER_TYPE | LEN |  SEQ   |    PAYLOAD    |  CRC16  |
|  0xA5  |  0x5A  |  uint8   |uint8| uint16 |   LEN 字节    | uint16  |
+--------+--------+----------+-----+--------+---------------+---------+
```

- 多字节字段一律 **小端**。
- `VER_TYPE`：高 4 位协议版本（当前 `1`），低 4 位帧类型：
  `0x1 WAVE`、`0x2 STATUS`、`0x3 EVENT`、`0x4 TEXT`（预留）、`0x5 ACK`。
- `SEQ`：每类流独立的 16 位环形计数（WAVE 一路，STATUS/EVENT/ACK 共用一路），上位机解缠绕为单调样本号。
- `CRC16`：CRC16-CCITT-FALSE（多项式 `0x1021`，初值 `0xFFFF`，不反转、不异或），
  覆盖 `VER_TYPE`、`LEN`、`SEQ`、`PAYLOAD`。校验向量 `"123456789" -> 0x29B1`。
- 总帧长 = `8 + LEN`。

## 2. 负载定义

### 2.1 `WAVE`（0x1）— 高速波形，默认 500 Hz

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `sample_tick` | uint32 | MCU `HAL_GetTick()` 毫秒 |
| `channel_mask` | uint32 | 32 位订阅掩码，bit n 置位表示后面携带通道 n |
| `values[K]` | float32 × K | `K = popcount(channel_mask)`，按 bit 从低到高排列 |

`LEN = 8 + 4K`，单帧最多 16 通道（`K ≤ 16`，`LEN ≤ 72`）。默认掩码 `0x040001FF`
（theta_e, iq_raw, vel_ctrl, vel_ref, id_filt, iq_filt, iq_ref, vd, vq, vbus_fast，10 通道，48 B/帧，24 kB/s）。

### 2.2 `STATUS`（0x2）— 10 Hz 状态心跳，与波形开关无关、上电即发

| 字段 | 类型 | 说明 | 上位机消费端 |
| --- | --- | --- | --- |
| `timestamp_ms` | uint32 | MCU 毫秒时间戳 | 顶栏心跳呼吸灯 + 运行时长 `HH:MM:SS` |
| `vbus_cvolts` | uint16 | 母线电压，0.01 V (厘伏) | 顶栏供电徽章 + 设备页电压展示 + 仪表盘 |
| `fault_code` | uint8 | 全局统一合并故障码 (0 无故障, 1..15 算法保护, 16..47 采样硬件故障；13 为 OVERTEMP 过温跳闸) | 顶栏状态指示灯 + 故障弹窗 + 诊断文本 |
| `state` | uint8 | 运行状态机 (0 IDLE / 1 RUN / 2 CALIB / 3 FAULT) | 顶栏主状态灯 + 使能/失能联动互锁 |
| `temp_c` | int8 | 板载功率级温度 °C（NTCG163JF103FT1 实时解算） | 顶栏 HUD 实时温度 (XX℃) + 超温报警着色 + 仪表盘 |
| `cpu_load_pct` | uint8 | CPU 快环峰值负荷率 (0..100%) | 顶栏与设备页算力占用实时指示 |

`LEN = 10`，总长 18 B（相比原 15B 节省近 22% 慢速通道带宽）。
解耦原则：转速、电流等高频控制量由 500Hz WAVE 驱动，STATUS 仅承担系统生命体征与全局安全监控。

### 2.3 `EVENT`（0x3）— 瞬态单触发

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `timestamp_ms` | uint32 | |
| `event_id` | uint8 | 1 FAULT_TRIP / 2 STATE_CHANGE / 3 CALIB_DONE / 4 WARN |
| `motor_fault` | uint8 | |
| `shunt_fault` | uint8 | |
| `detail` | uint32 | FAULT_TRIP：软限电流 ×100；STATE_CHANGE：新状态 |

`LEN = 11`。固件侧为单槽快照：未发出的 `FAULT_TRIP` 不会被随后的低优先级事件覆盖。

### 2.4 `ACK`（0x5）— 配置应答

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `cmd_code` | uint8 | 1 SET_MASK / 2 SET_RATE |
| `status` | uint8 | 0 OK / 1 REJECTED / 2 LIMITED |
| `effective_mask` | uint32 | 设备端实际生效掩码 |
| `effective_rate_hz` | uint16 | 设备端实际生效波形速率 |

`LEN = 8`。上位机以 `effective_rate_hz` 作为时间轴的唯一权威来源。

### 2.5 CLI 文本

CLI 回显仍以 **裸 ASCII** 发送（不封 `TEXT` 帧），保证 PuTTY/串口助手/Python 脚本
可直读。固件通过 `foc_cmd_print()` 保证文本与二进制帧 **不互相打断**：打印前挂起波形、
等待在途 DMA 帧发完（≤ 5 ms），再独占阻塞发送，然后恢复。上位机解码器把两个帧之间的
非同步字字节当作文本上交。`TEXT`（0x4）帧类型保留给未来需要封装的场景。

## 3. 32 位通道字典

| Bit | ID | 单位 | 含义 | Bit | ID | 单位 | 含义 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | `theta_e` | rad | 换相电角度 | 16 | `pos_ref` | rad | 位置给定（绝对） |
| 1 | `iq_raw` | A | Park 后原始 Iq | 17 | `position` | rad | 多圈机械位置 |
| 2 | `vel_ctrl` | rpm | 速度环反馈转速 | 18 | `duty_b` | 0..1 | B 相占空比 |
| 3 | `vel_ref` | rpm | 斜坡后转速给定 | 19 | `duty_c` | 0..1 | C 相占空比 |
| 4 | `id_filt` | A | Id 滤波 | 20 | `obs_theta` | rad | 观测器电角度 |
| 5 | `iq_filt` | A | Iq 滤波 | 21 | `obs_speed` | rpm | 观测器转速 |
| 6 | `iq_ref` | A | 转矩电流给定 | 22 | `obs_err` | deg | 观测角误差 |
| 7 | `vd` | V | d 轴电压指令 | 23 | `obs_conf` | 0..1 | 观测置信度 |
| 8 | `vq` | V | q 轴电压指令 | 24 | `obs_flux` | Wb | 观测磁链幅值 |
| 9 | `ia` | A | A 相电流 | 25 | `power_est` | W | 1.5(VdId+VqIq) |
| 10 | `ib` | A | B 相电流 | 26 | `vbus_fast` | V | 实时母线电压 |
| 11 | `ic` | A | C 相电流 | 27 | `torque_est` | N·m | 1.5·p·ψf·Iq |
| 12 | `duty_a` | 0..1 | A 相占空比 | 28 | `iq_err` | A | iq_ref − iq_raw |
| 13 | `id_raw` | A | Park 后原始 Id | 29 | `id_err` | A | id_ref − id_raw |
| 14 | `id_ref` | A | 弱磁 Id 给定 | 30 | `vel_err` | rpm | vel_ref − vel_ctrl |
| 15 | `vel_raw` | rpm | 编码器原始转速 | 31 | `diag_aux` | - | 角度管理器状态 |

字典的唯一真源是 `MDK-ARM/Code/foc/App/foc_telemetry.c: extract_channel_value()`；
上位机 `foc-studio/js/channels.js` 与 `tools/foc_stp.py: CHANNEL_NAMES` 必须同步。

## 4. CLI 控制命令

| 命令 | 作用 |
| --- | --- |
| `wave 0/1` | 波形流开/关（静默响应模式，返回 `wave=0/1 telem=0/1`，推荐上位机示波器使用）。STATUS 心跳不受影响 |
| `log 0/1` | 波形流开/关（同 `telem enable`，返回 `telem=0/1`）。STATUS 心跳不受影响 |
| `telem` | 打印 `enable / mask / rate` |
| `telem mask <hex\|dec>` | 设置订阅掩码；popcount > 16 时拒绝并回 `ACK LIMITED`，保持旧掩码 |
| `telem rate <hz>` | 波形速率 10..500 Hz；`16000/hz` 非整除时向下取整分频并 `ACK LIMITED`，回传真实速率 |
| `telem enable 0/1` | 波形流开/关 |

## 5. 固件实现要点（`foc_telemetry.c`）

- **快慢双缓冲**：`s_wave_buf[80]` 只归 16 kHz 快环 ISR，`s_slow_buf[32]` 只归主循环；物理隔离，互不踩踏。
- **TX 所有权仲裁**：`s_tx_state` 在短关中断临界区内与 `huart2.gState == READY` 一起判定后才认领 DMA；
  `HAL_UART_TxCpltCallback` 释放；`HAL_UART_ErrorCallback` 与 `foc_cmd_print` 超时路径调用
  `foc_telemetry_reset_tx_state()` 兜底。
- **快环预算**：WAVE 帧在 ISR 内打包，CRC 采用 256 项查表（512 B Flash），16 通道 76 B 仅需 76 次查表。
- **调度**：ISR 每 `s_telem_div` 拍发一帧 WAVE（默认 32 → 500 Hz）；主循环 `foc_telemetry_slow_tick()`
  按 `EVENT > ACK > STATUS` 优先级抢空闲 DMA 发送。

## 6. 参考实现与测试

| 端 | 文件 | 验证 |
| --- | --- | --- |
| 固件 C | `MDK-ARM/Code/foc/App/foc_stp.[ch]` | `tests/test_stp_cross.c` 生成 `tests/stp_golden.bin` |
| 上位机 JS | `foc-studio/js/protocol/stp.js` | `npm test`（`stp-decoder.mjs` / `stp-cross-verify.mjs` 消费上面的 golden） |
| Python 工具 | `tools/foc_stp.py` | `python tools/foc_stp.py` 自检；`tools/foc_capture.py`、`tools/hardware_closedloop_test.py` 使用 |

上机回归：`python tools/hardware_closedloop_test.py`（CLI 交互、500 Hz/10 Hz 帧率、CRC 零错误、
掩码/速率切换 ACK、文本交错、16 通道下快环 CPU 余量）。
