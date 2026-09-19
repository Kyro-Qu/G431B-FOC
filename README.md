# FOC_G431 —— STM32G431 FOC 电机控制工程（双轴框架预留）

基于火柴FOC（bd6s40a_mini_g431 V2）驱控一体板的磁场定向控制（FOC）工程。
架构与算法综合了 **SimpleFOC**（接口抽象、PID）、**ODrive**（轴对象化、
电流环带宽自整定、参数自动测量、轨迹规划）、**VESC**（SVPWM 写法、
中断调度、命令行）与 **ST MCSDK**（三电阻采样时序，硬件验证）的精华，
全中文注释。

## 功能

- 电流环（16 kHz）/ 速度环 / 位置环级联闭环 + 开环 V/f，四种模式串口一键切换
- 电流环增益按电机参数自动整定（Kp=Ls·ω，Ki=Rs·ω）
- **Rs/Ls 电机参数自动测量**（串口 `id`，2 秒测完直接喂给自整定，ODrive 方案）
- **位置模式梯形轨迹规划**（限速限加速度平滑运动 + 速度前馈，ODrive trap_traj）
- 上电自动完成三相电流零偏校准；编码器 D 轴对齐/Z 脉冲搜索仅在切入
  闭环后由按键或 `c` 命令启动，状态机全程限流保护
- **Flash 参数存储**（串口 `conf write`）：电机参数/PID/校准偏移掉电保存，
  重启自动加载 + 快速索引搜索（免对齐吸附，ODrive `index_search` 语义）
- **八重安全保护体系**：双阈值过流、NaN 防护、堵转保护、纯无感失锁安全停机、参数自检、
  母线欠压/过压保护 (UVLO/OVLO)、板载 NTC 功率级过温保护 (OVERTEMP)、独立看门狗与 TAMP 断电黑匣子（详见 Docs/06）
- **板载 NTC 温度感知与高低速解耦**：PB14 规则组时分复用采集 TDK NTCG163JF103FT1，B 参数方程精准解算，
  超温（>85°C）500ms 保护停机；10Hz STATUS 心跳帧携带实时温度并在上位机 HUD 预警
- **默认单电机，双轴框架完整预留**：算法层零全局状态，`FOC_NUM_AXES` 改 2 即开双轴
- 串口命令行（6.5 Mbaud）在线调模式/目标/PID + FOC-STP 自解释掩码遥测
  （32 通道字典任意订阅、10 Hz 状态心跳、故障事件、CRC16，配套网页上位机
  [foc-studio](https://kyroqu.xyz/foc-studio/)）+ DWT 实测快环 CPU 占用
- 预留/已实现特性：无感磁链观测器+PLL（VESC 纯无感启动与闭环接管）、霍尔传感器驱动、144 点齿槽转矩补偿

> 调研范围除 SimpleFOC / ODrive / VESC / ST MCSDK 外，还包括
> [MESC](https://github.com/davidmolony/MESC_Firmware)（HFI 低速无感）与
> [moteus](https://github.com/mjbots/moteus)（同为 STM32G4 的位置控制标杆）。

## 文档（完整体系；不知道读哪篇先看 [文档地图](Docs/README.md)）

| 文档 | 内容 |
| ---- | ---- |
| [Docs/00_底层原理_从变换到SVPWM与快环数据流.md](Docs/00_底层原理_从变换到SVPWM与快环数据流.md) | 数学推导 + 公式 + 逐行代码：Clarke/Park/SVPWM/采样链/快环/参数映射 |
| [Docs/01_FOC原理入门.md](Docs/01_FOC原理入门.md) | 小白向：FOC 是什么、Clarke/Park、SVPWM、级联环 |
| [Docs/02_工程架构.md](Docs/02_工程架构.md) | 分层设计、接口表、双电机原理、与开源项目对照 |
| [Docs/03_代码走读.md](Docs/03_代码走读.md) | 逐文件讲实现，关键函数逐段解释 |
| [Docs/04_上手指南.md](Docs/04_上手指南.md) | 硬件准备→编译→逐层验证→故障表 |
| [Docs/05_双电机扩展.md](Docs/05_双电机扩展.md) | 开启双轴、虚拟轴演示、接第二套硬件的完整步骤 |
| [Docs/06_稳定性与保护.md](Docs/06_稳定性与保护.md) | 八重保护逐层讲解：过流/NaN/堵转/欠压过压/过温/看门狗/Flash存储/黑匣子 |
| [Docs/07_移植指南.md](Docs/07_移植指南.md) | 换板/换芯片/换传感器/换采样拓扑的逐步清单 |
| [Docs/08_调参与调试手册.md](Docs/08_调参与调试手册.md) | 工具软件用法、适配新电机全流程、PID 逐环调法与波形判读 |
| [Docs/09_预留特性接入手册.md](Docs/09_预留特性接入手册.md) | 无感/HFI/抗齿槽/弱磁/CAN：预留代码怎么一步步接进工程并调试 |
| [Docs/11_FOC-STP遥测协议.md](Docs/11_FOC-STP遥测协议.md) | 遥测协议规格、通道字典、`telem` 命令与三端（C/JS/Python）参考实现 |

## 快速开始

```powershell
# 编译（Keil MDK，ARMCC 5.06；不要用 -j 并行参数）
& 'E:\Keil_v5\UV4\UV4.exe' -b 'MDK-ARM\FOC_G431.uvprojx' -o 'FOC_G431\build_out.txt'
```

烧录后（串口 6.5 Mbaud）：`help` 看命令 → `id` 自动测电机参数（需停机）
→ `id a` 应用 → 按键开环旋转验证 → 再按键停 → `m vel` → 按键校准 →
按键运行 → `t 1000` 转 1000 RPM → 调好后 `save` 固化参数。
详细步骤看 [上手指南](Docs/04_上手指南.md)。

## 代码结构

```
MDK-ARM/Code/foc/
├── Core/    纯算法：foc_motor(轴对象) foc_svm foc_pid foc_traj(轨迹)
│            foc_transform foc_observer(预留无感)
├── Driver/  设备驱动：abz_encoder current_shunt hall_sensor(预留)
├── HAL/     板级绑定：foc_config(用户配置) foc_board_g431(接口表)
└── App/     应用：foc_app(多轴调度) foc_calib foc_ident(参数测量)
             foc_cmd foc_telemetry
```

## 硬件说明

本板电流采样与 ST 官方 B-G431B-ESC1 等效（20mΩ×1.371 = 3mΩ×9.14），
两板固件可互相移植。厂家资料与 MCSDK 参考例程见
`D:\WorkSpace\Project\FOC\Matchstick_HFOC`（原理图 / 用户手册 / 有感无感例程）。
