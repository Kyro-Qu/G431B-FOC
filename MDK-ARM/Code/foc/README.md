# FOC 代码架构设计

当前代码已经具备基础分层：

- `App/`：应用入口、运行状态、控制模式管理。
- `Core/`：V/f 开环控制、坐标变换、SVPWM/SPWM。
- `Driver/`：ABZ 增量编码器与三电阻相电流采样驱动。
- `HAL/`：板级参数、PWM 输出以及电流/角度接口。
- `../vofa/`：调试数据上传，当前被 `Core` 直接调用。

## 1. 架构目标

FOC 架构应优先满足以下目标：

- **Core 与硬件解耦**：Clarke/Park、PI、SVPWM、状态量更新不直接包含 `main.h`、`stm32g4xx.h`、定时器句柄或 UART。
- **板级移植集中**：换 MCU、换定时器、换 ADC、换编码器时，主要改 `HAL/` 与 `Driver/`，不改控制算法。
- **控制链路清晰**：快环中断只做确定性计算，慢环和调试输出通过分频或主循环执行。
- **接口稳定**：App 只通过公开 API 设置目标、切换状态；Core 只通过抽象接口读取电流/角度并输出 PWM 占空比。
- **配置可分层**：电机参数、控制参数、板级外设参数分开，避免所有宏和句柄堆在一个头文件。

## 2. 当前代码职责

```text
foc/
+-- App/
|   +-- foc_app.c/.h          # foc_init、运行状态、控制模式、传感器类型
+-- Core/
|   +-- foc_controller.c/.h   # V/f 开环、定时器中断入口、角度推进、foc_feedback
|   +-- foc_math.c/.h         # Clarke/Park、角度归一化、SVPWM/SPWM
+-- Driver/
|   +-- encoder/
|   |   +-- abz_encoder.c/.h  # TIM 编码器模式读取 ABZ 角度/速度
|   |   +-- ABZ编码器.md      # 编码器驱动设计文档
|   +-- current/
|       +-- current_shunt.c/.h # 三电阻、双 ADC 注入采样、零偏校准与三相重构
+-- HAL/
    +-- foc_config.c/.h       # PWM、母线电压、电机参数、弱电流/角度接口
```

### 2.1 关键全局变量

| 变量 | 类型 | 定义位置 | 说明 |
| ---- | ---- | ---- | ---- |
| `foc_motor_info` | `foc_motor_info_t` | `HAL/foc_config.c` | 电机参数（极对数、Rs、Ls 等） |
| `foc_log_monitor` | `foc_log_monitor_t` | `HAL/foc_config.c` | 调试监视器指针集合 |
| `vf` | `foc_vf_state_t` | `Core/foc_controller.c` | V/f 开环状态（电角度、dq、ab） |
| `foc_feedback` | `foc_feedback_t` | `Core/foc_controller.c` | 传感器反馈（机械角度、转速） |

`foc_feedback_t` 是传感器的统一输出接口：

```c
// Core/foc_controller.h
typedef struct {
    float angle_rad;     /* 机械角度 [0, 2π)，由传感器驱动写入 */
    float velocity_rpm;  /* 机械转速 RPM，由传感器驱动写入 */
} foc_feedback_t;

extern foc_feedback_t foc_feedback;
```

编码器驱动通过指针绑定直接写入 `foc_feedback` 的字段，零拷贝、零开销：

```c
// App/foc_app.c → foc_init()
abz_encoder_init(&foc_feedback.angle_rad, &foc_feedback.velocity_rpm);
```

未来换传感器（霍尔、SPI 磁编码器、无感估算），只需换 init 调用，`foc_feedback` 接口不变。

### 2.2 当前初始化链路

```text
main.c
  -> MX_xxx_Init()          // CubeMX 生成的外设初始化（TIM4 编码器模式等）
  -> foc_init()
      -> foc_motor_init()           // 填充电机参数
      -> foc_contr_init()           // V/f 开环初始化
      -> foc_vf_set_voltage()       // 设置初始 Vq
      -> foc_log_monitor_init()     // 绑定监视器指针
      -> abz_encoder_init()         // 编码器初始化（绑定 foc_feedback，启动 TIM4）
      -> current_shunt_init()       // 启动 OPAMP、校准并使能 ADC 注入组
      -> foc_timer_init()           // 保持 TIM1 CH4 和 ADC 以 16 kHz 运行
      -> current_shunt_calibrate()  // U/V 各 1024 点，再采 W 1024 点
```

**关键顺序**：三相电流零偏校准成功前 TIM1 MOE 保持关闭；采样故障会立即清除 MOE 并进入 `FOC_STATE_FAULT`，编码器校准不会启动。

上电默认保持 `FOC_STATE_IDLE` 和功率输出关闭，以便先观察零偏与静态电流。第一次按键启动编码器校准；校准成功回到 IDLE 后，下一次按键才进入 RUN。

### 2.3 当前运行链路

```text
HAL_TIM_PeriodElapsedCallback(TIM1)  [每个完整 PWM 周期]
  -> current_shunt_tim_update_irq()
      -> 暂停 TIM1_TRGO
      -> 切换 OPAMP3 内/外部输出
      -> 提交 ADC1/ADC2 单 Rank JSQR
      -> 恢复 OC4REF 触发

ADC1_2_IRQHandler(ADC2 JEOS)         [16 kHz]
  -> current_shunt_adc_irq()
      -> 读取 ADC1/ADC2 JDR1
      -> 换算两相电流并由 Iu + Iv + Iw = 0 重构第三相
  -> foc_tim_irq()
      -> abz_encoder_update()       // 读 CNT → 更新 foc_feedback.angle_rad / velocity_rpm
      -> RUN 状态判断
      -> 开环角度推进（或闭环时读 foc_feedback）
      -> inverse_park_transform()
      -> svpwm_calc()
      -> foc_set_pwm()
      -> VOFA_Task() 分频上传

HAL_GPIO_EXTI_Callback(ABZ_Z_Pin)   [每转一次]
  -> abz_encoder_set_zero()         // Z 相归零校准，防累积误差
```

### 2.4 三电阻电流采样

- OPAMP1/2/3 使用 PGA IO0 bias、片内增益 x16 和工厂 trimming。
- ADC1/ADC2 为独立注入组，由 TIM1_TRGO 同时触发，12 位右对齐，采样时间 6.5 cycles。
- 换算系数为 `I = (offset - raw) * 0.0294755233 A/count`，理论零偏约 2545 counts。
- U 最大时采 V/W，V 最大时采 U/W，W 最大时采 U/V；通道路由依据 CCR1/2/3 的实际排序，不依赖扇区名称。
- 正常采样点为 `ARR - 1`；窄窗口采用 `Tbefore=41`、`Tafter=298` 调整 CCR4，跨越计数峰值时改用下降沿。
- `g_current_shunt_diag` 可在 Keil Watch 中查看 raw、offset、安培值、active/pending 组合、扇区、样本计数和故障标志。
- 当前阶段只提供可靠采集、重构和诊断，不包含 Id/Iq PI 电流闭环或软件过流阈值。

VOFA JustFloat 共 12 个通道：

| 通道 | 数据 |
| ---- | ---- |
| 0 | 开环电角度 |
| 1 | 编码器机械角度 |
| 2 | 编码器速度 RPM |
| 3 | ABZ Z/index 中断计数 |
| 4 | 编码器校准状态 |
| 5～7 | U/V/W PWM 等效相电压 |
| 8～10 | U/V/W 相电流 A |
| 11 | 电流采样状态与组合，值为 `state * 10 + pair` |

### 2.5 编码器数据流

```text
TIM4 硬件计数器 (CNT)
    │
    ▼
abz_encoder_update()          ← 16kHz PWM 中断中调用
    │  delta = CNT_now - CNT_last（半量程回绕处理）
    │  position_cnt += delta，取模到 [0, ABZ_ENCODER_CPR)
    │
    ├─→ foc_feedback.angle_rad    = position_cnt × ABZ_RAD_PER_CNT
    └─→ foc_feedback.velocity_rpm = delta × ABZ_RPM_COEFF
              │
              ▼
        控制器使用：
        theta_e = foc_feedback.angle_rad × pole_pairs  （电角度）
        速度环输入 = foc_feedback.velocity_rpm
```

这条链路适合当前 V/f 开环验证，但若要做闭环 FOC 和移植，需要进一步拆分依赖。

## 3. 推荐目标分层

```text
App
  - 状态机、启停、故障处理、目标命令、调试接口

Core
  - 控制器对象、快环/慢环调度、电流环、速度环、位置环、V/f 开环
  - 只依赖 foc_types、foc_math、抽象 board/sensor/current 接口

Math / Modulation
  - Clarke/Park、反 Park、限幅、角度归一化、SVPWM/SPWM
  - 输入电压矢量，输出占空比，不直接写硬件

Driver
  - 编码器、霍尔、无感估算、电流采样重构、母线电压采样
  - 可以依赖具体 MCU HAL，但对 Core 提供统一接口

HAL / Board
  - PWM 定时器、ADC 触发、DMA、硬件保护、板级句柄绑定
  - 每块板或每个 MCU 工程独立实现
```

建议逐步演进为以下目录：

```text
foc/
+-- App/
|   +-- foc_app.c/.h
|   +-- foc_state_machine.c/.h       # 可选：更完整状态机
+-- Core/
|   +-- foc_types.h                  # 通用结构体、枚举、错误码
|   +-- foc_controller.c/.h          # 控制器对象与快环入口
|   +-- foc_loop.c/.h                # 电流环/速度环/位置环，可后续拆出
|   +-- foc_math.c/.h                # 纯数学
|   +-- foc_modulation.c/.h          # SVPWM/SPWM，输出 duty
+-- Driver/
|   +-- encoder/
|   |   +-- encoder_interface.h
|   |   +-- encoder_abz.c/.h
|   |   +-- encoder_hall.c/.h
|   |   +-- encoder_spi.c/.h
|   +-- current/
|       +-- current_interface.h
|       +-- current_shunt.c/.h
+-- HAL/
    +-- foc_board.h                  # 板级抽象接口
    +-- foc_board_stm32g431.c/.h     # 当前板级实现
    +-- foc_config.h                 # 用户配置与编译开关
    +-- foc_motor_params.c/.h        # 电机参数
```

## 4. 核心接口设计

### 4.1 通用数据类型

通用类型建议集中到 `Core/foc_types.h`，避免 `foc_math.h`、`foc_config.h`、`foc_app.h` 之间互相前向声明。

```c
typedef struct {
    float a;
    float b;
    float c;
} foc_abc_t;

typedef struct {
    float alpha;
    float beta;
} foc_ab_t;

typedef struct {
    float d;
    float q;
} foc_dq_t;

typedef struct {
    float a;
    float b;
    float c;
} foc_pwm_duty_t;   /* 建议范围 0.0f ~ 1.0f，HAL 再换算为 CCR */
```

### 4.2 板级接口

`Core` 不应直接知道 `htim1`、`TIM_CHANNEL_1` 或 `__HAL_TIM_SET_COMPARE()`。建议由 `HAL/foc_board.h` 提供稳定接口：

```c
typedef struct {
    void (*pwm_start)(void);
    void (*pwm_stop)(void);
    void (*pwm_set_duty)(float duty_a, float duty_b, float duty_c);
    void (*get_phase_current)(float *ia, float *ib, float *ic);
    float (*get_bus_voltage)(void);
    float (*get_electrical_angle)(void);
    void (*fault_shutdown)(void);
} foc_board_if_t;
```

当前 `foc_timer_init()`、`foc_pwm_enable()`、`foc_pwm_disable()`、`foc_set_pwm()`、`foc_get_currents()`、`foc_get_electrical_angle()` 可以作为第一阶段接口保留，但建议后续收敛为接口表，便于多板卡复用。

### 4.3 传感器接口

当前实现采用**指针绑定模式**：编码器驱动通过 `abz_encoder_init()` 接收外部变量指针，
`abz_encoder_update()` 每次直接写入绑定的变量。这种方式零开销、零拷贝，适合单一传感器场景。

当前实际接口：

```c
// 初始化时绑定输出变量
abz_encoder_init(&foc_feedback.angle_rad, &foc_feedback.velocity_rpm);

// 16kHz 中断中调用，自动刷新 foc_feedback
abz_encoder_update();

// Z 相中断中调用
abz_encoder_set_zero();

// 查询校准状态
uint8_t abz_encoder_is_calibrated(void);
```

未来若需支持多种传感器动态切换，可扩展为函数指针表：

```c
typedef struct {
    void (*init)(void);
    void (*deinit)(void);
    void (*update)(void);
    float (*get_mech_angle_rad)(void);
    float (*get_electrical_angle_rad)(float pole_pairs);
    float (*get_velocity_rpm)(void);
    uint8_t (*is_ready)(void);
} foc_sensor_if_t;
```

但当前阶段指针绑定模式已经足够，不需要过度抽象。

### 4.4 调制输出接口

当前 `svpwm_calc()` 在 `foc_math.c` 中直接调用 `foc_set_pwm()`，这会让数学层依赖硬件输出。建议改为：

```c
foc_pwm_duty_t foc_svpwm_calc(const foc_ab_t *v_ab, float u_dc);
```

然后在 `Core` 中：

```c
foc_pwm_duty_t duty = foc_svpwm_calc(&v_ab, board->get_bus_voltage());
board->pwm_set_duty(duty.a, duty.b, duty.c);
```

这样可以在 PC 端或单元测试中验证 SVPWM，不需要真实定时器。

## 5. 控制任务调度

推荐把控制任务分为三类。

### 5.1 快环中断

执行频率通常等于 PWM 频率，目前为 16 kHz。

职责：

- 读取电流采样或重构相电流。
- 读取转子角度，或在开环模式下推进虚拟角度。
- 执行 Clarke/Park。
- 执行 Id/Iq 电流 PI。
- 执行反 Park 与 SVPWM。
- 刷新 PWM。
- 执行硬实时保护，例如过流、欠压、角度失效保护。

建议接口命名：

```c
void foc_fast_loop_irq(void);
```

当前对应 `foc_tim_irq()`。

### 5.2 中速控制任务

频率建议为 0.5 kHz ~ 2 kHz，可通过快环分频实现。

职责：

- 速度估算滤波。
- 速度环 PI，输出 `Iq_ref`。
- 位置环，输出 `speed_ref`。
- 电压/电流限幅与斜坡。

### 5.3 慢速应用任务

主循环或 RTOS 任务执行，频率通常为 10 Hz ~ 200 Hz。

职责：

- 状态机。
- 启停命令。
- 参数下发。
- VOFA/串口/上位机通信。
- 故障上报与清故障。

调试输出不要直接耦合在 `Core` 快环中。当前 `foc_controller.c` 直接调用 `VOFA_Task()`，建议后续改为 App 或 Monitor 模块通过分频读取观测量。

## 6. 状态机建议

当前状态只有 `IDLE`、`RUN`、`FAULT`，可以满足开环验证。闭环 FOC 建议扩展为：

```text
IDLE
  -> INIT
  -> CALIB_OFFSET       # ADC 电流零偏校准
  -> ALIGN              # 转子预定位/编码器零点校准
  -> OPEN_LOOP_RAMP     # 可选：无感或编码器闭环前的开环爬升
  -> CLOSED_LOOP
  -> FAULT
```

状态机原则：

- `IDLE` 和 `FAULT` 必须关闭 PWM 或输出安全占空比。
- 只有进入 `RUN/CLOSED_LOOP` 后才允许快环刷新有效 PWM。
- 故障状态只允许显式清除，不应自动恢复。
- App 负责状态跳转，Core 只执行控制步骤并返回错误码或状态标志。

## 7. 控制模式扩展路径

当前 `foc_ctrl_mode_t` 已预留模式，建议按以下顺序实现：

```text
FOC_CTRL_VF_OPENLOOP
  - 当前已有：虚拟电角度 + Vq 给定 + SVPWM

FOC_CTRL_TORQUE_IQ
  - 需要：电流采样、Park、Id/Iq PI、角度传感器或估算器

FOC_CTRL_SPEED
  - 需要：速度估算、速度 PI、Iq_ref 限幅、反积分饱和

FOC_CTRL_POSITION
  - 需要：机械角度、多圈计数、位置环、轨迹规划
```

建议控制器内部保留统一目标量：

```c
typedef struct {
    float vd;
    float vq;
    float id_ref;
    float iq_ref;
    float speed_rpm_ref;
    float position_rad_ref;
} foc_target_t;
```

不同模式只消费需要的字段，避免每个模式都定义一套互相割裂的 API。

## 8. 配置拆分建议

当前 `HAL/foc_config.h` 同时包含 `main.h`、PWM 参数、电机参数结构体、监视器结构体、HAL 函数声明。为了移植，建议拆分：

- `foc_config.h`：编译开关、控制频率、默认限幅、是否启用日志。
- `foc_motor_params.h/.c`：电机极对数、Rs、Ls、Ke、最大电流、最大转速。
- `foc_board.h`：板级接口声明。
- `foc_board_stm32g431.c/.h`：`htim1`、ADC、TIM4、UART 等 STM32G431 绑定。
- `foc_monitor.h/.c`：VOFA 或其他观测数据源，不放在 Core 内。

移植时只替换板级实现和参数文件，Core 文件应保持不变。

## 9. 移植清单

迁移到新板卡或新 MCU 时，按以下顺序处理：

1. 确认 PWM 频率、计数周期、中心对齐/边沿对齐、死区时间、互补输出、刹车输入。
2. 实现或适配 `pwm_start()`、`pwm_stop()`、`pwm_set_duty()`，并确认 A/B/C 三相通道顺序。
3. 配置 ADC 触发点、采样窗口、DMA，并实现 `get_phase_current()` 或电流重构模块。
4. 配置角度来源：ABZ、霍尔、SPI 磁编码器或无感估算，实现统一传感器接口。
5. 校准相电流符号、编码器方向、电角度零点和极对数。
6. 填写电机参数与保护阈值：母线电压、最大电流、最大转速、Rs、Ls、Ke。
7. 验证 `PWM_CNT`、`PWM_FREQ_HZ` 与实际定时器配置一致。
8. 若不使用 CMSIS-DSP，替换 `arm_sin_f32()`、`arm_cos_f32()` 或在配置中选择标准库实现。
9. 先跑 `FOC_CTRL_VF_OPENLOOP` 验证相序和 PWM，再进入电流闭环。
10. 最后接入 VOFA/串口日志，避免调试输出阻塞快环。

## 10. 当前重构优先级

建议按以下顺序重构，风险最低：

1. **修正依赖方向**：`Core` 不再包含 `../App/foc_app.h` 和 `../../vofa/vofa.h`，状态判断与监控分频移到 App 或调度层。
2. **拆出调制层**：让 `svpwm_calc()` 返回占空比或 CCR 结果，不在 `foc_math.c` 中直接调用 `foc_set_pwm()`。
3. **隔离 STM32 头文件**：`foc_math.h` 不包含 `stm32g4xx.h`，CMSIS-DSP 通过配置开关控制。
4. **拆分 `foc_config.h`**：把 `main.h`、`htim1`、电机参数、日志结构和板级函数分离。
5. **统一传感器接口**：让 ABZ、霍尔、SPI 编码器、无感估算都输出同样的机械角、电角度和速度。
6. **补齐电流采样链路**：实现电流偏置校准、采样同步、双/三电阻重构，再进入 `TORQUE_IQ`。
7. **完善保护机制**：过流、过压/欠压、角度丢失、堵转、温度等故障统一进入 `FAULT`。

## 11. 文件归属规则

为了保持可移植性，后续新增代码建议遵守：

- 出现 `TIM_HandleTypeDef`、`ADC_HandleTypeDef`、`HAL_`、`__HAL_` 的代码只能放在 `HAL/` 或具体 `Driver/`。
- 出现控制公式、坐标变换、PI、SVPWM 的代码放在 `Core/`。
- 出现启停、按键、串口命令、上位机协议、状态机的代码放在 `App/` 或独立监控模块。
- `Core/` 可以调用抽象接口，但不直接调用 CubeMX 生成的函数。
- 所有硬实时函数禁止阻塞式串口发送、动态内存分配和长时间循环等待。
