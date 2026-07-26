# ABZ 增量式编码器驱动设计文档

> **历史文档说明**：本文写于旧版单轴架构时期，原理讲解（采样时序、
> 半量程法、窗口规划等）仍然有效，但文中出现的旧 API 名
> （如 foc_set_pwm / foc_tim_irq / foc_vf_set_voltage / foc_feedback /
> abz_encoder_init 旧签名等）已在对象化重构中被接口表取代。
> 现行接口以 foc_types.h 与《Docs/03_代码走读.md》为准。

## 硬件配置

### 编码器参数

| 参数 | 宏/符号 | 说明 |
| ---- | ---- | ---- |
| 编码器线数 | — | 512 线（可更换） |
| 每转脉冲数 | `ABZ_ENCODER_CPR` | 线数 × 4（4 倍频） |
| 定时器句柄 | `ABZ_ENCODER_TIM_HANDLE` | 默认 htim4 |
| 定时器计数范围 | `ABZ_TIMER_COUNTER_RANGE` | ARR + 1（16 位 = 65536） |
| 回绕判断阈值 | `ABZ_TIMER_HALF_RANGE` | COUNTER_RANGE / 2 |
| 采样频率 | `PWM_FREQ_HZ` | FOC 控制频率（Hz） |

### 定时器配置

- 模式：编码器模式 TI1 和 TI2（双边沿 4 倍频）
- ARR = `ABZ_TIMER_COUNTER_RANGE - 1`（16 位满值，自由运行）
- 预分频 = 0
- 输入滤波 = 15（抗干扰）

**ARR 设为最大值的原因**：无论编码器线数多少，定时器都能自由计数，不需要针对特定编码器调整 ARR。通用性最强。

### Z 相配置

- Z 相信号特征：平时低电平，转到原点时产生短暂高电平脉冲
- GPIO 配置：下拉 + 上升沿触发外部中断
- 中断动作：仅做清零校准，不做复杂运算

## 核心算法思想

### 设计前提

**笃定电机在采样间隔内不可能转半圈以上。**

证明：

- 采样间隔 = 1 / `PWM_FREQ_HZ`
- 半圈对应 `ABZ_ENCODER_CPR / 2` 个脉冲
- 要在一个采样间隔内转半圈，所需转速 = 0.5 × `PWM_FREQ_HZ` × 60 RPM
- 以 16kHz 为例：0.5 × 16000 × 60 = 480,000 RPM，物理上不可能

只要满足这个前提，半量程判断法就是绝对可靠的。

### 编码器计数

- A 超前 B → CNT 自动 +1（正转）
- B 超前 A → CNT 自动 -1（反转）
- CNT 到达 `ABZ_TIMER_COUNTER_RANGE - 1` → 自动回绕到 0
- CNT 到达 0 → 自动回绕到 `ABZ_TIMER_COUNTER_RANGE - 1`

### 定时器回绕处理

CNT 范围 [0, `ABZ_TIMER_COUNTER_RANGE - 1`]，用 `ABZ_TIMER_HALF_RANGE` 作为阈值判断回绕方向：

```c
int32_t delta = (int32_t)CNT_now - (int32_t)CNT_last;

if (delta > ABZ_TIMER_HALF_RANGE)        // 反转跨越 0 边界
    delta -= ABZ_TIMER_COUNTER_RANGE;     // 修正为负增量
else if (delta < -ABZ_TIMER_HALF_RANGE)   // 正转跨越 ARR 边界
    delta += ABZ_TIMER_COUNTER_RANGE;     // 修正为正增量
```

**原理**：正常运转时 |delta| 远小于 `ABZ_TIMER_HALF_RANGE`。如果 |delta| 超过半量程，说明发生了边界回绕，需要修正方向。

### 单圈位置维护

软件维护 `position_cnt`，范围 [0, `ABZ_ENCODER_CPR`)：

```c
position_cnt = (position_cnt + delta) % ABZ_ENCODER_CPR;
if (position_cnt < 0) position_cnt += ABZ_ENCODER_CPR;
```

### 角度和速度计算

```c
angle_rad    = position_cnt * ABZ_RAD_PER_CNT     // = position_cnt × (2π / ABZ_ENCODER_CPR)
velocity_rpm = delta * ABZ_RPM_COEFF              // = delta × (60 × PWM_FREQ_HZ / ABZ_ENCODER_CPR)
```

- `ABZ_RAD_PER_CNT` = 2π / `ABZ_ENCODER_CPR`
- `ABZ_RPM_COEFF` = 60 × `PWM_FREQ_HZ` / `ABZ_ENCODER_CPR`

delta 就是在一个采样周期内真正的脉冲增量，直接乘系数就是速度。
position_cnt 就是转子的单圈绝对位置，直接乘系数就是机械角度。

## Z 相校准策略

### 为什么不用 Z 相做 ARR 限制？

因为 ARR = `ABZ_TIMER_COUNTER_RANGE - 1`，CNT 要增加 `ABZ_TIMER_COUNTER_RANGE` 次才会回绕，
而一圈只有 `ABZ_ENCODER_CPR` 个计数。CNT 不是每转一圈回零，
而是每 `ABZ_TIMER_COUNTER_RANGE / ABZ_ENCODER_CPR` 圈才回绕一次。

### 正确做法：Z 相每圈清零

电机每转一圈，Z 相来一个脉冲，触发中断将 CNT 清零。这样可以防止累积误差。

1. 系统上电时不知道转子位置
2. 电机启动后，第一次捕获到 Z 相中断时，将 CNT 强制写 0
3. 标记"已校准"
4. 后续 Z 相中断可用于丢步检测：如果 Z 触发时 position 不接近 0，说明丢步

### 采样间隔内不可能出现多次 Z 中断

- 控制周期间隔 = 1 / `PWM_FREQ_HZ`
- 即使极端转速下，一个采样间隔内电机转动角度远小于一圈
- 结论：不可能在一个采样间隔内出现多次 Z 中断

## 代码架构

### 初始化

```c
abz_encoder_init(&foc_feedback.angle_rad, &foc_feedback.velocity_rpm);
// 绑定 foc_feedback 字段 → 清零 CNT → 启动编码器模式
```

放在 `foc_app.c` 的 `foc_init()` 里，在 `foc_timer_init()` 之前：

```c
void foc_init(void)
{
    foc_motor_init();
    foc_contr_init(foc_motor_info.pole_pairs, 10.0f);
    foc_vf_set_voltage(2.0f);

#if FOC_LOG_MONITOR
    foc_log_monitor_init();
#endif

    /* 编码器初始化，绑定到 foc_feedback，必须在定时器启动前 */
    abz_encoder_init(&foc_feedback.angle_rad, &foc_feedback.velocity_rpm);

    app_state.state = FOC_STATE_IDLE;
    foc_timer_init();  // TIM1 中断启动后会调用 abz_encoder_update()
}
```

### Z 相外部中断回调

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ABZ_Z_Pin) {
        abz_encoder_set_zero();
        // 将 CNT 强制写 0，重置位置，置位校准标志
    }
}
```

### FOC PWM 中断中周期更新

```c
abz_encoder_update();
// 1. 读取当前 CNT
// 2. delta = CNT_now - CNT_last（含回绕处理）
// 3. position_cnt += delta，取模到 [0, ABZ_ENCODER_CPR)
// 4. angle = position_cnt × ABZ_RAD_PER_CNT
// 5. velocity_raw = delta × ABZ_RPM_COEFF
// 6. velocity = 一阶 IIR 低通滤波（ABZ_VELOCITY_LPF_ALPHA）
```

### 电角度计算（上层使用）

```c
float theta_e = motor_angle * pole_pairs;
theta_e = limit_angle_rad(theta_e);  // 限制到 [0, 2π)
```

## 文件清单

| 文件 | 说明 |
| ---- | ---- |
| `abz_encoder.h` | 接口定义、配置宏、数据结构 |
| `abz_encoder.c` | 驱动实现 |
| `ABZ编码器.md` | 本设计文档 |

## 速度估算方法

### 问题：单次差分法的量化噪声

直接用 `velocity = delta × ABZ_RPM_COEFF` 计算速度时，低速下会出现严重的量化噪声。

原因：CPR = `ABZ_ENCODER_CPR`，采样频率 = `PWM_FREQ_HZ`。低速时每个采样周期内 delta 只有 0 或 ±1，
导致速度输出在 0 和 ±(`ABZ_RPM_COEFF`) 之间跳变，呈现脉冲序列。

例如 468 RPM 时：
- 每秒脉冲数 = 468/60 × 2048 ≈ 16000
- 每个 16kHz 周期平均 1 个脉冲
- delta 序列：`[-1, -1, 0, -1, -1, 0, ...]`
- 速度输出：`[-468, -468, 0, -468, -468, 0, ...]`

角度不受影响，因为 position_cnt 是累积量（积分），量化误差被平滑。

### 当前方案：一阶 IIR 低通滤波 ✓（已实现）

```c
// y[n] = y[n-1] + α × (x[n] - y[n-1])
float velocity_raw = (float)delta * ABZ_RPM_COEFF;
enc.velocity_filtered += ABZ_VELOCITY_LPF_ALPHA * (velocity_raw - enc.velocity_filtered);
*enc.velocity_rpm = enc.velocity_filtered;
```

配置宏：`ABZ_VELOCITY_LPF_ALPHA`，默认 0.05

α 与截止频率的关系：`α ≈ 2π × fc / fs`

| α | 截止频率 fc | 效果 |
| ---- | ---- | ---- |
| 0.01 | ~25 Hz | 非常平滑，延迟大（适合显示/监控） |
| 0.05 | ~127 Hz | 平滑且响应尚可（速度环 1~2kHz 够用） |
| 0.1 | ~255 Hz | 轻度滤波，响应快（高动态场景） |
| 0.2 | ~510 Hz | 几乎不滤 |

优点：
- 实现极简，一行乘加运算
- 可调参数只有一个 α
- 16kHz 运行无额外开销
- 角度输出不受影响（滤波只作用于速度）

缺点：
- 有相位延迟（α 越小延迟越大）
- 不能同时兼顾平滑度和响应速度

适用场景：速度环反馈、一般 FOC 应用。

### 备选方案对比

#### 方案 B：降采样累积（Decimation）

思路：不是每个 16kHz 周期都算速度，而是每 N 个周期累积 delta 总和再算一次。

```c
// 伪代码
static int32_t sum_delta = 0;
static uint16_t dec_cnt = 0;

sum_delta += delta;
if (++dec_cnt >= N) {
    velocity = sum_delta * (ABZ_RPM_COEFF / N);
    sum_delta = 0;
    dec_cnt = 0;
}
```

| N | 速度更新频率 | 低速分辨率提升 |
| ---- | ---- | ---- |
| 8 | 2 kHz | 8× |
| 16 | 1 kHz | 16× |
| 32 | 500 Hz | 32× |

优点：分辨率随 N 线性提升，无相位延迟
缺点：速度更新频率降低，不适合高带宽速度环

#### 方案 C：M/T 法（测频测周结合）

- 高速时用 M 法（固定时间内数脉冲数）
- 低速时用 T 法（测两个脉冲之间的时间间隔）
- 中间速度用 M/T 法结合

优点：全速域精度最好
缺点：实现复杂，可能需要额外定时器资源，切换逻辑容易出 bug

#### 方案 D：PLL 速度观测器（锁相环跟踪）

```
角度误差 = θ_measured - θ_estimated
    → PI 控制器 → 输出 = 估计速度 ω_est
    → 积分器 → 输出 = 估计角度 θ_est → 反馈
```

优点：
- 速度和角度同时平滑
- 可输出插值角度（比编码器分辨率更细）
- 可扩展为二阶观测器输出加速度
- 工业伺服驱动器标准做法

缺点：
- 需要调 PI 参数（Kp、Ki），带宽和阻尼比要匹配电机动态
- PI 参数不对会振荡或跟不上
- 代码量和调试复杂度远高于 IIR

适用场景：高性能伺服、需要角度插值、需要加速度前馈、无感切换。

### 方案选型总结

| 方案 | 复杂度 | 平滑度 | 响应速度 | 适用阶段 |
| ---- | ---- | ---- | ---- | ---- |
| 一阶 IIR ✓ | ★ | ★★★ | ★★★ | 当前（速度环闭环） |
| 降采样累积 | ★★ | ★★★★ | ★★ | 低速精密控制 |
| M/T 法 | ★★★★ | ★★★★★ | ★★★★ | 全速域高精度 |
| PLL 观测器 | ★★★ | ★★★★★ | ★★★★★ | 高性能伺服 |

当前选择一阶 IIR，后续如需更高性能可升级到 PLL，结构上完全兼容。





在"D:\code\mcu\stm32\FOC\FOC_G431\MDK-ARM\Code\foc\Driver\encoder\ABZ编码器.md"这个文档里面完善一下接口，关于abz_encoder是输入和输出的接口定义。
