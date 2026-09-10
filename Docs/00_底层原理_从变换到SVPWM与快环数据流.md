# 00 · 底层原理：从坐标变换到 SVPWM 与 16 kHz 快环数据流

> 本文是本工程**最底层的一篇**：把"三相电流进 → 三相占空比出"这一条 16 kHz
> 快环里每一步的**数学原理、公式推导、对应代码**全部展开，一行行对上。
>
> 与 [01 FOC原理入门](01_FOC原理入门.md) 的关系：01 用直觉讲概念（适合第一次接触），
> 本文用数学和代码讲透（适合想真正吃透、想改代码、想调参的人）。建议先读 01 再读本文。
>
> **阅读约定**：文中所有代码行号均指向 `MDK-ARM/Code/foc` 下的源文件，
> 引用格式为 `文件:行号`。所有公式都是标准电机控制教科书结论 + 本工程代码的
> 直接映射，没有杜撰内容。

---

## 0. 符号表

| 符号 | 含义 | 本工程单位 |
| ---- | ---- | ---- |
| $u,v,w$（或 $a,b,c$） | 三相绕组，文中 $a,b,c$ 与代码一致 | A / V |
| $i_\alpha, i_\beta$ | 静止坐标系两相电流 | A |
| $v_\alpha, v_\beta$ | 静止坐标系两相电压 | V |
| $i_d, i_q$ | 旋转坐标系电流 | A |
| $v_d, v_q$ | 旋转坐标系电压 | V |
| $\theta_e$ | 电角度（电弧度） | rad |
| $\omega_e$ | 电角速度 | rad/s |
| $p$ | 极对数（pole pairs） | 1 |
| $R_s$ | 相电阻（星形等效单相） | Ω |
| $L_s$ | 相电感 | H |
| $\lambda_m$ | 转子永磁磁链幅值 | Wb |
| $U_{dc}$ | 母线电压 | V |
| $T_s$ | 快环周期（$1/16\text{kHz}=62.5\mu s$） | s |

本工程关键常量（`Core/foc_utils.h`）：

```c
#define _PI          3.141592653589793f
#define _2PI         6.283185307179586f
#define SQRT_3       1.732050807568877f   /* √3 */
#define SQRT_3_DIV_2 0.866025403784438f   /* √3/2 */
#define INV_SQRT_3   0.577350269189625f   /* 1/√3 */
#define FOC_RPM_TO_RADS (_2PI / 60.0f)
```

---

## 1. Clarke 变换：三相 → 两相静止（αβ）

### 1.1 为什么要"降维"：三相是冗余的

三相电流 $i_a+i_b+i_c=0$（基尔霍夫电流定律，星形中性点无引出线时），
所以三个交流量其实只有**两个自由度**。Clarke 变换把互差 120° 的三个量
投影到直角坐标系（α 轴与 a 相重合，β 轴超前 α 轴 90°），把"三矢量问题"
变成"二维平面问题"。

### 1.2 等幅值变换的推导

设三相电流为幅值 $I$、频率 $\omega$ 的平衡正弦量：

$$
i_a = I\cos(\omega t),\quad
i_b = I\cos(\omega t - 2\pi/3),\quad
i_c = I\cos(\omega t + 2\pi/3)
$$

**等幅值 Clarke**（变换后合成矢量幅值不变，仍为 $I$）定义为把三相矢量
直接投影到两个正交轴上（α 轴与 a 相重合）：

$$
\alpha = i_a - \frac{1}{2}i_b - \frac{1}{2}i_c
$$

$$
\beta = \frac{\sqrt{3}}{2}i_b - \frac{\sqrt{3}}{2}i_c
$$

利用 $i_a+i_b+i_c=0$ 消去 $i_c$：

$$
\alpha = i_a - \frac{1}{2}(i_b + i_c) = i_a + \frac{1}{2}i_a = \frac{3}{2}i_a
$$

咦，这里出现了系数 3/2？注意：这是**没有归一化**的投影。为了让合成矢量
幅值保持为 $I$，标准等幅值变换矩阵带系数 $2/3$：

$$
\begin{bmatrix} \alpha \\ \beta \end{bmatrix}
= \frac{2}{3}
\begin{bmatrix} 1 & -\frac{1}{2} & -\frac{1}{2} \\ 0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} \end{bmatrix}
\begin{bmatrix} i_a \\ i_b \\ i_c \end{bmatrix}
$$

代入平衡正弦量可验证：

$$
\alpha = \frac{2}{3}\cdot\frac{3}{2} I\cos(\omega t) = I\cos(\omega t)
$$

**但本工程代码不用矩阵，用了一个巧妙的化简**：既然 $i_c = -i_a-i_b$，
把 $i_c$ 代入 $\alpha$ 的投影式（未归一化形式）：

$$
\alpha = i_a - \frac{1}{2}i_b - \frac{1}{2}(-i_a-i_b) = i_a - \frac{1}{2}i_b + \frac{1}{2}i_a + \frac{1}{2}i_b = \frac{3}{2}i_a
$$

再整体乘以 $2/3$，就得到**代码里那两行**：

$$
\boxed{\alpha = i_a,\qquad \beta = \frac{i_a + 2i_b}{\sqrt{3}}}
$$

推导 $\beta$：$\beta = \frac{2}{3}\cdot\frac{\sqrt{3}}{2}(i_b-i_c)
= \frac{1}{\sqrt{3}}(i_b - i_c) = \frac{1}{\sqrt{3}}(i_b + i_a + i_b)
= \frac{i_a + 2i_b}{\sqrt{3}}$。∎

> **为什么 $\alpha=i_a$ 看起来"偷懒"？** 因为归一化系数 $2/3$ 被吸收进了
> $\beta$ 的 $\sqrt{3}$ 分母——数学上这两个式子就是标准等幅值 Clarke 的
> 显式展开，只是省掉了矩阵和一次减法（$i_c$ 的乘法）。

### 1.3 代码逐行（`Core/foc_transform.h:36-40`）

```c
static inline void foc_clarke(const abc_t *abc, ab_t *ab)
{
    ab->alpha = abc->a;
    ab->beta  = (abc->a + (2.0f * abc->b)) * INV_SQRT_3;
}
```

- 第一行直接抄 $\alpha = i_a$；
- 第二行：$(i_a + 2i_b)/\sqrt{3}$，`INV_SQRT_3` 即 $1/\sqrt{3}$。
  两次乘法（`2.0f*`、`*INV_SQRT_3`）比除法快，且不需要知道 $i_c$。

### 1.4 逆 Clarke（`Core/foc_transform.h:48-56`）

由 Clarke 的投影关系反解（把 $\beta$ 分量按 60°/120° 余弦规律分回三相）：

$$
a = \alpha,\qquad
b = -\frac{1}{2}\alpha + \frac{\sqrt{3}}{2}\beta,\qquad
c = -\frac{1}{2}\alpha - \frac{\sqrt{3}}{2}\beta
$$

```c
static inline void foc_inv_clarke(const ab_t *ab, abc_t *abc)
{
    const float half_alpha = -0.5f * ab->alpha;
    const float sqrt3_beta = SQRT_3_DIV_2 * ab->beta;
    abc->a = ab->alpha;
    abc->b = half_alpha + sqrt3_beta;
    abc->c = half_alpha - sqrt3_beta;
}
```

验证：$b+c = -2\cdot\frac{1}{2}\alpha = -\alpha$，所以 $a+b+c=0$，仍然满足
基尔霍夫定律——逆变换自动保持三相和为零。SVPWM 用逆 Clarke 把
$(v_\alpha,v_\beta)$ 变成三相目标电压，见 §3。

### 1.5 等幅值 vs 等功率

- **等幅值**（本工程）：合成矢量幅值 = 相电压幅值，系数 $2/3$。优点：
  SVPWM 的调制比、电压圆半径 $U_{dc}/\sqrt{3}$ 等直观；功率表达式多一个
  $3/2$ 系数（转矩公式 $T_e=\frac{3}{2}p\lambda_m i_q$）。
- **等功率**（系数 $\sqrt{2/3}$）：功率不变，但幅值不再是物理幅值。

本工程统一用等幅值（VESC、SimpleFOC、ST MCSDK 同），全链路一致，
文档里所有公式都按等幅值口径。

---

## 2. Park 变换：静止 → 旋转（dq）

### 2.1 为什么必须转到转子视角

αβ 坐标系固连在**定子**上，转子一转，$i_\alpha,i_\beta$ 仍是交流量，
PI 控制器跟踪交流量有稳态误差（带宽不够则幅值/相位都偏）。Park 变换把
坐标系转到与转子磁场**同步旋转**的位置，使电流变成直流量，PI 才能
无静差跟踪。

### 2.2 旋转矩阵的推导

设 d 轴与转子 N 极对齐，与 α 轴夹角为电角度 $\theta_e$。把矢量
$(\alpha,\beta)$ 投影到旋转坐标轴上：

- d 轴方向单位矢量：$(\cos\theta_e,\ \sin\theta_e)$
- q 轴方向单位矢量：$(-\sin\theta_e,\ \cos\theta_e)$（q 轴超前 d 轴 90°）

投影（点积）：

$$
d = \alpha\cos\theta_e + \beta\sin\theta_e
$$

$$
q = -\alpha\sin\theta_e + \beta\cos\theta_e
$$

写成矩阵就是标准旋转矩阵 $\mathbf{R}(\theta_e)$。

### 2.3 代码逐行（`Core/foc_transform.h:63-67`）

```c
static inline void foc_park(const ab_t *ab, float sin_th, float cos_th, dq_t *dq)
{
    dq->d = (ab->alpha * cos_th) + (ab->beta * sin_th);
    dq->q = (-ab->alpha * sin_th) + (ab->beta * cos_th);
}
```

注意参数：调用方传入**预先算好的 `sin_th`/`cos_th`**（`foc_motor.c:861-862`
用 `foc_sin(m->theta_e)`/`foc_cos(m->theta_e)` 算一次），四个三角函数调用
省成两次——16 kHz 下每拍省 4 次 sin/cos，是实打实的周期预算。

### 2.4 逆 Park（`Core/foc_transform.h:74-78`）

旋转矩阵的逆 = 转置（正交矩阵）：

$$
\alpha = d\cos\theta_e - q\sin\theta_e,\qquad
\beta = d\sin\theta_e + q\cos\theta_e
$$

```c
static inline void foc_inv_park(const dq_t *dq, float sin_th, float cos_th, ab_t *ab)
{
    ab->alpha = (dq->d * cos_th) - (dq->q * sin_th);
    ab->beta  = (dq->d * sin_th) + (dq->q * cos_th);
}
```

逆 Park 用在快环第 7 步：电流环输出 $(v_d,v_q)$ → 反变换回 $(v_\alpha,v_\beta)$
→ 交给 SVPWM。

### 2.5 dq 的物理意义：为什么 Iq 产生转矩

PMSM 在 dq 坐标系下的转矩方程（等幅值约定）：

$$
T_e = \frac{3}{2}p\left[\lambda_m i_q + (L_d - L_q)i_d i_q\right]
$$

对表贴式（$L_d\approx L_q$，本工程 DJI 2312S 属此类）简化为：

$$
\boxed{T_e = \frac{3}{2}p\,\lambda_m\, i_q}
$$

所以：
- $i_d=0$（控制目标，本工程 `id_ref` 默认 0）：不产生转矩，只建立磁场/耗损；
- $i_q$：**唯一控制转矩的自由度**，速度环输出就是 $i_q$ 给定。

### 2.6 电角度从哪来：$\theta_e = dir\cdot\theta_m\cdot p + offset$

Park 用的 $\theta_e$ 不是编码器机械角，而是电角度（`foc_angle_manager.c:290-293`）：

```c
float we = (float)m->calib.direction * m->velocity_observer_rpm * FOC_RPM_TO_RADS * m->params.pole_pairs;
float th_enc = foc_wrap_0_2pi(
    foc_wrap_0_2pi(((float)m->calib.direction * enc_mech_rad * m->params.pole_pairs) + m->calib.electrical_offset_rad) +
    (m->runtime.angle_delay_cycles * we * m->dt_fast));
```

- `direction`：编码器正方向与磁场正方向是否一致（本电机实测 `-1`，
  即编码器正转时电角度递减，见 `foc_config.h:220` 注释的 V/F 反向对称实验）；
- `electrical_offset_rad`：校准得到的"编码器读数 ↔ 电角度"偏移（在线 `calib offset` 查/设）；
- 最后一项 `angle_delay_cycles * we * dt_fast`：**角度超前补偿**——PWM 是
  上一拍的角度写进比较器、本拍才生效（一个周期延迟），按速度把角度
  往前推 `0.5` 拍（`FOC_M0_ANGLE_DELAY_CYCLES=0.5`）抵消采样/执行延迟。

---

## 3. SVPWM：把电压矢量变成三相占空比

### 3.1 逆变器的 8 种开关状态

三相半桥各两态（上通=1/下通=0），共 8 种组合。以 $U_{dc}$ 母线、各相
相对中性点电压记：

| 状态 | a | b | c | $v_a$ | $v_b$ | $v_c$ | 空间矢量 |
| ---- | - | - | - | ----- | ----- | ----- | -------- |
| U0 | 0 | 0 | 0 | 0 | 0 | 0 | 零矢量 |
| U1 | 1 | 0 | 0 | $2U_{dc}/3$ | $-U_{dc}/3$ | $-U_{dc}/3$ | $100$ |
| U2 | 1 | 1 | 0 | $U_{dc}/3$ | $U_{dc}/3$ | $-2U_{dc}/3$ | $110$ |
| U3 | 0 | 1 | 0 | $-U_{dc}/3$ | $2U_{dc}/3$ | $-U_{dc}/3$ | $010$ |
| U4 | 0 | 1 | 1 | $-2U_{dc}/3$ | $U_{dc}/3$ | $U_{dc}/3$ | $011$ |
| U5 | 0 | 0 | 1 | $-U_{dc}/3$ | $-U_{dc}/3$ | $2U_{dc}/3$ | $001$ |
| U6 | 1 | 0 | 1 | $U_{dc}/3$ | $-2U_{dc}/3$ | $U_{dc}/3$ | $101$ |
| U7 | 1 | 1 | 1 | 0 | 0 | 0 | 零矢量 |

6 个非零矢量幅值都是 $2U_{dc}/3$，指向平面内相隔 60° 的 6 个方向；
2 个零矢量（000/111）三相电位相同，线电压为 0——它们就是低边三电阻
采样的"黄金窗口"（见 §4）。

### 3.2 最大线性调制区：为什么是内切圆 $U_{dc}/\sqrt{3}$

6 个非零矢量顶点连成正六边形。要输出**幅值恒定、方向连续旋转**的
电压矢量，各方向能力必须一致，所以只能取**六边形内切圆**。

内切圆半径推导：六边形边长为 $2U_{dc}/3$（相邻矢量夹角 60°），
内切圆半径 = 边长 × $\cos 30° = \frac{2U_{dc}}{3}\cdot\frac{\sqrt{3}}{2} = \frac{U_{dc}}{\sqrt{3}}$。

这就是本工程三处 `u_dc * INV_SQRT_3`（电压圆限幅、电流环输出限幅、
`vq` 命令上限）的出处。**超过内切圆 = 过调制**，波形开始失真。

### 3.3 伏秒平衡：任意矢量怎么合成

目标矢量 $\vec{V}_{ref}$ 落在某扇区内，用该扇区两个相邻基础矢量
$\vec{V}_x,\vec{V}_y$ 和零矢量，按时间加权：

$$
\vec{V}_{ref}\cdot T_s = \vec{V}_x\cdot T_x + \vec{V}_y\cdot T_y + \vec{V}_z\cdot T_z,\qquad
T_x+T_y+T_z = T_s
$$

以扇区 1（$\vec{V}_1=100$、$\vec{V}_2=110$ 之间，夹角 60°）为例解出作用时间。
设 $\vec{V}_{ref}$ 幅值 $V_m$、与 $\vec{V}_1$ 夹角 $\theta$（$0\le\theta\le 60°$）：

在 $\vec{V}_1$ 方向投影：

$$
V_m\cos\theta = \frac{2}{3}U_{dc}\frac{T_1}{T_s} + \frac{2}{3}U_{dc}\cos 60°\frac{T_2}{T_s}
= \frac{2}{3}U_{dc}\frac{T_1}{T_s} + \frac{1}{3}U_{dc}\frac{T_2}{T_s}
$$

在 $\vec{V}_2$ 方向投影：

$$
V_m\sin\theta = \frac{2}{3}U_{dc}\sin 60°\frac{T_2}{T_s} = \frac{\sqrt{3}}{3}U_{dc}\frac{T_2}{T_s}
$$

解得：

$$
T_1 = \frac{\sqrt{3}\,T_s\,V_m}{U_{dc}}\sin\left(\frac{\pi}{3}-\theta\right),\qquad
T_2 = \frac{\sqrt{3}\,T_s\,V_m}{U_{dc}}\sin\theta,\qquad
T_0 = T_s - T_1 - T_2
$$

教科书 SVPWM 就是：判扇区 → 算 $T_1,T_2$ → 按扇区映射到三相比较值 → 插入
零矢量（七段式在周期中对称插入 000/111 减小开关次数与谐波）。

### 3.4 本工程为什么不查扇区表：中点注入法

`Core/foc_svm.h:5-13` 的注释说得很明白：本工程与 VESC、SimpleFOC 一样，
用**反 Clarke + 中点电压（零序）注入**，数学上与七段式 SVPWM 完全等价。

推导等价性。对目标电压矢量反 Clarke 得到三相目标相电压 $v_a,v_b,v_c$
（线电压只由两两之差决定）。注入任意共模电压 $v_{cm}$ 不影响线电压
（三相同时加减），于是**选择 $v_{cm}$ 让三相电压对称分布在 0~$U_{dc}$ 窗口中央**：

$$
v_{cm} = -\frac{\max(v_a,v_b,v_c) + \min(v_a,v_b,v_c)}{2}
$$

这样 $\max(v_x+v_{cm}) = -\min(v_x+v_{cm})$，三相占空比

$$
\boxed{d_x = \frac{1}{2} + \frac{v_x + v_{cm}}{U_{dc}}}
$$

自动以 0.5 为中心对称——这正是七段式在周期中点对称插入零矢量的效果。
**零序注入把相电压波形变成马鞍形（三次谐波叠加），使相电压峰值
降到 $U_{dc}/2$ 而线电压仍可达 $U_{dc}/\sqrt{3}$，比纯正弦 SPWM 的
$U_{dc}/2$ 上限提升 $\frac{U_{dc}/\sqrt{3}}{U_{dc}/2}-1 = \frac{2}{\sqrt{3}}-1 \approx 15.47\%$。**
这就是"SVPWM 比 SPWM 母线利用率高 15.47%"的来源，也是
`foc_svm.h` 注释里"比纯正弦调制多 15.5%"这句话的完整推导。

> 扇区号还保留着，但**只用于采样窗口规划与调试观测**（`foc_svm.c:9` 注释），
> 不参与占空比计算——占空比由上面的公式直接得出。

### 3.5 过调制保护：等比压缩（`foc_svm.c:50-58`）

当电压指令超出线性区（$|\vec{V}|>U_{dc}/\sqrt{3}$），本工程不截断单相，
而是**整体等比缩小三相摆幅**：

```c
span = v_max - v_min;
if (span > u_dc) {
    const float scale = u_dc / span;
    v.a *= scale;  v.b *= scale;  v.c *= scale;
    v_max *= scale;  v_min *= scale;
}
```

物理含义：相间摆幅 $span$ 是 PWM 必须容纳的总量，超过母线电压 $U_{dc}$
就按比例压缩——**保持电压矢量方向不变、幅值降到母线允许值**，
与电压圆限幅（`foc_voltage_circle_limit`，§6.3）互为冗余保护。

### 3.6 占空比钳位 0.06~0.94：采样窗口的硬约束（`foc_svm.c:70-72`）

```c
out->duty_a = foc_clampf(0.5f + ((v.a + v_common) * inv_udc), 0.060f, 0.940f);
```

钳位下限 0.06 / 上限 0.94 不是随便定的，对应 ST MCSDK 的
`MAX_MODULATION_100_PER_CENT` 思路：**保证任意时刻至少有 6% 周期是
下桥全通**，给三电阻采样留出 >1.8 µs 的干净低边窗口。

换算验证：中心对齐计数下完整 PWM 周期 = $2\times ARR = 10624$ count = 62.5 µs
（170 MHz）。占空比钳到 94% 意味着**每半周期（5312 count）至少有
$0.06\times5312\approx319$ tick 的连续下桥导通窗口**（$\approx1.88\,\mu s$；
整周期合计低边 3.75 µs），大于采样所需的 $T_{AFTER}=297$ tick
（= 1.747 µs，见 §4.2），余量足够。这就是调制限取 94% 而非 100% 的原因——
ST MCSDK 的 MAX_MODULATION_100_PER_CENT 同理。
这也是为什么高调制比时采样对要切换到非零扇区对（§4.3）——窗口始终存在。

### 3.7 扇区判定：$N=4C+2B+A$（`foc_svm.c:11-29`）

教科书扇区判定：三条直线把平面分成 6 个 60° 扇区，用三条判别式的
正负组合编码：

$$
u_1 = \beta,\qquad
u_2 = \sqrt{3}\,\alpha - \beta,\qquad
u_3 = -\sqrt{3}\,\alpha - \beta
$$

```c
static uint8_t foc_svm_sector(const ab_t *ab)
{
    const float u1 = ab->beta;
    const float u2 = (SQRT_3 * ab->alpha) - ab->beta;
    const float u3 = (-SQRT_3 * ab->alpha) - ab->beta;
    const uint8_t code = (uint8_t)((u1 > 0.0f) |
                                   ((u2 > 0.0f) << 1) |
                                   ((u3 > 0.0f) << 2));
    switch (code) {
    case 3U:  return 1U;   /* 011 -> 扇区1 */
    case 1U:  return 2U;   /* 001 -> 扇区2 */
    case 5U:  return 3U;   /* 101 -> 扇区3 */
    case 4U:  return 4U;   /* 100 -> 扇区4 */
    case 6U:  return 5U;   /* 110 -> 扇区5 */
    case 2U:  return 6U;   /* 010 -> 扇区6 */
    default:  return 0U;
    }
}
```

几何含义：三条线分别是 $\beta=0$（α 轴）、$\sqrt{3}\alpha-\beta=0$
（+60° 线）、$-\sqrt{3}\alpha-\beta=0$（-60° 线），它们的正负组合
唯一确定矢量所在 60° 扇区。`code` 的位组合 $4C+2B+A$ 对应
`u3`（高位）、`u2`、`u1`（低位），查表映射到 1~6。

### 3.8 `foc_svm_calc` 全函数逐行（`foc_svm.c:31-73`）

```c
void foc_svm_calc(const ab_t *v_ab, float u_dc, foc_svm_t *out)
{
    abc_t v;  float v_max, v_min, span, v_common, inv_udc;

    out->sector = foc_svm_sector(v_ab);          /* ① 扇区（采样窗口用） */

    foc_inv_clarke(v_ab, &v);                    /* ② 逆 Clarke: αβ -> abc */

    v_max = foc_max3(v.a, v.b, v.c);             /* ③ 找三相极值 */
    v_min = foc_min3(v.a, v.b, v.c);
    span = v_max - v_min;
    if (span > u_dc) { ... 等比压缩 ... }        /* ④ 过调制保护 */

    v_common = -0.5f * (v_max + v_min);          /* ⑤ 中点注入 */
    inv_udc = 1.0f / u_dc;

    out->duty_a = foc_clampf(0.5f + ((v.a + v_common) * inv_udc), 0.060f, 0.940f);
    out->duty_b = foc_clampf(...);
    out->duty_c = foc_clampf(...);               /* ⑥ 三相占空比 + 钳位 */
}
```

**数据流**：$(v_\alpha,v_\beta)\xrightarrow{\text{逆Clarke}}(v_a,v_b,v_c)
\xrightarrow{\text{中点注入+钳位}}(d_a,d_b,d_c)$，全程没有查表、没有除法
（`1/u_dc` 只算一次），是 $\le 30$ 行的极简实现。

---

## 4. 电流采样链：数字世界怎么"看见"电流

### 4.1 三电阻拓扑与采样时机

本板用**低边三电阻采样**（`Driver/current/current_shunt.c`，下称 cs 模块）：
三相各接一个 $0.02\,\Omega$ 采样电阻，经 OPAMP（PGA×16）放大后进 ADC。
要采某一相电流，必须在**该相下桥导通、且流过采样电阻**的时刻去采样——
也就是该相 PWM 为低的时间窗内。

- 零矢量 000（三下桥全通）期间：三相电流都流经下桥，是采三相的"黄金窗口"；
- 非零矢量期间：只有被钳位为低的那一相可采，能采的相组合随扇区变化。

### 4.2 采样窗口参数（`current_shunt.c:53-54`）

```c
#define CURRENT_SAMPLE_TBEFORE 41U
#define CURRENT_SAMPLE_TAFTER  297U
```

以中心对齐计数（ARR=5312）理解：
- 下桥开始导通后 **$T_{AFTER}=297$ tick**（$\approx 1.75\,\mu s$）再采：避开
  开关振铃和续流电流建立过程；
- 下桥结束前 **$T_{BEFORE}=41$ tick** 采完：留出 ADC 转换与注入组开销。

297 tick × (1/170 MHz) ≈ 1.747 µs，对齐 ST MCSDK 的 TW_AFTER 口径
（死区 750 ns + 开关振铃 1000 ns = 1747 ns = 297 tick，`current_shunt.c:44-52`
注释原文）。这就是 §3.6 说">1.8 µs 低边窗口"的设计依据（0.94 钳位给出
每半周期 319 tick ≈ 1.88 µs 连续窗口，略大于 297+裕量）。

### 4.3 采样对选择：UV / UW / VW 轮换（`current_shunt.c:825-897`）

`current_shunt_prepare_pwm()` 在每个 PWM 周期由 `set_compare` 调用
（`foc_board_g431.c:162`），按**三相占空比的最大值**决定本周期采哪两相：

```c
if ((PWM_CNT - max_duty) > CURRENT_SAMPLE_TAFTER) {
    pending_pair = CURRENT_PAIR_UV;        /* 中点附近低边时间足够 → UV */
} else if (...) {
    pending_pair = CURRENT_PAIR_VW;        /* 否则按 max_duty 归属选 VW/UW */
}
sampling_point = max_duty - CURRENT_SAMPLE_TBEFORE;   /* 上升沿前 */
/* 或 */
sampling_point = max_duty + CURRENT_SAMPLE_TAFTER;    /* 下降沿后 */
```

逻辑：**永远选"低边时间最长"的两相来采**，采样触发点放在
`max_duty` 两侧（低边窗口的中段），保证窗口足够 $T_{AFTER}$。`sector`
参数仅为 FOC 接口保留（`current_shunt.c:822` 注释：实际路由按 CCR 排序决定）。

### 4.4 第三相重建（`current_shunt.c:184-227`）

采到两相后，第三相用基尔霍夫定律补齐：

```c
case CURRENT_PAIR_UV:
    u = ((float)offset_u - (float)adc1_raw) * CURRENT_AMPS_PER_COUNT;
    v = ((float)offset_v - (float)adc2_raw) * CURRENT_AMPS_PER_COUNT;
    w = -(u + v);                       /* Iu+Iv+Iw=0 */
    break;
```

### 4.5 换算系数（`current_shunt.c:32-37`）

```c
#define CURRENT_SHUNT_OHM       0.02f      /* 采样电阻 */
#define CURRENT_EFFECTIVE_GAIN  1.367f     /* OPAMP PGA16 后的等效增益（含分压网络） */
#define CURRENT_AMPS_PER_COUNT  (3.3f / 4095.0f / (CURRENT_SHUNT_OHM * CURRENT_EFFECTIVE_GAIN))
```

推导：ADC 满量程 4095 count = 3.3 V；采样电阻电压 $= I \times 0.02$；
OPAMP 放大后 $= I \times 0.02 \times 1.367$。所以每 count 对应电流：

$$
\frac{3.3\,\text{V}}{4095 \times 0.02\,\Omega \times 1.367}
\approx 29.5\,\text{mA/count}
$$

`offset - raw` 是因为低边采样时电流方向与 ADC 参考方向相反（电流流进
采样电阻使 OPAMP 输出降低，零偏减当前读数即为电流）。

### 4.6 零偏校准（`current_shunt.c:411+`）

上电（PWM 关闭、三下桥导通）时采 1024 样本求每相零偏（分 UV / VW 两段），
`CURRENT_OFFSET_SAMPLES=1024`、最小计数门限 2048（`current_shunt.c:40-42`）
用于判链路是否异常（如 OPAMP 未起振）。

### 4.7 ADC 注入组触发机制

- ADC1/ADC2 各用**注入组（injected）**双上下文队列，硬件由 TIM1 的
  `TRGO` 事件（中心对齐周期更新）触发，软件在 `TIM1_UP_IRQHandler`
  （更新中断）里装载下一拍的 JSQR 上下文（`current_shunt.c:753-815`）；
- **ADC2 的 JEOS（注入序列结束）中断是控制节拍源**（`current_shunt.c:393`），
  它一来就代表本拍电流已就绪，中断里调用 `foc_motor_fast_loop()`；
- 有完善的故障路径：ADC 校准超时、上下文残留（JSQR 非空）、OPAMP 启动失败
  都会进 `current_shunt_fail()` 并给出诊断码（`fault` 命令可查）。

> 这解释了"为什么 PWM 更新中断（16 kHz）是唯一节拍"：采样完成 → JEOS 中断
> → 快环。电流环天然与 PWM 中心对齐、与采样点同步，延迟最小。

---

## 5. 16 kHz 快环完整数据流：逐行对照

### 5.1 节拍来源

TIM1 中心对齐计数，ARR=5312（`foc_config.h:53`），170 MHz 计数时钟：

$$
f_{PWM} = \frac{170\,\text{MHz}}{5312 \times 2} \approx 16.0\,\text{kHz},\qquad
T_s = 62.5\,\mu s
$$

（中心对齐一个完整周期计数值从 0 升到 ARR 再降回 0，所以 ×2。）
`FOC_DT_FAST = 1/16000`（`foc_config.h:34`）就是快环的 $T_s$。

### 5.2 快环 13 步（`Core/foc_motor.c:798-991`）

| 步 | 代码行 | 做什么 | 公式/说明 |
| -- | ------ | ------ | --------- |
| 1 | 806-819 | 传感器更新 | 编码器 `update()` → 机械角、速度、位置积分 |
| 2 | 834-847 | 读电流 + 硬过流保护 | `cur->get()`；单拍超 $12\,\text{A}$ 立即 `foc_motor_check_hard_current` 跳闸 |
| 3 | 849-859 | 角度选择 | CALIB/VF/开环 → 开环角度累加；闭环 → `foc_angle_mgr_update(m)`（无感/有感统一入口） |
| 4 | 861-866 | **Clarke + Park** | $i_{abc}\to i_{\alpha\beta}\to i_{dq}$，见 §1/§2 |
| 5 | 868-875 | 无感观测器逐拍更新 | 条件 `obs_enabled || obs_blending`，用**上一拍电压** `v_ab_last` 与本拍电流（观测器详情待无感定型后补写） |
| 6 | 876-881 | dq 电流滤波 + 软过流 | 一阶 LPF（遥测用）；$|i_{dq}|$ 持续超 5.2/6.6 A 连续 8 拍跳闸 |
| 7 | 889-893 | 慢环分频 | `slow_cnt % 16` → 1 kHz 速度/位置环（`foc_motor_slow_loop`） |
| 8 | 896-934 | 电压指令 | 开环 `v_openloop`；闭环：电流 PID + 动态限幅 + dq 解耦（见 §6） |
| 9 | 936-940 | NaN 防护 | `v_dq` 出现 NaN → `FOC_FAULT_CONTROL_NAN` |
| 10 | 942 | 电压圆限幅 | $|v_{dq}| \le U_{dc}/\sqrt{3}$（`foc_voltage_circle_limit`） |
| 11 | 944-966 | **逆 Park + 死区补偿** | $(v_d,v_q)\to(v_\alpha,v_\beta)$；按相电流符号回补 $0.17\,\text{V}$ 死区压降 |
| 12 | 968-974 | 无感调试台更新 | `foc_sensorless_bench_update()` 用真实施加电压对比 4 种观测器（调试台 §7） |
| 13 | 975-990 | **SVPWM + 写比较器 + 电压重建** | `foc_svm_calc` → `set_compare` → 由占空比反算 $v_{ab\_last}$ 供下拍观测器 |

### 5.3 电压重建：$v_{ab\_last}$ 从哪来（`foc_motor.c:983-990`）

观测器需要"**上一拍真实施加到电机上的电压**"。本工程不是直接用 PID
输出，而是用**实际写入的占空比 + 母线电压反推**（更接近真实端电压）：

```c
float udc_third = m->drv->u_dc * 0.33333333f;
float da = m->svm.duty_a, db = m->svm.duty_b, dc = m->svm.duty_c;
m->v_ab_last.alpha = udc_third * ((2.0f * da) - db - dc);
m->v_ab_last.beta  = m->drv->u_dc * 0.57735027f * (db - dc);
```

推导：三相端电压 $v_x = (2d_x-1)\cdot U_{dc}/2$（相对中点），
$\alpha\beta$ 分量经 Clarke：$v_\alpha = v_a$（等幅值展开后为
$\frac{U_{dc}}{3}(2d_a-d_b-d_c)$），$v_\beta = \frac{U_{dc}}{\sqrt{3}}(d_b-d_c)$。
`0.57735027` 即 $1/\sqrt{3}$。这消除了 PWM 装载延迟（$z^{-1}$）带来的
模型失真（`foc_motor.c:969` 注释：与采样电流严格同拍对齐）。

### 5.4 时间预算

16 kHz = 62.5 µs 内完成上述 13 步。实测记录：无感观测器单次约 5 µs
（`foc_motor.c:869` 注释），快环其余部分（变换 + 两个 PID + SVPWM）远小于
10 µs，余量充足；32 kHz 配置时观测器改隔拍执行（同注释）。

---

## 6. 底层参数映射：配置里的数字从哪来

### 6.1 电流环整定：$K_p = L_s\omega_c,\ K_i = R_s\omega_c$（零极点对消）

PMSM 电压方程（dq，稳态近似）：

$$
v_d = R_s i_d + L_s\frac{di_d}{dt} - \omega_e L_s i_q,\qquad
v_q = R_s i_q + L_s\frac{di_q}{dt} + \omega_e L_s i_d + \omega_e\lambda_m
$$

把交叉项 $\omega_e L_s i$ 和反电动势 $\omega_e\lambda_m$ 用解耦前馈抵消后，
剩下一阶对象 $G(s)=\frac{1}{R_s + sL_s}$。PI 控制器
$C(s)=K_p + \frac{K_i}{s}$，若取：

$$
K_p = L_s\omega_c,\qquad K_i = R_s\omega_c
$$

则开环传递函数 $C(s)G(s) = \frac{\omega_c}{s}$——**对象极点被 PI 零点精确
对消，闭环变成一阶低通，带宽就是 $\omega_c$**（单位 rad/s）。

代码出处（`foc_motor.c:704-708`）：

```c
v_max = drv->u_dc * INV_SQRT_3;
kp_i = params->ls_henry * cfg->current_bw_rads;
ki_i = params->rs_ohm * cfg->current_bw_rads;
foc_pid_init(&m->pid_id, kp_i, ki_i, 0.0f, v_max, 0.0f);
foc_pid_init(&m->pid_iq, kp_i, ki_i, 0.0f, v_max, 0.0f);
```

### 6.2 本板实例（`foc_config.h:96-97,108`）

当前基线 $R_s=0.100\,\Omega,\ L_s=20\,\mu\text{H},\ \omega_c=2000\,\text{rad/s}$：

$$
K_p = 20\times10^{-6}\times 2000 = 0.0400\,\text{V/A},\qquad
K_i = 0.100\times 2000 = 200\,\text{V/(A·s)}
$$

`foc_config.h:90-95` 的注释给出了这段历史的完整理由：0.1 Ω/20 µH 是
"长路验证通过的物理稳定参数"，能把电流环跑到 9000 RPM 极速；
若把辨识出的视在电感 336 µH 直接当设计参数，解耦项 $\omega_e L_s$ 会放大
16.8 倍，高速时产生巨大交叉扰动引发过流。**观测器/磁链提取用视在电感
（实测 301/333/336 µH 三次一致），电流环设计用动态等效电感（20 µH）**——
两个用途参数不同，不要混用（`foc_motor.c:739-743` 注释记录了用 80 µH
喂观测器导致 iq=1.5 A 时反电动势被 $\Delta L\cdot i$ 淹没而失步的教训）。

### 6.3 电压圆限幅（`foc_motor.c:942` 调用的 `foc_voltage_circle_limit`）

电流环输出、开环 vq 命令都要过电压圆：$v_d^2+v_q^2 \le (U_{dc}/\sqrt{3})^2$。
超出时按比例缩放到圆上，方向不变——与 §3.5 的 SVPWM 等比压缩形成
双保险（一个在 dq 域、一个在 abc 域）。

### 6.4 PID 内部实现（`Core/foc_pid.c:15-71`）

```c
float foc_pid_update(foc_pid_t *pid, float error, float dt)
{
    float proportional = pid->kp * error;                       /* P */

    /* Tustin 积分：integral += Ki·dt/2·(e[k] + e[k-1]) */
    float integral = pid->integral +
                     (pid->ki * dt * 0.5f * (error + pid->prev_error));

    /* 积分项单独钳位：防长期饱和退不出来 */
    if (pid->out_limit > 0.0f)
        integral = foc_clampf(integral, -pid->out_limit, pid->out_limit);

    float output_unclamped = proportional + integral + derivative;

    /* 条件抗饱和：输出已超限且误差同向 → 撤销本拍积分（能迅速退饱和） */
    if ((pid->out_limit > 0.0f) &&
        (((output_unclamped > pid->out_limit) && (error > 0.0f)) ||
         ((output_unclamped < -pid->out_limit) && (error < 0.0f)))) {
        integral = integral_prev;
        output_unclamped = proportional + integral + derivative;
    }

    output = foc_clampf(output_unclamped, -pid->out_limit, pid->out_limit);

    /* 输出斜坡：限制 d(output)/dt */
    if (pid->out_ramp > 0.0f) { ... }
}
```

三层结构：
1. **积分项独立钳位**（`pid->out_limit`）——单靠输出钳位时积分还会继续
   累积（windup），先钳积分；
2. **条件抗饱和**——误差与输出超限同向说明积分在把控制器"推得更深"，
   撤销本拍积分；反向误差仍允许积分，保证快速退出饱和（
   `foc_pid.c:41-52` 注释）；
3. **输出斜坡**（`out_ramp`）——速度环用（`FOC_VEL_IQ_RAMP_A_S`），
   限制 Iq 给定变化率，防阶跃冲击。

### 6.5 电流环限幅与母线动态同源（P0 修复，`foc_motor.c:909-913`）

```c
float v_limit = m->drv->u_dc * INV_SQRT_3;
foc_pid_set_limit(&m->pid_id, v_limit);
foc_pid_set_limit(&m->pid_iq, v_limit);
```

快环每拍先用**实时母线电压**刷新 PID 限幅（`drv->u_dc` 由慢环 100 Hz 采样
更新，`FOC_VBUS_AUTO_UPDATE_DRV=1`），再进电流环。母线塌陷时积分钳位和
条件抗饱和立即随物理电压收缩，杜绝"电压恢复瞬间积分残留 → 超高占空比
脉冲"的浪涌过流（08-31 实测 14 A 硬件跳闸的根因）。

### 6.6 死区补偿 $0.17\,\text{V}$（`foc_config.h:159-162`）

死区时间内上下管均关断，相电流流过续流二极管，等效平均电压损失：

$$
V_{loss} = U_{dc}\times t_{dead}\times f_{PWM}
= 14.4\,\text{V} \times 740\,\text{ns} \times 16\,\text{kHz}
\approx 0.17\,\text{V}
$$

方向与相电流相反。快环按相电流符号回补（`foc_motor.c:950-966`，
阈值 0.05 A 防过零抖振），无感观测器还另有一份独立死区补偿
（独立死区补偿在 `foc_sensorless_bench.c:479-499`）。

---

## 7. 往上一层：1 kHz 慢环（速度环 / 位置环 / 弱磁）

快环每 16 拍调一次慢环（`foc_motor.c:889-893` 的 `slow_cnt % 16`，
`foc_motor_slow_loop` 在 `foc_motor.c:236-560`）。慢环周期：

$$
T_{slow} = T_s \times slow\_div = 62.5\,\mu s \times 16 = 1\,\text{ms}
$$

慢环的职责：速度环/位置环 PI + 前馈，输出 $i_q$ 给定给快环电流环。
因为机械时间常数远大于电气时间常数，1 kHz 的慢环 + 16 kHz 的快环
已足够（ODrive/VESC 同款调度）。

### 7.1 速度反馈：编码器差分 + 双速率滤波混合（`foc_motor.c:242-292`）

编码器速度 `velocity_observer_rpm`（差分+平滑）先过两个一阶 LPF：

- `vel_filter`（快，`FOC_VEL_FILTER_MIN_HZ=15` Hz 起）——高速用，延迟小；
- `vel_filter_low`（低，`FOC_VEL_LOW_FILTER_HZ=15` Hz）——低速用，噪声小；
- 按**给定速度幅值**在 50~100 rpm 区间线性混合
  （`FOC_VEL_FILTER_BLEND_START_RPM/END_RPM`，`foc_motor.c:286-292`）。

为什么按给定而不是按实测混合：给定是用户命令、无噪声；实测在低速时
抖动大，若按它选滤波会来回切换产生电流毛刺。

### 7.2 速度环结构（`foc_motor.c:294-491`）

速度模式的完整组成（由慢到快三层）：

1. **目标斜坡**（`foc_motor.c:300-309`）：`vel_ramp_rpm_s` 限速率，
   $\omega_{ref}[k{+}1]=\operatorname{clamp}(\omega_{tgt},\ \omega_{ref}[k]\pm ramp\cdot dt)$
   ——避免阶跃目标直接变成电流冲击；
2. **速度 PI**（`foc_motor.c:407-410`）：$e = \omega_{ref}-\omega_{filt}$，
   `foc_pid_update(&m->pid_vel, e, dt)`，输出限幅 = `max_iq_sat`
   （受弱磁动态电流圆约束，见 7.4）；零速且转子静止时清积分、iq_ref=0
   （`foc_motor.c:387-395`），防止静摩擦把残留积分"冻住"；
3. **低速位置跟踪 + 速度阻尼**（`foc_motor.c:316-343, 401-422`）：
   低速（低于 `vel_track_rpm`）时把"目标速度积分"当作位置参考，
   用位置误差构成**移动位置控制器 + 速度 PD 阻尼**（`iq_track_damp =
   track_blend · Kp_vel · Δω`），随速度升高平滑交还给纯速度 PI。
   ——这正是 09-06 修复 30 RPM 极限环的落点：纯速度 PI 是欠阻尼弹簧，
   外转子齿槽激励下振荡（实测 -7~+55 RPM）；叠加位置跟踪和速度阻尼后
   低速波动从 57 RPM 降到 34 RPM（`foc_motor.c:327-343` 注释）。

**启动摩擦前馈**（`foc_motor.c:433-483`）：新启动/反转时先给
`boost = vel_start_a - vel_friction_a` 的**一次性挣脱电流**，
只在实测速度达到 `vel_start_rpm` 且连续 50 拍后才释放并单调衰减
（`FOC_VEL_START_RELEASE_TICKS=50`）。方向变化时重新武装；
齿槽脉动不会反复触发（释放后有迟滞），消除了此前 300 RPM 极限环的
一阶因素。

### 7.3 位置环：位置 PI + 梯形轨迹前馈 + 速度阻尼（`foc_motor.c:493-551`）

- `target` 是相对使能原点 `pos_origin_rad` 的偏移，位置参考
  $pos_{ref} = pos_{origin} + target$（`foc_motor.c:495`）；
- 目标变化时用**梯形轨迹**重新规划（`foc_traj_plan`/`foc_traj_eval`，
  ODrive trap_traj 方案，`foc_motor.c:504-525`）：每拍输出平滑位置参考
  `pos_r` 和**速度前馈** `vel_ff`；
- 位置 PI 直接输出电流（`foc_motor.c:542`），误差过零时清积分
  （`foc_motor.c:539-541`）——否则过零瞬间残留积分继续推转子，
  低惯量外转子会形成低频极限环（齿槽 + 保持电流的经典组合）；
- 速度阻尼项：$i_{damp} = pos\_vel\_kp\cdot(\omega_{ff}-\omega_{enc})$，
  限幅 `FOC_POS_DAMP_CURRENT_RATIO × Imax`（`foc_motor.c:543-547`）。
  轨迹前馈负责"跟上参考"，阻尼负责"不振荡"。

### 7.4 弱磁与动态电流圆（`foc_motor.c:347-385`）

高速时端电压利用率逼近极限（88% $U_{dc}/\sqrt{3}$ 目标，
`fieldweak_voltage_ratio`），反电动势占满电压后 Iq 无法再提速，需要
**负 Id 去磁**：

$$
v_{err} = v_{target} - |v_{dq}|,\qquad
id_{ref} = \operatorname{clamp}\!\Big(id_{ref} + K_{fw}\cdot v_{err}\cdot dt,\ id_{min},\ 0\Big)
$$

电压不足（$v_{err}<0$）时积分器输出负 Id；同时 Iq 限幅收窄为
**动态电流圆**（`foc_motor.c:371-377`）：

$$
i_{q,max} = \sqrt{I_{max}^2 - i_{d,ref}^2}
$$

保证合成电流幅值不超热极限。弱磁默认关闭
（`runtime.fieldweak_enable`），相关配置与启用方法见 08 篇调参手册。

---

## 8. 从底层到上层：全景图

```
                    ┌──────────────────────────────────────────────┐
 三相电流 ──► ADC 注入组(16kHz) ──► 第三相重建 ──► i_a,i_b,i_c     │
                    │                                                  │
                    ▼                                                  │
              [Clarke]  αβ      [Park]  dq        （§1/§2）            │
              i_α,i_β ────────► i_d,i_q                                │
                    │          （θe 由角度管理器提供）                    │
                    ▼                                                  │
              电流环 PID(id/iq)  +  动态限幅 + dq解耦      （§6.1/6.5）   │
                    │                                                  │
                    ▼                                                  │
              v_d,v_q ──► 电压圆限幅 ──► [逆Park] v_α,v_β              │
                    │            （§6.3）                               │
                    ▼                                                  │
              [SVPWM] 中点注入+钳位  ──► duty_a/b/c ──► CCR1/2/3      （§3）
                    │                                                  │
                    └──► 由占空比重建 v_ab_last ──► 下拍观测器/调试台    （§5.3）
```

每一环的"物理量 → 公式 → 代码"：

| 环节 | 物理量 | 核心公式 | 代码位置 |
| ---- | ------ | -------- | -------- |
| 采样 | 三相电流 | $i_c = -i_a-i_b$ | `current_shunt.c:184-227` |
| Clarke | αβ 电流 | $\alpha=i_a,\ \beta=(i_a+2i_b)/\sqrt3$ | `foc_transform.h:36-40` |
| Park | dq 电流 | 旋转矩阵投影 | `foc_transform.h:63-67` |
| 电流环 | dq 电压 | PI + 抗饱和 + 解耦 | `foc_pid.c:15-71`, `foc_motor.c:915-930` |
| 电压圆 | 限幅 | $|v|\le U_{dc}/\sqrt3$ | `foc_motor.c:942` |
| 逆 Park | αβ 电压 | 转置旋转矩阵 | `foc_transform.h:74-78` |
| SVPWM | 占空比 | $d_x=0.5+(v_x+v_{cm})/U_{dc}$ | `foc_svm.c:31-73` |
| PWM | 比较值 | $CCR_x = d_x\times ARR$ | `foc_board_g431.c:168-170` |
| 电压重建 | αβ 电压 | 由占空比反推 | `foc_motor.c:983-990` |

---

## 附：常见误区

1. **"Clarke 输出是直流"** —— 错。Clarke 只降维，输出仍是交流；Park 才变直流。
2. **"SVPWM 需要查扇区表算 Tx/Ty"** —— 教科书如此，但中点注入法数学等价且
   只需 max/min 和一次除法，本工程即后者。
3. **"占空比钳位 6%/94% 是为了保护 MOSFET"** —— 主因是保证三电阻采样窗口
   （>1.8 µs 低边时间），同时兼作调制限制。
4. **"电流环 Kp/Ki 随便调"** —— 本工程按 $L_s\omega_c/R_s\omega_c$ 自整定，
   改带宽只需 `current bw` 命令，越界（100~3000 rad/s）会被 CLI 拒绝。
5. **"反电动势/解耦项用实测大电感更准"** —— 电流环设计参数用动态等效电感
   （20 µH），观测器模型用视在电感（336 µH），两者用途不同（见 §6.2）
   
