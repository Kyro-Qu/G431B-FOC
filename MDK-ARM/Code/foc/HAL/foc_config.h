/**
 * @file    foc_config.h
 * @brief   FOC 工程用户配置（唯一需要按板卡/电机修改的头文件）
 *
 * 分三块：
 *   1. 轴数量与调度频率
 *   2. 板级参数（PWM / 母线电压）
 *   3. 电机参数与控制环默认值（每轴一组）
 *
 * 本文件只放宏与常量，不放代码、不包含硬件头文件，
 * 因此 Core 层与 PC 端单元测试都可以安全包含。
 */

#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* ======================== 1. 轴与调度 ======================== */

/**
 * 电机轴数量：1 或 2。本板只有一套功率级，默认 1（单电机）。
 * 双轴框架已完整预留：改成 2 即启用轴 1 ——
 *   默认的轴 1 是"虚拟轴"（无功率级/传感器，仅开环数学运行），
 *   用于演示与验证双轴调度框架；接入真实第二套功率板时按
 *   《Docs/05_双电机扩展.md》替换 foc_board_g431.c 中的 M1 绑定。
 */
#ifndef FOC_NUM_AXES
#define FOC_NUM_AXES            1
#endif

/** PWM / 快环频率 Hz（TIM1 中心对齐，ARR 与之对应） */
#define FOC_PWM_FREQ_HZ         16000.0f

/** 快环周期 s */
#define FOC_DT_FAST             (1.0f / FOC_PWM_FREQ_HZ)

/** 慢环（速度/位置环）分频：16 kHz / 16 = 1 kHz */
#define FOC_SLOW_DIV            16U

/** 遥测分频：16 kHz / 16 = 1 kHz 发送一帧 VOFA */
#define FOC_TELEMETRY_DIV       16U

/** 上电默认是否开启 VOFA 遥测流 */
#define FOC_TELEMETRY_DEFAULT_ON 1

/* ======================== 2. 板级参数（轴 0） ======================== */

/** TIM1 自动重装载值 ARR：170 MHz / 5312 / 2(中心对齐) ≈ 16 kHz
 *  （CKD=DIV2 只影响死区/滤波时钟 tDTS，不分频计数器时钟） */
#define FOC_PWM_ARR             5312U

/** 母线电压 V（4S 锂电标称。接可调电源时改这里） */
#define FOC_UDC_V               14.23f

/* ======================== 3. 电机 0 参数（DJI 2312S 实测） ======================== */

#define FOC_M0_POLE_PAIRS       6.0f
#define FOC_M0_RS_OHM           0.1f       /* 相电阻 Ω */
#define FOC_M0_LS_H             0.00002f   /* 相电感 H（20 µH） */
#define FOC_M0_KE               0.9f       /* 反电动势常数（备用） */
#define FOC_M0_MAX_CURRENT_A    5.2f       /* 软件电流限制（连续超限跳闸） */
/* 硬限刻意高于软限（基线版软硬同为 5.2A，8 拍容忍逻辑形同虚设）：
 * 软限 5.2A×8 拍容忍采样毛刺，硬限 6.5A 单拍针对真实短路/失控，
 * 远低于功率级承受能力 */
#define FOC_M0_HARD_CURRENT_A   6.5f       /* 硬电流限制（单拍即跳闸） */
#define FOC_M0_MAX_RPM          12450.0f

/* ---- 电机 0 控制环默认值 ---- */

/** 电流环带宽 rad/s：Kp=Ls·ω, Ki=Rs·ω。1000 rad/s ≈ 160 Hz，
 *  是 ODrive 的默认值，对绝大多数电机都是安全起点 */
#define FOC_M0_CURRENT_BW_RADS  1000.0f

/** 速度环 PI（输入 RPM 误差 → 输出 Iq 给定 A） */
#define FOC_M0_VEL_KP           0.005f     /* A/RPM */
#define FOC_M0_VEL_KI           0.02f      /* A/(RPM·s) */
#define FOC_M0_VEL_RAMP_RPM_S   2000.0f    /* 目标速度斜坡 */
#define FOC_M0_VEL_LPF_TF       0.005f     /* 速度反馈低通 5 ms */

/** 位置环 P（输入 rad 误差 → 输出速度给定 RPM） */
#define FOC_M0_POS_KP           60.0f      /* RPM/rad */
#define FOC_M0_POS_VEL_LIMIT    1000.0f    /* 位置模式速度上限 RPM */

/** 位置模式梯形轨迹（限速限加速度的平滑运动，ODrive trap_traj） */
#define FOC_M0_TRAJ_ENABLE      1
#define FOC_M0_TRAJ_ACC_RPM_S   4000.0f    /* 轨迹加/减速度 */

/** dq 解耦前馈（ω·L·i 交叉项补偿），低感电机低速时影响小，默认开 */
#define FOC_M0_DECOUPLE         1

/** 死区补偿电压 V（0 = 关闭）。理论值 = Udc×t_dead×f_pwm
 *  ≈ 14.23×740ns×16kHz ≈ 0.17V。默认关闭；上机看电流过零处
 *  有平顶畸变时再从 0.1 开始逐步加（08 篇有判读方法） */
#define FOC_M0_DEADTIME_COMP_V  0.0f

/** 堵转保护（仅速度/位置模式生效；力矩模式堵转是正常工况）：
 *  |Iq给定| ≥ 95% 限流 且 |转速| < 阈值 持续超时 → FOC_FAULT_STALL */
#define FOC_M0_STALL_ENABLE     1
#define FOC_M0_STALL_RPM        30.0f      /* 低于此转速视为"没在转" */
#define FOC_M0_STALL_TIMEOUT_MS 1000U

/* ======================== 3.5 系统级稳定性 ======================== */

/** 独立看门狗 IWDG（LSI 32kHz 时钟，与主时钟无关）：
 *  主循环挂死超过超时时间即硬件复位，TAMP 黑匣子会留下现场。
 *  调试器断点不会误触发（已配置 DBGMCU 冻结）。上限 4095ms */
#define FOC_WATCHDOG_ENABLE     1
#define FOC_WATCHDOG_TIMEOUT_MS 400U

/** Flash 参数存储（ODrive save_configuration 思路）：
 *  串口 `save` 把电机参数/控制环参数/校准偏移写入最后一页 Flash，
 *  上电自动加载；有存储偏移时校准走"快速索引搜索"（免对齐吸附）。 */
#define FOC_STORE_ENABLE        1

/* ---- 编码器（轴 0，TIM4 ABZ） ---- */

/** 4 倍频后的每转计数（512 线 × 4） */
#define FOC_M0_ENCODER_CPR      2048U

/* ======================== 4. 上电校准参数（轴 0） ======================== */
/* 电压模式校准，务必从小电压开始调试。流程见 App/foc_calib.c */

#define FOC_CALIB_ALIGN_VOLTAGE     0.30f   /* D 轴对齐电压 V */
#define FOC_CALIB_SEARCH_VOLTAGE    0.30f   /* 找 Z 脉冲的开环旋转电压 V */
#define FOC_CALIB_VOLTAGE_RAMP_MS   200U    /* 电压缓升时间，防电流阶跃 */
#define FOC_CALIB_BOOTSTRAP_MS      10U     /* 自举电容充电时间 */
#define FOC_CALIB_NEUTRAL_MS        2000U   /* 中点 PWM 观察时间（沿用已验证基线值） */
#define FOC_CALIB_ALIGN_MS          800U    /* D 轴对齐保持时间 */
#define FOC_CALIB_SETTLE_MS         20U     /* 强制清零后的等待 */
#define FOC_CALIB_SEARCH_RPM        20.0f   /* 找 Z 的开环转速 */
#define FOC_CALIB_SEARCH_TIMEOUT_MS 10000U  /* 找 Z 超时 → FAULT */
#define FOC_CALIB_ALIGN_THETA_E     0.0f    /* 对齐用电角度 rad */
#define FOC_CALIB_DIRECTION         (-1)    /* 编码器方向（本板实测为 -1） */
#define FOC_CALIB_CURRENT_LIMIT_A   1.5f    /* 校准期软电流限制 */
#define FOC_CALIB_HARD_LIMIT_A      3.0f    /* 校准期硬电流限制 */

/* ======================== 5. 按键行为 ======================== */

/**
 * 1：按键在未校准时先启动校准，校准完成后再按进入 RUN（闭环流程）
 * 0：按键直接以当前模式进入 RUN（开环调试流程）
 */
#define FOC_KEY_STARTS_CALIB    1

#endif /* FOC_CONFIG_H */
