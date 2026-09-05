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

/** 遥测分频：16 kHz / 32 = 500 Hz（VLink CDC 桥 68KB/s 吞吐上限
 *  边缘抖动会吞帧，500Hz 留 2 倍余量——2026-09-03 断流诊断） */
#define FOC_TELEMETRY_DIV       32U

/** 上电默认是否开启 VOFA 遥测流（0=关闭，串口纯命令行模式） */
#define FOC_TELEMETRY_DEFAULT_ON 0

/* ======================== 2. 板级参数（轴 0） ======================== */

/** TIM1 自动重装载值 ARR：170 MHz / 5312 / 2(中心对齐) ≈ 16 kHz
 *  （提频探索记录：64k 失败——TIM1 update ISR(装填注入上下文)超
 *  15.6µs 周期，中断风暴饿死主循环触发 IWDG 复位循环；若改 32k
 *  (ARR 2656) 需同步重验电流环 guardrail 与采样窗口。
 *  CKD=DIV2 只影响死区/滤波时钟 tDTS，不分频计数器时钟） */
#define FOC_PWM_ARR             5312U

/** 母线电压 V（4S 锂电标称。接可调电源时改这里） */
#define FOC_UDC_V               14.4f      /* 当前电机额定/实接母线电压 */

/* ======================== 3. 电机 0 参数（DJI 2312S 实测） ======================== */

/* 7 对极（12N14P 外转子）：2026-09-05 vf100 比值实验实测 85.0/100=0.850
 * （pp=7 预期 0.857，pp=6 预期 1.0）。历史工程 POLE_PAIR_NUM=6 是错的——
 * 这也是此前闭环全锁死（θe 电周期数错 1/7，扫 offset 修不平）的根因 */
#define FOC_M0_POLE_PAIRS       7.0f
#define FOC_M0_RS_OHM           0.100f     /* DJI 2312S 相电阻 Ω (对齐 ST 历史工程 0.10) */
#define FOC_M0_LS_H             0.000020f  /* DJI 2312S 相电感 H (20 uH，对齐 ST 历史工程 20 uH) */
#define FOC_M0_KE               0.9f       /* 反电动势常数 (0.9 V/krpm) */
#define FOC_M0_MAX_CURRENT_A    5.2f       /* 额定工作电流限制 5.2A（对齐历史工程与硬件规格） */
/* 硬限针对真实硬件短路（硬件测量量程达 +-60A，板载 MOSFET 额定 >30A），
 * 设为 12.0A 单拍即跳，既保证真实短路瞬间保护，又容纳高速正弦交流峰值 */
#define FOC_M0_HARD_CURRENT_A   12.0f      /* 硬电流限制（单拍即跳闸） */
#define FOC_M0_MAX_RPM          12000.0f   /* 最高机械转速 */

/* ---- 电机 0 控制环默认值 ---- */

/** 电流环带宽 rad/s：Kp=Ls·ω, Ki=Rs·ω。2000 rad/s ≈ 320 Hz */
#define FOC_M0_CURRENT_BW_RADS  2000.0f

/* Current-loop tuning guardrails for the 16 kHz PWM/current-sampling path.
 * 100..3000 rad/s was verified on the connected motor.  Higher values can
 * turn ADC/PWM delay into positive feedback and must not be accepted from
 * CLI or persisted configuration. */
#define FOC_CURRENT_BW_MIN_RADS 100.0f
#define FOC_CURRENT_BW_MAX_RADS 3000.0f

/** 速度环 PI（输入 RPM 误差 → 输出 Iq 给定 A） */
#define FOC_M0_VEL_KP           0.0010f    /* A/RPM；本电机实机稳定值 */
#define FOC_M0_VEL_KI           0.0010f    /* A/(RPM·s)，降低低速积分过冲 */
#define FOC_M0_VEL_RAMP_RPM_S   100.0f     /* 降低起步/反转电流冲击 */
#define FOC_M0_VEL_LPF_TF       0.00530516f /* 高速路径 30 Hz */
#define FOC_VEL_LOW_FILTER_HZ   15.0f      /* 低速控制反馈路径 */
#define FOC_M0_VEL_FRICTION_A   0.15f      /* 转动后的库仑摩擦前馈 */
#define FOC_M0_VEL_START_A      0.70f      /* 一次性起步脱槽峰值，随后衰减 */
#define FOC_M0_VEL_START_RPM     5.0f      /* 连续达到该速度50ms后释放起步补偿 */
#define FOC_M0_VEL_TRACK_KP     0.80f      /* <=50 RPM 移动位置轨迹刚度 */
#define FOC_M0_VEL_TRACK_LIMIT  0.40f      /* 最大位置滞后，限制跟踪电流 */
#define FOC_M0_VEL_TRACK_RPM    50.0f      /* <=50 全量，100 RPM 处完全退出 */

/** V/F 开环命令斜坡：避免 enable、变速和反转时给电角速度/电压阶跃。 */
#define FOC_M0_VF_BOOST_V            0.30f  /* Zero-speed voltage boost. */
#define FOC_M0_VF_SLOPE_V_PER_RPM    0.0006f/* Measured safe slope for this 14.4V/800KV motor. */
#define FOC_M0_VF_RPM_RAMP_RPM_S   100.0f  /* Open-loop acceleration must stay below pull-out torque. */
#define FOC_M0_VF_VQ_RAMP_V_S        5.0f  /* Build flux before accelerating. */
#define FOC_M0_VF_ZERO_RPM            0.5f  /* Remove voltage when speed command is zero. */

/** 位置 PI 直接输出 Iq，速度误差项提供轨迹前馈与阻尼 */
#define FOC_M0_POS_KP           3.00f      /* A/rad；已通过小位置阶跃验证 */
#define FOC_M0_POS_KI           0.10f      /* A/(rad*s)；补偿静差，避免积分过强 */
#define FOC_M0_POS_VEL_KP       0.000f     /* A/RPM；低速编码器噪声下先关闭阻尼 */
#define FOC_M0_POS_VEL_LIMIT    80.0f      /* 位置模式速度上限 RPM */

/** 位置模式梯形轨迹（限速限加速度的平滑运动，ODrive trap_traj） */
#define FOC_M0_TRAJ_ENABLE      1
#define FOC_M0_TRAJ_ACC_RPM_S   80.0f      /* 轨迹加/减速度；小步阶调试值 */

/** dq 解耦前馈（ω·L·i 交叉项补偿），低感电机低速时影响小，默认开 */
#define FOC_M0_DECOUPLE         1

/** 高速实验参数默认值（CLI tune 修改仅作用于 RAM） */
#define FOC_M0_ANGLE_DELAY_CYCLES       0.5f
#define FOC_M0_FIELDWEAK_ENABLE         1U
#define FOC_M0_FIELDWEAK_ENTER_RPM      1000.0f
#define FOC_M0_FIELDWEAK_VOLTAGE_RATIO  0.78f
#define FOC_M0_FIELDWEAK_GAIN           800.0f
#define FOC_M0_FIELDWEAK_ID_MIN_RATIO   0.85f

/** 死区补偿电压 V（0 = 关闭）。理论值 = Udc×t_dead×f_pwm
 *  ≈ 14.4×740ns×16kHz ≈ 0.17V。
 *  2026-09-03：开启以修正无感观测器电压模型（-19° 偏移主嫌） */
#define FOC_M0_DEADTIME_COMP_V  0.17f

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
 *  串口 `conf write` 把电机参数/控制环参数/校准偏移写入最后一页 Flash，
 *  上电自动加载；有存储偏移时校准走"快速索引搜索"（免对齐吸附）。 */
#define FOC_STORE_ENABLE        1

/** FDCAN1 formal control protocol.
 *
 * FOC_CAN_ENABLE=0 prevents the protocol from starting and leaves the
 * SIT1042T in standby.  CubeMX still initializes the FDCAN handle so that
 * regeneration cannot silently bypass this switch.  USART2 CLI and VOFA
 * telemetry are independent and remain available in either setting.
 *
 * The proven baseline uses ISO CAN FD without BRS, so the whole frame stays
 * at 500 kbit/s.  Set FOC_CAN_BRS_ENABLE to 1 only after the host data phase
 * has also been configured for 2 Mbit/s.
 */
#define FOC_CAN_ENABLE                 0
#define FOC_CAN_BRS_ENABLE             0
#define FOC_CAN_AUTO_RETRANSMISSION    1
#define FOC_CAN_STATUS_PERIOD_MS       100U

/* ---- 编码器（轴 0，TIM4 ABZ） ---- */

/** 4 倍频后的每转计数（512 线 × 4） */
#define FOC_M0_ENCODER_CPR      2048U

/* ======================== 4. 上电校准参数（轴 0） ======================== */
/* 电压模式校准，务必从小电压开始调试。流程见 App/foc_calib.c */

#define FOC_CALIB_ALIGN_VOLTAGE     0.60f   /* D 轴对齐电压 V：ident 实测 Rs=0.37Ω
                                             * → 稳态 1.6A，校准软限 3.0A 内。
                                             * （0.28V 拉不动转子，offset 两次差 67°） */
#define FOC_CALIB_SEARCH_VOLTAGE    0.60f   /* 找 Z 的开环旋转电压 V（旋转中
                                             * 反电动势限制电流，已验证） */
#define FOC_CALIB_VOLTAGE_RAMP_MS   200U    /* 电压缓升时间，防电流阶跃 */
#define FOC_CALIB_BOOTSTRAP_MS      10U     /* 自举电容充电时间 */
#define FOC_CALIB_NEUTRAL_MS        2000U   /* 中点 PWM 观察时间（沿用已验证基线值） */
#define FOC_CALIB_ALIGN_MS          1200U   /* 建场并从 -90° 电角度扫到 0° */
#define FOC_CALIB_SETTLE_MS         20U     /* 强制清零后的等待 */
#define FOC_CALIB_SEARCH_RPM        60.0f   /* 找 Z 的开环转速（1 秒转满一圈） */
#define FOC_CALIB_SEARCH_TIMEOUT_MS 10000U  /* 找 Z 超时 → FAULT */
#define FOC_CALIB_ALIGN_THETA_E     0.0f    /* 对齐用电角度 rad */
#define FOC_CALIB_DIRECTION         (-1)    /* 编码器方向：2026-09-05 vf 反对称测试
                                             * 改判 -1：vf +300rpm→转子 -210.9、
                                             * vf -300rpm→转子 +218.5（完美反对称，
                                             * 磁场方向与编码器正方向系统性相反）。
                                             * 昨日判 +1 的"vf 比值测试"当时正逢
                                             * 低速棘轮随机方向，判据不可靠。 */
#define FOC_CALIB_CURRENT_LIMIT_A   3.0f    /* 校准期软电流限制（0.25V 对齐在 0.1Ω 上为 2.5A） */
#define FOC_CALIB_HARD_LIMIT_A      6.0f    /* 校准期硬电流限制 */

/* ======================== 5. 按键行为 ======================== */

/**
 * 1：按键在未校准时先启动校准，校准完成后再按进入 RUN（闭环流程）
 * 0：按键直接以当前模式进入 RUN（开环调试流程）
 */
#define FOC_KEY_STARTS_CALIB    1

#endif /* FOC_CONFIG_H */
