# foc/ —— FOC 库本体

完整文档在仓库根目录 [Docs/](../../../Docs/)，请从
[01_FOC原理入门](../../../Docs/01_FOC原理入门.md) 开始读。

## 分层速查

```
Core/    纯算法层：禁止出现任何 HAL/寄存器代码
  foc_types.h      通用类型 + 三大硬件接口表（架构核心）
  foc_utils.h      数学常量与内联工具
  foc_transform.h  Clarke/Park（内联）
  foc_svm.c/.h     SVPWM：电压矢量 → 占空比（纯函数）
  foc_pid.c/.h     PID + 一阶低通
  foc_traj.c/.h    梯形轨迹规划（位置模式，ODrive trap_traj）
  foc_motor.c/.h   电机轴对象 + 级联控制环（工程心脏）
  foc_observer.c/.h【预留】无感磁链观测器 + PLL（VESC 式）

Driver/  设备驱动层：可用 HAL，对上提供统一接口
  encoder/abz_encoder    ABZ 编码器（TIM4）
  current/current_shunt  三电阻低边采样（本板硬件验证的时序）
  hall/hall_sensor       【预留】霍尔传感器

HAL/     板级绑定层：换板子只改这里
  foc_config.h      用户配置（轴数/频率/电机参数/校准参数）
  foc_board_g431    把本板硬件组装成接口表 + 轴1虚拟绑定

App/     应用层：业务逻辑，不写算法不碰寄存器
  foc_app        多轴对象、初始化时序、按键、任务调度
  foc_calib      上电校准状态机
  foc_ident      Rs/Ls 自动测量（串口 id / id a）
  foc_cmd        串口命令行（help 查看命令表）
  foc_telemetry  VOFA+ JustFloat 遥测（16 通道，见头文件通道表）
```

## 新增代码的归属规则

- 出现 `HAL_`、`__HAL_`、`TIM_HandleTypeDef` → 只能放 `HAL/` 或 `Driver/`
- 控制公式、坐标变换、PI、SVPWM → `Core/`
- 启停、命令、协议、状态机 → `App/`
- 快环（中断）内禁止：阻塞串口、动态内存、长循环等待
