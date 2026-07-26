#ifndef   __FOC_APP_H__
#define   __FOC_APP_H__


#include "foc_config.h"


// 系统/运行状态（安全与生命周期）
typedef enum {
    FOC_STATE_IDLE = 0,     // 空闲（不输出�?
    FOC_STATE_RUN  = 1,     // 运行（输出允许）
    FOC_STATE_CALIB = 2,
    FOC_STATE_FAULT= 3      // 故障（锁定，等待清故障）
} foc_state_t;

/* Global state mirror for Keil Watch. */
extern volatile uint8_t g_foc_state_diag;


// 控制目标模式
typedef enum {
    FOC_CTRL_NONE = 0,        // 不控（或占位�?
    FOC_CTRL_VF_OPENLOOP,     // 开�?V/f（你现在的）
    FOC_CTRL_TORQUE_IQ,       // 力矩模式：Iq 给定（电流环闭环�?
    FOC_CTRL_SPEED,           // 速度模式：速度�?+ 电流�?
    FOC_CTRL_POSITION         // 位置模式：位置环 + 速度�?+ 电流�?
} foc_ctrl_mode_t; 


// 传感器类型枚�?
typedef enum {
    FOC_SENS_NONE = 0,     // 无传感器（纯开环或无感估算�?
    FOC_SENS_ENCODER,      // 编码�?
    FOC_SENS_HALL,         // 霍尔
    FOC_SENS_SENSORLESS    // 无感（SMO/PLL等）
} foc_sensor_t;

// 初始化函�?
void foc_init(void);

// 状态管�?
foc_state_t foc_get_state(void);
void foc_set_state(foc_state_t state);

// 控制模式管理
foc_ctrl_mode_t foc_get_ctrl_mode(void);
void foc_set_ctrl_mode(foc_ctrl_mode_t mode);

// 传感器类型管�?
foc_sensor_t foc_get_sensor_type(void);
void foc_set_sensor_type(foc_sensor_t type);

#endif
