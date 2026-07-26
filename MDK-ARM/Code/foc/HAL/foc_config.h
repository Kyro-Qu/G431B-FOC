#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__

// 包含板级配置和接口定义

// 包含板级头文件包含
#include <stdint.h>
#include "main.h"

/* 前向声明，避免循环依赖 */
typedef struct dq_tag dq_t;
typedef struct ab_tag ab_t;

/* PWM 参数 (TIM1 170MHz, 中心对齐) */
#define PWM_CNT     5312u    /* ARR, 对应 16kHz */
#define PWM_NEUTRAL_CNT (PWM_CNT / 2u)
#define U_DC        14.23f   /* 母线电压 V  */
#define PWM_FREQ_HZ 16000.0f // 频率单位:Hz,16kHz  建议15kHz到20kHz
#define FOC_LOG_MONITOR 1        // 是否启用监视器日志输出,1启用，0禁用

/* 监视器数据（使用指针，更高效） */
typedef struct {
    float *angle;      // 电角度指针
    dq_t  *dq;         // DQ坐标系指针
    ab_t  *ab;         // AB坐标系指针
    uint16_t *pwm_a;   // PWM通道A占空比指针
    uint16_t *pwm_b;   // PWM通道B占空比指针
    uint16_t *pwm_c;   // PWM通道C占空比指针
} foc_log_monitor_t;


/* 电机参数 (需根据实际电机设置)  */
typedef struct {
    float pole_pairs;  /* 极对数  */
    float Rs;          /* 定子电阻 Ω */
    float Ls;          /* 定子电感 H */
    float max_current; /* 峰值最大电流 Apk */
    float max_voltage; /* 母线电压 V */
    float Ke;          /* 反电动势常数 Vrms/kRPM */
    float max_rpm;     /* 最大转速 RPM */
} foc_motor_info_t;

extern foc_motor_info_t foc_motor_info; // 全局电机参数实例
extern foc_log_monitor_t foc_log_monitor; // 全局监视器参数实例
extern TIM_HandleTypeDef htim1; // 定时器1句柄
extern volatile uint8_t g_foc_pwm_enabled;
/* 0=outputs off, 1=bootstrap low sides on, 2=normal complementary PWM. */
extern volatile uint8_t g_foc_pwm_stage;


// 传感器模式配置
// // ABZ 编码器    ADC无感测    霍尔传感器    SPI/IIC UVW霍尔

//电机参数初始化
void foc_motor_init(void);
void foc_log_monitor_init(void);


// PWM 输出接口
void foc_timer_init(void);            // 定时器初始化接口
void foc_pwm_bootstrap_start(void);   // 三路低侧导通，为高侧驱动 bootstrap 电容充电
void foc_pwm_enable(void);            // 启用PWM输出
void foc_pwm_disable(void);           // 禁用PWM输出
void foc_set_pwm(uint32_t ccr_a, uint32_t ccr_b, uint32_t ccr_c, uint8_t sector);

// 电流采样接口
void foc_get_currents(float *ia, float *ib, float *ic);

// 电角度获取接口
float foc_get_electrical_angle(void); // 角度传感器接口




#endif

