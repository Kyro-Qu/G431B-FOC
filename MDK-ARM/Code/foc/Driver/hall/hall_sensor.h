/**
 * @file    hall_sensor.h
 * @brief   【预留模块】霍尔传感器驱动（本板硬件支持，暂未绑定到任何轴）
 *
 * 火柴FOC 板的 H1/H2/H3 霍尔输入与 ABZ 编码器共用 PB6/PB7/PB8
 * （原理图带 22K 上拉 + 100R 串阻），接霍尔电机时不能同时接编码器。
 *
 * 原理：三个霍尔开关按 120° 分布，输出 3 位编码（1~6 有效），
 * 每个编码对应 60° 电角度扇区。仅用扇区中心角，角度分辨率只有 60°，
 * 转矩会有脉动；本驱动用"扇区边沿 + 转速积分插值"平滑角度
 * （SimpleFOC HallSensor 与 VESC hall 插值的通行做法）。
 *
 * 未来接入方式：
 *   1. CubeMX 里把 PB6/7/8 改为 EXTI 双边沿（或 TIM4 XOR 霍尔模式）；
 *   2. 在三个引脚的 EXTI 回调里调用 hall_sensor_on_edge()；
 *   3. 板级接口表换成：
 *        .update = hall_sensor_update, .angle_rad = hall_sensor_angle_rad,
 *        .velocity_rpm = hall_sensor_velocity_rpm（Z 相关接口置 NULL）；
 *   4. 校准：用 D 轴对齐法测每个扇区对应的电角度填入 sector_table
 *      （或沿用厂家例程 HALL_PHASE_SHIFT=300° 的约定）。
 */

#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pole_pairs;        /* 极对数（电角度→机械角换算用） */
    float phase_shift_rad;   /* 霍尔安装偏移（厂家例程约定 300°=5.24rad） */
    /* 状态 */
    volatile uint8_t hall_code;   /* 最近一次读到的 3 位霍尔编码 */
    volatile uint32_t edge_tick;  /* 最近一次扇区切换的时间戳 */
    float theta_e;           /* 插值后的电角度 [0, 2π) */
    float speed_e_rads;      /* 电角速度（由扇区切换周期估计） */
    int8_t direction;        /* 当前旋转方向 +1/-1 */
} hall_sensor_t;

/** 初始化（填极对数与相移） */
void hall_sensor_init(hall_sensor_t *h, float pole_pairs, float phase_shift_rad);

/** 霍尔引脚任意边沿中断中调用：code = (H3<<2)|(H2<<1)|H1 */
void hall_sensor_on_edge(hall_sensor_t *h, uint8_t code, uint32_t now_us);

/** 快环中调用：按当前转速插值推进角度 */
void hall_sensor_update(hall_sensor_t *h, float dt);

/** 插值后的电角度 [0, 2π) */
float hall_sensor_theta_e(const hall_sensor_t *h);

/** 机械转速 RPM */
float hall_sensor_velocity_rpm(const hall_sensor_t *h);

#ifdef __cplusplus
}
#endif

#endif /* HALL_SENSOR_H */
