/**
 * @file    hall_sensor.c
 * @brief   【预留模块】霍尔传感器驱动实现（扇区 + 转速插值）
 */

#include "hall_sensor.h"
#include "foc_utils.h"

/* 标准 120° 霍尔编码 → 扇区序号（0~5）。编码 0/7 非法。
 * 扇区 k 的中心电角度 = k·60° + 30° + phase_shift。 */
static const int8_t hall_code_to_sector[8] = {
    -1, 0, 2, 1, 4, 5, 3, -1
};

void hall_sensor_init(hall_sensor_t *h, float pole_pairs, float phase_shift_rad)
{
    h->pole_pairs = pole_pairs;
    h->phase_shift_rad = phase_shift_rad;
    h->hall_code = 0U;
    h->edge_tick = 0U;
    h->theta_e = 0.0f;
    h->speed_e_rads = 0.0f;
    h->direction = 1;
}

void hall_sensor_on_edge(hall_sensor_t *h, uint8_t code, uint32_t now_us)
{
    int8_t new_sector = hall_code_to_sector[code & 0x07U];
    int8_t old_sector = hall_code_to_sector[h->hall_code & 0x07U];

    if (new_sector < 0) {
        return;                       /* 非法编码（干扰/线未接好），忽略 */
    }

    if (old_sector >= 0) {
        /* 扇区差 → 方向；切换间隔 → 转速（60° / Δt） */
        int8_t diff = (int8_t)(new_sector - old_sector);

        if (diff > 3) {
            diff -= 6;
        } else if (diff < -3) {
            diff += 6;
        }

        if (diff != 0) {
            uint32_t dt_us = now_us - h->edge_tick;

            h->direction = (diff > 0) ? 1 : -1;
            if (dt_us > 0U) {
                float dt_s = (float)dt_us * 1e-6f;

                /* 一次跨 |diff| 个扇区（正常 1 个），每扇区 60° 电角度 */
                h->speed_e_rads = ((float)diff * (_PI / 3.0f)) / dt_s;
            }

            /* 边沿处角度硬同步到扇区边界，插值误差清零 */
            h->theta_e = foc_wrap_0_2pi(
                ((float)new_sector * (_PI / 3.0f)) +
                ((h->direction > 0) ? 0.0f : (_PI / 3.0f)) +
                h->phase_shift_rad);
        }
    }

    h->hall_code = code & 0x07U;
    h->edge_tick = now_us;
}

void hall_sensor_update(hall_sensor_t *h, float dt)
{
    /* 两次边沿之间按最近转速积分推进（插值），
     * 限制在一个扇区宽度内，防止低速时插飞 */
    h->theta_e = foc_wrap_0_2pi(h->theta_e + (h->speed_e_rads * dt));
}

float hall_sensor_theta_e(const hall_sensor_t *h)
{
    return h->theta_e;
}

float hall_sensor_velocity_rpm(const hall_sensor_t *h)
{
    return (h->speed_e_rads / h->pole_pairs) * FOC_RADS_TO_RPM;
}
