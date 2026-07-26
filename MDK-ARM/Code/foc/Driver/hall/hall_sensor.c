/**
 * @file    hall_sensor.c
 * @brief   【预留模块】霍尔传感器驱动实现（扇区 + 转速插值）
 *
 * 并发模型：hall_sensor_on_edge() 在 EXTI 中断里只写"事实"
 * （扇区边界角、转速、方向、序号），hall_sensor_update() 在快环里
 * 检测序号变化后基于这些事实做插值。两个上下文不共享读-改-写变量，
 * 边沿恰好落在 update 中间时最多引入一拍 ≤60° 的瞬时偏差，下一拍
 * 即被序号检测纠正。
 */

#include "hall_sensor.h"
#include "foc_utils.h"

/* 距上次边沿超过该时间视为停转：转速清零，插值停止。
 * 0.2s 对应约 50 eRPM（每扇区 60°），低于此速度霍尔本就无法测速。 */
#define HALL_SPEED_TIMEOUT_S 0.2f

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
    h->edge_seq = 0U;
    h->theta_base = 0.0f;
    h->speed_e_rads = 0.0f;
    h->direction = 1;
    h->seen_seq = 0U;
    h->interp_rad = 0.0f;
    h->since_edge_s = 0.0f;
    h->theta_e = 0.0f;
}

void hall_sensor_on_edge(hall_sensor_t *h, uint8_t code, uint32_t now_us)
{
    int8_t new_sector = hall_code_to_sector[code & 0x07U];
    int8_t old_sector = hall_code_to_sector[h->hall_code & 0x07U];

    if (new_sector < 0) {
        return;                       /* 非法编码（干扰/线未接好），忽略 */
    }

    if (old_sector < 0) {
        /* 第一个有效编码：只建立基准，不算速度 */
        h->hall_code = code & 0x07U;
        h->edge_tick = now_us;
        return;
    }

    /* 扇区差 → 方向；切换间隔 → 转速（60° / Δt） */
    {
        int8_t diff = (int8_t)(new_sector - old_sector);

        if (diff > 3) {
            diff -= 6;
        } else if (diff < -3) {
            diff += 6;
        }

        if (diff == 0) {
            /* 同扇区伪边沿（抖动/干扰）：不刷新 edge_tick，
             * 否则下一次真边沿的 Δt 偏小、转速被高估 */
            return;
        }

        {
            uint32_t dt_us = now_us - h->edge_tick;

            h->direction = (diff > 0) ? 1 : -1;
            if (dt_us > 0U) {
                float dt_s = (float)dt_us * 1e-6f;

                /* 一次跨 |diff| 个扇区（正常 1 个），每扇区 60° 电角度 */
                h->speed_e_rads = ((float)diff * (_PI / 3.0f)) / dt_s;
            }

            /* 硬同步基准角：进入新扇区时角度必然在"迎面"的扇区边界，
             * 正转是扇区下边界，反转是上边界 */
            h->theta_base = foc_wrap_0_2pi(
                ((float)new_sector * (_PI / 3.0f)) +
                ((h->direction > 0) ? 0.0f : (_PI / 3.0f)) +
                h->phase_shift_rad);
            ++h->edge_seq;
        }
    }

    h->hall_code = code & 0x07U;
    h->edge_tick = now_us;
}

void hall_sensor_update(hall_sensor_t *h, float dt)
{
    uint32_t seq = h->edge_seq;

    if (seq != h->seen_seq) {
        /* 新边沿：插值从新的基准角重新开始 */
        h->seen_seq = seq;
        h->interp_rad = 0.0f;
        h->since_edge_s = 0.0f;
    } else {
        h->since_edge_s += dt;
        if (h->since_edge_s > HALL_SPEED_TIMEOUT_S) {
            /* 长时间无边沿 = 停转/极低速：清转速，停止插值推进 */
            h->speed_e_rads = 0.0f;
        }
        h->interp_rad += h->speed_e_rads * dt;
        /* 插值限制在一个扇区宽度内：下一个边沿没来，
         * 角度就不可能已经越过下一个扇区边界 */
        h->interp_rad = foc_clampf(h->interp_rad, -(_PI / 3.0f), (_PI / 3.0f));
    }

    h->theta_e = foc_wrap_0_2pi(h->theta_base + h->interp_rad);
}

float hall_sensor_theta_e(const hall_sensor_t *h)
{
    return h->theta_e;
}

float hall_sensor_velocity_rpm(const hall_sensor_t *h)
{
    return (h->speed_e_rads / h->pole_pairs) * FOC_RADS_TO_RPM;
}
