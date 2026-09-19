/**
 * @file    abz_encoder.c
 * @brief   ABZ 位置跟踪与自适应滑窗速度估计。
 */

#include "abz_encoder.h"
#include "main.h"
#include <math.h>

/* 高速诊断开关（2026-09-03）：禁用 Z 脉冲的计数器重映射，
 * 验证"每转一次的位置阶跃激励"是否为高速爆发放大器。 */
#define DIAG_DISABLE_Z_REMAP 0
#define DIAG_SOFT_Z 1

extern TIM_HandleTypeDef ABZ_ENCODER_TIM_HANDLE;

/* ======================== 内部状态 ======================== */

/**
 * 单个滑动最小二乘窗口的 O(1) 状态。
 *
 * sum_y  = sum(y[i])
 * sum_iy = sum(i * y[i]), i=0..N-1
 *
 * 只维护这两个整数和，就不用在电流环中断里遍历 16/32/64 个历史点。
 * 这一点很关键：一长串浮点运算可能延误紧随其后的 ADC 注入转换。
 */
typedef struct {
    int64_t sum_y;
    int64_t sum_iy;
} abz_velocity_window_t;

typedef struct {
    uint16_t last_cnt;            /**< 上一次的 16 位 TIM 计数值。 */
    int32_t  position_cnt;        /**< 单圈位置 [0, CPR)。 */
    float    angle_rad;           /**< 经 PLL 滤波的单圈角度。 */
    float    angle_pll_count;     /**< PLL 位置，单位为编码器计数。 */
    float    angle_pll_velocity;  /**< PLL 速度，单位为计数/秒。 */
    float    velocity_rpm;        /**< 自适应拟合的机械转速。 */

    int64_t  total_count;         /**< 用于拟合的连续多圈计数。 */
    int64_t  velocity_history[ABZ_VELOCITY_HISTORY_LEN];
    uint16_t velocity_sample_div; /**< 16 kHz 到 1 kHz 的分频计数。 */
    uint8_t  velocity_history_head;
    abz_velocity_window_t velocity_low;
    abz_velocity_window_t velocity_mid;
    abz_velocity_window_t velocity_high;

    volatile uint8_t  zero_request;
    volatile uint8_t  index_request;
    volatile uint16_t index_hw_cnt;
    uint8_t  index_pending;
    int32_t  index_position_cnt;
    uint8_t  zero_on_index;
    uint8_t  is_calibrated;
} abz_encoder_t;

static abz_encoder_t enc;
static volatile float s_pll_bw_rad_s = ABZ_ANGLE_PLL_BW_RAD_S;


/**
 * 返回两次 16 位定时器采样之间的有符号位移。
 *
 * 物理电机不可能在一个 FOC 周期内走过半个定时器量程，
 * 因此跨越 0/65535 时可以无歧义地判定回绕方向。
 */
static int32_t abz_calc_delta(uint16_t cur_cnt, uint16_t last_cnt)
{
    int32_t delta = (int32_t)cur_cnt - (int32_t)last_cnt;

    if (delta > ABZ_TIMER_HALF_RANGE) {
        delta -= ABZ_TIMER_COUNTER_RANGE;
    } else if (delta < -ABZ_TIMER_HALF_RANGE) {
        delta += ABZ_TIMER_COUNTER_RANGE;
    }/* ======================== 计数器辅助函数 ======================== */

    return delta;
}

/** 把有符号位置回绕到一个机械圈以内。 */
static int32_t abz_wrap_position(int32_t position_cnt)
{
    position_cnt %= (int32_t)ABZ_ENCODER_CPR;
    if (position_cnt < 0) {
        position_cnt += (int32_t)ABZ_ENCODER_CPR;
    }
    return position_cnt;
}

/* ======================== 速度估计器 ======================== */

/**
 * 用同一个位置填满整段历史。
 *
 * 启动、强制清零或 Z 计数器重映射之后必须执行，
 * 否则位置跳变会被解释成极大的速度。
 */
static void abz_velocity_reset(int64_t position)
{
    uint32_t i;
    int32_t wrapped_position =
        abz_wrap_position((int32_t)(position % (int64_t)ABZ_ENCODER_CPR));

    enc.total_count = position;
    enc.angle_pll_count = (float)wrapped_position;
    enc.angle_pll_velocity = 0.0f;
    enc.angle_rad = enc.angle_pll_count * ABZ_RAD_PER_CNT;
    enc.velocity_rpm = 0.0f;
    enc.velocity_sample_div = 0U;
    enc.velocity_history_head = 0U;

    for (i = 0U; i < ABZ_VELOCITY_HISTORY_LEN; ++i) {
        enc.velocity_history[i] = position;
    }

    enc.velocity_low.sum_y =
        position * (int64_t)ABZ_VELOCITY_LOW_WINDOW;
    enc.velocity_low.sum_iy =
        position *
        ((int64_t)ABZ_VELOCITY_LOW_WINDOW *
         (int64_t)(ABZ_VELOCITY_LOW_WINDOW - 1U) / 2LL);
    enc.velocity_mid.sum_y =
        position * (int64_t)ABZ_VELOCITY_MID_WINDOW;
    enc.velocity_mid.sum_iy =
        position *
        ((int64_t)ABZ_VELOCITY_MID_WINDOW *
         (int64_t)(ABZ_VELOCITY_MID_WINDOW - 1U) / 2LL);
    enc.velocity_high.sum_y =
        position * (int64_t)ABZ_VELOCITY_HIGH_WINDOW;
    enc.velocity_high.sum_iy =
        position *
        ((int64_t)ABZ_VELOCITY_HIGH_WINDOW *
         (int64_t)(ABZ_VELOCITY_HIGH_WINDOW - 1U) / 2LL);
}

/**
 * 用二阶 PLL 跟踪量化后的单圈计数。
 *
 * 位置误差回绕到半圈以内，观测器跨越计数 0/CPR 时不会出现不连续。
 * 比例校正作用于位置，积分状态估计编码器计数/秒。
 */
static void abz_angle_pll_update(void)
{
    float error = (float)enc.position_cnt - enc.angle_pll_count;
    float pll_bw = s_pll_bw_rad_s;
    float pll_kp = 2.0f * pll_bw;
    float pll_ki = pll_bw * pll_bw;
    const float half_cpr = 0.5f * (float)ABZ_ENCODER_CPR;

    if (error > half_cpr) {
        error -= (float)ABZ_ENCODER_CPR;
    } else if (error < -half_cpr) {
        error += (float)ABZ_ENCODER_CPR;
    }

    enc.angle_pll_velocity +=
        pll_ki * ABZ_FAST_DT_S * error;
    enc.angle_pll_count += ABZ_FAST_DT_S *
        (enc.angle_pll_velocity + (pll_kp * error));

    /* NaN 与大误差安全复位 */
    if ((enc.angle_pll_count != enc.angle_pll_count) || (fabsf(error) > half_cpr)) {
        enc.angle_pll_count = (float)enc.position_cnt;
        enc.angle_pll_velocity = 0.0f;
    }

    while (enc.angle_pll_count >= (float)ABZ_ENCODER_CPR) {
        enc.angle_pll_count -= (float)ABZ_ENCODER_CPR;
    }
    while (enc.angle_pll_count < 0.0f) {
        enc.angle_pll_count += (float)ABZ_ENCODER_CPR;
    }
    enc.angle_rad = enc.angle_pll_count * ABZ_RAD_PER_CNT;
}

/**
 * 以恒定时间把一个滑窗推进一个样本。
 *
 * 现有点 y[1]..y[N-1] 左移一个下标，最旧的点移出，
 * new_position 进入下标 N-1。
 */
static void abz_velocity_window_push(abz_velocity_window_t *state,
                                     uint32_t window,
                                     int64_t oldest_position,
                                     int64_t new_position)
{
    state->sum_iy =
        state->sum_iy - (state->sum_y - oldest_position) +
        ((int64_t)(window - 1U) * new_position);
    state->sum_y += new_position - oldest_position;
}

/**
 * 把一个滚动窗口换算成 RPM。
 *
 * 对等间隔的 x=0..N-1：
 *   slope = [2*sum(i*y)-((N-1)*sum(y))] * 6/[N*(N^2-1)]
 *
 * 分子在 int64_t 中构造，很大的多圈绝对计数在转 float 之前精确对消。
 */
static float abz_velocity_window_rpm(const abz_velocity_window_t *state,
                                     uint32_t window)
{
    int64_t numerator_x2 =
        (2LL * state->sum_iy) -
        ((int64_t)(window - 1U) * state->sum_y);
    float n = (float)window;
    float scale =
        (6.0f * ABZ_RPM_PER_COUNT_PER_SAMPLE) /
        (n * ((n * n) - 1.0f));

    return (float)numerator_x2 * scale;
}

/**
 * 存储一个 1 kHz 历史样本并发布新的速度估计。
 *
 * 长窗口估计决定响应/噪声的折衷：
 *   <=60 RPM:     64 ms 拟合
 *   60..120 RPM:  64 -> 32 ms 交叉淡入淡出
 *   120..800 RPM: 32 ms 拟合
 *   800..1200:    32 -> 16 ms 交叉淡入淡出
 *   >=1200:       16 ms 拟合
 *
 * 窗口选择依据稳定的长窗口估计。若改用噪声大的短窗口估计来选窗口，
 * 估计器会在窗口之间来回抖动。
 */
static void abz_velocity_update(void)
{
    uint32_t head;
    uint32_t low_oldest_index;
    uint32_t mid_oldest_index;
    uint32_t high_oldest_index;
    int64_t low_oldest;
    int64_t new_position;
    int64_t endpoint_delta;
    float velocity_low;
    float velocity_mid;
    float velocity_high;
    float speed_abs;
    float velocity;

    if (++enc.velocity_sample_div < ABZ_VELOCITY_SAMPLE_DIV) {
        return;
    }
    enc.velocity_sample_div = 0U;

    head = (uint32_t)enc.velocity_history_head;
    low_oldest_index =
        (head + ABZ_VELOCITY_HISTORY_LEN -
         ABZ_VELOCITY_LOW_WINDOW) % ABZ_VELOCITY_HISTORY_LEN;
    mid_oldest_index =
        (head + ABZ_VELOCITY_HISTORY_LEN -
         ABZ_VELOCITY_MID_WINDOW) % ABZ_VELOCITY_HISTORY_LEN;
    high_oldest_index =
        (head + ABZ_VELOCITY_HISTORY_LEN -
         ABZ_VELOCITY_HIGH_WINDOW) % ABZ_VELOCITY_HISTORY_LEN;

    low_oldest = enc.velocity_history[low_oldest_index];
    new_position = enc.total_count;

    abz_velocity_window_push(
        &enc.velocity_low, ABZ_VELOCITY_LOW_WINDOW,
        low_oldest, new_position);
    abz_velocity_window_push(
        &enc.velocity_mid, ABZ_VELOCITY_MID_WINDOW,
        enc.velocity_history[mid_oldest_index], new_position);
    abz_velocity_window_push(
        &enc.velocity_high, ABZ_VELOCITY_HIGH_WINDOW,
        enc.velocity_history[high_oldest_index], new_position);

    enc.velocity_history[head] = new_position;
    if (++enc.velocity_history_head >= ABZ_VELOCITY_HISTORY_LEN) {
        enc.velocity_history_head = 0U;
    }

    velocity_low =
        abz_velocity_window_rpm(
            &enc.velocity_low, ABZ_VELOCITY_LOW_WINDOW);
    velocity_mid =
        abz_velocity_window_rpm(
            &enc.velocity_mid, ABZ_VELOCITY_MID_WINDOW);
    velocity_high =
        abz_velocity_window_rpm(
            &enc.velocity_high, ABZ_VELOCITY_HIGH_WINDOW);
    speed_abs = fabsf(velocity_low);

    if (speed_abs <= ABZ_VELOCITY_LOW_BLEND_START_RPM) {
        velocity = velocity_low;
    } else if (speed_abs < ABZ_VELOCITY_LOW_BLEND_END_RPM) {
        float blend =
            (speed_abs - ABZ_VELOCITY_LOW_BLEND_START_RPM) /
            (ABZ_VELOCITY_LOW_BLEND_END_RPM -
             ABZ_VELOCITY_LOW_BLEND_START_RPM);

        velocity = velocity_low + blend * (velocity_mid - velocity_low);
    } else if (speed_abs <= ABZ_VELOCITY_HIGH_BLEND_START_RPM) {
        velocity = velocity_mid;
    } else if (speed_abs < ABZ_VELOCITY_HIGH_BLEND_END_RPM) {
        float blend =
            (speed_abs - ABZ_VELOCITY_HIGH_BLEND_START_RPM) /
            (ABZ_VELOCITY_HIGH_BLEND_END_RPM -
             ABZ_VELOCITY_HIGH_BLEND_START_RPM);

        velocity = velocity_mid +
                   blend * (velocity_high - velocity_mid);
    } else {
        velocity = velocity_high;
    }

    endpoint_delta = new_position - low_oldest;
    if ((endpoint_delta <= ABZ_STATIONARY_ENDPOINT_COUNTS) &&
        (endpoint_delta >= -ABZ_STATIONARY_ENDPOINT_COUNTS) &&
        (fabsf(velocity_low) <= ABZ_STATIONARY_MAX_RPM)) {
        velocity = 0.0f;
    }
    enc.velocity_rpm = velocity;
}

/* ======================== 公开 API ======================== */

void abz_encoder_init(void)
{
    enc.last_cnt = 0U;
    enc.position_cnt = 0;
    enc.angle_rad = 0.0f;
    abz_velocity_reset(0);

    enc.zero_request = 0U;
    enc.index_request = 0U;
    enc.index_hw_cnt = 0U;
    enc.index_pending = 0U;
    enc.index_position_cnt = 0;
    /* Z 重映射仅用于校准。若启动时保持使能，V/F 模式下两个速度估计器
     * 每圈都会被复位一次，产生随速度变化的负测量偏置。 */
    enc.zero_on_index = 0U;
    enc.is_calibrated = 0U;

    __HAL_TIM_SET_COUNTER(&ABZ_ENCODER_TIM_HANDLE, 0);
    HAL_TIM_Encoder_Start(&ABZ_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);
}

void abz_encoder_deinit(void)
{
    HAL_TIM_Encoder_Stop(&ABZ_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);
}

void abz_encoder_update(void)
{
    uint16_t cur_cnt;
    uint16_t index_cnt = 0U;
    uint8_t index_request = 0U;
    uint32_t primask;
    int32_t delta;

    /*
     * 原子地消费 EXTI 产生的索引事件。这段短临界区防止在读取数据
     * 与清零标志之间到来新的边沿。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if (enc.index_request != 0U) {
        index_cnt = enc.index_hw_cnt;
        enc.index_request = 0U;
        index_request = 1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    /*
     * 保留物理 Z 边沿到本次快环调用之间的位移。
     * 若已请求，则重映射 TIM CNT，使 Z 成为机械计数零点。
     */
    if (index_request != 0U) {
        uint16_t post_cnt;
        int32_t index_delta = abz_calc_delta(index_cnt, enc.last_cnt);
        int32_t post_index_delta;

        cur_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&ABZ_ENCODER_TIM_HANDLE);
        post_index_delta = abz_calc_delta(cur_cnt, index_cnt);

        enc.index_position_cnt =
            abz_wrap_position(enc.position_cnt + index_delta);
        enc.index_pending = 1U;
        enc.is_calibrated = 1U;

#if !DIAG_DISABLE_Z_REMAP
        if (enc.zero_on_index != 0U) {
            post_cnt = (uint16_t)post_index_delta;
            __HAL_TIM_SET_COUNTER(&ABZ_ENCODER_TIM_HANDLE, post_cnt);
            enc.last_cnt = post_cnt;
            enc.position_cnt = abz_wrap_position(post_index_delta);
#if DIAG_SOFT_Z == 1
            /* 诊断（2026-09-03）：保留位置重映射但跳过速度估计重启——
             * 验证"每转速度估计重启瞬态"是否为爆发放大器。 */
#else
            /*
             * Z 重映射了硬件计数器。重启拟合，避免把重映射误解为真实
             * 的轴运动；短暂的重启瞬态由外环低通桥接。
             */
            abz_velocity_reset((int64_t)post_index_delta);
#endif
            return;
        }
#endif /* DIAG_DISABLE_Z_REMAP */
    }

    if (enc.zero_request != 0U) {
        __HAL_TIM_SET_COUNTER(&ABZ_ENCODER_TIM_HANDLE, 0);
        enc.last_cnt = 0U;
        enc.position_cnt = 0;
        enc.zero_request = 0U;
        enc.index_pending = 0U;
        enc.is_calibrated = 1U;
        abz_velocity_reset(0);
        return;
    }

    cur_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&ABZ_ENCODER_TIM_HANDLE);
    delta = abz_calc_delta(cur_cnt, enc.last_cnt);
    enc.last_cnt = cur_cnt;

    if ((delta > ABZ_MAX_DELTA) || (delta < -ABZ_MAX_DELTA)) {
        delta = 0;
    }

    enc.position_cnt = abz_wrap_position(enc.position_cnt + delta);
    enc.total_count += (int64_t)delta;
    abz_angle_pll_update();
    abz_velocity_update();
}

float abz_encoder_angle_rad(void)
{
    /* 换相电角度必须直接采用无滤波相移的硬件实时计数，
     * 避免二阶 PLL 在高速（>2000 RPM）下因滤波截止频率引入数十度甚至数百度的电角度相位滞后。 */
    return (float)enc.position_cnt * ABZ_RAD_PER_CNT;
}

float abz_encoder_velocity_rpm(void)
{
    return enc.velocity_rpm;
}

float abz_encoder_pll_velocity_rpm(void)
{
    return enc.angle_pll_velocity *
           (60.0f / (float)ABZ_ENCODER_CPR);
}

uint8_t abz_encoder_set_pll_bw_rad_s(float bw_rad_s)
{
    uint32_t primask;

    if ((bw_rad_s != bw_rad_s) ||
        ((bw_rad_s - bw_rad_s) != 0.0f) ||
        (bw_rad_s < ABZ_ANGLE_PLL_MIN_BW_RAD_S) ||
        (bw_rad_s > ABZ_ANGLE_PLL_MAX_BW_RAD_S)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    s_pll_bw_rad_s = bw_rad_s;
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

float abz_encoder_get_pll_bw_rad_s(void)
{
    return s_pll_bw_rad_s;
}

void abz_encoder_on_index(void)
{
    /* 仅在校准找零期间（未完成校准或使能了 zero_on_index）响应 Z 脉冲中断，
     * 正常运行中彻底旁路，避免高速旋转下极窄 Z 脉冲和边沿毛刺干扰正常换相累加。 */
    if ((enc.is_calibrated == 0U) || (enc.zero_on_index != 0U)) {
        enc.index_hw_cnt =
            (uint16_t)__HAL_TIM_GET_COUNTER(&ABZ_ENCODER_TIM_HANDLE);
        enc.index_request = 1U;
    }
}

void abz_encoder_force_zero(void)
{
    enc.zero_request = 1U;
}

void abz_encoder_set_zero_on_index(uint8_t enable)
{
    enc.zero_on_index = (enable != 0U) ? 1U : 0U;
}

uint8_t abz_encoder_consume_index(int32_t *position_cnt)
{
    if (enc.index_pending == 0U) {
        return 0U;
    }
    if (position_cnt != 0) {
        *position_cnt = enc.index_position_cnt;
    }
    enc.index_pending = 0U;
    return 1U;
}

uint8_t abz_encoder_is_calibrated(void)
{
    return enc.is_calibrated;
}

uint16_t abz_encoder_get_sample_div(void)
{
    return enc.velocity_sample_div;
}
