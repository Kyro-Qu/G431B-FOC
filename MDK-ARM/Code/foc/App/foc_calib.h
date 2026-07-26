#ifndef FOC_CALIB_H
#define FOC_CALIB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * FOC 上电校准模块。
 *
 * 当前版本不保存 Flash，每次上电都重新校准一次。
 * 校准流程大致是：
 *   1. D 轴固定给电压，把转子拉到已知电角度；
 *   2. 把当前位置临时作为增量编码器零点；
 *   3. 慢速开环旋转，等待 ABZ 编码器 Z/index 脉冲；
 *   4. 记录 Z 脉冲到来时的计数，换算成电角度 offset；
 *   5. RUN 阶段用“机械角度 * 极对数 + offset”作为 Park 变换角度。
 */

/* D 轴对齐电压，单位 V。这里是电压模式，不是电流模式，调试时要从小值开始。 */
#ifndef FOC_CALIB_ALIGN_VOLTAGE
#define FOC_CALIB_ALIGN_VOLTAGE     0.30f
#endif

/* 找 Z/index 时的 q 轴开环旋转电压，单位 V。 */
#ifndef FOC_CALIB_SEARCH_VOLTAGE
#define FOC_CALIB_SEARCH_VOLTAGE    0.30f
#endif

/* 校准电压从 0 缓升到目标值，避免静止低阻电机上的电流阶跃。 */
#ifndef FOC_CALIB_VOLTAGE_RAMP_MS
#define FOC_CALIB_VOLTAGE_RAMP_MS   200U
#endif

/* Same startup policy as the validated MCSDK project. */
#ifndef FOC_CALIB_BOOTSTRAP_MS
#define FOC_CALIB_BOOTSTRAP_MS      10U
#endif

/* Observe stable neutral PWM before applying a motor voltage vector. */
#ifndef FOC_CALIB_NEUTRAL_MS
#define FOC_CALIB_NEUTRAL_MS        2000U
#endif

/* 无电流 PI 时的校准保护阈值；连续超限由 16 kHz 电流任务关断 PWM。 */
#ifndef FOC_CALIB_CURRENT_LIMIT_A
#define FOC_CALIB_CURRENT_LIMIT_A   1.5f
#endif

/* Any single valid sample above this level disables PWM immediately. */
#ifndef FOC_CALIB_HARD_CURRENT_LIMIT_A
#define FOC_CALIB_HARD_CURRENT_LIMIT_A 3.0f
#endif

#ifndef FOC_OVERCURRENT_TRIP_SAMPLES
#define FOC_OVERCURRENT_TRIP_SAMPLES 8U
#endif

/* 找 Z/index 时的开环机械转速，单位 rpm。速度越低，捕获越稳，但等待时间越长。 */
#ifndef FOC_CALIB_SEARCH_RPM
#define FOC_CALIB_SEARCH_RPM        20.0f
#endif

/* D 轴固定对齐保持时间，单位 ms，用于等待转子机械稳定。 */
#ifndef FOC_CALIB_ALIGN_MS
#define FOC_CALIB_ALIGN_MS          800U
#endif

/* 强制清零后的等待时间，单位 ms，给编码器驱动/控制周期一点处理时间。 */
#ifndef FOC_CALIB_ZERO_SETTLE_MS
#define FOC_CALIB_ZERO_SETTLE_MS    20U
#endif

/* 找 Z/index 的超时时间，单位 ms。超时后进入 FAULT，避免一直带电旋转。 */
#ifndef FOC_CALIB_SEARCH_TIMEOUT_MS
#define FOC_CALIB_SEARCH_TIMEOUT_MS 10000U
#endif

/* D 轴对齐时使用的目标电角度，单位 rad。通常先使用 0。 */
#ifndef FOC_CALIB_ALIGN_THETA_E
#define FOC_CALIB_ALIGN_THETA_E     0.0f
#endif

/*
 * 编码器机械角到电角度的方向。
 * 如果校准后 q 轴给正电压时电机方向/力矩不符合预期，可以优先检查这个符号。
 */
#ifndef FOC_CALIB_DIRECTION
#define FOC_CALIB_DIRECTION         -1
#endif

typedef enum {
    /* 未开始校准。 */
    FOC_CALIB_IDLE = 0,

    /* Three low-side switches on: charge the high-side bootstrap capacitors. */
    FOC_CALIB_CHARGE_BOOTSTRAP,

    /* Normal complementary PWM enabled with a zero voltage vector. */
    FOC_CALIB_PWM_NEUTRAL,

    /* D 轴固定对齐中：给 vd，vq=0，让转子吸到已知电角度。 */
    FOC_CALIB_ALIGN_D,

    /* 已对齐并强制清零，短暂等待计数器状态稳定。 */
    FOC_CALIB_CLEAR_AT_ALIGN,

    /* 慢速开环旋转，等待 ABZ 的 Z/index 脉冲。 */
    FOC_CALIB_SEARCH_INDEX,

    /* 校准成功，RAM 中的 electrical_offset_rad 有效。 */
    FOC_CALIB_DONE,

    /* 校准失败，通常是超时没有等到 Z/index。 */
    FOC_CALIB_FAIL
} foc_calib_state_t;

/* 全局可见，便于在 Keil Watch 中直接观察校准状态。 */
extern volatile foc_calib_state_t g_foc_calib_state;
/* Runtime-visible calibration commands for Keil Watch and cautious tuning. */
extern volatile float g_foc_calib_align_voltage;
extern volatile float g_foc_calib_search_voltage;

typedef struct {
    /* 1 表示本次上电已经得到可信校准结果。 */
    uint8_t valid;

    /* 当前校准使用的方向，来自 FOC_CALIB_DIRECTION。 */
    int8_t direction;

    /* Z/index 到来瞬间捕获到的增量编码器计数。 */
    int32_t index_offset_cnt;

    /* index_offset_cnt 换算得到的机械角偏移，单位 rad。 */
    float index_offset_rad;

    /* 给 Park/反 Park 使用的电角度补偿值，单位 rad。 */
    float electrical_offset_rad;
} foc_calib_result_t;

/* 启动一次上电校准。通常在 foc_init() 之后调用。 */
void foc_calib_start(void);

/* 由高频安全检查调用，立即终止校准并锁存 FOC_STATE_FAULT。 */
void foc_calib_abort(void);

/* 校准状态机任务，需要在主循环中周期调用。 */
void foc_calib_task(void);

/* 返回 1 表示校准模块正在接管 PWM 电压命令，主循环不要覆盖 vd/vq。 */
uint8_t foc_calib_is_active(void);

/* 返回 1 表示本次上电校准成功，可以使用 electrical_offset_rad。 */
uint8_t foc_calib_is_valid(void);

/* 获取当前校准状态，可用于 VOFA/上位机观察流程卡在哪一步。 */
foc_calib_state_t foc_calib_get_state(void);

/* 获取完整校准结果，包含 Z 计数、机械角偏移、电角度 offset。 */
const foc_calib_result_t *foc_calib_get_result(void);

/* 只获取电角度 offset，RUN 阶段计算编码器电角度时会用到。 */
float foc_calib_get_electrical_offset(void);

#ifdef __cplusplus
}
#endif

#endif
