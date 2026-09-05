#ifndef CURRENT_SHUNT_H
#define CURRENT_SHUNT_H

#include <stdint.h>

/**
 * @brief 三电阻低边电流采样状态。
 *
 * 每个 PWM 周期只有两路 ADC 转换，因此零偏校准分两段进行。
 */
typedef enum {
    CURRENT_SHUNT_IDLE = 0,
    CURRENT_SHUNT_CAL_UV,
    CURRENT_SHUNT_CAL_W,
    CURRENT_SHUNT_CAL_W_INTERNAL,
    CURRENT_SHUNT_READY
} current_shunt_state_t;

/*
 * 锁存的电流采样故障原因。驱动退回 IDLE 后，
 * 该值仍保留在 g_current_shunt_diag.fault_code 中供调试查看。
 */
typedef enum {
    CURRENT_SHUNT_FAULT_NONE = 0,
    CURRENT_SHUNT_FAULT_OPAMP1_START,
    CURRENT_SHUNT_FAULT_OPAMP2_START,
    CURRENT_SHUNT_FAULT_OPAMP3_START,
    CURRENT_SHUNT_FAULT_ADC1_CALIBRATION,
    CURRENT_SHUNT_FAULT_ADC2_CALIBRATION,
    CURRENT_SHUNT_FAULT_ADC1_READY_TIMEOUT,
    CURRENT_SHUNT_FAULT_ADC2_READY_TIMEOUT,
    CURRENT_SHUNT_FAULT_ADC1_STOP_TIMEOUT,
    CURRENT_SHUNT_FAULT_ADC2_STOP_TIMEOUT,
    CURRENT_SHUNT_FAULT_ADC1_CONTEXT_FLUSH,
    CURRENT_SHUNT_FAULT_ADC2_CONTEXT_FLUSH,
    CURRENT_SHUNT_FAULT_CALIBRATION_TIMEOUT,
    CURRENT_SHUNT_FAULT_CONTEXT_NOT_ARMED,
    CURRENT_SHUNT_FAULT_ADC1_JEOS_MISSING,
    CURRENT_SHUNT_FAULT_QUEUE_OVERFLOW,
    CURRENT_SHUNT_FAULT_CAL_UV_PAIR,
    CURRENT_SHUNT_FAULT_CAL_W_PAIR,
    CURRENT_SHUNT_FAULT_CONTEXT_NOT_CONSUMED,
    CURRENT_SHUNT_FAULT_INVALID_PAIR,
    CURRENT_SHUNT_FAULT_OFFSET_RANGE,
    CURRENT_SHUNT_FAULT_INVALID_WINDOW,
    /* ADC 上下文已启动，但连续两个 TIM1 更新都未等到 ADC2 JEOS。 */
    CURRENT_SHUNT_FAULT_ADC_RESULT_TIMEOUT,
    CURRENT_SHUNT_FAULT_CURRENT_DISCONTINUITY
} current_shunt_fault_t;

/* 驱动最后到达的初始化/校准阶段。 */
typedef enum {
    CURRENT_SHUNT_STAGE_RESET = 0,
    CURRENT_SHUNT_STAGE_OPAMP1,
    CURRENT_SHUNT_STAGE_OPAMP2,
    CURRENT_SHUNT_STAGE_OPAMP3,
    CURRENT_SHUNT_STAGE_ADC1_CALIBRATION,
    CURRENT_SHUNT_STAGE_ADC2_CALIBRATION,
    CURRENT_SHUNT_STAGE_ADC1_ENABLE,
    CURRENT_SHUNT_STAGE_ADC2_ENABLE,
    CURRENT_SHUNT_STAGE_INITIALIZED,
    CURRENT_SHUNT_STAGE_CAL_UV,
    CURRENT_SHUNT_STAGE_CAL_W,
    CURRENT_SHUNT_STAGE_CAL_W_INTERNAL,
    CURRENT_SHUNT_STAGE_RUNNING
} current_shunt_stage_t;

/*
 * 供应用层和调试器只读查看的诊断快照。
 * 零偏字段为 12 位 ADC 码值，电流字段单位为安培。
 * 所有字段在中断上下文中更新。
 */
typedef struct {
    volatile uint16_t offset_u;
    volatile uint16_t offset_v;
    volatile uint16_t offset_w;          /* W external: PB1 -> ADC1_IN12 */
    volatile uint16_t offset_w_internal; /* W internal: OPAMP3 -> ADC2_IN18 */
    volatile uint16_t adc1_raw;
    volatile uint16_t adc2_raw;
    volatile float current_u;
    volatile float current_v;
    volatile float current_w;
    volatile uint32_t sample_count;
    volatile uint32_t tim_update_count;
    volatile uint32_t adc_irq_count;
    volatile uint32_t rejected_sample_count;
    volatile float rejected_current_u;
    volatile float rejected_current_v;
    volatile float rejected_current_w;
    volatile uint16_t rejected_adc1_raw;
    volatile uint16_t rejected_adc2_raw;
    volatile uint16_t rejected_ccr1;
    volatile uint16_t rejected_ccr2;
    volatile uint16_t rejected_ccr3;
    volatile uint16_t rejected_ccr4;
    volatile uint8_t rejected_pair;
    volatile uint8_t rejected_consecutive;
    volatile uint32_t adc_deferred_count; /* 累计被容忍/导致故障的延迟次数。 */
    volatile uint8_t adc_deferred_consecutive; /* 0/1 正常，2 触发故障 22。 */
    volatile uint32_t adc1_isr;
    volatile uint32_t adc2_isr;
    volatile uint32_t adc1_jsqr;
    volatile uint32_t adc2_jsqr;
    volatile uint32_t adc1_cr;
    volatile uint32_t adc2_cr;
    volatile uint32_t tim_cnt;
    volatile uint32_t tim_cr1;
    volatile uint8_t state;
    volatile uint8_t fault_code;
    volatile uint8_t init_stage;
    volatile uint8_t active_pair;
    volatile uint8_t pending_pair;
    volatile uint8_t contexts_armed;
    volatile uint8_t sector;
} current_shunt_diag_t;

extern volatile current_shunt_diag_t g_current_shunt_diag;

/* 启动 OPAMP，校准并使能 ADC，装填注入转换通路。 */
uint8_t current_shunt_init(void);

/*
 * 在 PWM 主输出关闭的状态下采集零电流偏置。
 * 本调用阻塞直到校准完成或 timeout_ms 超时。
 */
uint8_t current_shunt_calibrate(uint32_t timeout_ms);

/*
 * 处理已完成的 ADC1/ADC2 注入采样。
 * 每个 PWM 周期由 ADC1_2_IRQHandler 在 ADC2 JEOS 之后调用一次。
 * 仅当重建出有效运行电流时返回 1。
 */
uint8_t current_shunt_adc_irq(void);

/*
 * 在 TIM1 更新事件中提交待定的 ADC 注入上下文。
 * 本函数独占“关 TRGO -> 更新 JSQR -> 开 TRGO”的安全序列。
 */
void current_shunt_tim_update_irq(void);

/*
 * 根据 PWM 比较值规划下一周期的采样组合和 CH4 触发点。
 * sector 参数仅为 FOC 接口保留；实际路由按 CCR 大小排序决定。
 */
void current_shunt_prepare_pwm(uint32_t ccr_u,
                               uint32_t ccr_v,
                               uint32_t ccr_w,
                               uint8_t sector);

/* 将最新重建的 U/V/W 相电流（安培）拷贝到非空输出指针。 */
void current_shunt_get_currents(float *iu, float *iv, float *iw);

/* 仅在零偏校准完成且无采样故障后返回 1。 */
uint8_t current_shunt_is_ready(void);

/*
 * 在会阻塞 Flash 取指的操作（页擦除/编程）前后暂停/恢复
 * TIM1 触发的注入转换。仅在 PWM 输出关闭时调用。
 * 零偏和 READY 状态保持不变。
 */
uint8_t current_shunt_suspend(void);
uint8_t current_shunt_resume(void);

/*
 * 允许一次"预期的大电流暂态"（如无感角度源切换、参数辨识阶跃）：
 * 宽限期内单拍 step 判据放宽到 grace_a（不拒绝、不累计、不停机），
 * 任何一拍 step 回到 3.0A 正常判据内则立即结束宽限。
 * 不改 ready/fault 状态，硬过流保护全程有效。
 * grace_a<=0 立即结束宽限。
 */
void current_shunt_allow_transient(float grace_a);

/*
 * 清除连续采样拒绝与瞬态故障状态。
 * 在用户发送 fault clear 或系统尝试清除 FAULT 时调用，
 * 恢复采样链路的正常接收与连续性判据，消除误判导致的快环假死死锁。
 */
void current_shunt_reset_discontinuity(void);

#endif /* CURRENT_SHUNT_H */
