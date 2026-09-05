/**
 * @file    foc_store.h
 * @brief   Flash 参数存储（ODrive save_configuration 思路）
 *
 * 解决什么问题？
 *   - 电机参数（Rs/Ls，可能来自 `ident` 自动测量）、PID 整定、
 *     校准出的电角度偏移，每次上电都要重来一遍很麻烦；
 *   - 尤其校准偏移：Z 脉冲在电机上的机械位置是固定的，
 *     "对齐点→Z 点"的偏移天然可以跨上电复用。
 *
 * 方案：
 *   - MCU 最后一页 Flash（0x0801F800，2KB）存一个带魔数+CRC 的参数块；
 *   - 串口 `conf write` 写入（仅 IDLE 允许——页擦除会阻塞总线约 22ms）；
 *   - 上电自动加载：参数覆盖 foc_config.h 的默认值；
 *   - 存储的校准偏移标记为 from_store：上电后编码器零点是随机的，
 *     必须等 Z 脉冲重建零点后偏移才生效——所以还是要过一次校准，
 *     但走"快速索引搜索"（跳过对齐吸附，直接慢转找 Z），
 *     这就是 ODrive encoder.index_search 的语义。
 *
 * 注意：代码增长到接近 126KB（当前约 50KB Flash 镜像）时需要给存储页挪位置。
 */

#ifndef FOC_STORE_H
#define FOC_STORE_H

#include "../Core/foc_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 加载结果 */
typedef enum {
    FOC_STORE_EMPTY = 0,   /* 无有效存储（首次使用/已擦除/CRC 错） */
    FOC_STORE_LOADED,      /* 参数已加载（无校准偏移） */
    FOC_STORE_LOADED_CALIB /* 参数 + 校准偏移都已加载 */
} foc_store_status_t;

/**
 * @brief 上电加载：校验通过则把存储值覆盖进 params/cfg，
 *        并输出校准偏移（供 foc_app 写进电机对象）
 */
foc_store_status_t foc_store_load(foc_motor_params_t *params,
                                  foc_ctrl_cfg_t *cfg,
                                  int8_t *calib_direction,
                                  float *calib_offset_rad);

/**
 * @brief 把轴当前的参数/控制配置/校准结果写入 Flash。
 *        仅在电机 IDLE 时调用（页擦除阻塞 CPU 约 22ms）。
 * @return 1 成功（已回读校验），0 失败
 */
uint8_t foc_store_save(const foc_motor_t *m);

/** 擦除存储（恢复出厂：下次上电用 foc_config.h 默认值） */
uint8_t foc_store_erase(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_STORE_H */
