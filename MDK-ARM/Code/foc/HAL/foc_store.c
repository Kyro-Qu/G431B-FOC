/**
 * @file    foc_store.c
 * @brief   Flash 参数存储实现（最后一页 2KB，魔数 + 版本 + CRC 校验）
 */

#include "foc_store.h"
#include "foc_config.h"
#include "main.h"
#include <string.h>
#include <stddef.h>

/* STM32G431CB：128KB Flash，2KB/页，共 64 页。用最后一页 */
#define STORE_ADDR   0x0801F800UL
#define STORE_PAGE   63U
#define STORE_MAGIC  0x464F4353UL  /* "FOCS" */
#define STORE_VER    7U  /* v7：极对数 6→7（12N14P 实测），旧块 params 内
                             * 的 pole_pairs=6 会在 load 时覆盖新编译默认，
                             * 必须使旧存储失效重建成编译默认 */

/* 参数块。改字段必须递增 STORE_VER（旧块会被当作无效丢弃） */
typedef struct {
    uint32_t magic;
    uint32_t version;

    foc_motor_params_t params;

    /* 控制环配置（逐字段存，不整包存 struct，避免对齐/改版陷阱） */
    float current_bw_rads;
    float vel_kp;
    float vel_ki;
    float vel_ramp_rpm_s;
    float vel_lpf_tf;
    float vel_friction_a;
    float vel_start_a;
    float vel_start_rpm;
    float vel_track_kp;
    float vel_track_limit_rad;
    float vel_track_rpm;
    float pos_kp;
    float pos_ki;
    float pos_vel_kp;
    float pos_vel_limit_rpm;
    float traj_accel_rpm_s;
    uint8_t traj_enable;
    uint8_t decouple_enable;

    /* 校准结果 */
    uint8_t has_calib;
    int8_t calib_direction;
    float calib_offset_rad;

    uint32_t crc;              /* 覆盖 crc 字段之前的全部字节 */
} store_blob_t;

/* 保证按 8 字节（双字）整倍数编程 */
typedef union {
    store_blob_t blob;
    uint64_t dwords[(sizeof(store_blob_t) + 7U) / 8U];
} store_image_t;

/* 无查找表的 CRC32（反射多项式 0xEDB88320），一百来字节数据足够快 */
static uint32_t store_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint32_t b;

    for (i = 0U; i < len; i++) {
        crc ^= data[i];
        for (b = 0U; b < 8U; b++) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

foc_store_status_t foc_store_load(foc_motor_params_t *params,
                                  foc_ctrl_cfg_t *cfg,
                                  int8_t *calib_direction,
                                  float *calib_offset_rad)
{
#if FOC_STORE_ENABLE
    const store_blob_t *s = (const store_blob_t *)STORE_ADDR;

    if ((s->magic != STORE_MAGIC) || (s->version != STORE_VER)) {
        return FOC_STORE_EMPTY;
    }
    if (store_crc32((const uint8_t *)s, offsetof(store_blob_t, crc)) != s->crc) {
        return FOC_STORE_EMPTY;
    }

    *params = s->params;
    params->hard_current_a = FOC_M0_HARD_CURRENT_A;
    params->max_rpm        = FOC_M0_MAX_RPM;
    cfg->current_bw_rads   = s->current_bw_rads;
    cfg->vel_kp            = s->vel_kp;
    cfg->vel_ki            = s->vel_ki;
    cfg->vel_ramp_rpm_s    = s->vel_ramp_rpm_s;
    cfg->vel_lpf_tf        = s->vel_lpf_tf;
    cfg->vel_friction_a    = s->vel_friction_a;
    cfg->vel_start_a       = s->vel_start_a;
    cfg->vel_start_rpm     = s->vel_start_rpm;
    cfg->vel_track_kp      = s->vel_track_kp;
    cfg->vel_track_limit_rad = s->vel_track_limit_rad;
    cfg->vel_track_rpm     = s->vel_track_rpm;
    cfg->pos_kp            = s->pos_kp;
    cfg->pos_ki            = s->pos_ki;
    cfg->pos_vel_kp        = s->pos_vel_kp;
    cfg->pos_vel_limit_rpm = s->pos_vel_limit_rpm;
    cfg->traj_accel_rpm_s  = s->traj_accel_rpm_s;
    cfg->traj_enable       = s->traj_enable;
    cfg->decouple_enable   = s->decouple_enable;

    if (s->has_calib != 0U) {
        *calib_direction = s->calib_direction;
        *calib_offset_rad = s->calib_offset_rad;
        return FOC_STORE_LOADED_CALIB;
    }
    return FOC_STORE_LOADED;
#else
    (void)params;
    (void)cfg;
    (void)calib_direction;
    (void)calib_offset_rad;
    return FOC_STORE_EMPTY;
#endif
}

static uint8_t store_program(const store_image_t *img)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_err = 0U;
    uint32_t i;
    uint8_t ok = 1U;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return 0U;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = STORE_PAGE;
    erase.NbPages = 1U;
    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) {
        ok = 0U;
    }

    if (ok != 0U) {
        for (i = 0U; i < (sizeof(img->dwords) / 8U); i++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                  STORE_ADDR + (i * 8U),
                                  img->dwords[i]) != HAL_OK) {
                ok = 0U;
                break;
            }
        }
    }

    (void)HAL_FLASH_Lock();

    if (ok != 0U) {
        ok = (memcmp((const void *)STORE_ADDR, img,
                     sizeof(store_blob_t)) == 0) ? 1U : 0U;
    }
    return ok;
}

uint8_t foc_store_save(const foc_motor_t *m)
{
#if FOC_STORE_ENABLE
    static store_image_t img;   /* static：栈上省 ~200B，且仅主循环使用 */

    if ((m->state == FOC_STATE_RUN) || (m->state == FOC_STATE_CALIB)) {
        return 0U;             /* 页擦除阻塞 CPU ~22ms，不许带电操作 */
    }

    memset(&img, 0, sizeof(img));
    img.blob.magic = STORE_MAGIC;
    img.blob.version = STORE_VER;
    img.blob.params = m->params;
    img.blob.current_bw_rads   = m->cfg.current_bw_rads;
    img.blob.vel_kp            = m->pid_vel.kp;
    img.blob.vel_ki            = m->pid_vel.ki;
    img.blob.vel_ramp_rpm_s    = m->cfg.vel_ramp_rpm_s;
    img.blob.vel_lpf_tf        = m->cfg.vel_lpf_tf;
    img.blob.vel_friction_a    = m->cfg.vel_friction_a;
    img.blob.vel_start_a       = m->cfg.vel_start_a;
    img.blob.vel_start_rpm     = m->cfg.vel_start_rpm;
    img.blob.vel_track_kp      = m->cfg.vel_track_kp;
    img.blob.vel_track_limit_rad = m->cfg.vel_track_limit_rad;
    img.blob.vel_track_rpm     = m->cfg.vel_track_rpm;
    img.blob.pos_kp            = m->pid_pos.kp;
    img.blob.pos_ki            = m->pid_pos.ki;
    img.blob.pos_vel_kp        = m->cfg.pos_vel_kp;
    img.blob.pos_vel_limit_rpm = m->cfg.pos_vel_limit_rpm;
    img.blob.traj_accel_rpm_s  = m->cfg.traj_accel_rpm_s;
    img.blob.traj_enable       = m->cfg.traj_enable;
    img.blob.decouple_enable   = m->cfg.decouple_enable;

    /* 只有真正校准成功过（valid）或本就来自存储的偏移才值得保存 */
    if ((m->calib.valid != 0U) || (m->calib.from_store != 0U)) {
        img.blob.has_calib = 1U;
        img.blob.calib_direction = m->calib.direction;
        img.blob.calib_offset_rad = m->calib.electrical_offset_rad;
    }

    img.blob.crc = store_crc32((const uint8_t *)&img.blob,
                               offsetof(store_blob_t, crc));
    return store_program(&img);
#else
    (void)m;
    return 0U;
#endif
}

uint8_t foc_store_erase(void)
{
#if FOC_STORE_ENABLE
    FLASH_EraseInitTypeDef erase;
    uint32_t page_err = 0U;
    uint8_t ok;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return 0U;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = STORE_PAGE;
    erase.NbPages = 1U;
    ok = (HAL_FLASHEx_Erase(&erase, &page_err) == HAL_OK) ? 1U : 0U;
    (void)HAL_FLASH_Lock();
    return ok;
#else
    return 0U;
#endif
}
