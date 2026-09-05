#include "foc_svm.h"
#include "foc_transform.h"
#include "foc_utils.h"

/*
 * 扇区判断：把 αβ 平面按 60° 分成 6 个扇区。
 * 用三条判断线 u1/u2/u3 的符号组合直接映射扇区号，
 * 这是教科书式 SVPWM 的 N=4C+2B+A 判扇区法。
 * 扇区号只用于电流采样窗口规划和调试观察，不参与占空比计算。
 */
static uint8_t foc_svm_sector(const ab_t *ab)
{
    const float u1 = ab->beta;
    const float u2 = (SQRT_3 * ab->alpha) - ab->beta;
    const float u3 = (-SQRT_3 * ab->alpha) - ab->beta;
    const uint8_t code = (uint8_t)((u1 > 0.0f) |
                                   ((u2 > 0.0f) << 1) |
                                   ((u3 > 0.0f) << 2));

    switch (code) {
    case 3U:  return 1U;
    case 1U:  return 2U;
    case 5U:  return 3U;
    case 4U:  return 4U;
    case 6U:  return 5U;
    case 2U:  return 6U;
    default:  return 0U;
    }
}

void foc_svm_calc(const ab_t *v_ab, float u_dc, foc_svm_t *out)
{
    abc_t v;
    float v_max;
    float v_min;
    float span;
    float v_common;
    float inv_udc;

    out->sector = foc_svm_sector(v_ab);

    /* 反 Clarke：αβ → 三相目标电压 */
    foc_inv_clarke(v_ab, &v);

    v_max = foc_max3(v.a, v.b, v.c);
    v_min = foc_min3(v.a, v.b, v.c);

    /* 过调制保护：相间摆幅超过母线电压时整体等比缩小。
     * span = v_max - v_min 是 PWM 必须容纳的总摆幅。 */
    span = v_max - v_min;
    if (span > u_dc) {
        const float scale = u_dc / span;
        v.a *= scale;
        v.b *= scale;
        v.c *= scale;
        v_max *= scale;
        v_min *= scale;
    }

    /* 零序（中点电压）注入：把波形整体挪到 PWM 窗口中央 */
    v_common = -0.5f * (v_max + v_min);
    inv_udc = 1.0f / u_dc;

    /*
     * 占空比安全限幅（对齐 ST MCSDK MAX_MODULATION_100_PER_CENT）：
     * 最大占空比限制在 94.0%，保留足够的下桥导通时间（> 1.8us = 300 tick），
     * 保证三电阻采样在任何转速下都拥有纯净的公共低边窗口，
     * 彻底消除高调制比时切换运放内部通道和采样点镜像产生的虚假过流毛刺。
     */
    out->duty_a = foc_clampf(0.5f + ((v.a + v_common) * inv_udc), 0.060f, 0.940f);
    out->duty_b = foc_clampf(0.5f + ((v.b + v_common) * inv_udc), 0.060f, 0.940f);
    out->duty_c = foc_clampf(0.5f + ((v.c + v_common) * inv_udc), 0.060f, 0.940f);
}
