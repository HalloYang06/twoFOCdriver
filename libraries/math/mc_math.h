#ifndef H7FOC_LIBRARIES_MATH_MC_MATH_H
#define H7FOC_LIBRARIES_MATH_MC_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 数学常量 ─────────────────────────────────────────── */
#define MC_PI              3.14159265358979f
#define MC_TWO_PI          6.28318530717959f
#define MC_HALF_PI         1.57079632679490f
#define MC_SQRT3           1.73205080756888f
#define MC_SQRT3_BY_2      0.86602540378444f
#define MC_ONE_BY_SQRT3    0.57735026918963f
#define MC_ONE_BY_TWO_PI   0.15915494309190f  /* 1/(2*PI) */
#define MC_RAD_TO_DEG      57.2957795130823f
#define MC_DEG_TO_RAD      0.01745329251994f

/* ── 正弦查找表 (表本体在 mc_sin_table.c) ─────────────── */
#define MC_SIN_TABLE_SIZE  512U
extern const float mc_sin_table_f32[MC_SIN_TABLE_SIZE + 1U];

/* ── 热路径内联函数 ──────────────────────────────────── */

/* 限幅 */
static inline float mc_constrain_f32(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* 角度归一化到 [0, 2π)，不用 fmodf，避免软件库调用开销 */
static inline float mc_normalize_angle(float angle)
{
    while (angle >= MC_TWO_PI) angle -= MC_TWO_PI;
    while (angle < 0.0f)      angle += MC_TWO_PI;
    return angle;
}

/* 机械角度 → 电角度 */
static inline float mc_electrical_angle(float mech_rad, uint8_t pole_pairs)
{
    return mc_normalize_angle(mech_rad * (float)pole_pairs);
}

/* 角度单位转换 */
static inline float mc_deg_to_rad(float deg) { return deg * MC_DEG_TO_RAD; }
static inline float mc_rad_to_deg(float rad) { return rad * MC_RAD_TO_DEG; }

/*
 * 快速 sin/cos 计算，512 点查找表 + 三次 Hermite 插值。
 * 输入：弧度（支持任意正负值）。精度与 CMSIS-DSP arm_sin_cos_f32 一致。
 * 已内联，适合在 20kHz 电流环中断中调用。
 */
static inline void mc_sin_cos_f32(float theta_rad, float *p_sin, float *p_cos)
{
    float in, findex, fract;
    uint16_t idx_s, idx_c;
    float f1, f2, d1, d2, df, temp;
    const float dn = 0.0122718463030f;  /* 2*PI / TABLE_SIZE */

    /*
     * 先归一化到 [0, 2π) 再转换到 [0, 1)。
     * 不用 -in+1 的技巧，那个会导致 cos 在负角度时出错。
     */
    in = theta_rad * MC_ONE_BY_TWO_PI;
    in = in - (float)((int32_t)in);       /* 取小数部分，可能为负 */
    if (in < 0.0f) in += 1.0f;            /* 保证 [0, 1) */

    findex = (float)MC_SIN_TABLE_SIZE * in;
    idx_s = ((uint16_t)findex) & 0x1FFU;
    idx_c = (idx_s + (MC_SIN_TABLE_SIZE / 4U)) & 0x1FFU;
    fract = findex - (float)(uint16_t)findex;

    /* 余弦：三次 Hermite 插值 */
    f1 = mc_sin_table_f32[idx_c];
    f2 = mc_sin_table_f32[idx_c + 1U];
    d1 = -mc_sin_table_f32[idx_s];
    d2 = -mc_sin_table_f32[idx_s + 1U];
    df = f2 - f1;
    temp = dn * (d1 + d2) - 2.0f * df;
    temp = fract * temp + (3.0f * df - (d2 + 2.0f * d1) * dn);
    temp = fract * temp + d1 * dn;
    *p_cos = fract * temp + f1;

    /* 正弦：三次 Hermite 插值 */
    f1 = mc_sin_table_f32[idx_s];
    f2 = mc_sin_table_f32[idx_s + 1U];
    d1 = mc_sin_table_f32[idx_c];
    d2 = mc_sin_table_f32[idx_c + 1U];
    df = f2 - f1;
    temp = dn * (d1 + d2) - 2.0f * df;
    temp = fract * temp + (3.0f * df - (d2 + 2.0f * d1) * dn);
    temp = fract * temp + d1 * dn;
    *p_sin = fract * temp + f1;
}

/*
 * 快速平方根。Cortex-M7 有 VSQRT.F32 硬件指令，单周期完成。
 * 非 ARM 平台回退到 Newton-Raphson 近似。
 */
static inline float mc_sqrt_f32(float x)
{
#if defined(__ARM_FP) && (__ARM_FP >= 4)
    float result;
    __asm volatile ("vsqrt.f32 %0, %1" : "=t"(result) : "t"(x));
    return result;
#else
    if (x <= 0.0f) return 0.0f;
    float half = 0.5f * x;
    int32_t i;
    float y = x;
    __builtin_memcpy(&i, &y, sizeof(i));
    i = 0x5F3759DF - (i >> 1);
    __builtin_memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - half * y * y);
    y = y * (1.5f - half * y * y);
    return x * y;
#endif
}

/* ── 向后兼容别名（旧代码无需改动） ──────────────────── */
#define mc_math_constrain_f32    mc_constrain_f32
#define mc_math_normalize_angle  mc_normalize_angle
#define mc_math_electrical_angle mc_electrical_angle
#define mc_math_deg_to_rad       mc_deg_to_rad
#define mc_math_rad_to_deg       mc_rad_to_deg

#ifdef __cplusplus
}
#endif

#endif
