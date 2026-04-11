#ifndef H7FOC_LIBRARIES_FOC_FOC_SINCOS_H
#define H7FOC_LIBRARIES_FOC_FOC_SINCOS_H

/*
 * foc_sincos.h — 已废弃，sin/cos 实现已移至 mc_math.h 的 mc_sin_cos_f32()。
 * 本文件仅保留向后兼容的内联包装，新代码请直接使用 mc_sin_cos_f32()。
 */
#include "mc_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 兼容旧接口：输入为度，内部转弧度后调用 mc_sin_cos_f32 */
static inline void foc_sin_cos_f32(float theta_deg, float *p_sin, float *p_cos)
{
    mc_sin_cos_f32(theta_deg * MC_DEG_TO_RAD, p_sin, p_cos);
}

#ifdef __cplusplus
}
#endif

#endif
