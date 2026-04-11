#ifndef H7FOC_LIBRARIES_FOC_FOC_CLARKE_H
#define H7FOC_LIBRARIES_FOC_FOC_CLARKE_H

/* 直接迁移 CMSIS-DSP 的 f32 Clarke 计算，并改为工程内部命名。 */
static inline void foc_clarke_f32(float ia, float ib, float *i_alpha, float *i_beta)
{
    *i_alpha = ia;
    *i_beta = (0.57735026919f * ia + 1.15470053838f * ib);
}

#endif
