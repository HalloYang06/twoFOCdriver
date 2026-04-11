#ifndef H7FOC_LIBRARIES_FOC_FOC_PARK_H
#define H7FOC_LIBRARIES_FOC_FOC_PARK_H

/* 直接迁移 CMSIS-DSP 的 f32 Park 计算，并改为工程内部命名。 */
static inline void foc_park_f32(float i_alpha,
                                float i_beta,
                                float *i_d,
                                float *i_q,
                                float sin_val,
                                float cos_val)
{
    *i_d = i_alpha * cos_val + i_beta * sin_val;
    *i_q = -i_alpha * sin_val + i_beta * cos_val;
}

#endif
