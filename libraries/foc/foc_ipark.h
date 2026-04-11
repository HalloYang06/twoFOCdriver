#ifndef H7FOC_LIBRARIES_FOC_FOC_IPARK_H
#define H7FOC_LIBRARIES_FOC_FOC_IPARK_H

/* 直接迁移 CMSIS-DSP 的 f32 逆 Park 计算，并改为工程内部命名。 */
static inline void foc_inv_park_f32(float v_d,
                                    float v_q,
                                    float *v_alpha,
                                    float *v_beta,
                                    float sin_val,
                                    float cos_val)
{
    *v_alpha = v_d * cos_val - v_q * sin_val;
    *v_beta = v_d * sin_val + v_q * cos_val;
}

#endif
