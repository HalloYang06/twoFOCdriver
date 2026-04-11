#ifndef H7FOC_LIBRARIES_FOC_SVPWM_H
#define H7FOC_LIBRARIES_FOC_SVPWM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;
} svpwm_result_t;

void svpwm_calculate(float v_alpha, float v_beta, float v_dc, svpwm_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
