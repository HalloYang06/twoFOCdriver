#include "svpwm.h"

#include "mc_math.h"

void svpwm_calculate(float v_alpha, float v_beta, float v_dc, svpwm_result_t *result)
{
    float va;
    float vb;
    float vc;
    float v_max;
    float v_min;
    float v_offset;

    va = v_alpha;
    vb = -0.5f * v_alpha + MC_SQRT3_BY_2 * v_beta;
    vc = -0.5f * v_alpha - MC_SQRT3_BY_2 * v_beta;

    v_max = va;
    if (vb > v_max) v_max = vb;
    if (vc > v_max) v_max = vc;

    v_min = va;
    if (vb < v_min) v_min = vb;
    if (vc < v_min) v_min = vc;

    v_offset = 0.5f * (v_max + v_min);

    result->duty_a = mc_math_constrain_f32((va - v_offset) / v_dc + 0.5f, 0.0f, 1.0f);
    result->duty_b = mc_math_constrain_f32((vb - v_offset) / v_dc + 0.5f, 0.0f, 1.0f);
    result->duty_c = mc_math_constrain_f32((vc - v_offset) / v_dc + 0.5f, 0.0f, 1.0f);
}
