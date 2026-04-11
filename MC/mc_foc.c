#include "mc_foc.h"

#include <string.h>

#include "foc_clarke.h"
#include "foc_ipark.h"
#include "foc_park.h"
#include "mc_math.h"

void foc_init(foc_t *foc, uint8_t pole_pairs, float voltage_supply)
{
    memset(foc, 0, sizeof(*foc));

    foc->pole_pairs = pole_pairs;
    foc->voltage_supply = voltage_supply;

    
    pid_init(&foc->pid_id, 1.2f, 60.0f, 0.0f, 0.00005f, FOC_VOLTAGE_LIMIT);
    pid_init(&foc->pid_iq, 1.2f, 60.0f, 0.0f, 0.00005f, FOC_VOLTAGE_LIMIT);
    pid_init(&foc->pid_velocity, 0.5f, 2.0f, 0.01f, 0.001f, FOC_CURRENT_LIMIT);
}

void foc_set_velocity(foc_t *foc, float target_velocity)
{
    foc->target_velocity = mc_math_constrain_f32(target_velocity, -FOC_VELOCITY_LIMIT, FOC_VELOCITY_LIMIT);
    pid_set_target(&foc->pid_velocity, foc->target_velocity);
}

void foc_set_current(foc_t *foc, float target_iq)
{
    foc->target_iq = mc_math_constrain_f32(target_iq, -FOC_CURRENT_LIMIT, FOC_CURRENT_LIMIT);
    pid_set_target(&foc->pid_iq, foc->target_iq);
}

void foc_update_angle(foc_t *foc, float electrical_angle)
{
    foc->theta_elec = mc_math_normalize_angle(electrical_angle);
}

void foc_set_speed_feedback(foc_t *foc, float electrical_speed)
{
    foc->omega_elec = electrical_speed;
}

void foc_update_currents(foc_t *foc, const phase_currents_t *currents)
{
    float sin_val;
    float cos_val;

    foc->i_abc = *currents;

    foc_clarke_f32(foc->i_abc.ia, foc->i_abc.ib, &foc->i_alpha_beta.alpha, &foc->i_alpha_beta.beta);
    mc_sin_cos_f32(foc->theta_elec, &sin_val, &cos_val);
    foc_park_f32(foc->i_alpha_beta.alpha,
                 foc->i_alpha_beta.beta,
                 &foc->i_dq.d,
                 &foc->i_dq.q,
                 sin_val,
                 cos_val);
}

void foc_cal_current_loop(foc_t *foc)
{
    float v_magnitude;

    foc->v_dq.d = pid_calc(&foc->pid_id, foc->i_dq.d);
    foc->v_dq.q = pid_calc(&foc->pid_iq, foc->i_dq.q);

    v_magnitude = mc_sqrt_f32(foc->v_dq.d * foc->v_dq.d + foc->v_dq.q * foc->v_dq.q);
    if (v_magnitude > FOC_VOLTAGE_LIMIT)
    {
        float scale = FOC_VOLTAGE_LIMIT / v_magnitude;
        foc->v_dq.d *= scale;
        foc->v_dq.q *= scale;
    }
}

void foc_cal_velocity_loop(foc_t *foc)
{
    foc->target_iq = pid_calc(&foc->pid_velocity, foc->omega_elec);
    pid_set_target(&foc->pid_iq, foc->target_iq);
}

void foc_build_voltage_vector(foc_t *foc)
{
    float sin_val;
    float cos_val;

    mc_sin_cos_f32(foc->theta_elec, &sin_val, &cos_val);
    foc_inv_park_f32(foc->v_dq.d,
                     foc->v_dq.q,
                     &foc->v_alpha_beta.alpha,
                     &foc->v_alpha_beta.beta,
                     sin_val,
                     cos_val);
}

void foc_enable(foc_t *foc)
{
    foc->enabled = 1U;
}

void foc_disable(foc_t *foc)
{
    foc->enabled = 0U;
    pid_reset(&foc->pid_id);
    pid_reset(&foc->pid_iq);
    pid_reset(&foc->pid_velocity);
}
