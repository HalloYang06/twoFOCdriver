#ifndef H7FOC_MC_FOC_H
#define H7FOC_MC_FOC_H

#include <stdint.h>

#include "mc_types.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_VOLTAGE_LIMIT   24.0f
#define FOC_CURRENT_LIMIT   10.0f
#define FOC_VELOCITY_LIMIT  1000.0f

typedef struct
{
    pid_t pid_id;
    pid_t pid_iq;
    pid_t pid_velocity;

    phase_currents_t i_abc;
    alpha_beta_t i_alpha_beta;
    dq_frame_t i_dq;

    dq_frame_t v_dq;
    alpha_beta_t v_alpha_beta;

    float duty_a;
    float duty_b;
    float duty_c;

    float theta_elec;
    float omega_elec;

    float target_velocity;
    float target_id;
    float target_iq;

    uint8_t pole_pairs;
    float voltage_supply;

    uint8_t enabled;
    uint8_t open_loop;
} foc_t;

void foc_init(foc_t *foc, uint8_t pole_pairs, float voltage_supply);
void foc_set_velocity(foc_t *foc, float target_velocity);
void foc_set_current(foc_t *foc, float target_iq);
void foc_update_angle(foc_t *foc, float electrical_angle);
void foc_set_speed_feedback(foc_t *foc, float electrical_speed);
void foc_update_currents(foc_t *foc, const phase_currents_t *currents);
void foc_cal_current_loop(foc_t *foc);
void foc_cal_velocity_loop(foc_t *foc);
void foc_build_voltage_vector(foc_t *foc);
void foc_enable(foc_t *foc);
void foc_disable(foc_t *foc);

#ifdef __cplusplus
}
#endif

#endif
