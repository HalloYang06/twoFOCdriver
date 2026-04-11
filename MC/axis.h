#ifndef H7FOC_MC_AXIS_H
#define H7FOC_MC_AXIS_H

#include "bsp_current_sense.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "mc_foc.h"
#include "pid.h"

typedef struct
{
    axis_id_t id;
    axis_mode_t mode;
    axis_state_t state;
    uint32_t error_flags;
    float position_mech_rad;
    float speed_mech_rad_s;
    float target_position;
    uint8_t move_flag;
    uint8_t move_done;
    uint8_t enable_pending;

    pid_t pid_position;
    foc_t foc;
    bsp_pwm_t pwm;
    bsp_current_sense_t *current_sense;

    const encoder_if_t *encoder_ops;
    void *encoder_ctx;
    encoder_kind_t encoder_kind;
} axis_t;

typedef struct
{
    axis_id_t id;
    uint8_t pole_pairs;
    float voltage_supply;

    const encoder_if_t *encoder_ops;
    void *encoder_ctx;
    encoder_kind_t encoder_kind;

    bsp_current_sense_t *current_sense;
    ADC_HandleTypeDef *hadc;

    TIM_HandleTypeDef *pwm_htim;
    uint32_t pwm_channel_u;
    uint32_t pwm_channel_v;
    uint32_t pwm_channel_w;
    uint16_t pwm_period;
} axis_cfg_t;

void axis_init(axis_t *axis, const axis_cfg_t *cfg);
void axis_enable(axis_t *axis);
void axis_disable(axis_t *axis);
void axis_set_mode(axis_t *axis, axis_mode_t mode);
void axis_set_current_ref(axis_t *axis, float iq_ref);
void axis_set_velocity_ref(axis_t *axis, float velocity_ref);
void axis_set_position_ref(axis_t *axis, float position_ref);
void axis_request_move(axis_t *axis);
void axis_stop_move(axis_t *axis);
void axis_calibrate_current_sense(axis_t *axis, uint32_t sample_count);
void axis_calibrate_encoder(axis_t *axis);
void axis_slow_loop_handler(axis_t *axis, float dt);
void axis_current_loop_irq_handler(axis_t *axis,
                                   uint16_t sample_ch15,
                                   uint16_t sample_ch3,
                                   uint16_t sample_ch8);

#endif
