#include "axis.h"

#include <math.h>

#include "svpwm.h"

static uint8_t axis_encoder_is_ready(const axis_t *axis)
{
    if ((axis == 0) || (axis->encoder_ops == 0) || (axis->encoder_ops->is_ready == 0))
    {
        return 1U;
    }

    return (axis->encoder_ops->is_ready(axis->encoder_ctx) != 0) ? 1U : 0U;
}

static void axis_start_closed_loop(axis_t *axis)
{
    if ((axis == 0) || (axis->foc.enabled != 0U))
    {
        return;
    }

    bsp_pwm_start(&axis->pwm);
    foc_enable(&axis->foc);
    axis->state = AXIS_STATE_RUNNING;
}

void axis_init(axis_t *axis, const axis_cfg_t *cfg)
{
    const float position_output_limit =
        (cfg->pole_pairs > 0U) ? (FOC_VELOCITY_LIMIT / (float)cfg->pole_pairs) : FOC_VELOCITY_LIMIT;

    axis->id = cfg->id;
    axis->mode = AXIS_MODE_DISABLED;
    axis->state = AXIS_STATE_IDLE;
    axis->error_flags = 0U;
    axis->position_mech_rad = 0.0f;
    axis->speed_mech_rad_s = 0.0f;
    axis->target_position = 0.0f;
    axis->move_flag = 0U;
    axis->move_done = 1U;
    axis->enable_pending = 0U;
    axis->current_sense = cfg->current_sense;
    axis->encoder_ops = cfg->encoder_ops;
    axis->encoder_ctx = cfg->encoder_ctx;
    axis->encoder_kind = cfg->encoder_kind;

    pid_init(&axis->pid_position, 12.0f, 0.01f, 0.0f, 0.001f, position_output_limit);
    pid_set_target(&axis->pid_position, 0.0f);

    foc_init(&axis->foc, cfg->pole_pairs, cfg->voltage_supply);
    bsp_pwm_init(&axis->pwm,
                 cfg->pwm_htim,
                 cfg->pwm_channel_u,
                 cfg->pwm_channel_v,
                 cfg->pwm_channel_w,
                 cfg->pwm_period);

    if (axis->current_sense != 0)
    {
        bsp_current_sense_init(axis->current_sense, cfg->hadc);
    }

    if ((axis->encoder_ops != 0) && (axis->encoder_ops->init != 0))
    {
        axis->encoder_ops->init(axis->encoder_ctx);
    }
}

void axis_enable(axis_t *axis)
{
    axis->move_done = 1U;

    if ((axis->encoder_ops != 0) && (axis->encoder_ops->start != 0))
    {
        axis->encoder_ops->start(axis->encoder_ctx);
    }

    if (axis_encoder_is_ready(axis) == 0U)
    {
        axis->enable_pending = 1U;
        axis->state = AXIS_STATE_READY;
        return;
    }

    axis->enable_pending = 0U;
    axis_start_closed_loop(axis);
}

void axis_disable(axis_t *axis)
{
    axis->enable_pending = 0U;
    foc_disable(&axis->foc);
    bsp_pwm_emergency_stop(&axis->pwm);
    axis->move_flag = 0U;
    axis->move_done = 1U;
    axis->target_position = axis->position_mech_rad;
    pid_set_target(&axis->pid_position, axis->target_position);
    pid_reset(&axis->pid_position);

    if ((axis->encoder_ops != 0) && (axis->encoder_ops->stop != 0))
    {
        axis->encoder_ops->stop(axis->encoder_ctx);
    }

    axis->state = AXIS_STATE_IDLE;
}

void axis_set_mode(axis_t *axis, axis_mode_t mode)
{
    axis->mode = mode;

    if (mode != AXIS_MODE_POSITION)
    {
        axis->move_flag = 0U;
        axis->move_done = 1U;
        pid_reset(&axis->pid_position);
    }

    if (mode == AXIS_MODE_DISABLED)
    {
        axis_stop_move(axis);
    }
}

void axis_set_current_ref(axis_t *axis, float iq_ref)
{
    foc_set_current(&axis->foc, iq_ref);
}

void axis_set_velocity_ref(axis_t *axis, float velocity_ref)
{
    foc_set_velocity(&axis->foc, velocity_ref);
}

void axis_set_position_ref(axis_t *axis, float position_ref)
{
    axis->target_position = position_ref;
    pid_set_target(&axis->pid_position, position_ref);
}

void axis_request_move(axis_t *axis)
{
    axis->move_flag = 1U;
    axis->move_done = 0U;
}

void axis_stop_move(axis_t *axis)
{
    axis->move_flag = 0U;
    axis->move_done = 1U;
    axis->target_position = axis->position_mech_rad;
    pid_set_target(&axis->pid_position, axis->target_position);
    pid_reset(&axis->pid_position);
    foc_set_velocity(&axis->foc, 0.0f);
    foc_set_current(&axis->foc, 0.0f);
}

void axis_calibrate_current_sense(axis_t *axis, uint32_t sample_count)
{
    axis->state = AXIS_STATE_CALIBRATING;

    if (axis->current_sense != 0)
    {
        bsp_current_sense_calibrate(axis->current_sense, sample_count);
    }

    axis->state = AXIS_STATE_READY;
}

void axis_calibrate_encoder(axis_t *axis)
{
    axis->state = AXIS_STATE_CALIBRATING;

    if ((axis->encoder_ops != 0) && (axis->encoder_ops->align_electric_zero != 0))
    {
        axis->encoder_ops->align_electric_zero(axis->encoder_ctx);
    }

    axis->state = AXIS_STATE_READY;
}

void axis_feedback_update_handler(axis_t *axis, float dt)
{
    float speed_rad_s = 0.0f;
    float position_rad = axis->position_mech_rad;

    if ((axis->encoder_ops != 0) && (axis->encoder_ops->update != 0))
    {
        axis->encoder_ops->update(axis->encoder_ctx, dt);
    }

    if ((axis->encoder_ops != 0) && (axis->encoder_ops->get_angle_mech_rad != 0))
    {
        position_rad = axis->encoder_ops->get_angle_mech_rad(axis->encoder_ctx);
    }

    if ((axis->encoder_ops != 0) && (axis->encoder_ops->get_speed_rad_s != 0))
    {
        speed_rad_s = axis->encoder_ops->get_speed_rad_s(axis->encoder_ctx);
    }

    axis->position_mech_rad = position_rad;
    axis->speed_mech_rad_s = speed_rad_s;
    foc_set_speed_feedback(&axis->foc, speed_rad_s * (float)axis->foc.pole_pairs);

    if ((axis->enable_pending != 0U) && (axis_encoder_is_ready(axis) != 0U))
    {
        axis->enable_pending = 0U;
        axis_start_closed_loop(axis);
    }
}

void axis_outer_loop_handler(axis_t *axis)
{
    if (axis->state != AXIS_STATE_RUNNING)
    {
        return;
    }

    if (axis->mode == AXIS_MODE_VELOCITY)
    {
        foc_cal_velocity_loop(&axis->foc);
    }

    if (axis->mode == AXIS_MODE_POSITION)
    {
        const float position_error = axis->target_position - axis->position_mech_rad;

        if (axis->move_flag != 0U)
        {
            const float position_velocity_ref = pid_calc(&axis->pid_position, axis->position_mech_rad);

            foc_set_velocity(&axis->foc, position_velocity_ref * (float)axis->foc.pole_pairs);
            foc_cal_velocity_loop(&axis->foc);

            if ((fabsf(position_error) <= 0.01f) && (fabsf(axis->speed_mech_rad_s) <= 0.01f))
            {
                axis->move_flag = 0U;
                axis->move_done = 1U;
                pid_set_target(&axis->pid_position, axis->position_mech_rad);
                pid_reset(&axis->pid_position);
                foc_set_velocity(&axis->foc, 0.0f);
                foc_set_current(&axis->foc, 0.0f);
            }
        }
        else
        {
            foc_set_velocity(&axis->foc, 0.0f);
            foc_cal_velocity_loop(&axis->foc);
        }
    }
}

void axis_slow_loop_handler(axis_t *axis, float dt)
{
    axis_feedback_update_handler(axis, dt);
    axis_outer_loop_handler(axis);
}

void axis_current_loop_irq_handler(axis_t *axis,
                                   uint16_t sample_ch15,
                                   uint16_t sample_ch3,
                                   uint16_t sample_ch8)
{
    phase_currents_t currents;
    svpwm_result_t svpwm;
    float angle_rad = 0.0f;

    if (axis->current_sense != 0)
    {
        bsp_current_sense_update_injected(axis->current_sense, sample_ch15, sample_ch3, sample_ch8);
    }

    if (axis->foc.enabled == 0U)
    {
        return;
    }

    if ((axis->encoder_ops != 0) && (axis->encoder_ops->get_angle_elec_rad != 0))
    {
        angle_rad = axis->encoder_ops->get_angle_elec_rad(axis->encoder_ctx);
    }

    foc_update_angle(&axis->foc, angle_rad);

    if (axis->current_sense != 0)
    {
        bsp_current_sense_get_currents(axis->current_sense, &currents.ia, &currents.ib, &currents.ic);
    }
    else
    {
        currents.ia = 0.0f;
        currents.ib = 0.0f;
        currents.ic = 0.0f;
    }

    foc_update_currents(&axis->foc, &currents);
    foc_cal_current_loop(&axis->foc);
    foc_build_voltage_vector(&axis->foc);

    svpwm_calculate(axis->foc.v_alpha_beta.alpha,
                    axis->foc.v_alpha_beta.beta,
                    axis->foc.voltage_supply,
                    &svpwm);

    axis->foc.duty_a = svpwm.duty_a;
    axis->foc.duty_b = svpwm.duty_b;
    axis->foc.duty_c = svpwm.duty_c;

    bsp_pwm_set_duty(&axis->pwm, svpwm.duty_a, svpwm.duty_b, svpwm.duty_c);
}
