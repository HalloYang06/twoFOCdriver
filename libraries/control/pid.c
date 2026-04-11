#include "pid.h"

#include "mc_math.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float dt, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->output_limit = output_limit;
    pid->integral_limit = output_limit * 0.8f;

    pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
    pid->target = 0.0f;
    pid->output = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_set_target(pid_t *pid, float target)
{
    pid->target = target;
}

float pid_calc(pid_t *pid, float actual_value)
{
    float error = pid->target - actual_value;
    float p_term = pid->kp * error;

    pid->integral += error * pid->dt;
    pid->integral = mc_math_constrain_f32(pid->integral, -pid->integral_limit, pid->integral_limit);

    float i_term = pid->ki * pid->integral;
    float derivative = (error - pid->prev_error) / pid->dt;
    float d_term = pid->kd * derivative;

    pid->output = p_term + i_term + d_term;
    pid->output = mc_math_constrain_f32(pid->output, -pid->output_limit, pid->output_limit);
    pid->prev_error = error;

    return pid->output;
}
