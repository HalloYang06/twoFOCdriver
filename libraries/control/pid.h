#ifndef H7FOC_LIBRARIES_CONTROL_PID_H
#define H7FOC_LIBRARIES_CONTROL_PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* 简单 PID 控制器，保留当前工程已经调过的接口风格。 */
typedef struct
{
    float kp;
    float ki;
    float kd;

    float dt;
    float target;
    float output;

    float integral;
    float prev_error;

    float output_limit;
    float integral_limit;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float dt, float output_limit);
void pid_reset(pid_t *pid);
void pid_set_target(pid_t *pid, float target);
float pid_calc(pid_t *pid, float actual_value);

#ifdef __cplusplus
}
#endif

#endif
