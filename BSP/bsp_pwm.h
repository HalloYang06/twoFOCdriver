#ifndef H7FOC_BSP_BSP_PWM_H
#define H7FOC_BSP_BSP_PWM_H

#include "main.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel_u;
    uint32_t channel_v;
    uint32_t channel_w;
    uint16_t period;
} bsp_pwm_t;

void bsp_pwm_init(bsp_pwm_t *pwm,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel_u,
                  uint32_t channel_v,
                  uint32_t channel_w,
                  uint16_t period);
void bsp_pwm_start(bsp_pwm_t *pwm);
void bsp_pwm_stop(bsp_pwm_t *pwm);
void bsp_pwm_set_duty(bsp_pwm_t *pwm, float duty_u, float duty_v, float duty_w);
void bsp_pwm_emergency_stop(bsp_pwm_t *pwm);

#endif
