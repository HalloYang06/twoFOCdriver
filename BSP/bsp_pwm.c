#include "bsp_pwm.h"

#include "mc_math.h"

void bsp_pwm_init(bsp_pwm_t *pwm,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel_u,
                  uint32_t channel_v,
                  uint32_t channel_w,
                  uint16_t period)
{
    pwm->htim = htim;
    pwm->channel_u = channel_u;
    pwm->channel_v = channel_v;
    pwm->channel_w = channel_w;
    pwm->period = period;
}

void bsp_pwm_start(bsp_pwm_t *pwm)
{
    /* 启动三相 PWM 及其互补输出，保持和当前硬件配置一致。 */
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel_u);
    HAL_TIMEx_PWMN_Start(pwm->htim, pwm->channel_u);

    HAL_TIM_PWM_Start(pwm->htim, pwm->channel_v);
    HAL_TIMEx_PWMN_Start(pwm->htim, pwm->channel_v);

    HAL_TIM_PWM_Start(pwm->htim, pwm->channel_w);
    HAL_TIMEx_PWMN_Start(pwm->htim, pwm->channel_w);
}

void bsp_pwm_stop(bsp_pwm_t *pwm)
{
    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel_u);
    HAL_TIMEx_PWMN_Stop(pwm->htim, pwm->channel_u);

    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel_v);
    HAL_TIMEx_PWMN_Stop(pwm->htim, pwm->channel_v);

    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel_w);
    HAL_TIMEx_PWMN_Stop(pwm->htim, pwm->channel_w);
}

void bsp_pwm_set_duty(bsp_pwm_t *pwm, float duty_u, float duty_v, float duty_w)
{
    uint32_t ccr_u;
    uint32_t ccr_v;
    uint32_t ccr_w;

    duty_u = mc_math_constrain_f32(duty_u, 0.0f, 1.0f);
    duty_v = mc_math_constrain_f32(duty_v, 0.0f, 1.0f);
    duty_w = mc_math_constrain_f32(duty_w, 0.0f, 1.0f);

    ccr_u = (uint32_t)(duty_u * (float)pwm->period);
    ccr_v = (uint32_t)(duty_v * (float)pwm->period);
    ccr_w = (uint32_t)(duty_w * (float)pwm->period);

    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel_u, ccr_u);
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel_v, ccr_v);
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel_w, ccr_w);
}

void bsp_pwm_emergency_stop(bsp_pwm_t *pwm)
{
    bsp_pwm_set_duty(pwm, 0.0f, 0.0f, 0.0f);
    bsp_pwm_stop(pwm);
}
