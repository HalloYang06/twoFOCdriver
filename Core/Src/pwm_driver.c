#include "pwm_driver.h"
#include "tim.h"

/**
 * @brief  PWM初始化函数
 * @note   启动TIM1的3路互补PWM输出
 * @retval None
 */
void PWM_Init(void)
{
    /* 启动PWM通道1及其互补通道 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

    /* 启动PWM通道2及其互补通道 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

    /* 启动PWM通道3及其互补通道 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief  设置PWM占空比
 * @param  htim: 定时器句柄指针
 * @param  channel: PWM通道 (TIM_CHANNEL_1/2/3)
 * @param  duty_cycle: 占空比值 (0-2999)
 * @retval None
 */
void PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t duty_cycle)
{
    /* 限制占空比范围 */
    if (duty_cycle > 2999) {
        duty_cycle = 2999;
    }

    /* 设置比较值 */
    __HAL_TIM_SET_COMPARE(htim, channel, duty_cycle);
}

/**
 * @brief  紧急停止PWM输出
 * @note   立即停止所有PWM输出并将占空比设为0
 * @retval None
 */
void PWM_EmergencyStop(void)
{
    /* 将所有占空比设为0 */
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, 0);
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, 0);
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, 0);

    /* 停止PWM输出 */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}
