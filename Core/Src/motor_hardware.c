#include "motor_hardware.h"
#include <string.h>

/* ==================== PWM驱动函数实现 ==================== */

/**
 * @brief  初始化PWM驱动
 * @param  hal: 硬件抽象层指针
 * @retval None
 */
void MotorHAL_PWM_Init(MotorHAL_TypeDef *hal)
{

    uint16_t half_duty = hal->pwm_config.period / 2;

    __HAL_TIM_SET_COMPARE(hal->pwm_config.timer, hal->pwm_config.channel_u, half_duty);
    __HAL_TIM_SET_COMPARE(hal->pwm_config.timer, hal->pwm_config.channel_v, half_duty);
    __HAL_TIM_SET_COMPARE(hal->pwm_config.timer, hal->pwm_config.channel_w, half_duty);
}

/**
 * @brief  启动PWM输出
 * @param  hal: 硬件抽象层指针
 * @retval None
 */
void MotorHAL_PWM_Start(MotorHAL_TypeDef *hal)
{
    // 启动PWM和互补PWM输出
    HAL_TIM_PWM_Start(hal->pwm_config.timer, hal->pwm_config.channel_u);
    HAL_TIMEx_PWMN_Start(hal->pwm_config.timer, hal->pwm_config.channel_u);

    HAL_TIM_PWM_Start(hal->pwm_config.timer, hal->pwm_config.channel_v);
    HAL_TIMEx_PWMN_Start(hal->pwm_config.timer, hal->pwm_config.channel_v);

    HAL_TIM_PWM_Start(hal->pwm_config.timer, hal->pwm_config.channel_w);
    HAL_TIMEx_PWMN_Start(hal->pwm_config.timer, hal->pwm_config.channel_w);
}

/**
 * @brief  停止PWM输出
 * @param  hal: 硬件抽象层指针
 * @retval None
 */
void MotorHAL_PWM_Stop(MotorHAL_TypeDef *hal)
{
    // 停止PWM和互补PWM输出
    HAL_TIM_PWM_Stop(hal->pwm_config.timer, hal->pwm_config.channel_u);
    HAL_TIMEx_PWMN_Stop(hal->pwm_config.timer, hal->pwm_config.channel_u);

    HAL_TIM_PWM_Stop(hal->pwm_config.timer, hal->pwm_config.channel_v);
    HAL_TIMEx_PWMN_Stop(hal->pwm_config.timer, hal->pwm_config.channel_v);

    HAL_TIM_PWM_Stop(hal->pwm_config.timer, hal->pwm_config.channel_w);
    HAL_TIMEx_PWMN_Stop(hal->pwm_config.timer, hal->pwm_config.channel_w);
}

/**
 * @brief  设置三相PWM占空比
 * @param  hal: 硬件抽象层指针
 * @param  duty_u: U相占空比 (0.0 ~ 1.0)
 * @param  duty_v: V相占空比 (0.0 ~ 1.0)
 * @param  duty_w: W相占空比 (0.0 ~ 1.0)
 * @retval None
 */
void MotorHAL_PWM_SetDutyCycle(MotorHAL_TypeDef *hal, float duty_u, float duty_v, float duty_w)
{
    // 限制占空比范围
    if (duty_u < 0.0f) duty_u = 0.0f;
    if (duty_u > 1.0f) duty_u = 1.0f;
    if (duty_v < 0.0f) duty_v = 0.0f;
    if (duty_v > 1.0f) duty_v = 1.0f;
    if (duty_w < 0.0f) duty_w = 0.0f;
    if (duty_w > 1.0f) duty_w = 1.0f;

    // 转换为比较值并调用原有的PWM驱动
    uint32_t ccr_u = (uint32_t)(duty_u * hal->pwm_config.period);
    uint32_t ccr_v = (uint32_t)(duty_v * hal->pwm_config.period);
    uint32_t ccr_w = (uint32_t)(duty_w * hal->pwm_config.period);

    PWM_SetDutyCycle(hal->pwm_config.timer, hal->pwm_config.channel_u, ccr_u);
    PWM_SetDutyCycle(hal->pwm_config.timer, hal->pwm_config.channel_v, ccr_v);
    PWM_SetDutyCycle(hal->pwm_config.timer, hal->pwm_config.channel_w, ccr_w);
}

/* ==================== ADC电流采样函数实现 ==================== */

/**
 * @brief  初始化ADC电流采样
 * @param  hal: 硬件抽象层指针
 * @retval None
 */
void MotorHAL_CurrentSense_Init(MotorHAL_TypeDef *hal)
{
    // 初始化原有的电流采样驱动（已在hal->current_sense中创建实例）
    // 注意：CurrentSense_Init 需要传入ADC句柄，需要在调用前设置
    // 这里假设已经在外部通过配置结构体设置了ADC句柄

    // 启动DMA电流采样
    CurrentSense_Start(&hal->current_sense);
}

/**
 * @brief  读取三相电流
 * @param  hal: 硬件抽象层指针
 * @param  current_u: U相电流输出 (A)
 * @param  current_v: V相电流输出 (A)
 * @param  current_w: W相电流输出 (A)
 * @retval None
 */
void MotorHAL_CurrentSense_Read(MotorHAL_TypeDef *hal, float *current_u, float *current_v, float *current_w)
{
    CurrentSense_GetCurrents(&hal->current_sense, current_u, current_v, current_w);
}

/**
 * @brief  校准电流传感器零点
 * @param  hal: 硬件抽象层指针
 * @retval None
 * @note   在电机静止且没有电流时调用
 */
void MotorHAL_CurrentSense_Calibrate(MotorHAL_TypeDef *hal)
{
    CurrentSense_Calibrate(&hal->current_sense, 100);  // 采样100次求平均
}

/* ==================== 编码器读取函数实现 ==================== */

/**
 * @brief  初始化编码器
 * @param  hal: 硬件抽象层指针
 * @retval None
 * @note   调用原有的Encoder_Init和Encoder_Start
 */
void MotorHAL_Encoder_Init(MotorHAL_TypeDef *hal)
{
    // 启动原有的编码器驱动
    Encoder_Start(&hal->encoder);

    hal->angle_offset = 0.0f;
}

/**
 * @brief  获取编码器机械角度
 * @param  hal: 硬件抽象层指针
 * @retval 机械角度 (rad)，范围 [0, 2π)
 * @note   调用原有的Encoder_GetCount并转换为弧度
 */
float MotorHAL_Encoder_GetAngle(MotorHAL_TypeDef *hal)
{
    // 获取编码器计数值
    int16_t count = Encoder_GetCount(&hal->encoder);

    // 转换为角度 (rad)
    float angle = ((float)count / (float)ENCODER_PPR) * 2.0f * 3.14159265359f;

    // 应用偏移
    angle += hal->angle_offset;

    // 归一化到 [0, 2π)
    while (angle < 0.0f) angle += 2.0f * 3.14159265359f;
    while (angle >= 2.0f * 3.14159265359f) angle -= 2.0f * 3.14159265359f;

    return angle;
}

/**
 * @brief  获取编码器机械角速度
 * @param  hal: 硬件抽象层指针
 * @retval 机械角速度 (rad/s)
 * @note   调用原有的Encoder_GetSpeed_RPS并转换为rad/s
 */
float MotorHAL_Encoder_GetVelocity(MotorHAL_TypeDef *hal)
{
    // 更新速度计算（应定期调用，如1kHz）
    Encoder_UpdateSpeed(&hal->encoder);

    // 获取转速 (RPS)
    float speed_rps = Encoder_GetSpeed_RPS(&hal->encoder);

    // 转换为角速度 (rad/s)
    float velocity = speed_rps * 2.0f * 3.14159265359f;

    return velocity;
}

/**
 * @brief  重置编码器
 * @param  hal: 硬件抽象层指针
 * @retval None
 */
void MotorHAL_Encoder_Reset(MotorHAL_TypeDef *hal)
{
    Encoder_Reset(&hal->encoder);
    hal->angle_offset = 0.0f;
}

/* ==================== 通用硬件函数实现 ==================== */

/**
 * @brief  初始化电机硬件抽象层
 * @param  hal: 硬件抽象层指针
 * @retval None
 * @note   调用所有子模块的初始化函数
 */
void MotorHAL_Init(MotorHAL_TypeDef *hal)
{
    MotorHAL_PWM_Init(hal);
    MotorHAL_CurrentSense_Init(hal);
    MotorHAL_Encoder_Init(hal);
}

/**
 * @brief  紧急停止
 * @param  hal: 硬件抽象层指针
 * @retval None
 */
void MotorHAL_EmergencyStop(MotorHAL_TypeDef *hal)
{
    // 立即停止PWM输出
    MotorHAL_PWM_Stop(hal);

    uint16_t half_duty = hal->pwm_config.period / 2;
    __HAL_TIM_SET_COMPARE(hal->pwm_config.timer, hal->pwm_config.channel_u, half_duty);
    __HAL_TIM_SET_COMPARE(hal->pwm_config.timer, hal->pwm_config.channel_v, half_duty);
    __HAL_TIM_SET_COMPARE(hal->pwm_config.timer, hal->pwm_config.channel_w, half_duty);
}
