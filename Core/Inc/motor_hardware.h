#ifndef MOTOR_HARDWARE_H
#define MOTOR_HARDWARE_H

#include "main.h"
#include "tim.h"
#include "adc.h"


#include "pwm_driver.h"
#include "current_sense.h"
#include "encoder.h"

/* ==================== 硬件抽象层结构体 ==================== */

/* PWM驱动配置 */
typedef struct {
    TIM_HandleTypeDef *timer;      // PWM定时器句柄
    uint32_t channel_u;            // U相通道 (TIM_CHANNEL_1/2/3/4)
    uint32_t channel_v;            // V相通道
    uint32_t channel_w;            // W相通道
    uint16_t period;               // PWM周期值 (ARR)
} PWM_Config_TypeDef;

/* 电机硬件抽象层 */
typedef struct {
    /* 配置信息 */
    PWM_Config_TypeDef pwm_config;

    /* 底层驱动实例 - 复用原有的成熟驱动 */
    CurrentSense_TypeDef current_sense;  // 电流采样驱动实例
    Encoder_TypeDef encoder;             // 编码器驱动实例

    /* 运行时数据 */
    float angle_offset;            // 角度偏移校准值
} MotorHAL_TypeDef;

/* ==================== PWM驱动函数 ==================== */
void MotorHAL_PWM_Init(MotorHAL_TypeDef *hal);
void MotorHAL_PWM_Start(MotorHAL_TypeDef *hal);
void MotorHAL_PWM_Stop(MotorHAL_TypeDef *hal);
void MotorHAL_PWM_SetDutyCycle(MotorHAL_TypeDef *hal, float duty_u, float duty_v, float duty_w);

/* ==================== ADC电流采样函数 ==================== */
void MotorHAL_CurrentSense_Init(MotorHAL_TypeDef *hal);
void MotorHAL_CurrentSense_Read(MotorHAL_TypeDef *hal, float *current_u, float *current_v, float *current_w);
void MotorHAL_CurrentSense_Calibrate(MotorHAL_TypeDef *hal);

/* ==================== 编码器读取函数 ==================== */
void MotorHAL_Encoder_Init(MotorHAL_TypeDef *hal);
float MotorHAL_Encoder_GetAngle(MotorHAL_TypeDef *hal);        // 获取机械角度 (rad)
float MotorHAL_Encoder_GetVelocity(MotorHAL_TypeDef *hal);     // 获取机械角速度 (rad/s)
void MotorHAL_Encoder_Reset(MotorHAL_TypeDef *hal);

/* ==================== 通用硬件函数 ==================== */
void MotorHAL_Init(MotorHAL_TypeDef *hal);
void MotorHAL_EmergencyStop(MotorHAL_TypeDef *hal);

#endif /* MOTOR_HARDWARE_H */
