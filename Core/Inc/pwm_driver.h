#ifndef __PWM_DRIVER_H__
#define __PWM_DRIVER_H__

#include "main.h"
#define PWM_FREQUENCY 20000 // 20 kHz PWM frequency

void PWM_Init(void);
void PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t duty_cycle);
void PWM_EmergencyStop(void);

#endif /* __PWM_DRIVER_H__ */
