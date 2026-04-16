#ifndef H7FOC_APP_APP_AXIS_H
#define H7FOC_APP_APP_AXIS_H

#include "axis.h"
#include "main.h"

typedef enum
{
    APP_AXIS_SLOW_LOOP_SRC_TIM7 = 0,
    APP_AXIS_SLOW_LOOP_SRC_SYNC0 = 1,
} app_axis_slow_loop_source_t;

extern axis_t g_axis0;

void App_AxisInit(void);
void App_AxisEnable(void);
void App_AxisDisable(void);
void App_AxisSetSlowLoopSource(app_axis_slow_loop_source_t source);
app_axis_slow_loop_source_t App_AxisGetSlowLoopSource(void);
void App_AxisSlowLoopTick(void);
void App_AxisSync0Tick(void);
void App_AxisCurrentLoopIrqHandler(uint16_t sample_ch15, uint16_t sample_ch3, uint16_t sample_ch8);
void App_AxisUartTxCplt(UART_HandleTypeDef *huart);
void App_AxisUartRxCplt(UART_HandleTypeDef *huart);
void App_AxisUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);

#endif
