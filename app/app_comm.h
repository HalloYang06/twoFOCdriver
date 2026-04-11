#ifndef H7FOC_APP_APP_COMM_H
#define H7FOC_APP_APP_COMM_H

#include "main.h"

void App_CommInit(void);
void App_CommTick(void);
void App_CommUartTxCplt(UART_HandleTypeDef *huart);
void App_CommUartRxCplt(UART_HandleTypeDef *huart);
void App_CommUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);

#endif
