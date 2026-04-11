#ifndef H7FOC_COMM_DEBUG_COMM_VOFA_H
#define H7FOC_COMM_DEBUG_COMM_VOFA_H

#include "main.h"

#define COMM_VOFA_CHANNEL_NUM    8U
#define COMM_VOFA_TAIL_SIZE      4U
#define COMM_VOFA_FRAME_SIZE     (COMM_VOFA_CHANNEL_NUM * 4U + COMM_VOFA_TAIL_SIZE)

typedef struct
{
    float channels[COMM_VOFA_CHANNEL_NUM];
    uint8_t frame[COMM_VOFA_FRAME_SIZE];
    uint8_t tx_busy;
    UART_HandleTypeDef *huart;
} comm_vofa_t;

extern comm_vofa_t g_comm_vofa;

void comm_vofa_init(UART_HandleTypeDef *huart);
void comm_vofa_send_data(float *data, uint8_t num);
void comm_vofa_send_float(float ch0,
                          float ch1,
                          float ch2,
                          float ch3,
                          float ch4,
                          float ch5,
                          float ch6,
                          float ch7);
void comm_vofa_uart_tx_cplt_callback(UART_HandleTypeDef *huart);

#endif
