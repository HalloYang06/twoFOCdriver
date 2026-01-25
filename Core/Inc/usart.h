/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

/* USER CODE BEGIN Private defines */
#define RX_BUF_SIZE 128
extern  uint8_t rx_buf[RX_BUF_SIZE];
extern  uint8_t process_buf[RX_BUF_SIZE];
extern  float open_loop_velocity;
  typedef  struct {
    uint8_t header1 ;
    uint8_t header2;
    uint8_t len;
    uint8_t cmd;
    float data;
    uint8_t crc;
    uint8_t tail;
  }Data_frame_t;
  void UART_Data_process(uint8_t *buf,uint8_t len);
/* USER CODE END Private defines */

void MX_UART4_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

