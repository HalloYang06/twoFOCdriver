
//
// Created by 27352 on 26-1-27.
//

#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

#include "main.h"
#include "usart.h"
#include <string.h>
#include "FOC.h"
/* VOFA+ JustFloat协议配置 */
#define VOFA_CHANNEL_NUM    8       // 最多8个通道（可调整）
#define VOFA_TAIL_SIZE      4       // 帧尾大小
#define VOFA_FRAME_SIZE     (VOFA_CHANNEL_NUM * 4 + VOFA_TAIL_SIZE)  // 总帧大小

/* VOFA+数据结构 */
typedef  struct {
    float channels[VOFA_CHANNEL_NUM];   // 通道数据
    uint8_t frame[VOFA_FRAME_SIZE];     // 完整帧（用于DMA发送）
    uint8_t tx_busy;                     // 发送忙标志
    UART_HandleTypeDef *huart;           // 串口句柄
} VOFA_TypeDef;

/* 全局VOFA实例 */
extern VOFA_TypeDef vofa;

/* ==================== VOFA+接口函数 ==================== */

/**
 * @brief  初始化VOFA+调试
 * @param  huart: 串口句柄指针
 * @retval None
 */
void VOFA_Init(UART_HandleTypeDef *huart);

/**
 * @brief  发送VOFA+数据帧
 * @param  data: 浮点数数组（最多VOFA_CHANNEL_NUM个）
 * @param  num: 数据个数
 * @retval None
 * @note   使用DMA异步发送，不阻塞
 */
void VOFA_SendData(float *data, uint8_t num);

/**
 * @brief  快速发送指定通道数据
 * @param  ch0~ch7: 各通道数据
 * @retval None
 */
void VOFA_SendFloat(float ch0, float ch1, float ch2, float ch3,
                    float ch4, float ch5, float ch6, float ch7);

/**
 * @brief  UART DMA发送完成回调（需在HAL回调中调用）
 * @param  huart: 串口句柄指针
 * @retval None
 */
void VOFA_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif //VOFA_DEBUG_H
