//
// Created by 27352 on 26-1-27.
//

#include "vofa_debug.h"
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)  /* Keil MDK */
__attribute__((section(".ARM.__at_0x30000000"))) VOFA_TypeDef vofa;
#elif defined(__GNUC__)  /* GCC / CubeIDE */
__attribute__((section(".RAM_D2"))) VOFA_TypeDef vofa;
#endif
/* 全局VOFA实例 */
VOFA_TypeDef vofa;

/* VOFA+ JustFloat协议帧尾：0x00 0x00 0x80 0x7f */
static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

/**
 * @brief  初始化VOFA+调试
 * @param  huart: 串口句柄指针
 * @retval None
 */
void VOFA_Init(UART_HandleTypeDef *huart)
{
    vofa.huart = huart;
    vofa.tx_busy = 0;

    /* 清零数据 */
    memset(vofa.channels, 0, sizeof(vofa.channels));
    memset(vofa.frame, 0, sizeof(vofa.frame));
}

/**
 * @brief  发送VOFA+数据帧
 * @param  data: 浮点数数组
 * @param  num: 数据个数（最多VOFA_CHANNEL_NUM）
 * @retval None
 */
void VOFA_SendData(float *data, uint8_t num)
{
    /* 如果上次发送还未完成，直接返回（避免DMA冲突）*/
		
    if (vofa.tx_busy) {
        return;
    }
		
    /* 限制通道数 */
    if (num > VOFA_CHANNEL_NUM) {
        num = VOFA_CHANNEL_NUM;
    }

    /* 复制数据到通道 */
    memcpy(vofa.channels, data, num * sizeof(float));

    /* 构建帧：浮点数 + 帧尾 */
    memcpy(vofa.frame, vofa.channels, VOFA_CHANNEL_NUM * sizeof(float));
    memcpy(vofa.frame + VOFA_CHANNEL_NUM * sizeof(float), vofa_tail, VOFA_TAIL_SIZE);

    /* 使用DMA发送 */
    vofa.tx_busy = 1;
    HAL_UART_Transmit_DMA(vofa.huart, vofa.frame, VOFA_FRAME_SIZE);
}

/**
 * @brief  快速发送指定通道数据
 * @param  ch0~ch7: 各通道数据
 * @retval None
 */
void VOFA_SendFloat(float ch0, float ch1, float ch2, float ch3,
                    float ch4, float ch5, float ch6, float ch7)
{
    vofa.channels[0] = ch0;
    vofa.channels[1] = ch1;
    vofa.channels[2] = ch2;
    vofa.channels[3] = ch3;
    vofa.channels[4] = ch4;
    vofa.channels[5] = ch5;
    vofa.channels[6] = ch6;
    vofa.channels[7] = ch7;
    HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    VOFA_SendData(vofa.channels, VOFA_CHANNEL_NUM);
}

/**
 * @brief  UART DMA发送完成回调
 * @param  huart: 串口句柄指针
 * @retval None
 */
void VOFA_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == vofa.huart) {
        vofa.tx_busy = 0;  /* 发送完成，清除忙标志 */
    }
}
