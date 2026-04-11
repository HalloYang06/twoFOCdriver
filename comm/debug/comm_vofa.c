#include "comm_vofa.h"

#include <string.h>

/* 直接移植原有 VOFA 调试逻辑，只调整命名和归属目录。 */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
__attribute__((section(".ARM.__at_0x30000000"))) comm_vofa_t g_comm_vofa;
#elif defined(__GNUC__)
__attribute__((section(".RAM_D2"))) comm_vofa_t g_comm_vofa;
#endif

static const uint8_t g_vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

void comm_vofa_init(UART_HandleTypeDef *huart)
{
    g_comm_vofa.huart = huart;
    g_comm_vofa.tx_busy = 0U;
    memset(g_comm_vofa.channels, 0, sizeof(g_comm_vofa.channels));
    memset(g_comm_vofa.frame, 0, sizeof(g_comm_vofa.frame));
}

void comm_vofa_send_data(float *data, uint8_t num)
{
    if (g_comm_vofa.tx_busy != 0U)
    {
        return;
    }

    if (num > COMM_VOFA_CHANNEL_NUM)
    {
        num = COMM_VOFA_CHANNEL_NUM;
    }

    memcpy(g_comm_vofa.channels, data, (size_t)num * sizeof(float));
    memcpy(g_comm_vofa.frame, g_comm_vofa.channels, COMM_VOFA_CHANNEL_NUM * sizeof(float));
    memcpy(g_comm_vofa.frame + COMM_VOFA_CHANNEL_NUM * sizeof(float), g_vofa_tail, COMM_VOFA_TAIL_SIZE);

    g_comm_vofa.tx_busy = 1U;
    HAL_UART_Transmit_DMA(g_comm_vofa.huart, g_comm_vofa.frame, COMM_VOFA_FRAME_SIZE);
}

void comm_vofa_send_float(float ch0,
                          float ch1,
                          float ch2,
                          float ch3,
                          float ch4,
                          float ch5,
                          float ch6,
                          float ch7)
{
    g_comm_vofa.channels[0] = ch0;
    g_comm_vofa.channels[1] = ch1;
    g_comm_vofa.channels[2] = ch2;
    g_comm_vofa.channels[3] = ch3;
    g_comm_vofa.channels[4] = ch4;
    g_comm_vofa.channels[5] = ch5;
    g_comm_vofa.channels[6] = ch6;
    g_comm_vofa.channels[7] = ch7;
    comm_vofa_send_data(g_comm_vofa.channels, COMM_VOFA_CHANNEL_NUM);
}

void comm_vofa_uart_tx_cplt_callback(UART_HandleTypeDef *huart)
{
    if (huart == g_comm_vofa.huart)
    {
        g_comm_vofa.tx_busy = 0U;
    }
}
