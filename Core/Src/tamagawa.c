#include "tamagawa.h"
#include "string.h"
#include "math.h"
#include "usart.h"

/* ==================== DMA缓冲区 (D2 SRAM) ==================== */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
__attribute__((section(".ARM.__at_0x30020200"))) uint8_t tama_tx_buf[TAMA_TX_BUF_SIZE];
__attribute__((section(".ARM.__at_0x30020220"))) uint8_t tama_rx_buf[TAMA_RX_BUF_SIZE];
#elif defined(__GNUC__)
__attribute__((section(".RAM_D2"))) uint8_t tama_tx_buf[TAMA_TX_BUF_SIZE];
__attribute__((section(".RAM_D2"))) uint8_t tama_rx_buf[TAMA_RX_BUF_SIZE];
#endif

#define PI_F  3.14159265358979f
#define TWO_PI_F  6.28318530718f

/* RS485方向切换延时 */
static void Tamagawa_Delay(void)
{
    for (volatile uint8_t i = 0; i < 50; i++) { __NOP(); }
}

/* ==================== 初始化 ==================== */

void Tamagawa_Init(Tamagawa_TypeDef *tama, UART_HandleTypeDef *huart, uint8_t pole_pairs)
{
    memset(tama, 0, sizeof(Tamagawa_TypeDef));
    tama->huart = huart;
    tama->pole_pairs = pole_pairs;
    tama->rx_flag = 0;
    tama->elec_zero_offset = 0.0f;

    /* 初始方向设为接收 */
    Tamagawa_Delay();
    TAMA_RS485_RX();
    Tamagawa_Delay();

    /* 使用 ReceiveToIdle DMA 持续监听，不手动开启 RXNE 中断。
     * 手动开 RXNE 但未配置 huart->RxISR 会在 HAL_UART_IRQHandler 中触发空指针调用。 */
    memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/* ==================== 数据请求 ==================== */

void Tamagawa_RequestData(Tamagawa_TypeDef *tama, uint8_t data_id)
{
    tama->data_id = data_id;

    uint8_t tx_cmd = 0;
    uint8_t tx_size = 0;
    uint8_t rx_size = 0;

    switch (data_id)
    {
        case TAMA_DATA_ID_0:
            tx_cmd = TAMA_CMD_ID0; tx_size = 1; rx_size = 6; break;
        case TAMA_DATA_ID_1:
            tx_cmd = TAMA_CMD_ID1; tx_size = 1; rx_size = 6; break;
        case TAMA_DATA_ID_2:
            tx_cmd = TAMA_CMD_ID2; tx_size = 1; rx_size = 4; break;
        case TAMA_DATA_ID_3:
            tx_cmd = TAMA_CMD_ID3; tx_size = 1; rx_size = 11; break;
        case TAMA_DATA_ID_7:
            tx_cmd = TAMA_CMD_ID7; tx_size = 1; rx_size = 6; break;
        case TAMA_DATA_ID_8:
            tx_cmd = TAMA_CMD_ID8; tx_size = 1; rx_size = 6; break;
        case TAMA_DATA_ID_C:
            tx_cmd = TAMA_CMD_IDC; tx_size = 1; rx_size = 6; break;
        default:
            return;
    }

    tama->tx_size = tx_size;
    tama->rx_size = rx_size;

    /* 逐字节发送（和参考代码一致） */
    tama_tx_buf[0] = tx_cmd;

    /* 切换到发送模式 */
    Tamagawa_Delay();
    TAMA_RS485_TX();
    Tamagawa_Delay();

    /* 清除DCache */
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_tx_buf, TAMA_TX_BUF_SIZE);

    /* DMA发送1字节 */
    if (HAL_UART_Transmit_DMA(tama->huart, tama_tx_buf, 1) != HAL_OK)
    {
        TAMA_RS485_RX();
        tama->rx_flag = 0;
        return;
    }

    /* 立刻切回接收模式（该项目原始可工作时序） */
    Tamagawa_Delay();
    TAMA_RS485_RX();
    Tamagawa_Delay();

    /* 标记等待接收 */
    tama->rx_flag = 2;
}

/* ==================== RxEvent回调（IDLE检测） ==================== */

/**
 * @brief  UART ReceiveToIdle回调 - 在HAL_UARTEx_RxEventCallback中调用
 * @param  tama: 多摩川结构体指针
 * @param  Size: 实际接收到的字节数
 */
void Tamagawa_UART_RxEventCallback(Tamagawa_TypeDef *tama, uint16_t Size)
{
    /* 无效化DCache */
    SCB_InvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);

    /* 解析接收数据 */
    if (Size >= tama->rx_size && tama->rx_size > 0)
    {
        Tamagawa_RxParse(tama, tama_rx_buf);
        tama->rx_flag = 1;
    }
    else
    {
        /* 收到不完整帧（常见于噪声/错误触发），立即回到空闲重试，避免长时间卡在等待态 */
        tama->rx_flag = 0;
    }

    /* 清空缓冲区，重新启动ReceiveToIdle监听 */
    memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(tama->huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
    if (tama->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(tama->huart->hdmarx, DMA_IT_HT);
    }
}

/* ==================== 发送完成回调（不再需要切换方向） ==================== */

void Tamagawa_UART_TxCpltCallback_Handler(Tamagawa_TypeDef *tama)
{
    /* 发送完成后无需额外处理，方向已在RequestData中切到RX */
    (void)tama;
}

/* ==================== 接收完成回调（保留但不再主要使用） ==================== */

void Tamagawa_UART_RxCpltCallback(Tamagawa_TypeDef *tama)
{
    /* ReceiveToIdle模式下，如果DMA接收满了也会触发这个回调 */
    SCB_InvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
    if (tama->rx_size > 0)
    {
        Tamagawa_RxParse(tama, tama_rx_buf);
        tama->rx_flag = 1;
    }

    memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(tama->huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
    if (tama->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(tama->huart->hdmarx, DMA_IT_HT);
    }
}

/* ==================== 数据解析 ==================== */

void Tamagawa_RxParse(Tamagawa_TypeDef *tama, uint8_t *buf)
{
    uint32_t abs_raw;

    switch (tama->data_id)
    {
        case TAMA_DATA_ID_0:
            tama->rx.cf  = buf[0];
            tama->rx.sf  = buf[1];
            abs_raw = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tama->rx.abs = abs_raw & TAMA_POSITION_MASK;
            tama->rx.crc = buf[5];
            tama->position = tama->rx.abs;
            break;

        case TAMA_DATA_ID_1:
            tama->rx.cf  = buf[0];
            tama->rx.sf  = buf[1];
            tama->rx.abm = buf[2] | (buf[3] << 8);
            tama->rx.crc = buf[5];
            tama->turns = tama->rx.abm;
            break;

        case TAMA_DATA_ID_2:
            tama->rx.cf   = buf[0];
            tama->rx.sf   = buf[1];
            tama->rx.enid = buf[2];
            tama->rx.crc  = buf[3];
            break;

        case TAMA_DATA_ID_3:
            tama->rx.cf   = buf[0];
            tama->rx.sf   = buf[1];
            abs_raw = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tama->rx.abs  = abs_raw & TAMA_POSITION_MASK;
            tama->rx.enid = buf[5];
            tama->rx.abm  = buf[6] | (buf[7] << 8);
            tama->rx.almc = buf[9];
            tama->rx.crc  = buf[10];
            tama->position = tama->rx.abs;
            tama->turns = tama->rx.abm;
            break;

        case TAMA_DATA_ID_C:
            tama->rx.cf  = buf[0];
            tama->rx.sf  = buf[1];
            abs_raw = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tama->rx.abs = abs_raw & TAMA_POSITION_MASK;
            tama->rx.crc = buf[5];
            break;

        case TAMA_DATA_ID_D:
            tama->rx.cf  = buf[0];
            tama->rx.adf = buf[1];
            tama->rx.edf = buf[2];
            tama->rx.crc = buf[3];
            break;

        default:
            break;
    }
}

/* ==================== 周期性更新 ==================== */

void Tamagawa_Update(Tamagawa_TypeDef *tama)
{
    static uint32_t wait_count = 0;
    int32_t delta_pos = 0;

    if (tama->rx_flag == 0)
    {
        /* 空闲状态，发送读取请求 */
        wait_count = 0;
        Tamagawa_RequestData(tama, TAMA_DATA_ID_0);
    }
    else if (tama->rx_flag == 1)
    {
        /* 接收完成，计算角度 */
        wait_count = 0;
        tama->position_last = tama->position_total;

        /* Unwrap single-turn position from ID0 frames.
         * We avoid relying on 'turns' because ID0 does not carry ABM.
         */
        if (tama->position_raw_valid == 0U)
        {
            tama->position_total = tama->position;
            tama->position_raw_last = tama->position;
            tama->position_raw_valid = 1U;
        }
        else
        {
            delta_pos = tama->position - tama->position_raw_last;
            if (delta_pos > (TAMA_ENCODER_RESOLUTION / 2))
            {
                delta_pos -= TAMA_ENCODER_RESOLUTION;
            }
            else if (delta_pos < -(TAMA_ENCODER_RESOLUTION / 2))
            {
                delta_pos += TAMA_ENCODER_RESOLUTION;
            }
            tama->position_total += delta_pos;
            tama->position_raw_last = tama->position;
        }

        tama->angle_mech_rad = ((float)tama->position / TAMA_ENCODER_RESOLUTION_F) * TWO_PI_F;

        float elec_angle = fmodf(tama->angle_mech_rad * (float)tama->pole_pairs, TWO_PI_F);
        elec_angle -= tama->elec_zero_offset;

        while (elec_angle >= TWO_PI_F) elec_angle -= TWO_PI_F;
        while (elec_angle < 0.0f) elec_angle += TWO_PI_F;

        tama->angle_elec_rad = elec_angle;
        tama->angle_update_seq++;

        tama->rx_flag = 0;
    }
    else if (tama->rx_flag == 2)
    {
        /* 等待接收中，超时保护 (1kHz调用，1000次=1s) */
        wait_count++;
        if (wait_count > TAMA_RX_TIMEOUT_TICKS)
        {
            HAL_UART_AbortReceive(tama->huart);
            HAL_UART_AbortTransmit(tama->huart);
            TAMA_RS485_RX();

            /* 重新启动ReceiveToIdle监听 */
            memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
            HAL_UARTEx_ReceiveToIdle_DMA(tama->huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
            if (tama->huart->hdmarx != NULL)
            {
                __HAL_DMA_DISABLE_IT(tama->huart->hdmarx, DMA_IT_HT);
            }

            tama->rx_flag = 0;
            wait_count = 0;
        }
    }
}

/* ==================== 角度获取 ==================== */

float Tamagawa_GetAngle_Elec_Rad(Tamagawa_TypeDef *tama)
{
    return tama->angle_elec_rad;
}

float Tamagawa_GetAngle_Mech_Rad(Tamagawa_TypeDef *tama)
{
    return tama->angle_mech_rad;
}

/* ==================== 速度计算 ==================== */

void Tamagawa_UpdateSpeed(Tamagawa_TypeDef *tama, float dt)
{
    if (dt <= 0.0f)
    {
        return;
    }

    int32_t delta = tama->position_total - tama->position_total_last;
    tama->position_total_last = tama->position_total;

    tama->speed_rps = (float)delta / TAMA_ENCODER_RESOLUTION_F / dt;
    tama->speed_rpm = tama->speed_rps * 60.0f;
}

float Tamagawa_GetSpeed_RPS(Tamagawa_TypeDef *tama)
{
    return tama->speed_rps;
}

/* ==================== 电角度零点校准 ==================== */

void Tamagawa_AlignElectricZero(Tamagawa_TypeDef *tama)
{
    float raw_elec = fmodf(tama->angle_mech_rad * (float)tama->pole_pairs, TWO_PI_F);
    while (raw_elec < 0.0f) raw_elec += TWO_PI_F;
    while (raw_elec >= TWO_PI_F) raw_elec -= TWO_PI_F;

    tama->elec_zero_offset = raw_elec;
}

/* ==================== CRC校验 ==================== */

uint8_t Tamagawa_CRC(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    int i, j;

    for (i = 0; i < len; i++)
    {
        uint8_t in = data[i];
        for (j = 0; j < 8; j++)
        {
            uint8_t val = (in >> 7) ^ (crc >> 7);
            crc <<= 1;
            in <<= 1;
            crc |= val;
        }
    }

    return crc;
}
