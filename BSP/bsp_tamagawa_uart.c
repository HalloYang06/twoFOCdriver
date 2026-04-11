#include "bsp_tamagawa_uart.h"

#include <math.h>
#include <string.h>

#include "bsp_encoder.h"
#include "usart.h"

/* 多摩川链路时序敏感，这里集中实现，避免被拆散后难以维护。 */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
__attribute__((section(".ARM.__at_0x30020200"))) uint8_t tama_tx_buf[TAMA_TX_BUF_SIZE];
__attribute__((section(".ARM.__at_0x30020220"))) uint8_t tama_rx_buf[TAMA_RX_BUF_SIZE];
#elif defined(__GNUC__)
__attribute__((section(".RAM_D2"))) uint8_t tama_tx_buf[TAMA_TX_BUF_SIZE];
__attribute__((section(".RAM_D2"))) uint8_t tama_rx_buf[TAMA_RX_BUF_SIZE];
#endif

#define TWO_PI_F 6.28318530718f

static void Tamagawa_Delay(void)
{
    for (volatile uint8_t i = 0; i < 50U; ++i)
    {
        __NOP();
    }
}

void Tamagawa_Init(Tamagawa_TypeDef *tama, UART_HandleTypeDef *huart, uint8_t pole_pairs)
{
    memset(tama, 0, sizeof(Tamagawa_TypeDef));
    tama->huart = huart;
    tama->pole_pairs = pole_pairs;
    tama->rx_flag = 0U;
    tama->elec_zero_offset = 0.0f;

    Tamagawa_Delay();
    TAMA_RS485_RX();
    Tamagawa_Delay();

    memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void Tamagawa_RequestData(Tamagawa_TypeDef *tama, uint8_t data_id)
{
    uint8_t tx_cmd = 0U;
    uint8_t tx_size = 0U;
    uint8_t rx_size = 0U;

    tama->data_id = data_id;

    switch (data_id)
    {
        case TAMA_DATA_ID_0: tx_cmd = TAMA_CMD_ID0; tx_size = 1U; rx_size = 6U; break;
        case TAMA_DATA_ID_1: tx_cmd = TAMA_CMD_ID1; tx_size = 1U; rx_size = 6U; break;
        case TAMA_DATA_ID_2: tx_cmd = TAMA_CMD_ID2; tx_size = 1U; rx_size = 4U; break;
        case TAMA_DATA_ID_3: tx_cmd = TAMA_CMD_ID3; tx_size = 1U; rx_size = 11U; break;
        case TAMA_DATA_ID_7: tx_cmd = TAMA_CMD_ID7; tx_size = 1U; rx_size = 6U; break;
        case TAMA_DATA_ID_8: tx_cmd = TAMA_CMD_ID8; tx_size = 1U; rx_size = 6U; break;
        case TAMA_DATA_ID_C: tx_cmd = TAMA_CMD_IDC; tx_size = 1U; rx_size = 6U; break;
        default: return;
    }

    tama->tx_size = tx_size;
    tama->rx_size = rx_size;
    tama_tx_buf[0] = tx_cmd;

    Tamagawa_Delay();
    TAMA_RS485_TX();
    Tamagawa_Delay();

    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_tx_buf, TAMA_TX_BUF_SIZE);

    if (HAL_UART_Transmit_DMA(tama->huart, tama_tx_buf, 1U) != HAL_OK)
    {
        TAMA_RS485_RX();
        tama->rx_flag = 0U;
        return;
    }

    /* 这里保持和原工程一致：发完命令后立刻切回接收，不能改时序。 */
    Tamagawa_Delay();
    TAMA_RS485_RX();
    Tamagawa_Delay();

    tama->rx_flag = 2U;
}

void Tamagawa_UART_RxEventCallback(Tamagawa_TypeDef *tama, uint16_t size)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);

    if ((size >= tama->rx_size) && (tama->rx_size > 0U))
    {
        Tamagawa_RxParse(tama, tama_rx_buf);
        tama->rx_flag = 1U;
    }
    else
    {
        tama->rx_flag = 0U;
    }

    memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(tama->huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
    if (tama->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(tama->huart->hdmarx, DMA_IT_HT);
    }
}

void Tamagawa_UART_TxCpltCallback_Handler(Tamagawa_TypeDef *tama)
{
    (void)tama;
}

void Tamagawa_UART_RxCpltCallback(Tamagawa_TypeDef *tama)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
    if (tama->rx_size > 0U)
    {
        Tamagawa_RxParse(tama, tama_rx_buf);
        tama->rx_flag = 1U;
    }

    memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(tama->huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
    if (tama->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(tama->huart->hdmarx, DMA_IT_HT);
    }
}

void Tamagawa_RxParse(Tamagawa_TypeDef *tama, uint8_t *buf)
{
    uint32_t abs_raw;

    switch (tama->data_id)
    {
        case TAMA_DATA_ID_0:
            tama->rx.cf = buf[0];
            tama->rx.sf = buf[1];
            abs_raw = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tama->rx.abs = abs_raw & TAMA_POSITION_MASK;
            tama->rx.crc = buf[5];
            tama->position = (int32_t)tama->rx.abs;
            break;

        case TAMA_DATA_ID_1:
            tama->rx.cf = buf[0];
            tama->rx.sf = buf[1];
            tama->rx.abm = (int16_t)(buf[2] | (buf[3] << 8));
            tama->rx.crc = buf[5];
            tama->turns = tama->rx.abm;
            break;

        case TAMA_DATA_ID_2:
            tama->rx.cf = buf[0];
            tama->rx.sf = buf[1];
            tama->rx.enid = buf[2];
            tama->rx.crc = buf[3];
            break;

        case TAMA_DATA_ID_3:
            tama->rx.cf = buf[0];
            tama->rx.sf = buf[1];
            abs_raw = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tama->rx.abs = abs_raw & TAMA_POSITION_MASK;
            tama->rx.enid = buf[5];
            tama->rx.abm = (int16_t)(buf[6] | (buf[7] << 8));
            tama->rx.almc = buf[9];
            tama->rx.crc = buf[10];
            tama->position = (int32_t)tama->rx.abs;
            tama->turns = tama->rx.abm;
            break;

        case TAMA_DATA_ID_C:
            tama->rx.cf = buf[0];
            tama->rx.sf = buf[1];
            abs_raw = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tama->rx.abs = abs_raw & TAMA_POSITION_MASK;
            tama->rx.crc = buf[5];
            break;

        case TAMA_DATA_ID_D:
            tama->rx.cf = buf[0];
            tama->rx.adf = buf[1];
            tama->rx.edf = buf[2];
            tama->rx.crc = buf[3];
            break;

        default:
            break;
    }
}

void Tamagawa_Update(Tamagawa_TypeDef *tama)
{
    static uint32_t wait_count = 0U;
    int32_t delta_pos = 0;

    if (tama->rx_flag == 0U)
    {
        wait_count = 0U;
        Tamagawa_RequestData(tama, TAMA_DATA_ID_0);
    }
    else if (tama->rx_flag == 1U)
    {
        wait_count = 0U;
        tama->position_last = tama->position_total;

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

        {
            float elec_angle = fmodf(tama->angle_mech_rad * (float)tama->pole_pairs, TWO_PI_F);
            elec_angle -= tama->elec_zero_offset;

            while (elec_angle >= TWO_PI_F)
            {
                elec_angle -= TWO_PI_F;
            }
            while (elec_angle < 0.0f)
            {
                elec_angle += TWO_PI_F;
            }

            tama->angle_elec_rad = elec_angle;
        }

        tama->angle_update_seq++;
        tama->rx_flag = 0U;
    }
    else if (tama->rx_flag == 2U)
    {
        wait_count++;
        if (wait_count > TAMA_RX_TIMEOUT_TICKS)
        {
            HAL_UART_AbortReceive(tama->huart);
            HAL_UART_AbortTransmit(tama->huart);
            TAMA_RS485_RX();

            memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
            HAL_UARTEx_ReceiveToIdle_DMA(tama->huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
            if (tama->huart->hdmarx != NULL)
            {
                __HAL_DMA_DISABLE_IT(tama->huart->hdmarx, DMA_IT_HT);
            }

            tama->rx_flag = 0U;
            wait_count = 0U;
        }
    }
}

float Tamagawa_GetAngle_Elec_Rad(Tamagawa_TypeDef *tama)
{
    return tama->angle_elec_rad;
}

float Tamagawa_GetAngle_Mech_Rad(Tamagawa_TypeDef *tama)
{
    return tama->angle_mech_rad;
}

void Tamagawa_UpdateSpeed(Tamagawa_TypeDef *tama, float dt)
{
    int32_t delta;

    if (dt <= 0.0f)
    {
        return;
    }

    delta = tama->position_total - tama->position_total_last;
    tama->position_total_last = tama->position_total;
    tama->speed_rps = (float)delta / TAMA_ENCODER_RESOLUTION_F / dt;
    tama->speed_rpm = tama->speed_rps * 60.0f;
}

float Tamagawa_GetSpeed_RPS(Tamagawa_TypeDef *tama)
{
    return tama->speed_rps;
}

void Tamagawa_AlignElectricZero(Tamagawa_TypeDef *tama)
{
    float raw_elec = fmodf(tama->angle_mech_rad * (float)tama->pole_pairs, TWO_PI_F);

    while (raw_elec < 0.0f)
    {
        raw_elec += TWO_PI_F;
    }
    while (raw_elec >= TWO_PI_F)
    {
        raw_elec -= TWO_PI_F;
    }

    tama->elec_zero_offset = raw_elec;
}

uint8_t Tamagawa_CRC(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0U;
    int i;
    int j;

    for (i = 0; i < len; ++i)
    {
        uint8_t in = data[i];
        for (j = 0; j < 8; ++j)
        {
            uint8_t val = (uint8_t)((in >> 7) ^ (crc >> 7));
            crc <<= 1;
            in <<= 1;
            crc |= val;
        }
    }

    return crc;
}

void bsp_tamagawa_uart_init(Tamagawa_TypeDef *tama, UART_HandleTypeDef *huart, uint8_t pole_pairs)
{
    Tamagawa_Init(tama, huart, pole_pairs);
}

void bsp_tamagawa_uart_tx_cplt(Tamagawa_TypeDef *tama)
{
    Tamagawa_UART_TxCpltCallback_Handler(tama);
}

void bsp_tamagawa_uart_rx_cplt(Tamagawa_TypeDef *tama)
{
    Tamagawa_UART_RxCpltCallback(tama);
}

void bsp_tamagawa_uart_rx_event(Tamagawa_TypeDef *tama, uint16_t size)
{
    Tamagawa_UART_RxEventCallback(tama, size);
}

static int bsp_encoder_tamagawa_update_impl(void *ctx, float dt)
{
    Tamagawa_TypeDef *tama = (Tamagawa_TypeDef *)ctx;
    Tamagawa_Update(tama);
    Tamagawa_UpdateSpeed(tama, dt);
    return 0;
}

static float bsp_encoder_tamagawa_get_angle_mech_impl(void *ctx)
{
    return Tamagawa_GetAngle_Mech_Rad((Tamagawa_TypeDef *)ctx);
}

static float bsp_encoder_tamagawa_get_angle_elec_impl(void *ctx)
{
    return Tamagawa_GetAngle_Elec_Rad((Tamagawa_TypeDef *)ctx);
}

static float bsp_encoder_tamagawa_get_speed_impl(void *ctx)
{
    return Tamagawa_GetSpeed_RPS((Tamagawa_TypeDef *)ctx) * TWO_PI_F;
}

static int bsp_encoder_tamagawa_align_impl(void *ctx)
{
    Tamagawa_AlignElectricZero((Tamagawa_TypeDef *)ctx);
    return 0;
}

static int bsp_encoder_tamagawa_is_ready_impl(void *ctx)
{
    Tamagawa_TypeDef *tama = (Tamagawa_TypeDef *)ctx;
    return (tama->angle_update_seq > 0U) ? 1 : 0;
}

static uint32_t bsp_encoder_tamagawa_get_fault_impl(void *ctx)
{
    Tamagawa_TypeDef *tama = (Tamagawa_TypeDef *)ctx;
    return (uint32_t)tama->rx.almc;
}

const encoder_if_t g_bsp_encoder_tamagawa_ops = {
    .init = 0,
    .start = 0,
    .stop = 0,
    .update = bsp_encoder_tamagawa_update_impl,
    .get_angle_mech_rad = bsp_encoder_tamagawa_get_angle_mech_impl,
    .get_angle_elec_rad = bsp_encoder_tamagawa_get_angle_elec_impl,
    .get_speed_rad_s = bsp_encoder_tamagawa_get_speed_impl,
    .align_electric_zero = bsp_encoder_tamagawa_align_impl,
    .is_ready = bsp_encoder_tamagawa_is_ready_impl,
    .get_fault = bsp_encoder_tamagawa_get_fault_impl,
};
