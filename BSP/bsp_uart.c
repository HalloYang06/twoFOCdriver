/*
 * bsp_uart.c — UART/RS485 BSP 驱动
 * RS485 方向控制通过结构体中的 GPIO 配置，支持多个 RS485 口。
 */
#include "bsp_uart.h"

#include <stdint.h>
#include <string.h>

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
__attribute__((section(".ARM.__at_0x30000100"))) bsp_uart_t g_bsp_uart_comm;
#elif defined(__GNUC__)
__attribute__((aligned(32), section(".RAM_D2"))) bsp_uart_t g_bsp_uart_comm;
#else
bsp_uart_t g_bsp_uart_comm;
#endif

/* ── RS485 方向控制（基于结构体配置） ────────────────── */

static void bsp_uart_dir_tx(bsp_uart_t *uart)
{
    if (uart->rs485.port != 0)
    {
        HAL_GPIO_WritePin(uart->rs485.port, uart->rs485.pin, GPIO_PIN_SET);
    }
}

static void bsp_uart_dir_rx(bsp_uart_t *uart)
{
    if (uart->rs485.port != 0)
    {
        HAL_GPIO_WritePin(uart->rs485.port, uart->rs485.pin, GPIO_PIN_RESET);
    }
}

/* ── D-Cache 操作 ────────────────────────────────────── */

static void bsp_uart_clean_dcache(const void *addr, uint32_t size)
{
    const uintptr_t start = ((uintptr_t)addr) & ~(uintptr_t)31U;
    const uintptr_t end = (((uintptr_t)addr + size) + 31U) & ~(uintptr_t)31U;
    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

static void bsp_uart_invalidate_dcache(const void *addr, uint32_t size)
{
    const uintptr_t start = ((uintptr_t)addr) & ~(uintptr_t)31U;
    const uintptr_t end = (((uintptr_t)addr + size) + 31U) & ~(uintptr_t)31U;
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

/* ── 帧缓存 ─────────────────────────────────────────── */

static void bsp_uart_store_rx_frame(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    if ((uart == 0) || (data == 0) || (len == 0U))
    {
        return;
    }

    if (len > BSP_UART_FRAME_BUFFER_SIZE)
    {
        len = BSP_UART_FRAME_BUFFER_SIZE;
    }

    memcpy(uart->rx_frame_buffer, data, len);
    uart->rx_frame_length = len;
    uart->rx_frame_ready = 1U;
}

/* ── 公开接口 ────────────────────────────────────────── */

void bsp_uart_init(bsp_uart_t *uart, UART_HandleTypeDef *huart)
{
    if (uart == 0)
    {
        return;
    }

    memset(uart, 0, sizeof(*uart));
    uart->huart = huart;
}

void bsp_uart_set_rs485(bsp_uart_t *uart, GPIO_TypeDef *port, uint16_t pin)
{
    if (uart == 0)
    {
        return;
    }

    uart->rs485.port = port;
    uart->rs485.pin = pin;
}

void bsp_uart_bind(bsp_uart_t *uart, UART_HandleTypeDef *huart)
{
    if (uart == 0)
    {
        return;
    }

    uart->huart = huart;
}

HAL_StatusTypeDef bsp_uart_start_receive(bsp_uart_t *uart)
{
    if ((uart == 0) || (uart->huart == 0))
    {
        return HAL_ERROR;
    }

    uart->last_rx_size = 0U;
    bsp_uart_dir_rx(uart);

    if (HAL_UARTEx_ReceiveToIdle_DMA(uart->huart,
                                     uart->rx_dma_buffer,
                                     BSP_UART_RX_DMA_BUFFER_SIZE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (uart->huart->hdmarx != 0)
    {
        __HAL_DMA_DISABLE_IT(uart->huart->hdmarx, DMA_IT_HT);
    }

    return HAL_OK;
}
HAL_StatusTypeDef bsp_uart_transmit_dma(bsp_uart_t *uart, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((uart == 0) || (uart->huart == 0) || (data == 0) || (len == 0U))
    {
        return HAL_ERROR;
    }

    bsp_uart_dir_tx(uart);
    uart->tx_busy = 1U;
    status = HAL_UART_Transmit_DMA(uart->huart, data, len);
    if (status != HAL_OK)
    {
        uart->tx_busy = 0U;
        bsp_uart_dir_rx(uart);
    }

    return status;
}

void bsp_uart_mark_tx_complete(bsp_uart_t *uart, UART_HandleTypeDef *huart)
{
    if ((uart == 0) || (huart != uart->huart))
    {
        return;
    }

    uart->tx_busy = 0U;
    bsp_uart_dir_rx(uart);
}

uint8_t bsp_uart_is_bound(const bsp_uart_t *uart)
{
    if ((uart == 0) || (uart->huart == 0))
    {
        return 0U;
    }

    return 1U;
}

uint8_t *bsp_uart_get_rx_buffer(bsp_uart_t *uart)
{
    if (uart == 0)
    {
        return 0;
    }

    return uart->rx_dma_buffer;
}

/* ── 全局通信口便捷接口 ─────────────────────────────── */

void bsp_uart_comm_init(void)
{
    memset(&g_bsp_uart_comm, 0, sizeof(g_bsp_uart_comm));
    bsp_uart_init(&g_bsp_uart_comm, 0);
    /* 配置 RS485 方向引脚（与原工程硬件一致） */
    bsp_uart_set_rs485(&g_bsp_uart_comm, RS485_RE_GPIO_Port, RS485_RE_Pin);
}

void bsp_uart_comm_bind(UART_HandleTypeDef *huart)
{
    bsp_uart_bind(&g_bsp_uart_comm, huart);
    bsp_uart_invalidate_dcache(g_bsp_uart_comm.rx_dma_buffer, BSP_UART_RX_DMA_BUFFER_SIZE);
    (void)bsp_uart_start_receive(&g_bsp_uart_comm);
}

uint16_t bsp_uart_comm_fetch_frame(uint8_t *data, uint16_t max_len)
{
    uint16_t copy_length;

    if ((data == 0) || (max_len == 0U) || (g_bsp_uart_comm.rx_frame_ready == 0U))
    {
        return 0U;
    }

    copy_length = g_bsp_uart_comm.rx_frame_length;
    if (copy_length > max_len)
    {
        copy_length = max_len;
    }

    memcpy(data, g_bsp_uart_comm.rx_frame_buffer, copy_length);
    g_bsp_uart_comm.rx_frame_length = 0U;
    g_bsp_uart_comm.rx_frame_ready = 0U;
    return copy_length;
}

int bsp_uart_comm_send_frame(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0U) || (len > BSP_UART_FRAME_BUFFER_SIZE))
    {
        return -1;
    }

    if (bsp_uart_is_bound(&g_bsp_uart_comm) == 0U)
    {
        return 0;
    }

    if (g_bsp_uart_comm.tx_busy != 0U)
    {
        return -1;
    }

    memcpy(g_bsp_uart_comm.tx_shadow_buffer, data, len);
    g_bsp_uart_comm.tx_shadow_length = len;
    bsp_uart_clean_dcache(g_bsp_uart_comm.tx_shadow_buffer, len);

    if (bsp_uart_transmit_dma(&g_bsp_uart_comm, g_bsp_uart_comm.tx_shadow_buffer, len) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

void bsp_uart_comm_tx_cplt_callback(UART_HandleTypeDef *huart)
{
    bsp_uart_mark_tx_complete(&g_bsp_uart_comm, huart);
}

void bsp_uart_comm_rx_cplt_callback(UART_HandleTypeDef *huart)
{
    if ((huart == 0) || (huart != g_bsp_uart_comm.huart))
    {
        return;
    }

    bsp_uart_invalidate_dcache(g_bsp_uart_comm.rx_dma_buffer, BSP_UART_RX_DMA_BUFFER_SIZE);
    bsp_uart_store_rx_frame(&g_bsp_uart_comm,
                            bsp_uart_get_rx_buffer(&g_bsp_uart_comm),
                            BSP_UART_RX_DMA_BUFFER_SIZE);
    (void)bsp_uart_start_receive(&g_bsp_uart_comm);
}

void bsp_uart_comm_rx_event_callback(UART_HandleTypeDef *huart, uint16_t size)
{
    if ((huart == 0) || (huart != g_bsp_uart_comm.huart))
    {
        return;
    }

    g_bsp_uart_comm.last_rx_size = size;
    bsp_uart_invalidate_dcache(g_bsp_uart_comm.rx_dma_buffer, size);
    bsp_uart_store_rx_frame(&g_bsp_uart_comm,
                            bsp_uart_get_rx_buffer(&g_bsp_uart_comm),
                            size);
    (void)bsp_uart_start_receive(&g_bsp_uart_comm);
}
