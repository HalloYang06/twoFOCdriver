#ifndef H7FOC_BSP_BSP_UART_H
#define H7FOC_BSP_BSP_UART_H

/*
 * bsp_uart.h — UART/RS485 BSP 驱动
 * RS485 方向控制 GPIO 放入结构体，支持多个 RS485 口互不冲突。
 */

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_UART_RX_DMA_BUFFER_SIZE 256U
#define BSP_UART_FRAME_BUFFER_SIZE  256U

/* RS485 方向控制引脚配置（可选，纯 UART 模式设为 NULL） */
typedef struct
{
    GPIO_TypeDef *port;     /* RS485 RE/DE 引脚端口，NULL 表示纯 UART */
    uint16_t pin;           /* RS485 RE/DE 引脚号 */
} bsp_uart_rs485_t;

typedef struct
{
    UART_HandleTypeDef *huart;
    bsp_uart_rs485_t rs485;  /* RS485 方向控制，port==NULL 时不控制方向 */

    uint8_t rx_dma_buffer[BSP_UART_RX_DMA_BUFFER_SIZE];
    uint16_t last_rx_size;
    uint8_t tx_busy;

    uint8_t rx_frame_buffer[BSP_UART_FRAME_BUFFER_SIZE];
    uint16_t rx_frame_length;
    uint8_t rx_frame_ready;

    uint8_t tx_shadow_buffer[BSP_UART_FRAME_BUFFER_SIZE];
    uint16_t tx_shadow_length;
} bsp_uart_t;

extern bsp_uart_t g_bsp_uart_comm;

/* 初始化（清零结构体，绑定 UART 句柄） */
void bsp_uart_init(bsp_uart_t *uart, UART_HandleTypeDef *huart);

/* 配置 RS485 方向控制引脚 */
void bsp_uart_set_rs485(bsp_uart_t *uart, GPIO_TypeDef *port, uint16_t pin);

void bsp_uart_bind(bsp_uart_t *uart, UART_HandleTypeDef *huart);
HAL_StatusTypeDef bsp_uart_start_receive(bsp_uart_t *uart);
HAL_StatusTypeDef bsp_uart_transmit_dma(bsp_uart_t *uart, uint8_t *data, uint16_t len);
void bsp_uart_mark_tx_complete(bsp_uart_t *uart, UART_HandleTypeDef *huart);
uint8_t bsp_uart_is_bound(const bsp_uart_t *uart);
uint8_t *bsp_uart_get_rx_buffer(bsp_uart_t *uart);

/* 全局通信口便捷接口 */
void bsp_uart_comm_init(void);
void bsp_uart_comm_bind(UART_HandleTypeDef *huart);
uint16_t bsp_uart_comm_fetch_frame(uint8_t *data, uint16_t max_len);
int bsp_uart_comm_send_frame(const uint8_t *data, uint16_t len);
void bsp_uart_comm_tx_cplt_callback(UART_HandleTypeDef *huart);
void bsp_uart_comm_rx_cplt_callback(UART_HandleTypeDef *huart);
void bsp_uart_comm_rx_event_callback(UART_HandleTypeDef *huart, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
