#ifndef H7FOC_BSP_BSP_LAN9252_H
#define H7FOC_BSP_BSP_LAN9252_H

#include <stdint.h>

#include "main.h"

typedef void (*bsp_lan9252_irq_cb_t)(void);

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *irq_port;
    uint16_t irq_pin;
    GPIO_TypeDef *sync0_port;
    uint16_t sync0_pin;
    GPIO_TypeDef *sync1_port;
    uint16_t sync1_pin;
} bsp_lan9252_cfg_t;

void bsp_lan9252_init(const bsp_lan9252_cfg_t *cfg);
void bsp_lan9252_init_default(void);
uint8_t bsp_lan9252_is_initialized(void);

void bsp_lan9252_cs_low(void);
void bsp_lan9252_cs_high(void);
uint8_t bsp_lan9252_spi_transfer(uint8_t tx_byte);

void bsp_lan9252_register_irq_cb(bsp_lan9252_irq_cb_t cb);
void bsp_lan9252_register_sync0_cb(bsp_lan9252_irq_cb_t cb);
void bsp_lan9252_register_sync1_cb(bsp_lan9252_irq_cb_t cb);
void bsp_lan9252_handle_exti(uint16_t gpio_pin);

uint32_t bsp_lan9252_irq_lock(void);
void bsp_lan9252_irq_unlock(uint32_t irq_state);

void bsp_lan9252_on_1ms_tick(void);
uint32_t bsp_lan9252_get_1ms_ticks(void);
void bsp_lan9252_reset_1ms_ticks(void);

#endif
