#include "bsp_lan9252.h"

#include "spi.h"

#ifndef BSP_LAN9252_CS_GPIO_Port
#define BSP_LAN9252_CS_GPIO_Port GPIOA
#endif

#ifndef BSP_LAN9252_CS_Pin
#define BSP_LAN9252_CS_Pin GPIO_PIN_4
#endif

#if !defined(BSP_LAN9252_IRQ_GPIO_Port) || !defined(BSP_LAN9252_IRQ_Pin)
#error "LAN9252 IRQ pin is not configured. Define BSP_LAN9252_IRQ_GPIO_Port and BSP_LAN9252_IRQ_Pin."
#endif
#if (BSP_LAN9252_IRQ_Pin == 0U)
#error "LAN9252 IRQ pin is invalid. BSP_LAN9252_IRQ_Pin must not be 0."
#endif

#if !defined(BSP_LAN9252_SYNC0_GPIO_Port) || !defined(BSP_LAN9252_SYNC0_Pin)
#error "LAN9252 SYNC0 pin is not configured. Define BSP_LAN9252_SYNC0_GPIO_Port and BSP_LAN9252_SYNC0_Pin."
#endif
#if (BSP_LAN9252_SYNC0_Pin == 0U)
#error "LAN9252 SYNC0 pin is invalid. BSP_LAN9252_SYNC0_Pin must not be 0."
#endif

#if !defined(BSP_LAN9252_SYNC1_GPIO_Port) || !defined(BSP_LAN9252_SYNC1_Pin)
#error "LAN9252 SYNC1 pin is not configured. Define BSP_LAN9252_SYNC1_GPIO_Port and BSP_LAN9252_SYNC1_Pin."
#endif
#if (BSP_LAN9252_SYNC1_Pin == 0U)
#error "LAN9252 SYNC1 pin is invalid. BSP_LAN9252_SYNC1_Pin must not be 0."
#endif

static bsp_lan9252_cfg_t g_cfg;
static uint8_t g_initialized = 0U;
static volatile uint32_t g_tick_1ms = 0U;
static bsp_lan9252_irq_cb_t g_irq_cb = 0;
static bsp_lan9252_irq_cb_t g_sync0_cb = 0;
static bsp_lan9252_irq_cb_t g_sync1_cb = 0;

void bsp_lan9252_init(const bsp_lan9252_cfg_t *cfg)
{
    if ((cfg == 0) || (cfg->hspi == 0) || (cfg->cs_port == 0) || (cfg->cs_pin == 0U)
        || (cfg->irq_port == 0) || (cfg->irq_pin == 0U)
        || (cfg->sync0_port == 0) || (cfg->sync0_pin == 0U)
        || (cfg->sync1_port == 0) || (cfg->sync1_pin == 0U))
    {
        g_initialized = 0U;
        return;
    }

    g_cfg = *cfg;
    g_initialized = 1U;
    bsp_lan9252_cs_high();
}

void bsp_lan9252_init_default(void)
{
    const bsp_lan9252_cfg_t default_cfg = {
        .hspi = &hspi3,
        .cs_port = BSP_LAN9252_CS_GPIO_Port,
        .cs_pin = BSP_LAN9252_CS_Pin,
        .irq_port = BSP_LAN9252_IRQ_GPIO_Port,
        .irq_pin = BSP_LAN9252_IRQ_Pin,
        .sync0_port = BSP_LAN9252_SYNC0_GPIO_Port,
        .sync0_pin = BSP_LAN9252_SYNC0_Pin,
        .sync1_port = BSP_LAN9252_SYNC1_GPIO_Port,
        .sync1_pin = BSP_LAN9252_SYNC1_Pin,
    };

    bsp_lan9252_init(&default_cfg);
}

uint8_t bsp_lan9252_is_initialized(void)
{
    return g_initialized;
}

void bsp_lan9252_cs_low(void)
{
    if (!g_initialized)
    {
        return;
    }

    g_cfg.cs_port->BSRR = ((uint32_t)g_cfg.cs_pin << 16U);
}

void bsp_lan9252_cs_high(void)
{
    if (!g_initialized)
    {
        return;
    }

    g_cfg.cs_port->BSRR = g_cfg.cs_pin;
}

uint8_t bsp_lan9252_spi_transfer(uint8_t tx_byte)
{
    uint8_t rx_byte = 0xFFU;

    if (!g_initialized)
    {
        return 0xFFU;
    }

    if (HAL_SPI_TransmitReceive(g_cfg.hspi, &tx_byte, &rx_byte, 1U, 10U) != HAL_OK)
    {
        return 0xFFU;
    }

    return rx_byte;
}

void bsp_lan9252_register_irq_cb(bsp_lan9252_irq_cb_t cb)
{
    g_irq_cb = cb;
}

void bsp_lan9252_register_sync0_cb(bsp_lan9252_irq_cb_t cb)
{
    g_sync0_cb = cb;
}

void bsp_lan9252_register_sync1_cb(bsp_lan9252_irq_cb_t cb)
{
    g_sync1_cb = cb;
}

void bsp_lan9252_handle_exti(uint16_t gpio_pin)
{
    if (!g_initialized)
    {
        return;
    }

    if ((g_cfg.irq_pin != 0U) && (gpio_pin == g_cfg.irq_pin) && (g_irq_cb != 0))
    {
        g_irq_cb();
    }

    if ((g_cfg.sync0_pin != 0U) && (gpio_pin == g_cfg.sync0_pin) && (g_sync0_cb != 0))
    {
        g_sync0_cb();
    }

    if ((g_cfg.sync1_pin != 0U) && (gpio_pin == g_cfg.sync1_pin) && (g_sync1_cb != 0))
    {
        g_sync1_cb();
    }
}

uint32_t bsp_lan9252_irq_lock(void)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    return irq_state;
}

void bsp_lan9252_irq_unlock(uint32_t irq_state)
{
    if ((irq_state & 0x1U) == 0U)
    {
        __enable_irq();
    }
}

void bsp_lan9252_on_1ms_tick(void)
{
    g_tick_1ms++;
}

uint32_t bsp_lan9252_get_1ms_ticks(void)
{
    return g_tick_1ms;
}

void bsp_lan9252_reset_1ms_ticks(void)
{
    g_tick_1ms = 0U;
}
