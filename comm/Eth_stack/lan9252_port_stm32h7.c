#include "lan9252_port.h"

#include "bsp_lan9252.h"

void lan9252_port_open(void)
{
    if (!bsp_lan9252_is_initialized())
    {
        bsp_lan9252_init_default();
    }
}

void lan9252_port_cs_low(void)
{
    bsp_lan9252_cs_low();
}

void lan9252_port_cs_high(void)
{
    bsp_lan9252_cs_high();
}

uint8_t lan9252_port_spi_transfer(uint8_t tx_byte)
{
    return bsp_lan9252_spi_transfer(tx_byte);
}

uint32_t lan9252_port_irq_lock(void)
{
    return bsp_lan9252_irq_lock();
}

void lan9252_port_irq_unlock(uint32_t irq_state)
{
    bsp_lan9252_irq_unlock(irq_state);
}

void lan9252_port_set_irq_cb(lan9252_port_irq_cb_t cb)
{
    bsp_lan9252_register_irq_cb(cb);
}

void lan9252_port_set_sync0_cb(lan9252_port_irq_cb_t cb)
{
    bsp_lan9252_register_sync0_cb(cb);
}

void lan9252_port_set_sync1_cb(lan9252_port_irq_cb_t cb)
{
    bsp_lan9252_register_sync1_cb(cb);
}

uint32_t lan9252_port_get_tick_ms(void)
{
    return bsp_lan9252_get_1ms_ticks();
}

void lan9252_port_reset_tick_ms(void)
{
    bsp_lan9252_reset_1ms_ticks();
}
