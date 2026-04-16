#include "SPIDriver_port.h"

#include "ecatappl.h"
#include "9252_HW.h"
#include "lan9252_port.h"

static uint32_t g_timer_base_ms = 0U;

void SPIOpen(void)
{
    lan9252_port_open();
}

void CSLOW(void)
{
    lan9252_port_cs_low();
}

void CSHIGH(void)
{
    lan9252_port_cs_high();
}

void SPIWrite(UINT8 value)
{
    (void)lan9252_port_spi_transfer((uint8_t)value);
}

UINT8 SPIRead(void)
{
    return (UINT8)lan9252_port_spi_transfer(0xFFU);
}

UINT32 PDI_Disable_Global_Interrupt(void)
{
    return (UINT32)lan9252_port_irq_lock();
}

void PDI_Restore_Global_Interrupt(UINT32 int_sts)
{
    lan9252_port_irq_unlock((uint32_t)int_sts);
}

void PDI_Enable_Global_interrupt(void)
{
}

void PDI_IRQ_Interrupt(void)
{
    lan9252_port_set_irq_cb(PDI_Isr);
}

void PDI_Init_SYNC_Interrupts(void)
{
    lan9252_port_set_sync0_cb(Sync0_Isr);
    lan9252_port_set_sync1_cb(Sync1_Isr);
}

void PDI_Timer_Interrupt(void)
{
    g_timer_base_ms = lan9252_port_get_tick_ms();
}

UINT16 PDI_GetTimer(void)
{
    uint32_t elapsed_ms = lan9252_port_get_tick_ms() - g_timer_base_ms;
    uint32_t elapsed_ticks = elapsed_ms * (uint32_t)ECAT_TIMER_INC_P_MS;

    if (elapsed_ticks > 0xFFFFU)
    {
        return 0xFFFFU;
    }

    return (UINT16)elapsed_ticks;
}

void PDI_ClearTimer(void)
{
    g_timer_base_ms = lan9252_port_get_tick_ms();
}
