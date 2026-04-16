#include "SPIDriver_port.h"

#include "bsp_lan9252.h"
#include "ecatappl.h"
#include "9252_HW.h"

static uint32_t g_timer_base_ms = 0U;

void SPIOpen(void)
{
    if (!bsp_lan9252_is_initialized())
    {
        bsp_lan9252_init_default();
    }
}

void CSLOW(void)
{
    bsp_lan9252_cs_low();
}

void CSHIGH(void)
{
    bsp_lan9252_cs_high();
}

void SPIWrite(UINT8 value)
{
    (void)bsp_lan9252_spi_transfer((uint8_t)value);
}

UINT8 SPIRead(void)
{
    return (UINT8)bsp_lan9252_spi_transfer(0xFFU);
}

UINT32 PDI_Disable_Global_Interrupt(void)
{
    return (UINT32)bsp_lan9252_irq_lock();
}

void PDI_Restore_Global_Interrupt(UINT32 int_sts)
{
    bsp_lan9252_irq_unlock((uint32_t)int_sts);
}

void PDI_Enable_Global_interrupt(void)
{
}

void PDI_IRQ_Interrupt(void)
{
    bsp_lan9252_register_irq_cb(PDI_Isr);
}

void PDI_Init_SYNC_Interrupts(void)
{
    bsp_lan9252_register_sync0_cb(Sync0_Isr);
    bsp_lan9252_register_sync1_cb(Sync1_Isr);
}

void PDI_Timer_Interrupt(void)
{
    g_timer_base_ms = bsp_lan9252_get_1ms_ticks();
}

UINT16 PDI_GetTimer(void)
{
    uint32_t elapsed_ms = bsp_lan9252_get_1ms_ticks() - g_timer_base_ms;
    uint32_t elapsed_ticks = elapsed_ms * (uint32_t)ECAT_TIMER_INC_P_MS;

    if (elapsed_ticks > 0xFFFFU)
    {
        return 0xFFFFU;
    }

    return (UINT16)elapsed_ticks;
}

void PDI_ClearTimer(void)
{
    g_timer_base_ms = bsp_lan9252_get_1ms_ticks();
}
