#ifndef H7FOC_COMM_ETH_STACK_SPIDRIVER_PORT_H
#define H7FOC_COMM_ETH_STACK_SPIDRIVER_PORT_H

#include "ecat_def.h"

typedef union
{
    UINT16 Val;
    struct
    {
        UINT8 LB;
        UINT8 HB;
    } byte;
} UINT16_VAL;

typedef union
{
    UINT32 Val;
    UINT16 w[2];
    UINT8 v[4];
    struct
    {
        UINT8 LB;
        UINT8 HB;
        UINT8 UB;
        UINT8 MB;
    } byte;
} UINT32_VAL;

typedef union
{
    UINT64 Val;
    UINT32 d[2];
    UINT16 w[4];
    UINT8 v[8];
} UINT64_VAL;

void SPIOpen(void);
void CSLOW(void);
void CSHIGH(void);
void SPIWrite(UINT8 value);
UINT8 SPIRead(void);

UINT32 PDI_Disable_Global_Interrupt(void);
void PDI_Restore_Global_Interrupt(UINT32 int_sts);
void PDI_Enable_Global_interrupt(void);

void PDI_IRQ_Interrupt(void);
void PDI_Init_SYNC_Interrupts(void);
void PDI_Timer_Interrupt(void);

UINT16 PDI_GetTimer(void);
void PDI_ClearTimer(void);

#endif
