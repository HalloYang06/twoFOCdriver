#ifndef H7FOC_COMM_ETH_STACK_ECAT_PLATFORM_STM32H7_H
#define H7FOC_COMM_ETH_STACK_ECAT_PLATFORM_STM32H7_H

/*
 * 平台固定配置（STM32H7 + LAN9252 SPI）
 * 通过在 ecat_def.h 前置包含本文件，将历史评估板宏统一锁定为当前平台配置。
 */

#define EL9800_HW 0
#define MCI_HW 0
#define FC1100_HW 0
#define TIESC_HW 0

#define CONTROLLER_16BIT 0
#define CONTROLLER_32BIT 1
#define MEMORY_UNIT_16BIT 0

#define _PIC18 0
#define _PIC24 0

#define BIG_ENDIAN_16BIT 0
#define BIG_ENDIAN_FORMAT 0

#define AL_EVENT_ENABLED 1
#define DC_SUPPORTED 1
#define ECAT_TIMER_INT 0
#define INTERRUPTS_SUPPORTED 1

#if (CONTROLLER_32BIT != 1) || (CONTROLLER_16BIT != 0)
#error "STM32H7 平台要求 CONTROLLER_32BIT=1 且 CONTROLLER_16BIT=0"
#endif

#if (BIG_ENDIAN_FORMAT != 0) || (BIG_ENDIAN_16BIT != 0)
#error "STM32H7 平台要求小端字节序配置"
#endif

#endif
