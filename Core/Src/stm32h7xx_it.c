/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "FOC.h"
#include "current_sense.h"
#include "vofa_debug.h"
#include "usart.h"
#include "stdlib.h"
#include "tamagawa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern Encoder_TypeDef encoder_M2;      // 在main.c中定�?
extern Encoder_TypeDef encoder_M0;      // FOC控制的编码器
extern Tamagawa_TypeDef tamagawa_M0;    // 多摩川编码器
extern FOC_TypeDef foc;                 // FOC控制�?
extern CurrentSense_TypeDef current_sense; // 电流采样
extern TIM_HandleTypeDef htim1;         // PWM定时�?
extern uint8_t rx_buf[];                // 串口接收缓冲
extern uint8_t open_loop_enabled;       // 开环使能标�?
extern float open_loop_velocity;        // 开环速度
#define RX_BUF_SIZE 16                  // 接收缓冲区大�?
/* Electrical angle offset for quick phase test: 0, +/-1.0472, +/-2.0944, 3.1416 */
#define FOC_THETA_OFFSET_RAD 0.0f
#define FOC_THETA_DIR_SIGN   (-1.0f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
#include "stdio.h"
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  volatile uint32_t cfsr = SCB->CFSR;
  volatile uint32_t hfsr = SCB->HFSR;
  volatile uint32_t bfar = SCB->BFAR;

  volatile uint32_t *stack_ptr;
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
  /* Keil ARMCC/ARMClang */
  register uint32_t lr_val __asm("lr");
  if (lr_val & 0x04)
    stack_ptr = (volatile uint32_t *)__get_PSP();
  else
    stack_ptr = (volatile uint32_t *)__get_MSP();
#else
  /* GCC */
  __asm volatile (
    "TST LR, #4 \n"
    "ITE EQ \n"
    "MRSEQ %0, MSP \n"
    "MRSNE %0, PSP \n"
    : "=r" (stack_ptr)
  );
#endif

  volatile uint32_t fault_lr  = stack_ptr[5];
  volatile uint32_t fault_pc  = stack_ptr[6];

  char buf[128];
  int len = snprintf(buf, sizeof(buf),
    "\r\n!!! HardFault !!!\r\n"
    "PC=0x%08lX LR=0x%08lX\r\n"
    "CFSR=0x%08lX HFSR=0x%08lX BFAR=0x%08lX\r\n",
    fault_pc, fault_lr, cfsr, hfsr, bfar);

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* 每次循环都发送故障信�?*/
    for (int i = 0; i < len; i++)
    {
      while (!(UART4->ISR & USART_ISR_TXE_TXFNF)) {}
      UART4->TDR = (uint8_t)buf[i];
    }
    while (!(UART4->ISR & USART_ISR_TC)) {}


    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(M1_ENC_Z_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
	
  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(M2_ENC_Z_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */


  

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMAMUX1 overrun interrupt.
  */
void DMAMUX1_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMAMUX1_OVR_IRQn 0 */

  /* USER CODE END DMAMUX1_OVR_IRQn 0 */
  // Handle DMA1_Stream4
  HAL_DMAEx_MUX_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMAMUX1_OVR_IRQn 1 */

  /* USER CODE END DMAMUX1_OVR_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/* ==================== FOC电流环相关回�?(20kHz) ==================== */

/**
 * @brief  ADC转换完成回调 - FOC电流环主循环 (20kHz)
 * @note   由TIM1 CH4触发，频�?0kHz (50us周期)
 * @param  hadc: ADC句柄
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{		
		/*
    static uint32_t callback_count = 0;
    callback_count++;

    // LED1闪烁指示ADC中断工作（每1秒，20000次）
    if (callback_count % 20000 == 0) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
		*/
    if (hadc == &hadc2)
    {
        /* ===== 步骤1：更新电流采�?===== */
        CurrentSense_DMA_CpltCallback(&current_sense);
        /* 注意：Tamagawa_Update 已移�?TIM7 1kHz 中断�?
         * 不能�?0kHz ADC中断里调用HAL_UART_Transmit_DMA�?
         * 会打断UART回调导致HAL状态机破坏 -> HardFault */

        /* ===== 步骤2：仅在FOC使能时执行控�?===== */
        if (foc.enabled)
        {
            /* 2.1 在电流环中直接从position计算电角度 */
            {
                foc.theta_elec = _normalizeAngle(
                    FOC_THETA_DIR_SIGN * tamagawa_M0.angle_elec_rad + FOC_THETA_OFFSET_RAD
                );
            }

            /* 2.2 获取三相电流 */
            PhaseCurrents_TypeDef i_abc;
            CurrentSense_GetCurrents(&current_sense, &i_abc.Ia, &i_abc.Ib, &i_abc.Ic);

            /* 2.3 FOC变换和PID计算 */
            FOC_UpdateCurrents(&foc, &i_abc);
            FOC_CalCurrentLoop(&foc);

            /* 2.4 逆Park + SVPWM */
            Inverse_Park_Transform(&foc.v_dq, foc.theta_elec, &foc.v_alphabeta);

            SVPWM_TypeDef svpwm;
            SVPWM_Calculate(&foc.v_alphabeta, foc.voltage_supply, &svpwm);
            SVPWM_GetDutyCycles(&svpwm, &foc.duty_a, &foc.duty_b, &foc.duty_c);

            /* 2.5 更新PWM */
            PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, (uint32_t)(foc.duty_a * FOC_PWM_PERIOD));
            PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, (uint32_t)(foc.duty_b * FOC_PWM_PERIOD));
            PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, (uint32_t)(foc.duty_c * FOC_PWM_PERIOD));
        }
    }
}
/**
 * @brief  ADC DMA半完成回�?- FOC不使用此回调
 * @note   如果使用双缓冲模式，可以在这里处理前半部分数�?
 * @param  hadc: ADC句柄
 * @retval None
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc2)
    {
        CurrentSense_DMA_HalfCpltCallback(&current_sense);
    }
}

/* ==================== 编码器相关回�?==================== */
/**
  * @brief  GPIO外部中断回调函数
  * @param  GPIO_Pin: 触发中断的GPIO引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // M2编码器Z相中�?

    if (GPIO_Pin == M2_ENC_Z_Pin)
    {
        Encoder_ZPulse_Callback(&encoder_M2);
    }

}

/**
 *@ UART发送终端回调函�?
 * @param  huart: UART句柄
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART4) {
      // tx_done = 1;  // 修复：应该是赋值，不是比较

      /* VOFA+ DMA发送完成回�?*/
      VOFA_UART_TxCpltCallback(huart);
  }
  /* 多摩川编码器：发送完成后切换到接收模�?*/
  if (huart->Instance == USART2) {
      Tamagawa_UART_TxCpltCallback_Handler(&tamagawa_M0);
  }
}

/**
 * @brief  UART接收完成回调（DMA满）
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  /* 多摩川编码器：ReceiveToIdle DMA满时也会触发 */
  if (huart->Instance == USART2) {
      Tamagawa_UART_RxCpltCallback(&tamagawa_M0);
  }
}

/**
 * @brief  UART ReceiveToIdle回调（IDLE检测，多摩川主要用这个�?
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART2) {
      Tamagawa_UART_RxEventCallback(&tamagawa_M0, Size);
  }
}

/* USER CODE END 1 */
