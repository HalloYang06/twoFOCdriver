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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern Encoder_TypeDef encoder_M2;      // 在main.c中定义
extern Encoder_TypeDef encoder_M0;      // FOC控制的编码器
extern FOC_TypeDef foc;                 // FOC控制器
extern CurrentSense_TypeDef current_sense; // 电流采样
extern TIM_HandleTypeDef htim1;         // PWM定时器
extern uint8_t rx_buf[];                // 串口接收缓冲
extern uint8_t open_loop_enabled;       // 开环使能标志
extern float open_loop_velocity;        // 开环速度
#define RX_BUF_SIZE 16                  // 接收缓冲区大小
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)  /* Keil MDK */
					__attribute__((section(".ARM.__at_0x30000050"))) char buf;;
					#elif defined(__GNUC__)  /* GCC / CubeIDE */
					__attribute__((section(".RAM_D2"))) VOFA_TypeDef vofa;
					#endif
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
extern UART_HandleTypeDef huart4;
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
  /* 进入 HardFault，LED2 快闪提示 */
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    for(volatile uint32_t i = 0; i < 500000; i++);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    for(volatile uint32_t i = 0; i < 500000; i++);
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
  static uint32_t dma_irq_count = 0;
  dma_irq_count++;
  if (dma_irq_count % 1000 == 0) {
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);  // LED2闪烁表示DMA中断工作
  }
  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
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

/* USER CODE BEGIN 1 */
/* ==================== FOC电流环相关回调 (20kHz) ==================== */

/**
 * @brief  ADC转换完成回调 - FOC电流环主循环
 * @note   由TIM1 CH4触发，频率20kHz (50us周期)
 * @param  hadc: ADC句柄
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    static uint32_t callback_count = 0;  // 必须是static，否则每次重置
    callback_count++;

    // 测试1：每1000次回调闪烁LED，验证回调是否被调用
    if (callback_count % 1000 == 0) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }

    if (hadc == &hadc2)
    {

        /* 更新电流采样数据 */
        CurrentSense_DMA_CpltCallback(&current_sense);
        /*更新编码器的数据*/
        Encoder_UpdateSpeed(&encoder_M0);
        /* 仅在FOC使能时执行控制算法 */
        if (foc.enabled)
        {
            /* 0. 实时更新电角度（必须在Park变换前更新！） */
            float elec_angle_rad = Encoder_GetAngle_Elec_Rad(&encoder_M0, foc.pole_pairs);
            foc.theta_elec = elec_angle_rad;  // 直接更新，不用FOC_UpdateAngle避免函数调用开销

            /* 1. 获取三相电流 */
            PhaseCurrents_TypeDef i_abc;
            CurrentSense_GetCurrents(&current_sense, &i_abc.Ia, &i_abc.Ib, &i_abc.Ic);

            /* 2. 更新FOC电流（Clarke + Park变换） */
            FOC_UpdateCurrents(&foc, &i_abc);

            /* 3. 电流环PID计算 */
            FOC_CalCurrentLoop(&foc);//

            /* 4. 逆Park变换 (dq -> αβ) */
            Inverse_Park_Transform(&foc.v_dq, foc.theta_elec, &foc.v_alphabeta);

            /* 5. SVPWM调制 */
            SVPWM_TypeDef svpwm;
            SVPWM_Calculate(&foc.v_alphabeta, foc.voltage_supply, &svpwm);
            SVPWM_GetDutyCycles(&svpwm, &foc.duty_a, &foc.duty_b, &foc.duty_c);

            /* 6. 更新PWM占空比 */
            PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, (uint32_t)(foc.duty_a * FOC_PWM_PERIOD));
            PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, (uint32_t)(foc.duty_b * FOC_PWM_PERIOD));
            PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, (uint32_t)(foc.duty_c * FOC_PWM_PERIOD));
        }
    }
}
/**
 * @brief  ADC DMA半完成回调 - FOC不使用此回调
 * @note   如果使用双缓冲模式，可以在这里处理前半部分数据
 * @param  hadc: ADC句柄
 * @retval None
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{


        /* FOC电流环不使用半完成回调 */
        /* 如果需要双缓冲，可以在这里处理 */
        CurrentSense_DMA_HalfCpltCallback(&current_sense);
    
}

/* ==================== 编码器相关回调 ==================== */
/**
  * @brief  GPIO外部中断回调函数
  * @param  GPIO_Pin: 触发中断的GPIO引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // M2编码器Z相中断

    if (GPIO_Pin == M2_ENC_Z_Pin)
    {
        Encoder_ZPulse_Callback(&encoder_M2);
    }

}

/**
 *@ UART发送终端回调函数
 * @param  huart: UART句柄
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART4) {
      // tx_done = 1;  // 修复：应该是赋值，不是比较

      /* VOFA+ DMA发送完成回调 */
      VOFA_UART_TxCpltCallback(huart);
  }
}

/* USER CODE END 1 */
