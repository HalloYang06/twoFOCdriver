/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "current_sense.h"
#include "pwm_driver.h"
#include "FOC.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "vofa_debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Encoder_TypeDef encoder_M0, encoder_M1, encoder_M2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FOC_TypeDef foc;
Motor_TypeDef motor1;
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)  /* Keil MDK */
__attribute__((section(".ARM.__at_0x30020000"))) CurrentSense_TypeDef current_sense;
#elif defined(__GNUC__)  /* GCC / CubeIDE */
__attribute__((section(".RAM_D2"))) CurrentSense_TypeDef current_sense;
#endif


/* ==================== 开环速度测试相关变量 ==================== */
float open_loop_angle = 0.0f;       // 开环电角度
float open_loop_velocity = 600.0f;  // 开环速度（rad/s）最大920
float open_loop_voltage = 15.0f;    // 开环电压（V）
uint8_t open_loop_enabled = 0;      // 开环使能标志
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* ===== 强制启用 FPU ===== */

  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));


  /* 配置FPU在中断中的自动保存（避免中断中使用浮点时HardFault） */
  __DSB();  // 数据同步屏障
  __ISB();  // 指令同步屏障
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM7_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* ===== 初始化PWM驱动 ===== */
  PWM_Init();


  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, 0); // U相
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, 0); // V相
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, 0); // W相

  /* 启动TIM1定时器*/
  HAL_TIM_Base_Start(&htim1);

  FOC_Enable(&foc);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

  /* ===== 初始化编码器 ===== */
  Encoder_Init(&encoder_M0, &htim3);  // 使用TIM3作为编码器接口
  Encoder_Start(&encoder_M0);

  /* ===== 初始化电流传感器 ===== */
  CurrentSense_Init(&current_sense, &hadc2);

  CurrentSense_Calibrate(&current_sense, 200);

  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)current_sense.adc_buffer, CURRENT_BUFFER_SIZE * 2);

  /* ===== 初始化FOC控制器 ===== */
  FOC_Init(&foc, 11, 24.0f);  // 11极对数，24V供电（根据你的电机参数修改）

  /* ===== 初始化VOFA+调试 ===== */
  VOFA_Init(&huart4);

  /* ===== 启动定时器中断 ===== */
  HAL_TIM_Base_Start_IT(&htim7); // 启动定时器7中断，10kHz




  /*
  const osThreadAttr_t uart_send_task_attributes = {
    .name = "uart_send_task",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  uart_send_task_handle = osThreadNew(uart_send_task, NULL, &uart_send_task_attributes);
  */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  开环速度测试 - SVPWM输出
 * @note   在定时器中断中调用，每1ms更新一次
 * @retval None
 */
void OpenLoop_SpeedTest(void)
{
    if (!open_loop_enabled) {
        return;
    }

    /* 1. 更新开环电角度（积分） */
    open_loop_angle += open_loop_velocity * 0.0001f;  // dt = 0.1ms, rad/s转度/s

    /* 2. 归一化角度到 [0, 360°] */
    if (open_loop_angle >=2*PI) {
        open_loop_angle -= 2*PI;
    }
    if (open_loop_angle < 0.0f) {
        open_loop_angle += 2*PI;
    }

    /* 3. 计算dq轴电压 (开环模式下，id=0, iq产生转矩) */
    foc.v_dq.d = 0.0f;
    foc.v_dq.q = open_loop_voltage;

    /* 4. dq -> αβ 反Park变换（使用度数） */
    Inverse_Park_Transform(&foc.v_dq, open_loop_angle, &foc.v_alphabeta);

    /* 5. SVPWM调制 */
    SVPWM_TypeDef svpwm;
    SVPWM_Calculate(&foc.v_alphabeta, 24.0f, &svpwm);
    SVPWM_GetDutyCycles(&svpwm, &foc.duty_a, &foc.duty_b, &foc.duty_c);
    /* 6. 更新PWM占空比 */
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, (uint32_t)(foc.duty_a * FOC_PWM_PERIOD));
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, (uint32_t)(foc.duty_b * FOC_PWM_PERIOD));
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, (uint32_t)(foc.duty_c * FOC_PWM_PERIOD));
}



/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* ==================== TIM7中断回调 (1kHz) ==================== */

 /* TIM7: 1ms周期，用于FOC控制和数据采集 */
    if (htim == &htim7)
    {
        /* 更新编码器速度 */
       Encoder_UpdateSpeed(&encoder_M0);
      /* 开环速度测试模式 */
        if (open_loop_enabled)
        {
            OpenLoop_SpeedTest();    // 执行开环SVPWM输出
        }
        /* 闭环FOC控制模式 */
        else if (foc.enabled)
        {
            /* 1. 获取电角度（度数，0~2pi） */
            float elec_angle_rad = Encoder_GetAngle_Elec_Rad(&encoder_M0, foc.pole_pairs);

            /* 2. 更新FOC角度 */
            FOC_UpdateAngle(&foc, elec_angle_rad);

            /* 3. 计算电角速度 (rad/s) */
            foc.omega_elec = Encoder_GetSpeed_RPS(&encoder_M0) * 2.0f * PI * foc.pole_pairs;

            /* 4. 速度环PID计算（输出q轴电流目标值） */
            FOC_CalVelocityLoop(&foc);
        }

        /* ===== VOFA+数据发送===== */
        static uint16_t vofa_count = 0;
        vofa_count++;
        if (vofa_count >= 1000) {  
            vofa_count = 0;
            /* 获取当前数据 */
            float encoder_angle = Encoder_GetAngle_Mech_Deg(&encoder_M0); // 机械角度(度)
            float encoder_speed = Encoder_GetSpeed_RPS(&encoder_M0);  // 编码器速度(RPS)
            float current_ia, current_ib, current_ic;
            CurrentSense_GetCurrents(&current_sense, &current_ia, &current_ib, &current_ic);
            
            static char buffer[100];
            int len =sprintf(buffer,"angle:%.1f,speed:%.1f,ia:%.5f,ib:%.5f,ic%.5f,id:%.1f,iq:%.1f\r\n",encoder_angle,encoder_speed,
                              current_ia,current_ib,current_ic,foc.i_dq.d,foc.i_dq.q,foc.target_iq);

            /* 发送到VOFA+ (8个通道) */

            HAL_UART_Transmit_DMA(&huart4, (uint8_t*) buffer, strlen(buffer));
            /*
            VOFA_SendFloat(
                encoder_angle,          // CH0: 编码器电角度 (度)
                encoder_speed,          // CH1: 编码器速度 (RPS)
                current_ia,             // CH2: A相电流 (A)
                current_ib,             // CH3: B相电流 (A)
                current_ic,             // CH4: C相电流 (A)
                foc.i_dq.d,            // CH5: d轴电流 (A)
                foc.i_dq.q,            // CH6: q轴电流 (A)
                foc.target_iq          // CH7: 目标q轴电流 (A)
            );
            */

        }
    }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
