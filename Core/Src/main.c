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
CurrentSense_TypeDef current_sense;

/* ==================== 开环速度测试相关变量 ==================== */
float open_loop_angle = 0.0f;
float open_loop_velocity = 60.0f;    // 降低速度：便于启动
float open_loop_voltage = 10.0f;     // 提高电压：2V → 6V（24V母线的25%）
uint8_t open_loop_enabled = 1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* ===== 强制启用 FPU ===== */
  /* STM32H7 的 Cortex-M7 有硬件FPU，必须先启用才能使用浮点运算 */
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // CP10和CP11全访问权限

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
  /* 初始化PWM驱动，启动6路互补PWM输出 */

  PWM_Init();

  /* 设置初始占空比为0（电机静止） */
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, 0); // U相
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, 0); // V相
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, 0); // W相

  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4); // 启动定时器1通道4，用于电流采样触发ADC1

  /* 初始化编码器 - 用于开环测试速度反馈 */
  Encoder_Init(&encoder_M0, &htim3);  // 使用TIM3作为编码器接口
  Encoder_Start(&encoder_M0);

  /* 初始化FOC控制器 - 用于开环测试 */
  FOC_Init(&foc, 11, 24.0f);  // 7极对数，24V供电（根据你的电机参数修改）

  /* ===== IR2101S 自举电容充电 ===== */
  /* IR2101S 高侧驱动需要自举电容，必须先让低侧导通给电容充电 */
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, 0);  // 低侧导通
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, 0);
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, 0);
  HAL_Delay(100);  // 等待100ms让自举电容充电

  HAL_TIM_Base_Start_IT(&htim7); // 启动定时器7中断，用于编码器速度更新和开环角度更新

  /* 串口输出提示信息 */
  char welcome_msg[] = "openloop test";
	HAL_UART_Transmit(&huart4,(uint8_t*)welcome_msg,10,100);


  uint32_t  last_time=0;
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
    open_loop_angle += open_loop_velocity * 0.0001f;  // dt = 0.1ms

    /* 2. 归一化角度到 [0, 2π] */
    if (open_loop_angle > TWO_PI) {
        open_loop_angle -= TWO_PI;
    }

    /* 3. 计算dq轴电压 (开环模式下，id=0, iq产生转矩) */
    foc.v_dq.d = 0.0f;
    foc.v_dq.q = open_loop_voltage;

    /* 4. dq -> αβ 反Park变换 */
    float cos_theta, sin_theta;
    arm_sin_cos_f32(open_loop_angle * 57.2987f, &sin_theta, &cos_theta);  // 弧度转度数

    foc.v_alphabeta.alpha = foc.v_dq.d * cos_theta - foc.v_dq.q * sin_theta;
    foc.v_alphabeta.beta  = foc.v_dq.d * sin_theta + foc.v_dq.q * cos_theta;

    /* 5. SVPWM调制 */
    SVPWM_TypeDef svpwm;
    SVPWM_Calculate(&foc.v_alphabeta, 24.0f, &svpwm);  // 24V供电
    SVPWM_GetDutyCycles(&svpwm, &foc.duty_a, &foc.duty_b, &foc.duty_c);
    //SPWM_Calculate(&foc,&foc.v_alphabeta,24.0f);
    /* 6. 更新PWM占空比 */
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, (uint32_t)(foc.duty_a * FOC_PWM_PERIOD));
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, (uint32_t)(foc.duty_b * FOC_PWM_PERIOD));
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, (uint32_t)(foc.duty_c * FOC_PWM_PERIOD));
}

/**
 * @brief  打印速度信息到串口
 * @note   每500ms打印一次
 * @retval None
 */
void OpenLoop_PrintSpeed(void)
{
    static uint32_t print_counter = 0;
    static uint8_t led_state = 0;

    if (!open_loop_enabled) {
        return;
    }

    print_counter++;
    if (print_counter >= 500) {  // 每500ms打印一次
        print_counter = 0;

        /* 切换LED状态，确认程序运行 */
        led_state = !led_state;
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

        float speed_rpm = Encoder_GetSpeed_RPM(&encoder_M0);
        float speed_rps = Encoder_GetSpeed_RPS(&encoder_M0);
        float elec_angle_deg = open_loop_angle * 180.0f / PI;

        char msg[200];
        sprintf(msg, "[OpenLoop] Angle:%.1f deg | Duty: A=%.2f B=%.2f C=%.2f | Vq=%.2f V\r\n",
                elec_angle_deg, foc.duty_a, foc.duty_b, foc.duty_c, open_loop_voltage);
        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 100);
    }
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
  /* ==================== 开环测试 / FOC速度环相关回调 (1kHz) ==================== */

/**
 * @brief  TIM定时器周期回调 - 开环测试 / FOC速度环主循环
 * @note   TIM7每1ms触发一次
 * @param  htim: 定时器句柄
 * @retval None
 */
 /* TIM7: 开环测试 / 闭环速度环控制 (1kHz) */
    if (htim == &htim7)
    {
        /* 更新编码器速度 */
        Encoder_UpdateSpeed(&encoder_M0);

        /* 开环速度测试模式 */
        if (open_loop_enabled)
        {
            OpenLoop_SpeedTest();    // 执行开环SVPWM输出
            OpenLoop_PrintSpeed();   // 打印速度信息
        }
        /* 闭环FOC控制模式 */
        else if (foc.enabled)
        {
            /* 1. 计算机械角度 (rad) */
            float mechanical_angle = (float)Encoder_GetTotalCount(&encoder_M0)
                                   / (4.0f * ENCODER_PPR) * 2.0f * PI;

            /* 2. 更新FOC角度（机械角 → 电角） */
            FOC_UpdateAngle(&foc, mechanical_angle);

            /* 3. 计算电角速度 (rad/s) */
            foc.omega_elec = Encoder_GetSpeed_RPS(&encoder_M0) * 2.0f * PI * foc.pole_pairs;

            /* 4. 速度环PID计算（输出q轴电流目标值） */
            FOC_CalVelocityLoop(&foc);
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
