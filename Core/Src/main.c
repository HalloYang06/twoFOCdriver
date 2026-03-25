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
#include "tamagawa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Encoder_TypeDef encoder_M0, encoder_M1, encoder_M2;
Tamagawa_TypeDef tamagawa_M0;  /* 澶氭懇宸濈紪鐮佸�?Motor0 */
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
void Forcejiaozhun(float voltage, uint32_t time_ms);//寮€鐜牎鍑?
void OpenLoop_SpeedTest(void);//寮€鐜祴璇?
void Tamagawa_ReadBlocking(Tamagawa_TypeDef *tama); // 阻塞式读取多摩川(初始化用)
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

volatile uint32_t g_trace = 0;


/* ==================== 开环速度测试相关变量 ==================== */
float open_loop_angle = 0.0f;       // 开环电角度
float open_loop_velocity = 100.0f;  // 寮€鐜€熷害锛坮ad/s锛夋渶澶?20
float open_loop_voltage = 10.0f;    // 寮€鐜數鍘嬶紙V�?
uint8_t open_loop_enabled = 0;      // 寮€鐜娇鑳芥爣�?
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


  /* 閰嶇疆FPU鍦ㄤ腑鏂腑鐨勮嚜鍔ㄤ繚瀛橈紙閬垮厤涓柇涓娇鐢ㄦ诞鐐规椂HardFault�?*/
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ===== 初始化PWM驱动 ===== */
  PWM_Init();
  /* ===== 初始化VOFA+调试 ===== */
  VOFA_Init(&huart4);
  /* 鍚姩TIM1瀹氭椂鍣?/
  HAL_TIM_Base_Start(&htim1);

  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, 0);
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, 0);
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, 0);

  /* ===== 初始化编码器(ABZ保留，可用于其他电机) ===== */
  //Encoder_Init(&encoder_M0, &htim3);
  //Encoder_Start(&encoder_M0);

  /* ===== 鍒濆鍖栧鎽╁窛缂栫爜�?(Motor0鐢ㄥ鎽╁窛) ===== */
  Tamagawa_Init(&tamagawa_M0, &huart2, MOTOR_POLE_PAIRS);

  /* 闃诲寮忚鍙栦竴娆″鎽╁窛鏁版嵁锛岀‘淇濆垵濮嬩綅缃湁�?*/
  /* 如果通信失败会超时跳过，不阻塞后续初始化 */
  Tamagawa_ReadBlocking(&tamagawa_M0);

  /* LED1浜〃绀哄鎽╁窛鍒濆鍖栧畬鎴?涓嶇鎴愬姛澶辫�? */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /* ===== 鍒濆鍖栫數娴侀噰鏍?===== */
  CurrentSense_Init(&current_sense, &hadc2);

  HAL_Delay(100);  // 等待ADC稳定

  
  //Forcejiaozhun(3.0f, 1500);  // 3V鐢靛帇锛屽�?.5�?

  /* LED1鐏〃绀哄榻愬畬鎴?*/
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  HAL_Delay(100);  // 等待电机完全停止

  /* Start TIM1 CH4 trigger before calibration so ADC2 external trigger is active. */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_Delay(5);
  /* ===== 电流零点校准（在电机静止且不通电时）===== */
  CurrentSense_Calibrate(&current_sense, 500);

  /* ===== 启动ADC DMA采样 ===== */
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)current_sense.adc_buffer, CURRENT_BUFFER_SIZE * 2);

  /* ===== 鍒濆鍖朏OC鎺у埗�?===== */
  FOC_Init(&foc, MOTOR_POLE_PAIRS, 24.0f);

 
  // 第一步调试：先用纯P控制，不加I和D
  PID_Init(&foc.pid_id, 1.2f, 60.0f, 0.0f, 0.00005f, FOC_VOLTAGE_LIMIT);   // d�?
  PID_Init(&foc.pid_iq, 1.2f, 60.0f, 0.0f, 0.00005f, FOC_VOLTAGE_LIMIT);   // q�?
  PID_SetTarget(&foc.pid_id, 0.0f);      
  PID_SetTarget(&foc.pid_iq, 0.05f);    
  /* ===== 先启动所有硬件，最后再使能FOC ===== */
  HAL_TIM_Base_Start_IT(&htim7);  // 鍚姩瀹氭椂鍣?涓柇锛?kHz

  /* 所有硬件就绪后再使能FOC */
  FOC_Enable(&foc);

  /* USER CODE END 2 */

  /* Init scheduler */
  //osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

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
 * @brief  开环强制对齐电角度零点
 * @note   姝ゅ嚱鏁颁細杈撳嚭鍥哄畾d杞寸數鍘嬶紝寮哄埗杞瓙瀵归綈鍒板凡鐭ヤ綅缃?
 * @param  voltage: 对齐电压 (V)
 * @param  time_ms: 对齐时间 (ms)
 * @retval None
 */
void Forcejiaozhun(float voltage, uint32_t time_ms)
{

    DQ_TypeDef align_v_dq;
    align_v_dq.d = voltage;
    align_v_dq.q = 0.0f;

    /* 2. 璁剧疆瀵归綈瑙掑害�?锛堣浆瀛愪細瀵归綈鍒拌繖涓搴︼級 */
    float jiao_angle = 0.0f;

    /* 3. 反Park变换：dq -> αβ */
    AlphaBeta_TypeDef align_v_alphabeta;
    Inverse_Park_Transform(&align_v_dq, jiao_angle, &align_v_alphabeta);

    /* 4. SVPWM调制 */
    SVPWM_TypeDef svpwm;
    SVPWM_Calculate(&align_v_alphabeta, 24.0f, &svpwm);

    float duty_a, duty_b, duty_c;
    SVPWM_GetDutyCycles(&svpwm, &duty_a, &duty_b, &duty_c);

    /* 5. 鏇存柊PWM鍗犵┖姣?*/
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, (uint32_t)(duty_a * FOC_PWM_PERIOD));
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, (uint32_t)(duty_b * FOC_PWM_PERIOD));
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, (uint32_t)(duty_c * FOC_PWM_PERIOD));

    /* 6. 等待转子物理对齐 */
    HAL_Delay(time_ms);

    /* 7. 读取多摩川编码器位置并设置为零点偏移 */
    Tamagawa_ReadBlocking(&tamagawa_M0);
    Tamagawa_AlignElectricZero(&tamagawa_M0);

    /* 8. 停止PWM输出 */
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, 0);
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, 0);
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, 0);
}

/**
 * @brief  闃诲寮忚鍙栧鎽╁窛缂栫爜鍣ㄦ暟鎹紙绾疆璇紝涓嶄緷璧朌MA鍥炶皟锛?
 * @note   用于初始化阶段测试通信是否正常
 * @param  tama: 多摩川结构体指针
 */
void Tamagawa_ReadBlocking(Tamagawa_TypeDef *tama)
{
    uint8_t tx_cmd = TAMA_CMD_ID0;  /* 璇诲崟鍦堬紝鍝嶅�?瀛楄�?*/
    uint8_t rx_temp[16] = {0};
    HAL_StatusTypeDef status;

    /* 先中止之前的DMA操作 */
    HAL_UART_AbortReceive(tama->huart);
    HAL_UART_AbortTransmit(tama->huart);

    /* 鍒囨崲鍒板彂�?*/
    for (volatile uint8_t i = 0; i < 50; i++) __NOP();
    TAMA_RS485_TX();
    for (volatile uint8_t i = 0; i < 50; i++) __NOP();

    /* 杞鍙戦€?瀛楄�?*/
    status = HAL_UART_Transmit(tama->huart, &tx_cmd, 1, 100);

    /* 鍒囨崲鍒版帴鏀?*/
    for (volatile uint8_t i = 0; i < 50; i++) __NOP();
    TAMA_RS485_RX();
    for (volatile uint8_t i = 0; i < 50; i++) __NOP();

    if (status != HAL_OK) return;

    /* 杞鎺ユ敹6瀛楄妭锛岃秴�?00ms */
    status = HAL_UART_Receive(tama->huart, rx_temp, 6, 100);

    if (status == HAL_OK)
    {
        /* 解析 */
        tama->data_id = TAMA_DATA_ID_0;
        Tamagawa_RxParse(tama, rx_temp);

        tama->position_total = tama->position;
        tama->angle_mech_rad = ((float)tama->position / TAMA_ENCODER_RESOLUTION_F) * 6.28318530718f;

        float elec_angle = fmodf(tama->angle_mech_rad * (float)tama->pole_pairs, 6.28318530718f);
        while (elec_angle < 0.0f) elec_angle += 6.28318530718f;
        while (elec_angle >= 6.28318530718f) elec_angle -= 6.28318530718f;
        tama->angle_elec_rad = elec_angle;
    }

    /* 重新启动ReceiveToIdle DMA监听 */
    memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(tama->huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
    if (tama->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(tama->huart->hdmarx, DMA_IT_HT);
    }

    tama->rx_flag = 0;
}

/**
 * @brief  开环速度测试 - SVPWM输出
 * @note   鍦ㄥ畾鏃跺櫒涓柇涓皟鐢紝姣?ms鏇存柊涓€娆?
 * @retval None
 */
void OpenLoop_SpeedTest(void)
{
    if (!open_loop_enabled) {
        return;
    }

    /* 1. 更新开环电角度（积分） */
    open_loop_angle += open_loop_velocity * 0.001f;  // dt = 0.1ms, rad/s转度/s

    /* 2. 归一化角度到 [0, 360°] */
    if (open_loop_angle >=2*PI) {
        open_loop_angle -= 2*PI;
    }
    if (open_loop_angle < 0.0f) {
        open_loop_angle += 2*PI;
    }

    /* 3. 璁＄畻dq杞寸數鍘?(寮€鐜ā寮忎笅锛宨d=0, iq浜х敓杞�? */
    foc.v_dq.d = 0.0f;
    foc.v_dq.q = open_loop_voltage;

    /* 4. dq -> αβ 反Park变换（使用度数） */
    Inverse_Park_Transform(&foc.v_dq, open_loop_angle, &foc.v_alphabeta);

    /* 5. SVPWM调制 */
    SVPWM_TypeDef svpwm;
    SVPWM_Calculate(&foc.v_alphabeta, 24.0f, &svpwm);
    SVPWM_GetDutyCycles(&svpwm, &foc.duty_a, &foc.duty_b, &foc.duty_c);
    /* 6. 鏇存柊PWM鍗犵┖姣?*/
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

 /* TIM7: 1ms鍛ㄦ湡锛岀敤浜庨€熷害鏇存柊鍜屼綆棰戜换鍔?*/
    if (htim == &htim7)
    {
        /* 鍏堥┍鍔ㄥ鎽╁窛閫氫俊鐘舵€佹満锛氬彂閫佽�?鎺ユ敹瑙ｆ�?瓒呮椂鎭㈠ */
        Tamagawa_Update(&tamagawa_M0);

        /* 更新多摩川编码器速度 (1kHz, dt=0.001s) */
        Tamagawa_UpdateSpeed(&tamagawa_M0, 0.001f);

        /* 鏇存柊ABZ缂栫爜鍣ㄩ€熷害锛堜繚鐣欏吋瀹癸紝鏈垵濮嬪寲鏃朵笉璋冪敤锛?*/
        // Encoder_UpdateSpeed(&encoder_M0);

      /* 开环速度测试模式 */
        if (open_loop_enabled)
        {
            OpenLoop_SpeedTest();    // 执行开环SVPWM输出
        }
        /* 闭环FOC控制模式 */
        else if (foc.enabled)
        {
            /* 计算电角速度 (rad/s) - 使用多摩川速度 */
            //foc.omega_elec = Tamagawa_GetSpeed_RPS(&tamagawa_M0) * 2.0f * PI * foc.pole_pairs;

            /* 速度环PID计算（如果启用） */
            //FOC_CalVelocityLoop(&foc);
        }

        /* ===== VOFA+���ݷ��ͣ�200Hz�����ڶ�λż�������� ===== */
        static uint16_t vofa_count = 0;
        vofa_count++;
        if (vofa_count >= 2) {
            float theta_raw = tamagawa_M0.angle_elec_rad;
            float theta_ctrl = foc.theta_elec;
            float i_sum = current_sense.Ia_filtered + current_sense.Ib_filtered + current_sense.Ic_filtered;

            vofa_count = 0;
            VOFA_SendFloat(
                           theta_raw,                       // CH0: ԭʼ��Ƕ�(rad)
                           theta_ctrl,                      // CH1: ���Ƶ�Ƕ�(rad)
                           current_sense.Ia_filtered,       // CH2: Ia
                           current_sense.Ib_filtered,       // CH3: Ib
                           current_sense.Ic_filtered,       // CH4: Ic
                           i_sum,                           // CH5: Ia+Ib+Ic��Ӧ�ӽ�0��
                           foc.i_dq.q,                      // CH6: Iq����
                           foc.target_iq      							// 
                );
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
