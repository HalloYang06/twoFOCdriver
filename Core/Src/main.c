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
Tamagawa_TypeDef tamagawa_M0;  /* 婢舵碍鎳囧婵堢椽閻礁锟?Motor0 */
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
void Forcejiaozhun(float voltage, uint32_t time_ms);//瀵偓閻滎垱鐗庨崙?
void OpenLoop_SpeedTest(void);//瀵偓閻滎垱绁寸拠?
void Tamagawa_ReadBlocking(Tamagawa_TypeDef *tama); // 闃诲寮忚鍙栧鎽╁窛(鍒濆鍖栫敤)
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


/* ==================== 寮€鐜€熷害娴嬭瘯鐩稿叧鍙橀噺 ==================== */
float open_loop_angle = 0.0f;       // 寮€鐜數瑙掑害
float open_loop_velocity = 100.0f;  //寮€鐜€熷害
float open_loop_voltage = 10.0f;    //寮€鐜數鍘?
uint8_t open_loop_enabled = 1;      // 寮€鐜娇鑳芥爣蹇?
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* ===== 寮哄埗鍚敤 FPU ===== */

  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));


  
  __DSB();  // 鏁版嵁鍚屾灞忛殰
  __ISB();  // 鎸囦护鍚屾灞忛殰
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
  /* ===== 鍒濆鍖朠WM椹卞姩 ===== */
  PWM_Init();
  /* ===== 鍒濆鍖朧OFA+璋冭瘯 ===== */
  VOFA_Init(&huart4);
  
  HAL_TIM_Base_Start(&htim1);

  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, 0);
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, 0);
  PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, 0);

  /* ===== 鍒濆鍖栫紪鐮佸櫒(ABZ淇濈暀锛屽彲鐢ㄤ簬鍏朵粬鐢垫満) ===== */
  //Encoder_Init(&encoder_M0, &htim3);
  //Encoder_Start(&encoder_M0);

  /* ===== 澶氭懇宸濆垵濮嬪寲===== */
  Tamagawa_Init(&tamagawa_M0, &huart2, MOTOR_POLE_PAIRS);

 
  /* 濡傛灉閫氫俊澶辫触浼氳秴鏃惰烦杩囷紝涓嶉樆濉炲悗缁垵濮嬪寲 */
  Tamagawa_ReadBlocking(&tamagawa_M0);

  /* LED1娴滎喛銆冪粈鍝勵樋閹解晛绐涢崚婵嗩潗閸栨牕鐣幋?娑撳秶顓搁幋鎰婢惰精锟? */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /* ===== 鐢垫祦閲囨牱鍒濆鍖?==== */
  CurrentSense_Init(&current_sense, &hadc2);

  HAL_Delay(100);  // 绛夊緟ADC绋冲畾

  
  //Forcejiaozhun(3.0f, 1500);  // 3V寮哄埗瀵归綈

 
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  HAL_Delay(100);  // 绛夊緟鐢垫満瀹屽叏鍋滄

  /* Start TIM1 CH4 trigger before calibration so ADC2 external trigger is active. */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_Delay(5);
  /* ===== 鐢垫祦闆剁偣鏍″噯锛堝湪鐢垫満闈欐涓斾笉閫氱數鏃讹級===== */
  CurrentSense_Calibrate(&current_sense, 2000);

  /* ===== 鍚姩ADC DMA閲囨牱 ===== */
  CurrentSense_Start(&current_sense);

  /* =====鍒濆鍖杅oc===== */
  FOC_Init(&foc, MOTOR_POLE_PAIRS, 24.0f);

 
  // 绗竴姝ヨ皟璇曪細鍏堢敤绾疨鎺у埗锛屼笉鍔營鍜孌
  PID_Init(&foc.pid_id, 1.2f, 60.0f, 0.0f, 0.00005f, FOC_VOLTAGE_LIMIT);  
  PID_Init(&foc.pid_iq, 1.2f, 60.0f, 0.0f, 0.00005f, FOC_VOLTAGE_LIMIT);   
  PID_SetTarget(&foc.pid_id, 0.0f);      
  PID_SetTarget(&foc.pid_iq, 0.05f);    
  /* ===== 鍏堝惎鍔ㄦ墍鏈夌‖浠讹紝鏈€鍚庡啀浣胯兘FOC ===== */
  HAL_TIM_Base_Start_IT(&htim7);  // 瀹氭椂涓冨垵濮嬪寲涓?000hz

  /* 鎵€鏈夌‖浠跺氨缁悗鍐嶄娇鑳紽OC */
  //FOC_Enable(&foc);

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
 * @brief  寮€鐜己鍒跺榻愮數瑙掑害闆剁偣
 * @note   濮濄倕鍤遍弫棰佺窗鏉堟挸鍤崶鍝勭暰d鏉炲鏁搁崢瀣剁礉瀵搫鍩楁潪顒€鐡欑€靛綊缍堥崚鏉垮嚒閻儰缍呯純?
 * @param  voltage: 瀵归綈鐢靛帇 (V)
 * @param  time_ms: 瀵归綈鏃堕棿 (ms)
 * @retval None
 */
void Forcejiaozhun(float voltage, uint32_t time_ms)
{

    DQ_TypeDef align_v_dq;
    align_v_dq.d = voltage;
    align_v_dq.q = 0.0f;

    /* 2. 鐠佸墽鐤嗙€靛綊缍堢憴鎺戝锟?閿涘牐娴嗙€涙劒绱扮€靛綊缍堥崚鎷岀箹娑擃亣顫楁惔锔肩礆 */
    float jiao_angle = 0.0f;

    /* 3. 鍙峆ark鍙樻崲锛歞q -> 伪尾 */
    AlphaBeta_TypeDef align_v_alphabeta;
    Inverse_Park_Transform(&align_v_dq, jiao_angle, &align_v_alphabeta);

    /* 4. SVPWM璋冨埗 */
    SVPWM_TypeDef svpwm;
    SVPWM_Calculate(&align_v_alphabeta, 24.0f, &svpwm);

    float duty_a, duty_b, duty_c;
    SVPWM_GetDutyCycles(&svpwm, &duty_a, &duty_b, &duty_c);

    /* 5. 閺囧瓨鏌奝WM閸楃姷鈹栧В?*/
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, (uint32_t)(duty_a * FOC_PWM_PERIOD));
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, (uint32_t)(duty_b * FOC_PWM_PERIOD));
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, (uint32_t)(duty_c * FOC_PWM_PERIOD));

    /* 6. 绛夊緟杞瓙鐗╃悊瀵归綈 */
    HAL_Delay(time_ms);

    /* 7. 璇诲彇澶氭懇宸濈紪鐮佸櫒浣嶇疆骞惰缃负闆剁偣鍋忕Щ */
    Tamagawa_ReadBlocking(&tamagawa_M0);
    Tamagawa_AlignElectricZero(&tamagawa_M0);

    /* 8. 鍋滄PWM杈撳嚭 */
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, 0);
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, 0);
    PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, 0);
}

/**
 * @brief  闂冭顢ｅ蹇氼嚢閸欐牕顦块幗鈺佺獩缂傛牜鐖滈崳銊︽殶閹诡噯绱欑痪顖濈枂鐠囶澁绱濇稉宥勭贩鐠ф湆MA閸ョ偠鐨熼敍?
 * @note   鐢ㄤ簬鍒濆鍖栭樁娈垫祴璇曢€氫俊鏄惁姝ｅ父
 * @param  tama: 澶氭懇宸濈粨鏋勪綋鎸囬拡
 */
void Tamagawa_ReadBlocking(Tamagawa_TypeDef *tama)
{
    uint8_t tx_cmd = TAMA_CMD_ID0;  /* 鐠囪宕熼崷鍫礉閸濆秴锟?鐎涙锟?*/
    uint8_t rx_temp[16] = {0};
    HAL_StatusTypeDef status;

    /* 鍏堜腑姝箣鍓嶇殑DMA鎿嶄綔 */
    HAL_UART_AbortReceive(tama->huart);
    HAL_UART_AbortTransmit(tama->huart);

    /* 閸掑洦宕查崚鏉垮絺锟?*/
    for (volatile uint8_t i = 0; i < 50; i++) __NOP();
    TAMA_RS485_TX();
    for (volatile uint8_t i = 0; i < 50; i++) __NOP();

    /* 鏉烆喛顕楅崣鎴︹偓?鐎涙锟?*/
    status = HAL_UART_Transmit(tama->huart, &tx_cmd, 1, 100);

    /* 閸掑洦宕查崚鐗堝复閺€?*/
    for (volatile uint8_t i = 0; i < 50; i++) __NOP();
    TAMA_RS485_RX();
    for (volatile uint8_t i = 0; i < 50; i++) __NOP();

    if (status != HAL_OK) return;

    /* 鏉烆喛顕楅幒銉︽暪6鐎涙濡敍宀冪Т锟?00ms */
    status = HAL_UART_Receive(tama->huart, rx_temp, 6, 100);

    if (status == HAL_OK)
    {
        /* 瑙ｆ瀽 */
        tama->data_id = TAMA_DATA_ID_0;
        Tamagawa_RxParse(tama, rx_temp);

        tama->position_total = tama->position;
        tama->angle_mech_rad = ((float)tama->position / TAMA_ENCODER_RESOLUTION_F) * 6.28318530718f;

        float elec_angle = fmodf(tama->angle_mech_rad * (float)tama->pole_pairs, 6.28318530718f);
        while (elec_angle < 0.0f) elec_angle += 6.28318530718f;
        while (elec_angle >= 6.28318530718f) elec_angle -= 6.28318530718f;
        tama->angle_elec_rad = elec_angle;
    }

    /* 閲嶆柊鍚姩ReceiveToIdle DMA鐩戝惉 */
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
 * @brief  寮€鐜€熷害娴嬭瘯 - SVPWM杈撳嚭
 * @note   閸︺劌鐣鹃弮璺烘珤娑擃厽鏌囨稉顓＄殶閻㈩煉绱濆В?ms閺囧瓨鏌婃稉鈧▎?
 * @retval None
 */
void OpenLoop_SpeedTest(void)
{
    if (!open_loop_enabled) {
        return;
    }

    /* 1. 鏇存柊寮€鐜數瑙掑害锛堢Н鍒嗭級 */
    open_loop_angle += open_loop_velocity * 0.001f;  // dt = 0.1ms, rad/s杞害/s

    /* 2. 褰掍竴鍖栬搴﹀埌 [0, 360掳] */
    if (open_loop_angle >=2*PI) {
        open_loop_angle -= 2*PI;
    }
    if (open_loop_angle < 0.0f) {
        open_loop_angle += 2*PI;
    }

    /* 3. 鐠侊紕鐣籨q鏉炲鏁搁崢?(瀵偓閻滎垱膩瀵繋绗呴敍瀹╠=0, iq娴溠呮晸鏉烆剛锟? */
    foc.v_dq.d = 0.0f;
    foc.v_dq.q = open_loop_voltage;

    /* 4. dq -> 伪尾 鍙峆ark鍙樻崲锛堜娇鐢ㄥ害鏁帮級 */
    Inverse_Park_Transform(&foc.v_dq, open_loop_angle, &foc.v_alphabeta);

    /* 5. SVPWM璋冨埗 */
    SVPWM_TypeDef svpwm;
    SVPWM_Calculate(&foc.v_alphabeta, 24.0f, &svpwm);
    SVPWM_GetDutyCycles(&svpwm, &foc.duty_a, &foc.duty_b, &foc.duty_c);
    /* 6. 閺囧瓨鏌奝WM閸楃姷鈹栧В?*/
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
  /* ==================== TIM7涓柇鍥炶皟 (1kHz) ==================== */

 /* TIM7: 1ms閸涖劍婀￠敍宀€鏁ゆ禍搴ㄢ偓鐔峰閺囧瓨鏌婇崪灞肩秵妫版垳鎹㈤崝?*/
    if (htim == &htim7)
    {
        /* 閸忓牓鈹嶉崝銊ヮ樋閹解晛绐涢柅姘繆閻樿埖鈧焦婧€閿涙艾褰傞柅浣筋嚞锟?閹恒儲鏁圭憴锝嗭拷?鐡掑懏妞傞幁銏狀槻 */
        Tamagawa_Update(&tamagawa_M0);

        /* 鏇存柊澶氭懇宸濈紪鐮佸櫒閫熷害 (1kHz, dt=0.001s) */
        Tamagawa_UpdateSpeed(&tamagawa_M0, 0.001f);

        /* 閺囧瓨鏌夾BZ缂傛牜鐖滈崳銊┾偓鐔峰閿涘牅绻氶悾娆忓悑鐎圭櫢绱濋張顏勫灥婵瀵查弮鏈电瑝鐠嬪啰鏁ら敍?*/
        // Encoder_UpdateSpeed(&encoder_M0);

      /* 寮€鐜€熷害娴嬭瘯妯″紡 */
        if (open_loop_enabled)
        {
            OpenLoop_SpeedTest();    // 鎵ц寮€鐜疭VPWM杈撳嚭
        }
        /* 闂幆FOC鎺у埗妯″紡 */
        else if (foc.enabled)
        {
            /* 璁＄畻鐢佃閫熷害 (rad/s) - 浣跨敤澶氭懇宸濋€熷害 */
            //foc.omega_elec = Tamagawa_GetSpeed_RPS(&tamagawa_M0) * 2.0f * PI * foc.pole_pairs;

            /* 閫熷害鐜疨ID璁＄畻锛堝鏋滃惎鐢級 */
            //FOC_CalVelocityLoop(&foc);
        }

        /* ===== VOFA+璋冭瘯 ===== */
        static uint16_t vofa_count = 0;
        vofa_count++;
        if (vofa_count >= 2) {
            float theta_raw = tamagawa_M0.angle_elec_rad;
            float theta_ctrl = foc.theta_elec;
            float i_sum = current_sense.Ia_filtered + current_sense.Ib_filtered + current_sense.Ic_filtered;

            vofa_count = 0;
            VOFA_SendFloat(
                           theta_raw,                       // CH0: 
                           theta_ctrl,                      // CH1: 
                           current_sense.Ia_filtered,       // CH2: Ia filtered
                           current_sense.Ib_filtered,       // CH3: Ib filtered
                           current_sense.Ic_filtered,       // CH4: Ic filtered
                           i_sum,                           // CH5: filtered sum
                           foc.i_dq.q,                      // CH6: Iq
                           foc.target_iq                    // CH7: target Iq
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

