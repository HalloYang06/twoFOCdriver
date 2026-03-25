



#include "current_sense.h"
#include <string.h>

/*
 * 二阶Butterworth低通滤波器系数
 * Fs = 20kHz, Fc = 2kHz
 *
 * 用MATLAB/Python计算:
 *   from scipy.signal import butter, bilinear
 *   b, a = butter(2, 2000, fs=20000)
 *   b = [0.06745527, 0.13491055, 0.06745527]
 *   a = [1.0, -1.14298050, 0.41280159]
 *
 * CMSIS-DSP Biquad DF1 系数格式: {b0, b1, b2, a1, a2}
 * 注意: CMSIS-DSP的a1/a2符号取反 (内部�?y = b0*x + b1*x1 + b2*x2 + a1*y1 + a2*y2)
 * 所�?a1 = -(-1.14298050) = 1.14298050, a2 = -(0.41280159) = -0.41280159
 */
static const float biquad_coeffs[5] = {
    0.06745527f,    /* b0 */
    0.13491055f,    /* b1 */
    0.06745527f,    /* b2 */
    1.14298050f,    /* a1 (CMSIS取反) */
   -0.41280159f     /* a2 (CMSIS取反) */
};

/**
 * @brief  初始化电流采样模�?
 * @param  cs: 电流采样结构体指�?
 * @param  hadc: ADC句柄
 * @retval None
 */
void CurrentSense_Init(CurrentSense_TypeDef *cs, ADC_HandleTypeDef *hadc)
{
    cs->hadc = hadc;

    // 清零缓冲�?
    memset(cs->adc_buffer, 0, sizeof(cs->adc_buffer));

    // 初始化电流�?
    cs->Ia = 0.0f;
    cs->Ib = 0.0f;
    cs->Ic = 0.0f;

    cs->Ia_filtered = 0.0f;
    cs->Ib_filtered = 0.0f;
    cs->Ic_filtered = 0.0f;

    // 默认零点偏移（对�?.65V的ADC值）
    cs->offset_a = (uint16_t)(CURRENT_SENSE_OFFSET / ADC_VREF * ADC_RESOLUTION);
    cs->offset_b = (uint16_t)(CURRENT_SENSE_OFFSET / ADC_VREF * ADC_RESOLUTION);
    cs->offset_c = (uint16_t)(CURRENT_SENSE_OFFSET / ADC_VREF * ADC_RESOLUTION);

    cs->data_ready = 0;
    cs->buffer_index = 0;

#if CURRENT_FILTER_ENABLED
    /* 初始化三相Biquad IIR滤波�?*/
    memset(cs->filter_state_a, 0, sizeof(cs->filter_state_a));
    memset(cs->filter_state_b, 0, sizeof(cs->filter_state_b));
    memset(cs->filter_state_c, 0, sizeof(cs->filter_state_c));

    arm_biquad_cascade_df1_init_f32(&cs->filter_a, BIQUAD_NUM_STAGES,
                                     (float *)biquad_coeffs, cs->filter_state_a);
    arm_biquad_cascade_df1_init_f32(&cs->filter_b, BIQUAD_NUM_STAGES,
                                     (float *)biquad_coeffs, cs->filter_state_b);
    arm_biquad_cascade_df1_init_f32(&cs->filter_c, BIQUAD_NUM_STAGES,
                                     (float *)biquad_coeffs, cs->filter_state_c);
#endif
}

/**
 * @brief  启动电流采样（DMA模式�?
 * @param  cs: 电流采样结构体指�?
 * @retval None
 */
void CurrentSense_Start(CurrentSense_TypeDef *cs)
{
    // 启动ADC校准（可选）
    HAL_ADCEx_Calibration_Start(cs->hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    // 启动ADC DMA循环模式
    HAL_ADC_Start_DMA(cs->hadc, (uint32_t*)cs->adc_buffer, CURRENT_BUFFER_SIZE * 2);
}

/**
 * @brief  停止电流采样
 * @param  cs: 电流采样结构体指�?
 * @retval None
 */
void CurrentSense_Stop(CurrentSense_TypeDef *cs)
{
    HAL_ADC_Stop_DMA(cs->hadc);
}

/**
 * @brief  电流传感器零点校�?
 * @note   在电机不通电时调用此函数进行零点校准
 * @param  cs: 电流采样结构体指�?
 * @param  sample_count: 采样次数（用于平均）
 * @retval None
 */
void CurrentSense_Calibrate(CurrentSense_TypeDef *cs, uint32_t sample_count)
{
    uint32_t sum_a = 0, sum_b = 0, sum_c = 0;

    // 使用current_sense自身的adc_buffer（位于D2 SRAM，DMA可访问）
    // 不能用栈上局部变量，因为栈在DTCM RAM，STM32H7 DMA无法访问DTCM
    HAL_ADC_Start_DMA(cs->hadc, (uint32_t*)cs->adc_buffer, CURRENT_BUFFER_SIZE);

    // 丢弃前几次采样，让ADC和电路稳�?
    HAL_Delay(50);

    for (uint32_t i = 0; i < sample_count; i++)
    {
        // 等待DMA传输完成（等待一个完整的扫描序列�?
        HAL_Delay(2);  // 给ADC足够的时间完成扫�?

        // 无效化DCache，确保读到DMA写入的最新数�?
        SCB_InvalidateDCache_by_Addr((uint32_t *)cs->adc_buffer, sizeof(cs->adc_buffer));
        // ADC2扫描顺序: CH15(SO2=B�?, CH3(SO1=A�?, CH8(SO3=C�?
        sum_a += cs->adc_buffer[CURRENT_PHASE_A_INDEX];
        sum_b += cs->adc_buffer[CURRENT_PHASE_B_INDEX];
        sum_c += cs->adc_buffer[CURRENT_PHASE_C_INDEX];
    }

    // 停止DMA采样
    HAL_ADC_Stop_DMA(cs->hadc);

    // 计算平均值作为零点偏�?
    cs->offset_a = (uint16_t)(sum_a / sample_count);
    cs->offset_b = (uint16_t)(sum_b / sample_count);
    cs->offset_c = (uint16_t)(sum_c / sample_count);

#if CURRENT_FILTER_ENABLED
    /* 校准后重置滤波器状态，避免残留旧数�?*/
    memset(cs->filter_state_a, 0, sizeof(cs->filter_state_a));
    memset(cs->filter_state_b, 0, sizeof(cs->filter_state_b));
    memset(cs->filter_state_c, 0, sizeof(cs->filter_state_c));
#endif
}

/**
 * @brief  ADC值转换为电流�?
 * @param  adc_value: ADC原始�?
 * @param  offset: 零点偏移�?
 * @retval 电流�?(A)
 */
float CurrentSense_ADCToCurrent(uint16_t adc_value, uint16_t offset)
{
    // 转换ADC值为电压
    float voltage = (float)adc_value / ADC_RESOLUTION * ADC_VREF;

    // 转换电压为电�?
    // 公式: Vout = 16.5 * 0.01 * I + 1.65
    // 反推: I = (Vout - offset_voltage) / 0.165
    float offset_voltage = (float)offset / ADC_RESOLUTION * ADC_VREF;
    float current = (voltage - offset_voltage) / CURRENT_SENSE_SENSITIVITY;

    return current;
}

/**
 * @brief  更新电流值（从DMA缓冲区读取并处理�?
 * @param  cs: 电流采样结构体指�?
 * @param  buffer_half: 处理的缓冲区部分 (0=前半部分, 1=后半部分)
 * @retval None
 */
void CurrentSense_Update(CurrentSense_TypeDef *cs, uint8_t buffer_half)
{
    uint16_t *buffer_ptr;

    // 选择要处理的缓冲区部�?
    if (buffer_half == 0) {
        buffer_ptr = &cs->adc_buffer[0];  // 前半部分
    } else {
        buffer_ptr = &cs->adc_buffer[CURRENT_BUFFER_SIZE];  // 后半部分
    }

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    /* Ensure CPU reads fresh ADC samples written by DMA. */
    SCB_InvalidateDCache_by_Addr((uint32_t *)buffer_ptr, CURRENT_BUFFER_SIZE * sizeof(uint16_t));
#endif

    // 从缓冲区读取ADC值并转换为电�?
    // ADC2扫描顺序: CH15(SO2=B�?, CH3(SO1=A�?, CH8(SO3=C�?
    float Ia_raw = CURRENT_PHASE_A_SIGN * CurrentSense_ADCToCurrent(buffer_ptr[CURRENT_PHASE_A_INDEX], cs->offset_a);
    float Ib_raw = CURRENT_PHASE_B_SIGN * CurrentSense_ADCToCurrent(buffer_ptr[CURRENT_PHASE_B_INDEX], cs->offset_b);
    float Ic_raw = CURRENT_PHASE_C_SIGN * CurrentSense_ADCToCurrent(buffer_ptr[CURRENT_PHASE_C_INDEX], cs->offset_c);

    // 更新原始电流�?
    cs->Ia = Ia_raw;
    cs->Ib = Ib_raw;
    cs->Ic = Ic_raw;

#if CURRENT_FILTER_ENABLED
    /* 二阶Butterworth IIR滤波 (CMSIS-DSP Biquad DF1)
     * 每次处理1个采样点(blockSize=1)，运算量极小
     */
    float in, out;

    in = Ia_raw;
    arm_biquad_cascade_df1_f32(&cs->filter_a, &in, &out, 1);
    cs->Ia_filtered = out;

    in = Ib_raw;
    arm_biquad_cascade_df1_f32(&cs->filter_b, &in, &out, 1);
    cs->Ib_filtered = out;

    in = Ic_raw;
    arm_biquad_cascade_df1_f32(&cs->filter_c, &in, &out, 1);
    cs->Ic_filtered = out;
#else
    // 不滤波，直接使用原始�?
    cs->Ia_filtered = Ia_raw;
    cs->Ib_filtered = Ib_raw;
    cs->Ic_filtered = Ic_raw;
#endif

#if CURRENT_ZERO_SUM_COMPENSATION
    /* Remove common-mode offset so Ia+Ib+Ic tends to zero. */
    float i_avg = (cs->Ia_filtered + cs->Ib_filtered + cs->Ic_filtered) * 0.33333334f;
    cs->Ia_filtered -= i_avg;
    cs->Ib_filtered -= i_avg;
    cs->Ic_filtered -= i_avg;
#endif

    // 设置数据就绪标志
    cs->data_ready = 1;
}

/**
 * @brief  获取三相电流�?
 * @param  cs: 电流采样结构体指�?
 * @param  Ia: A相电流指�?
 * @param  Ib: B相电流指�?
 * @param  Ic: C相电流指�?
 * @retval None
 */
void CurrentSense_GetCurrents(CurrentSense_TypeDef *cs, float *Ia, float *Ib, float *Ic)
{
    *Ia = cs->Ia_filtered;
    *Ib = cs->Ib_filtered;
    *Ic = cs->Ic_filtered;

    // 清除数据就绪标志
    cs->data_ready = 0;
}

/**
 * @brief  DMA半完成中断回�?
 * @note   在HAL_ADC_ConvHalfCpltCallback中调用此函数
 * @param  cs: 电流采样结构体指�?
 * @retval None
 */
void CurrentSense_DMA_HalfCpltCallback(CurrentSense_TypeDef *cs)
{
    // DMA传输到一半，处理前半部分缓冲�?
    CurrentSense_Update(cs, 0);
    cs->buffer_index = 0;
}

/**
 * @brief  DMA完成中断回调
 * @note   在HAL_ADC_ConvCpltCallback中调用此函数
 * @param  cs: 电流采样结构体指�?
 * @retval None
 */
void CurrentSense_DMA_CpltCallback(CurrentSense_TypeDef *cs)
{
    // DMA传输完成，处理后半部分缓冲区
    CurrentSense_Update(cs, 1);
    cs->buffer_index = 1;
}



