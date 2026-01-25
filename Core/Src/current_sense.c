

//
// Created by Claude for FOC Current Sensing
//

#include "current_sense.h"
#include <string.h>

/**
 * @brief  初始化电流采样模块
 * @param  cs: 电流采样结构体指针
 * @param  hadc: ADC句柄
 * @retval None
 */
void CurrentSense_Init(CurrentSense_TypeDef *cs, ADC_HandleTypeDef *hadc)
{
    cs->hadc = hadc;

    // 清零缓冲区
    memset(cs->adc_buffer, 0, sizeof(cs->adc_buffer));

    // 初始化电流值
    cs->Ia = 0.0f;
    cs->Ib = 0.0f;
    cs->Ic = 0.0f;

    cs->Ia_filtered = 0.0f;
    cs->Ib_filtered = 0.0f;
    cs->Ic_filtered = 0.0f;

    // 默认零点偏移（对应1.65V的ADC值）
    // ADC值 = 电压 / VREF * ADC_RESOLUTION
    cs->offset_a = (uint16_t)(CURRENT_SENSE_OFFSET / ADC_VREF * ADC_RESOLUTION);
    cs->offset_b = (uint16_t)(CURRENT_SENSE_OFFSET / ADC_VREF * ADC_RESOLUTION);
    cs->offset_c = (uint16_t)(CURRENT_SENSE_OFFSET / ADC_VREF * ADC_RESOLUTION);

    cs->data_ready = 0;
    cs->buffer_index = 0;
}

/**
 * @brief  启动电流采样（DMA模式）
 * @param  cs: 电流采样结构体指针
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
 * @param  cs: 电流采样结构体指针
 * @retval None
 */
void CurrentSense_Stop(CurrentSense_TypeDef *cs)
{
    HAL_ADC_Stop_DMA(cs->hadc);
}

/**
 * @brief  电流传感器零点校准
 * @note   在电机不通电时调用此函数进行零点校准
 * @param  cs: 电流采样结构体指针
 * @param  sample_count: 采样次数（用于平均）
 * @retval None
 */
void CurrentSense_Calibrate(CurrentSense_TypeDef *cs, uint32_t sample_count)
{
    uint32_t sum_a = 0, sum_b = 0, sum_c = 0;

    for (uint32_t i = 0; i < sample_count; i++)
    {
        // 启动单次转换
        HAL_ADC_Start(cs->hadc);
        HAL_ADC_PollForConversion(cs->hadc, 100);
        uint32_t adc_a = HAL_ADC_GetValue(cs->hadc);

        HAL_ADC_PollForConversion(cs->hadc, 100);
        uint32_t adc_b = HAL_ADC_GetValue(cs->hadc);

        HAL_ADC_PollForConversion(cs->hadc, 100);
        uint32_t adc_c = HAL_ADC_GetValue(cs->hadc);

        HAL_ADC_Stop(cs->hadc);

        sum_a += adc_a;
        sum_b += adc_b;
        sum_c += adc_c;

        HAL_Delay(1);
    }

    // 计算平均值作为零点偏移
    cs->offset_a = (uint16_t)(sum_a / sample_count);
    cs->offset_b = (uint16_t)(sum_b / sample_count);
    cs->offset_c = (uint16_t)(sum_c / sample_count);
}

/**
 * @brief  ADC值转换为电流值
 * @param  adc_value: ADC原始值
 * @param  offset: 零点偏移值
 * @retval 电流值 (A)
 */
float CurrentSense_ADCToCurrent(uint16_t adc_value, uint16_t offset)
{
    // 转换ADC值为电压
    float voltage = (float)adc_value / ADC_RESOLUTION * ADC_VREF;

    // 转换电压为电流
    // 公式: Vout = 16.5 * 0.01 * I + 1.65
    // 反推: I = (Vout - offset_voltage) / 0.165
    float offset_voltage = (float)offset / ADC_RESOLUTION * ADC_VREF;
    float current = (voltage - offset_voltage) / CURRENT_SENSE_SENSITIVITY;

    return current;
}

/**
 * @brief  更新电流值（从DMA缓冲区读取并处理）
 * @param  cs: 电流采样结构体指针
 * @param  buffer_half: 处理的缓冲区部分 (0=前半部分, 1=后半部分)
 * @retval None
 */
void CurrentSense_Update(CurrentSense_TypeDef *cs, uint8_t buffer_half)
{
    uint16_t *buffer_ptr;

    // 选择要处理的缓冲区部分
    if (buffer_half == 0) {
        buffer_ptr = &cs->adc_buffer[0];  // 前半部分
    } else {
        buffer_ptr = &cs->adc_buffer[CURRENT_BUFFER_SIZE];  // 后半部分
    }

    // 从缓冲区读取ADC值并转换为电流
    // 根据adc.c的配置顺序: Channel 11 (Rank 1), Channel 4 (Rank 2), Channel 18 (Rank 3)
    // 假设对应 A, B, C 三相
    float Ia_raw = CurrentSense_ADCToCurrent(buffer_ptr[0], cs->offset_a);
    float Ib_raw = CurrentSense_ADCToCurrent(buffer_ptr[1], cs->offset_b);
    float Ic_raw = CurrentSense_ADCToCurrent(buffer_ptr[2], cs->offset_c);

    // 更新原始电流值
    cs->Ia = Ia_raw;
    cs->Ib = Ib_raw;
    cs->Ic = Ic_raw;

#if CURRENT_FILTER_ENABLED
    // 一阶低通滤波: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    cs->Ia_filtered = CURRENT_FILTER_ALPHA * Ia_raw + (1.0f - CURRENT_FILTER_ALPHA) * cs->Ia_filtered;
    cs->Ib_filtered = CURRENT_FILTER_ALPHA * Ib_raw + (1.0f - CURRENT_FILTER_ALPHA) * cs->Ib_filtered;
    cs->Ic_filtered = CURRENT_FILTER_ALPHA * Ic_raw + (1.0f - CURRENT_FILTER_ALPHA) * cs->Ic_filtered;
#else
    // 不滤波，直接使用原始值
    cs->Ia_filtered = Ia_raw;
    cs->Ib_filtered = Ib_raw;
    cs->Ic_filtered = Ic_raw;
#endif

    // 设置数据就绪标志
    cs->data_ready = 1;
}

/**
 * @brief  获取三相电流值
 * @param  cs: 电流采样结构体指针
 * @param  Ia: A相电流指针
 * @param  Ib: B相电流指针
 * @param  Ic: C相电流指针
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
 * @brief  DMA半完成中断回调
 * @note   在HAL_ADC_ConvHalfCpltCallback中调用此函数
 * @param  cs: 电流采样结构体指针
 * @retval None
 */
void CurrentSense_DMA_HalfCpltCallback(CurrentSense_TypeDef *cs)
{
    // DMA传输到一半，处理前半部分缓冲区
    CurrentSense_Update(cs, 0);
    cs->buffer_index = 0;
}

/**
 * @brief  DMA完成中断回调
 * @note   在HAL_ADC_ConvCpltCallback中调用此函数
 * @param  cs: 电流采样结构体指针
 * @retval None
 */
void CurrentSense_DMA_CpltCallback(CurrentSense_TypeDef *cs)
{
    // DMA传输完成，处理后半部分缓冲区
    CurrentSense_Update(cs, 1);
    cs->buffer_index = 1;
}
