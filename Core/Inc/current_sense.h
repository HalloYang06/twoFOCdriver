


#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "main.h"
#include "adc.h"

// 电流传感器参数
#define CURRENT_SENSE_GAIN          16.5f       // 传感器增益 (mV/A)
#define CURRENT_SENSE_OFFSET        1.65f       // 零点偏移电压 (V)
#define CURRENT_SENSE_SENSITIVITY   0.165f      // 灵敏度 (V/A) = GAIN * 0.01

// ADC参数
#define ADC_RESOLUTION              65535.0f    // 16位ADC
#define ADC_VREF                    3.3f        // 参考电压 (V)

// 双缓冲区大小（每个缓冲区存储一次完整采样：3个通道）
#define CURRENT_BUFFER_SIZE         3           // A, B, C三相

// 滤波器参数
#define CURRENT_FILTER_ENABLED      1           // 1=启用滤波, 0=禁用滤波
#define CURRENT_FILTER_ALPHA        0.2f        // 一阶低通滤波系数 (0-1)，越小越平滑

// 电流采样数据结构
typedef struct {
    // ADC原始值双缓冲区
    uint16_t adc_buffer[CURRENT_BUFFER_SIZE * 2];  // DMA双缓冲区

    // 当前电流值 (A)
    float Ia;           // A相电流
    float Ib;           // B相电流
    float Ic;           // C相电流

    // 滤波后的电流值 (A)
    float Ia_filtered;
    float Ib_filtered;
    float Ic_filtered;

    // 零点偏移校准值 (ADC原始值)
    uint16_t offset_a;
    uint16_t offset_b;
    uint16_t offset_c;

    // 状态标志
    uint8_t  data_ready;        // 数据就绪标志
    uint8_t  buffer_index;      // 当前处理的缓冲区索引 (0或1)

    // ADC句柄
    ADC_HandleTypeDef *hadc;

} CurrentSense_TypeDef;

// 函数声明
void CurrentSense_Init(CurrentSense_TypeDef *cs, ADC_HandleTypeDef *hadc);
void CurrentSense_Start(CurrentSense_TypeDef *cs);
void CurrentSense_Stop(CurrentSense_TypeDef *cs);
void CurrentSense_Calibrate(CurrentSense_TypeDef *cs, uint32_t sample_count);
void CurrentSense_Update(CurrentSense_TypeDef *cs, uint8_t buffer_half);
float CurrentSense_ADCToCurrent(uint16_t adc_value, uint16_t offset);
void CurrentSense_GetCurrents(CurrentSense_TypeDef *cs, float *Ia, float *Ib, float *Ic);
void CurrentSense_DMA_HalfCpltCallback(CurrentSense_TypeDef *cs);
void CurrentSense_DMA_CpltCallback(CurrentSense_TypeDef *cs);

#endif // CURRENT_SENSE_H
