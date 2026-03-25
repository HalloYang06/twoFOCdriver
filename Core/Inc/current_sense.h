


#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "main.h"
#include "adc.h"
#include "arm_math.h"

// 电流传感器参�?
#define CURRENT_SENSE_GAIN          16.5f       // 传感器增�?(mV/A)
#define CURRENT_SENSE_OFFSET        1.65f       // 零点偏移电压 (V)
#define CURRENT_SENSE_SENSITIVITY   0.165f      // 灵敏�?(V/A) = GAIN * 0.01

// ADC参数
#define ADC_RESOLUTION              65535.0f    // 16位ADC
#define ADC_VREF                    3.3f        // 参考电�?(V)

// Dual-buffer size (one full ADC2 scan: 3 channels)
#define CURRENT_BUFFER_SIZE         3           // A, B, C phases

// Filter and compensation options
#define CURRENT_FILTER_ENABLED      1           // 1=Enable biquad low-pass filter
#define CURRENT_ZERO_SUM_COMPENSATION 0         // 0=keep true measured phase relation for debug
/* Phase-to-ADC mapping and sign.
 * ADC2 scan order is [0]=CH15(SO2), [1]=CH3(SO1), [2]=CH8(SO3).
 * Change these to quickly test phase wiring/order without editing C source.
 */
#define CURRENT_PHASE_A_INDEX       1
#define CURRENT_PHASE_B_INDEX       0
#define CURRENT_PHASE_C_INDEX       2
#define CURRENT_PHASE_A_SIGN        1.0f
#define CURRENT_PHASE_B_SIGN        -1.0f
#define CURRENT_PHASE_C_SIGN        1.0f

/* 二阶Butterworth低通滤波器参数
 * 采样频率: 20kHz, 截止频率: 2kHz
 * 使用CMSIS-DSP Biquad IIR (Direct Form I)
 * 每相一个二阶节(numStages=1), 系数5�? {b0, b1, b2, a1, a2}
 */
#define BIQUAD_NUM_STAGES   1

// 电流采样数据结构
typedef struct {
    // ADC原始值双缓冲�?
    uint16_t adc_buffer[CURRENT_BUFFER_SIZE * 2];  // DMA双缓冲区

    // 当前电流�?(A)
    float Ia;           // A相电�?
    float Ib;           // B相电�?
    float Ic;           // C相电�?

    // 滤波后的电流�?(A)
    float Ia_filtered;
    float Ib_filtered;
    float Ic_filtered;

    // 零点偏移校准�?(ADC原始�?
    uint16_t offset_a;
    uint16_t offset_b;
    uint16_t offset_c;

    // 状态标�?
    uint8_t  data_ready;        // 数据就绪标志
    uint8_t  buffer_index;      // 当前处理的缓冲区索引 (0�?)

    // ADC句柄
    ADC_HandleTypeDef *hadc;

    // 二阶IIR滤波器实�?(每相一�?
    arm_biquad_casd_df1_inst_f32 filter_a;
    arm_biquad_casd_df1_inst_f32 filter_b;
    arm_biquad_casd_df1_inst_f32 filter_c;

    // 滤波器状态缓冲区 (每个二阶节需�?个状态变�?
    float filter_state_a[4 * BIQUAD_NUM_STAGES];
    float filter_state_b[4 * BIQUAD_NUM_STAGES];
    float filter_state_c[4 * BIQUAD_NUM_STAGES];

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

