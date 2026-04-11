#ifndef H7FOC_BSP_BSP_CURRENT_SENSE_H
#define H7FOC_BSP_BSP_CURRENT_SENSE_H

/*
 * bsp_current_sense.h — 三相电流采样 BSP 驱动
 * 已移除 CMSIS-DSP 依赖，使用自有 filter_so_t 做 biquad 滤波。
 */

#include "adc.h"
#include "main.h"
#include "filter_so.h"

#define BSP_CURRENT_SENSE_GAIN                  16.5f
#define BSP_CURRENT_SENSE_OFFSET                1.65f
#define BSP_CURRENT_SENSE_SENSITIVITY           0.165f
#define BSP_CURRENT_SENSE_ADC_RESOLUTION        4095.0f
#define BSP_CURRENT_SENSE_ADC_VREF              3.3f
#define BSP_CURRENT_SENSE_BUFFER_SIZE           3U
#define BSP_CURRENT_SENSE_FILTER_ENABLED        1
#define BSP_CURRENT_SENSE_ZERO_SUM_COMPENSATION 1
#define BSP_CURRENT_SENSE_PHASE_A_INDEX         0U
#define BSP_CURRENT_SENSE_PHASE_B_INDEX         1U
#define BSP_CURRENT_SENSE_PHASE_C_INDEX         2U
#define BSP_CURRENT_SENSE_PHASE_A_SIGN          1.0f
#define BSP_CURRENT_SENSE_PHASE_B_SIGN          1.0f
#define BSP_CURRENT_SENSE_PHASE_C_SIGN          1.0f

typedef struct
{
    uint16_t adc_buffer[BSP_CURRENT_SENSE_BUFFER_SIZE * 2U];

    float Ia;
    float Ib;
    float Ic;

    float Ia_filtered;
    float Ib_filtered;
    float Ic_filtered;

    uint16_t offset_a;
    uint16_t offset_b;
    uint16_t offset_c;

    uint8_t data_ready;
    uint8_t buffer_index;

    ADC_HandleTypeDef *hadc;

    /* 自有二阶 IIR 滤波器，替代 CMSIS-DSP biquad */
    filter_so_t filter_a;
    filter_so_t filter_b;
    filter_so_t filter_c;
} bsp_current_sense_t;

void bsp_current_sense_init(bsp_current_sense_t *cs, ADC_HandleTypeDef *hadc);
void bsp_current_sense_start(bsp_current_sense_t *cs);
void bsp_current_sense_stop(bsp_current_sense_t *cs);
void bsp_current_sense_calibrate(bsp_current_sense_t *cs, uint32_t sample_count);
void bsp_current_sense_update_dma(bsp_current_sense_t *cs, uint8_t buffer_half);
void bsp_current_sense_update_injected(bsp_current_sense_t *cs,
                                       uint16_t sample_ch15,
                                       uint16_t sample_ch3,
                                       uint16_t sample_ch8);
float bsp_current_sense_adc_to_current(uint16_t adc_value, uint16_t offset);
void bsp_current_sense_get_currents(bsp_current_sense_t *cs, float *ia, float *ib, float *ic);
void bsp_current_sense_dma_half_cplt_callback(bsp_current_sense_t *cs);
void bsp_current_sense_dma_cplt_callback(bsp_current_sense_t *cs);

#endif
