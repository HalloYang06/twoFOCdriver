#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "main.h"
#include "adc.h"
#include "arm_math.h"

#define CURRENT_SENSE_GAIN          16.5f
#define CURRENT_SENSE_OFFSET        1.65f
#define CURRENT_SENSE_SENSITIVITY   0.165f

#define ADC_RESOLUTION              4095.0f
#define ADC_VREF                    3.3f

/* We keep samples in physical channel order: [0]=CH15, [1]=CH3, [2]=CH8. */
#define CURRENT_BUFFER_SIZE         3

/* Disable filtering while bringing up injected current sampling. */
#define CURRENT_FILTER_ENABLED      1
#define CURRENT_ZERO_SUM_COMPENSATION 1

#define CURRENT_PHASE_A_INDEX       0
#define CURRENT_PHASE_B_INDEX       1
#define CURRENT_PHASE_C_INDEX       2
#define CURRENT_PHASE_A_SIGN        1.0f
#define CURRENT_PHASE_B_SIGN       	1.0f
#define CURRENT_PHASE_C_SIGN        1.0f

#define BIQUAD_NUM_STAGES           1

typedef struct {
    uint16_t adc_buffer[CURRENT_BUFFER_SIZE * 2];

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

    arm_biquad_casd_df1_inst_f32 filter_a;
    arm_biquad_casd_df1_inst_f32 filter_b;
    arm_biquad_casd_df1_inst_f32 filter_c;

    float filter_state_a[4 * BIQUAD_NUM_STAGES];
    float filter_state_b[4 * BIQUAD_NUM_STAGES];
    float filter_state_c[4 * BIQUAD_NUM_STAGES];
} CurrentSense_TypeDef;

void CurrentSense_Init(CurrentSense_TypeDef *cs, ADC_HandleTypeDef *hadc);
void CurrentSense_Start(CurrentSense_TypeDef *cs);
void CurrentSense_Stop(CurrentSense_TypeDef *cs);
void CurrentSense_Calibrate(CurrentSense_TypeDef *cs, uint32_t sample_count);
void CurrentSense_Update(CurrentSense_TypeDef *cs, uint8_t buffer_half);
void CurrentSense_UpdateInjectedValues(CurrentSense_TypeDef *cs,
                                       uint16_t sample_ch15,
                                       uint16_t sample_ch3,
                                       uint16_t sample_ch8);
float CurrentSense_ADCToCurrent(uint16_t adc_value, uint16_t offset);
void CurrentSense_GetCurrents(CurrentSense_TypeDef *cs, float *Ia, float *Ib, float *Ic);
void CurrentSense_DMA_HalfCpltCallback(CurrentSense_TypeDef *cs);
void CurrentSense_DMA_CpltCallback(CurrentSense_TypeDef *cs);

#endif
