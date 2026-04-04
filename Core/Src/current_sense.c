#include "current_sense.h"
#include <string.h>

static const float biquad_coeffs[5] = {
    0.06745527f,
    0.13491055f,
    0.06745527f,
    1.14298050f,
   -0.41280159f
};

static void CurrentSense_ResetFilters(CurrentSense_TypeDef *cs)
{
#if CURRENT_FILTER_ENABLED
    memset(cs->filter_state_a, 0, sizeof(cs->filter_state_a));
    memset(cs->filter_state_b, 0, sizeof(cs->filter_state_b));
    memset(cs->filter_state_c, 0, sizeof(cs->filter_state_c));
#else
    (void)cs;
#endif
}

static void CurrentSense_ProcessSamples(CurrentSense_TypeDef *cs, const uint16_t *samples)
{
    float Ia_raw = CURRENT_PHASE_A_SIGN *
                   CurrentSense_ADCToCurrent(samples[CURRENT_PHASE_A_INDEX], cs->offset_a);
    float Ib_raw = CURRENT_PHASE_B_SIGN *
                   CurrentSense_ADCToCurrent(samples[CURRENT_PHASE_B_INDEX], cs->offset_b);
    float Ic_raw = CURRENT_PHASE_C_SIGN *
                   CurrentSense_ADCToCurrent(samples[CURRENT_PHASE_C_INDEX], cs->offset_c);

    cs->Ia = Ia_raw;
    cs->Ib = Ib_raw;
    cs->Ic = Ic_raw;

#if CURRENT_FILTER_ENABLED
    float in;
    float out;

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
    cs->Ia_filtered = Ia_raw;
    cs->Ib_filtered = Ib_raw;
    cs->Ic_filtered = Ic_raw;
#endif

#if CURRENT_ZERO_SUM_COMPENSATION
    {
        float i_avg = (cs->Ia_filtered + cs->Ib_filtered + cs->Ic_filtered) * 0.33333334f;
        cs->Ia_filtered -= i_avg;
        cs->Ib_filtered -= i_avg;
        cs->Ic_filtered -= i_avg;
    }
#endif

    cs->data_ready = 1;
}

void CurrentSense_Init(CurrentSense_TypeDef *cs, ADC_HandleTypeDef *hadc)
{
    memset(cs->adc_buffer, 0, sizeof(cs->adc_buffer));

    cs->hadc = hadc;
    cs->Ia = 0.0f;
    cs->Ib = 0.0f;
    cs->Ic = 0.0f;
    cs->Ia_filtered = 0.0f;
    cs->Ib_filtered = 0.0f;
    cs->Ic_filtered = 0.0f;

    cs->offset_a = (uint16_t)(CURRENT_SENSE_OFFSET / ADC_VREF * ADC_RESOLUTION);
    cs->offset_b = (uint16_t)(CURRENT_SENSE_OFFSET / ADC_VREF * ADC_RESOLUTION);
    cs->offset_c = (uint16_t)(CURRENT_SENSE_OFFSET / ADC_VREF * ADC_RESOLUTION);

    cs->data_ready = 0;
    cs->buffer_index = 0;

#if CURRENT_FILTER_ENABLED
    CurrentSense_ResetFilters(cs);
    arm_biquad_cascade_df1_init_f32(&cs->filter_a, BIQUAD_NUM_STAGES,
                                    (float *)biquad_coeffs, cs->filter_state_a);
    arm_biquad_cascade_df1_init_f32(&cs->filter_b, BIQUAD_NUM_STAGES,
                                    (float *)biquad_coeffs, cs->filter_state_b);
    arm_biquad_cascade_df1_init_f32(&cs->filter_c, BIQUAD_NUM_STAGES,
                                    (float *)biquad_coeffs, cs->filter_state_c);
#endif
}

void CurrentSense_Start(CurrentSense_TypeDef *cs)
{
    HAL_ADCEx_InjectedStart_IT(cs->hadc);
}

void CurrentSense_Stop(CurrentSense_TypeDef *cs)
{
    HAL_ADCEx_InjectedStop_IT(cs->hadc);
}

void CurrentSense_Calibrate(CurrentSense_TypeDef *cs, uint32_t sample_count)
{
    uint32_t sum_a = 0;
    uint32_t sum_b = 0;
    uint32_t sum_c = 0;
    uint32_t valid_samples = 0;

    HAL_ADCEx_InjectedStart(cs->hadc);
    HAL_Delay(5);

    for (uint32_t i = 0; i < sample_count; i++)
    {
        if (HAL_ADCEx_InjectedPollForConversion(cs->hadc, 10) != HAL_OK)
        {
            continue;
        }

        /* Keep the same physical channel order used by the old mapping. */
        uint16_t samples[CURRENT_BUFFER_SIZE];
        samples[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(cs->hadc, ADC_INJECTED_RANK_2);
        samples[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(cs->hadc, ADC_INJECTED_RANK_1);
        samples[2] = (uint16_t)HAL_ADCEx_InjectedGetValue(cs->hadc, ADC_INJECTED_RANK_3);

        sum_a += samples[CURRENT_PHASE_A_INDEX];
        sum_b += samples[CURRENT_PHASE_B_INDEX];
        sum_c += samples[CURRENT_PHASE_C_INDEX];
        valid_samples++;
    }

    HAL_ADCEx_InjectedStop(cs->hadc);

    if (valid_samples > 0U)
    {
        cs->offset_a = (uint16_t)(sum_a / valid_samples);
        cs->offset_b = (uint16_t)(sum_b / valid_samples);
        cs->offset_c = (uint16_t)(sum_c / valid_samples);
    }

    CurrentSense_ResetFilters(cs);
}

float CurrentSense_ADCToCurrent(uint16_t adc_value, uint16_t offset)
{
    float voltage = (float)adc_value / ADC_RESOLUTION * ADC_VREF;
    float offset_voltage = (float)offset / ADC_RESOLUTION * ADC_VREF;
    return (voltage - offset_voltage) / CURRENT_SENSE_SENSITIVITY;
}

void CurrentSense_Update(CurrentSense_TypeDef *cs, uint8_t buffer_half)
{
    uint16_t *buffer_ptr;

    if (buffer_half == 0U) {
        buffer_ptr = &cs->adc_buffer[0];
    } else {
        buffer_ptr = &cs->adc_buffer[CURRENT_BUFFER_SIZE];
    }

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr((uint32_t *)buffer_ptr, CURRENT_BUFFER_SIZE * sizeof(uint16_t));
#endif

    CurrentSense_ProcessSamples(cs, buffer_ptr);
}

void CurrentSense_UpdateInjectedValues(CurrentSense_TypeDef *cs,
                                       uint16_t sample_ch15,
                                       uint16_t sample_ch3,
                                       uint16_t sample_ch8)
{
    uint16_t samples[CURRENT_BUFFER_SIZE];

    samples[0] = sample_ch15;
    samples[1] = sample_ch3;
    samples[2] = sample_ch8;

    CurrentSense_ProcessSamples(cs, samples);
}

void CurrentSense_GetCurrents(CurrentSense_TypeDef *cs, float *Ia, float *Ib, float *Ic)
{
    *Ia = cs->Ia_filtered;
    *Ib = cs->Ib_filtered;
    *Ic = cs->Ic_filtered;
    cs->data_ready = 0;
}

void CurrentSense_DMA_HalfCpltCallback(CurrentSense_TypeDef *cs)
{
    CurrentSense_Update(cs, 0);
    cs->buffer_index = 0;
}

void CurrentSense_DMA_CpltCallback(CurrentSense_TypeDef *cs)
{
    CurrentSense_Update(cs, 1);
    cs->buffer_index = 1;
}
