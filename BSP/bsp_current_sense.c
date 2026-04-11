/*
 * bsp_current_sense.c — 三相电流采样驱动
 * 已移除 CMSIS-DSP 依赖，使用自有 filter_so_t 做二阶 IIR 滤波。
 */
#include "bsp_current_sense.h"

#include <string.h>

/*
 * 原 CMSIS-DSP biquad 系数 {b0, b1, b2, -a1, -a2}:
 *   {0.06745527, 0.13491055, 0.06745527, 1.14298050, -0.41280159}
 *
 * 转换为 filter_so_t 约定 (差分方程 y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2):
 *   b0 =  0.06745527
 *   b1 =  0.13491055
 *   b2 =  0.06745527
 *   a1 = -1.14298050  (CMSIS 的 -a1 = 1.14298050，所以 a1 = -1.14298050)
 *   a2 =  0.41280159  (CMSIS 的 -a2 = -0.41280159，所以 a2 = 0.41280159)
 */
#define CS_FILTER_B0  ( 0.06745527f)
#define CS_FILTER_B1  ( 0.13491055f)
#define CS_FILTER_B2  ( 0.06745527f)
#define CS_FILTER_A1  (-1.14298050f)
#define CS_FILTER_A2  ( 0.41280159f)

static void bsp_current_sense_init_filters(bsp_current_sense_t *cs)
{
#if BSP_CURRENT_SENSE_FILTER_ENABLED
    filter_so_init(&cs->filter_a);
    filter_so_init(&cs->filter_b);
    filter_so_init(&cs->filter_c);
    filter_so_set_coeffs(&cs->filter_a, CS_FILTER_A1, CS_FILTER_A2,
                         CS_FILTER_B0, CS_FILTER_B1, CS_FILTER_B2);
    filter_so_set_coeffs(&cs->filter_b, CS_FILTER_A1, CS_FILTER_A2,
                         CS_FILTER_B0, CS_FILTER_B1, CS_FILTER_B2);
    filter_so_set_coeffs(&cs->filter_c, CS_FILTER_A1, CS_FILTER_A2,
                         CS_FILTER_B0, CS_FILTER_B1, CS_FILTER_B2);
#else
    (void)cs;
#endif
}

static void bsp_current_sense_reset_filters(bsp_current_sense_t *cs)
{
#if BSP_CURRENT_SENSE_FILTER_ENABLED
    filter_so_reset(&cs->filter_a);
    filter_so_reset(&cs->filter_b);
    filter_so_reset(&cs->filter_c);
#else
    (void)cs;
#endif
}
static void bsp_current_sense_process_samples(bsp_current_sense_t *cs, const uint16_t *samples)
{
    float ia_raw = BSP_CURRENT_SENSE_PHASE_A_SIGN *
                   bsp_current_sense_adc_to_current(samples[BSP_CURRENT_SENSE_PHASE_A_INDEX], cs->offset_a);
    float ib_raw = BSP_CURRENT_SENSE_PHASE_B_SIGN *
                   bsp_current_sense_adc_to_current(samples[BSP_CURRENT_SENSE_PHASE_B_INDEX], cs->offset_b);
    float ic_raw = BSP_CURRENT_SENSE_PHASE_C_SIGN *
                   bsp_current_sense_adc_to_current(samples[BSP_CURRENT_SENSE_PHASE_C_INDEX], cs->offset_c);

    cs->Ia = ia_raw;
    cs->Ib = ib_raw;
    cs->Ic = ic_raw;

#if BSP_CURRENT_SENSE_FILTER_ENABLED
    cs->Ia_filtered = filter_so_run(&cs->filter_a, ia_raw);
    cs->Ib_filtered = filter_so_run(&cs->filter_b, ib_raw);
    cs->Ic_filtered = filter_so_run(&cs->filter_c, ic_raw);
#else
    cs->Ia_filtered = ia_raw;
    cs->Ib_filtered = ib_raw;
    cs->Ic_filtered = ic_raw;
#endif

#if BSP_CURRENT_SENSE_ZERO_SUM_COMPENSATION
    {
        float i_avg = (cs->Ia_filtered + cs->Ib_filtered + cs->Ic_filtered) * 0.33333334f;
        cs->Ia_filtered -= i_avg;
        cs->Ib_filtered -= i_avg;
        cs->Ic_filtered -= i_avg;
    }
#endif

    cs->data_ready = 1U;
}

void bsp_current_sense_init(bsp_current_sense_t *cs, ADC_HandleTypeDef *hadc)
{
    memset(cs, 0, sizeof(*cs));

    cs->hadc = hadc;
    cs->offset_a = (uint16_t)(BSP_CURRENT_SENSE_OFFSET / BSP_CURRENT_SENSE_ADC_VREF * BSP_CURRENT_SENSE_ADC_RESOLUTION);
    cs->offset_b = (uint16_t)(BSP_CURRENT_SENSE_OFFSET / BSP_CURRENT_SENSE_ADC_VREF * BSP_CURRENT_SENSE_ADC_RESOLUTION);
    cs->offset_c = (uint16_t)(BSP_CURRENT_SENSE_OFFSET / BSP_CURRENT_SENSE_ADC_VREF * BSP_CURRENT_SENSE_ADC_RESOLUTION);

    bsp_current_sense_init_filters(cs);
}

void bsp_current_sense_start(bsp_current_sense_t *cs)
{
    HAL_ADCEx_InjectedStart_IT(cs->hadc);
}

void bsp_current_sense_stop(bsp_current_sense_t *cs)
{
    HAL_ADCEx_InjectedStop_IT(cs->hadc);
}
void bsp_current_sense_calibrate(bsp_current_sense_t *cs, uint32_t sample_count)
{
    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;
    uint32_t sum_c = 0U;
    uint32_t valid_samples = 0U;

    HAL_ADCEx_InjectedStart(cs->hadc);
    HAL_Delay(5U);

    for (uint32_t i = 0U; i < sample_count; ++i)
    {
        uint16_t samples[BSP_CURRENT_SENSE_BUFFER_SIZE];

        if (HAL_ADCEx_InjectedPollForConversion(cs->hadc, 10U) != HAL_OK)
        {
            continue;
        }

        /* 保持和原工程一致的物理通道顺序：CH15、CH3、CH8。 */
        samples[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(cs->hadc, ADC_INJECTED_RANK_2);
        samples[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(cs->hadc, ADC_INJECTED_RANK_1);
        samples[2] = (uint16_t)HAL_ADCEx_InjectedGetValue(cs->hadc, ADC_INJECTED_RANK_3);

        sum_a += samples[BSP_CURRENT_SENSE_PHASE_A_INDEX];
        sum_b += samples[BSP_CURRENT_SENSE_PHASE_B_INDEX];
        sum_c += samples[BSP_CURRENT_SENSE_PHASE_C_INDEX];
        valid_samples++;
    }

    HAL_ADCEx_InjectedStop(cs->hadc);

    if (valid_samples > 0U)
    {
        cs->offset_a = (uint16_t)(sum_a / valid_samples);
        cs->offset_b = (uint16_t)(sum_b / valid_samples);
        cs->offset_c = (uint16_t)(sum_c / valid_samples);
    }

    bsp_current_sense_reset_filters(cs);
}

float bsp_current_sense_adc_to_current(uint16_t adc_value, uint16_t offset)
{
    float voltage = (float)adc_value / BSP_CURRENT_SENSE_ADC_RESOLUTION * BSP_CURRENT_SENSE_ADC_VREF;
    float offset_voltage = (float)offset / BSP_CURRENT_SENSE_ADC_RESOLUTION * BSP_CURRENT_SENSE_ADC_VREF;
    return (voltage - offset_voltage) / BSP_CURRENT_SENSE_SENSITIVITY;
}

void bsp_current_sense_update_dma(bsp_current_sense_t *cs, uint8_t buffer_half)
{
    uint16_t *buffer_ptr;

    if (buffer_half == 0U)
    {
        buffer_ptr = &cs->adc_buffer[0];
    }
    else
    {
        buffer_ptr = &cs->adc_buffer[BSP_CURRENT_SENSE_BUFFER_SIZE];
    }

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr((uint32_t *)buffer_ptr,
                                 (int32_t)(BSP_CURRENT_SENSE_BUFFER_SIZE * sizeof(uint16_t)));
#endif

    bsp_current_sense_process_samples(cs, buffer_ptr);
}

void bsp_current_sense_update_injected(bsp_current_sense_t *cs,
                                       uint16_t sample_ch15,
                                       uint16_t sample_ch3,
                                       uint16_t sample_ch8)
{
    uint16_t samples[BSP_CURRENT_SENSE_BUFFER_SIZE];

    samples[0] = sample_ch15;
    samples[1] = sample_ch3;
    samples[2] = sample_ch8;
    bsp_current_sense_process_samples(cs, samples);
}

void bsp_current_sense_get_currents(bsp_current_sense_t *cs, float *ia, float *ib, float *ic)
{
    *ia = cs->Ia_filtered;
    *ib = cs->Ib_filtered;
    *ic = cs->Ic_filtered;
    cs->data_ready = 0U;
}

void bsp_current_sense_dma_half_cplt_callback(bsp_current_sense_t *cs)
{
    bsp_current_sense_update_dma(cs, 0U);
    cs->buffer_index = 0U;
}

void bsp_current_sense_dma_cplt_callback(bsp_current_sense_t *cs)
{
    bsp_current_sense_update_dma(cs, 1U);
    cs->buffer_index = 1U;
}
