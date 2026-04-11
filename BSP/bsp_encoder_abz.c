#include "bsp_encoder_abz.h"
#include "bsp_encoder.h"
#include "mc_math.h"

void Encoder_Init(Encoder_TypeDef *encoder, TIM_HandleTypeDef *htim)
{
    encoder->htim = htim;
    encoder->totalCount = 0;
    encoder->lastCount = 0;
    encoder->deltaCount = 0;
    encoder->speed_rpm = 0.0f;
    encoder->speed_rps = 0.0f;
    encoder->zPulseCount = 0U;
    encoder->direction = 0U;
    encoder->pole_pairs = 1U;
    encoder->zero_electric_offset = 0.0f;
}

void Encoder_Start(Encoder_TypeDef *encoder)
{
    HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(encoder->htim, 0U);
    encoder->lastCount = 0;
}

void Encoder_Stop(Encoder_TypeDef *encoder)
{
    HAL_TIM_Encoder_Stop(encoder->htim, TIM_CHANNEL_ALL);
}

int16_t Encoder_GetCount(Encoder_TypeDef *encoder)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(encoder->htim);
}

int32_t Encoder_GetTotalCount(Encoder_TypeDef *encoder)
{
    int16_t currentCount = Encoder_GetCount(encoder);
    int16_t diff = currentCount - encoder->lastCount;

    encoder->totalCount += diff;
    encoder->lastCount = currentCount;
    return encoder->totalCount;
}

void Encoder_UpdateTotalCount(Encoder_TypeDef *encoder)
{
    int16_t currentCount = Encoder_GetCount(encoder);
    int16_t diff = currentCount - encoder->lastCount;

    encoder->totalCount += diff;
    encoder->lastCount = currentCount;
}

void Encoder_UpdateSpeed(Encoder_TypeDef *encoder)
{
    int16_t currentCount = Encoder_GetCount(encoder);

    encoder->deltaCount = currentCount - encoder->lastCount;
    encoder->totalCount += encoder->deltaCount;
    encoder->lastCount = currentCount;
    encoder->direction = (encoder->deltaCount < 0) ? 1U : 0U;
    encoder->speed_rps = (float)encoder->deltaCount / (4.0f * (float)ENCODER_PPR * ENCODER_SAMPLE_TIME);
    encoder->speed_rpm = encoder->speed_rps * 60.0f;
}

float Encoder_GetSpeed_RPM(Encoder_TypeDef *encoder)
{
    return encoder->speed_rpm;
}

float Encoder_GetSpeed_RPS(Encoder_TypeDef *encoder)
{
    return encoder->speed_rps;
}

void Encoder_Reset(Encoder_TypeDef *encoder)
{
    __HAL_TIM_SET_COUNTER(encoder->htim, 0U);
    encoder->totalCount = 0;
    encoder->lastCount = 0;
    encoder->deltaCount = 0;
    encoder->speed_rpm = 0.0f;
    encoder->speed_rps = 0.0f;
    encoder->zPulseCount = 0U;
}

void Encoder_ZPulse_Callback(Encoder_TypeDef *encoder)
{
    encoder->zPulseCount++;
}

uint32_t Encoder_GetRevolutions(Encoder_TypeDef *encoder)
{
    return encoder->zPulseCount;
}

void Encoder_SetPolePairs(Encoder_TypeDef *encoder, uint8_t pole_pairs)
{
    encoder->pole_pairs = pole_pairs;
}

float Encoder_GetAngle_Mech_Deg(Encoder_TypeDef *encoder)
{
    float angle_deg = ((float)encoder->totalCount / (4.0f * (float)ENCODER_PPR)) * 360.0f;

    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

float Encoder_GetAngle_Mech_Rad(Encoder_TypeDef *encoder)
{
    return Encoder_GetAngle_Mech_Deg(encoder) * MC_DEG_TO_RAD;
}

float Encoder_GetAngle_Elec_Deg(Encoder_TypeDef *encoder, uint8_t pole_pairs)
{
    float elec_angle_deg = Encoder_GetAngle_Mech_Deg(encoder) * (float)pole_pairs;

    while (elec_angle_deg >= 360.0f)
    {
        elec_angle_deg -= 360.0f;
    }
    while (elec_angle_deg < 0.0f)
    {
        elec_angle_deg += 360.0f;
    }

    return elec_angle_deg;
}

float Encoder_GetAngle_Elec_Rad(Encoder_TypeDef *encoder, uint8_t pole_pairs)
{
    float elec_angle_rad = Encoder_GetAngle_Elec_Deg(encoder, pole_pairs) * MC_DEG_TO_RAD;

    elec_angle_rad -= encoder->zero_electric_offset;
    return mc_normalize_angle(elec_angle_rad);
}

void Encoder_SetElectricZeroOffset(Encoder_TypeDef *encoder, float offset_rad)
{
    encoder->zero_electric_offset = mc_normalize_angle(offset_rad);
}

float Encoder_GetElectricZeroOffset(Encoder_TypeDef *encoder)
{
    return encoder->zero_electric_offset;
}

void Encoder_AlignElectricZero(Encoder_TypeDef *encoder, uint8_t pole_pairs)
{
    Encoder_SetElectricZeroOffset(encoder,
                                  Encoder_GetAngle_Elec_Deg(encoder, pole_pairs) * MC_DEG_TO_RAD);
}


static int bsp_encoder_abz_start_impl(void *ctx)
{
    Encoder_Start((Encoder_TypeDef *)ctx);
    return 0;
}

static int bsp_encoder_abz_stop_impl(void *ctx)
{
    Encoder_Stop((Encoder_TypeDef *)ctx);
    return 0;
}

static int bsp_encoder_abz_update_impl(void *ctx, float dt)
{
    (void)dt;
    Encoder_UpdateSpeed((Encoder_TypeDef *)ctx);
    return 0;
}

static float bsp_encoder_abz_get_angle_mech_impl(void *ctx)
{
    return Encoder_GetAngle_Mech_Rad((Encoder_TypeDef *)ctx);
}

static float bsp_encoder_abz_get_angle_elec_impl(void *ctx)
{
    Encoder_TypeDef *encoder = (Encoder_TypeDef *)ctx;
    return Encoder_GetAngle_Elec_Rad(encoder, encoder->pole_pairs);
}

static float bsp_encoder_abz_get_speed_impl(void *ctx)
{
    return Encoder_GetSpeed_RPS((Encoder_TypeDef *)ctx) * MC_TWO_PI;
}

static int bsp_encoder_abz_align_impl(void *ctx)
{
    Encoder_TypeDef *encoder = (Encoder_TypeDef *)ctx;
    Encoder_AlignElectricZero(encoder, encoder->pole_pairs);
    return 0;
}

const encoder_if_t g_bsp_encoder_abz_ops = {
    .init = 0,
    .start = bsp_encoder_abz_start_impl,
    .stop = bsp_encoder_abz_stop_impl,
    .update = bsp_encoder_abz_update_impl,
    .get_angle_mech_rad = bsp_encoder_abz_get_angle_mech_impl,
    .get_angle_elec_rad = bsp_encoder_abz_get_angle_elec_impl,
    .get_speed_rad_s = bsp_encoder_abz_get_speed_impl,
    .align_electric_zero = bsp_encoder_abz_align_impl,
    .is_ready = 0,
    .get_fault = 0,
};
