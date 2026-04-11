#ifndef H7FOC_BSP_BSP_ENCODER_ABZ_H
#define H7FOC_BSP_BSP_ENCODER_ABZ_H

#include "main.h"
#include "tim.h"

/* ABZ 增量编码器参数。 */
#define ENCODER_PPR                 1024U
#define ENCODER_SAMPLE_TIME         0.0001f

typedef struct
{
    TIM_HandleTypeDef *htim;
    int32_t totalCount;
    int16_t lastCount;
    int32_t deltaCount;
    float speed_rpm;
    float speed_rps;
    uint32_t zPulseCount;
    uint8_t direction;
    uint8_t pole_pairs;
    float zero_electric_offset;
} Encoder_TypeDef;

void Encoder_Init(Encoder_TypeDef *encoder, TIM_HandleTypeDef *htim);
void Encoder_Start(Encoder_TypeDef *encoder);
void Encoder_Stop(Encoder_TypeDef *encoder);
int16_t Encoder_GetCount(Encoder_TypeDef *encoder);
int32_t Encoder_GetTotalCount(Encoder_TypeDef *encoder);
void Encoder_UpdateSpeed(Encoder_TypeDef *encoder);
void Encoder_UpdateTotalCount(Encoder_TypeDef *encoder);
void Encoder_AlignElectricZero(Encoder_TypeDef *encoder, uint8_t pole_pairs);
float Encoder_GetSpeed_RPM(Encoder_TypeDef *encoder);
float Encoder_GetSpeed_RPS(Encoder_TypeDef *encoder);
void Encoder_Reset(Encoder_TypeDef *encoder);
void Encoder_ZPulse_Callback(Encoder_TypeDef *encoder);
uint32_t Encoder_GetRevolutions(Encoder_TypeDef *encoder);
float Encoder_GetAngle_Mech_Deg(Encoder_TypeDef *encoder);
float Encoder_GetAngle_Mech_Rad(Encoder_TypeDef *encoder);
float Encoder_GetAngle_Elec_Deg(Encoder_TypeDef *encoder, uint8_t pole_pairs);
float Encoder_GetAngle_Elec_Rad(Encoder_TypeDef *encoder, uint8_t pole_pairs);
void Encoder_SetPolePairs(Encoder_TypeDef *encoder, uint8_t pole_pairs);
void Encoder_SetElectricZeroOffset(Encoder_TypeDef *encoder, float offset_rad);
float Encoder_GetElectricZeroOffset(Encoder_TypeDef *encoder);

#endif
