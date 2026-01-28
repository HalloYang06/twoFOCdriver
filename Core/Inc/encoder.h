//
// Created by 27352 on 26-1-15.
//

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h"

// 编码器参数定义
#define ENCODER_PPR         1024    // 编码器线数（每转脉冲数，根据实际编码器修改）
#define ENCODER_SAMPLE_TIME 0.001f   // 采样时间（秒），即多久计算一次转速

// 编码器数据结构
typedef struct {
    TIM_HandleTypeDef *htim;        // 定时器句柄
    int32_t  totalCount;            // 累计计数值（考虑溢出）
    int16_t  lastCount;             // 上次的编码器计数值
    int32_t  deltaCount;            // 本次采样周期内的计数差值
    float    speed_rpm;             // 转速（RPM）
    float    speed_rps;             // 转速（RPS，转/秒）
    uint32_t zPulseCount;           // Z相脉冲计数（圈数）
    uint8_t  direction;             // 旋转方向（0=正转，1=反转）
    uint8_t  pole_pairs;            // 电机极对数（用于计算电角度）
    float    zero_electric_offset;  // 电角度零点偏移（弧度，0~2π）
} Encoder_TypeDef;

// 函数声明
void Encoder_Init(Encoder_TypeDef *encoder, TIM_HandleTypeDef *htim);
void Encoder_Start(Encoder_TypeDef *encoder);
void Encoder_Stop(Encoder_TypeDef *encoder);
int16_t Encoder_GetCount(Encoder_TypeDef *encoder);
int32_t Encoder_GetTotalCount(Encoder_TypeDef *encoder);
void Encoder_UpdateSpeed(Encoder_TypeDef *encoder);
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

// 电角度零点偏移相关函数
void Encoder_SetElectricZeroOffset(Encoder_TypeDef *encoder, float offset_rad);
float Encoder_GetElectricZeroOffset(Encoder_TypeDef *encoder);
void Encoder_AlignElectricZero(Encoder_TypeDef *encoder, uint8_t pole_pairs);
void Encoder_ForceAlignElectricZero(Encoder_TypeDef *encoder, uint8_t pole_pairs, float align_voltage);

#endif //ENCODER_H

