//
// Created by 27352 on 26-1-15.
//

#include "encoder.h"

/**
 * @brief  初始化编码器
 * @param  encoder: 编码器结构体指针
 * @param  htim: 定时器句柄
 * @retval None
 */
void Encoder_Init(Encoder_TypeDef *encoder, TIM_HandleTypeDef *htim)
{
    encoder->htim = htim;
    encoder->totalCount = 0;
    encoder->lastCount = 0;
    encoder->deltaCount = 0;
    encoder->speed_rpm = 0.0f;
    encoder->speed_rps = 0.0f;
    encoder->zPulseCount = 0;
    encoder->direction = 0;
}

/**
 * @brief  启动编码器
 * @param  encoder: 编码器结构体指针
 * @retval None
 */
void Encoder_Start(Encoder_TypeDef *encoder)
{
    HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);

    // 设置初始计数值为0
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    encoder->lastCount = 0;
}

/**
 * @brief  停止编码器
 * @param  encoder: 编码器结构体指针
 * @retval None
 */
void Encoder_Stop(Encoder_TypeDef *encoder)
{
    HAL_TIM_Encoder_Stop(encoder->htim, TIM_CHANNEL_ALL);
}

/**
 * @brief  获取编码器当前计数值
 * @param  encoder: 编码器结构体指针
 * @retval 当前计数值
 */
int16_t Encoder_GetCount(Encoder_TypeDef *encoder)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(encoder->htim);
}

/**
 * @brief  获取编码器累计计数值（考虑溢出）
 * @param  encoder: 编码器结构体指针
 * @retval 累计计数值
 */
int32_t Encoder_GetTotalCount(Encoder_TypeDef *encoder)
{
    int16_t currentCount = Encoder_GetCount(encoder);
    int16_t diff = currentCount - encoder->lastCount;

    encoder->totalCount += diff;
    encoder->lastCount = currentCount;

    return encoder->totalCount;
}

/**
 * @brief  更新编码器转速
 * @note   此函数应该在固定的时间间隔内调用（如10ms）
 * @param  encoder: 编码器结构体指针
 * @retval None
 */
void Encoder_UpdateSpeed(Encoder_TypeDef *encoder)
{
    int16_t currentCount = Encoder_GetCount(encoder);

    // 计算本次采样周期内的计数差值
    encoder->deltaCount = currentCount - encoder->lastCount;
    encoder->lastCount = currentCount;

    // 判断旋转方向
    if (encoder->deltaCount >= 0) {
        encoder->direction = 0;  // 正转
    } else {
        encoder->direction = 1;  // 反转
    }

    // 计算转速
    // deltaCount是采样周期内的计数值变化
    // 编码器模式TI12下，一个脉冲产生4个计数（四倍频）
    // 转速(RPM) = (deltaCount / 4) / PPR / 采样时间(秒) * 60
    encoder->speed_rps = (float)encoder->deltaCount / (4.0f * ENCODER_PPR * ENCODER_SAMPLE_TIME);
    encoder->speed_rpm = encoder->speed_rps * 60.0f;
}

/**
 * @brief  获取转速（RPM）
 * @param  encoder: 编码器结构体指针
 * @retval 转速（转/分钟）
 */
float Encoder_GetSpeed_RPM(Encoder_TypeDef *encoder)
{
    return encoder->speed_rpm;
}

/**
 * @brief  获取转速（RPS）
 * @param  encoder: 编码器结构体指针
 * @retval 转速（转/秒）
 */
float Encoder_GetSpeed_RPS(Encoder_TypeDef *encoder)
{
    return encoder->speed_rps;
}

/**
 * @brief  复位编码器
 * @param  encoder: 编码器结构体指针
 * @retval None
 */
void Encoder_Reset(Encoder_TypeDef *encoder)
{
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    encoder->totalCount = 0;
    encoder->lastCount = 0;
    encoder->deltaCount = 0;
    encoder->speed_rpm = 0.0f;
    encoder->speed_rps = 0.0f;
    encoder->zPulseCount = 0;
}

/**
 * @brief  Z相脉冲中断回调函数
 * @note   在外部中断回调中调用此函数
 * @param  encoder: 编码器结构体指针
 * @retval None
 */
void Encoder_ZPulse_Callback(Encoder_TypeDef *encoder)
{
    encoder->zPulseCount++;

    // 可选：在Z相脉冲时校准编码器位置
    // __HAL_TIM_SET_COUNTER(encoder->htim, 0);
}

/**
 * @brief  获取圈数（通过Z相脉冲计数）
 * @param  encoder: 编码器结构体指针
 * @retval 圈数
 */
uint32_t Encoder_GetRevolutions(Encoder_TypeDef *encoder)
{
    return encoder->zPulseCount;
}