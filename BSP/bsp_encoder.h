#ifndef H7FOC_BSP_BSP_ENCODER_H
#define H7FOC_BSP_BSP_ENCODER_H

/*
 * bsp_encoder.h — 编码器统一接口定义
 * 只定义接口（函数指针表），不 include 具体编码器实现的头文件。
 * 具体编码器的头文件由使用方按需 include。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ENCODER_KIND_ABZ = 0,
    ENCODER_KIND_TAMAGAWA = 1,
} encoder_kind_t;

/* 编码器虚函数表 — 所有编码器类型实现此接口 */
typedef struct
{
    int (*init)(void *ctx);
    int (*start)(void *ctx);
    int (*stop)(void *ctx);
    int (*update)(void *ctx, float dt);
    float (*get_angle_mech_rad)(void *ctx);
    float (*get_angle_elec_rad)(void *ctx);
    float (*get_speed_rad_s)(void *ctx);
    int (*align_electric_zero)(void *ctx);
    int (*is_ready)(void *ctx);
    uint32_t (*get_fault)(void *ctx);
} encoder_if_t;

/* 各编码器实现的 ops 表（定义在各自的 .c 文件中） */
extern const encoder_if_t g_bsp_encoder_abz_ops;
extern const encoder_if_t g_bsp_encoder_tamagawa_ops;

#ifdef __cplusplus
}
#endif

#endif
