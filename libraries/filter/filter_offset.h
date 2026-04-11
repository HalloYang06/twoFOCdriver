#ifndef H7FOC_LIBRARIES_FILTER_FILTER_OFFSET_H
#define H7FOC_LIBRARIES_FILTER_FILTER_OFFSET_H

/*
 * filter_offset.h — 偏置估计滤波器
 * 参照 TI C2000 MCSDK OFFSET 实现。
 *
 * 两种模式：
 * 1. 在线模式：一阶低通持续跟踪直流偏置（运行时动态补偿漂移）
 *    offset[n] = offset[n-1] + alpha * (input - offset[n-1])
 * 2. 校准模式：累加 N 个样本取平均（兼容旧接口）
 *
 * run() 内联在头文件中。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float offset;        /* 当前偏置估计值 */
    float alpha;         /* 一阶低通系数 (0~1)，越小越平滑 */
    uint32_t cal_count;  /* 校准模式：已累加样本数 */
    float cal_sum;       /* 校准模式：累加和 */
} filter_offset_t;

/* 初始化（设置低通系数，清零状态） */
void filter_offset_init(filter_offset_t *f, float alpha);

/* 清零状态（保留 alpha） */
void filter_offset_reset(filter_offset_t *f);

/* 设置低通系数 */
void filter_offset_set_alpha(filter_offset_t *f, float alpha);

/*
 * 在线偏置跟踪 — 内联。
 * 每次采样调用，持续估计直流偏置。
 * 返回当前偏置估计值。
 */
static inline float filter_offset_run(filter_offset_t *f, float input)
{
    float alpha  = f->alpha;
    float offset = f->offset;

    offset = offset + alpha * (input - offset);
    f->offset = offset;

    return offset;
}

/* ── 校准模式接口（一次性累加取平均） ──────────────── */

/* 开始校准（清零累加器） */
void filter_offset_cal_start(filter_offset_t *f);

/* 推入一个校准样本 */
void filter_offset_cal_push(filter_offset_t *f, float sample);

/* 推入一个 uint16 校准样本（兼容 ADC 原始值） */
void filter_offset_cal_push_u16(filter_offset_t *f, uint16_t sample);

/* 结束校准，将平均值写入 offset */
void filter_offset_cal_finish(filter_offset_t *f);

/* 获取当前偏置值 */
static inline float filter_offset_get(const filter_offset_t *f)
{
    return f->offset;
}

/* 向后兼容旧接口 */
static inline void filter_offset_push_u16(filter_offset_t *f, uint16_t sample)
{
    filter_offset_cal_push_u16(f, sample);
    filter_offset_cal_finish(f);
}

#ifdef __cplusplus
}
#endif

#endif
