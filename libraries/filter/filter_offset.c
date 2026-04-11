/*
 * filter_offset.c — 偏置估计滤波器的初始化和校准模式实现。
 * 在线跟踪函数 filter_offset_run() 在 filter_offset.h 中内联。
 */
#include "filter_offset.h"

void filter_offset_init(filter_offset_t *f, float alpha)
{
    f->offset    = 0.0f;
    f->alpha     = alpha;
    f->cal_count = 0U;
    f->cal_sum   = 0.0f;
}

void filter_offset_reset(filter_offset_t *f)
{
    f->offset    = 0.0f;
    f->cal_count = 0U;
    f->cal_sum   = 0.0f;
}

void filter_offset_set_alpha(filter_offset_t *f, float alpha)
{
    f->alpha = alpha;
}

void filter_offset_cal_start(filter_offset_t *f)
{
    f->cal_count = 0U;
    f->cal_sum   = 0.0f;
}

void filter_offset_cal_push(filter_offset_t *f, float sample)
{
    f->cal_sum += sample;
    f->cal_count++;
}

void filter_offset_cal_push_u16(filter_offset_t *f, uint16_t sample)
{
    f->cal_sum += (float)sample;
    f->cal_count++;
}

void filter_offset_cal_finish(filter_offset_t *f)
{
    if (f->cal_count > 0U)
    {
        f->offset = f->cal_sum / (float)f->cal_count;
    }
}
