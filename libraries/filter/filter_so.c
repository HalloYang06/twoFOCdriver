/*
 * filter_so.c — 二阶 IIR 滤波器的初始化和系数计算。
 * 运行函数 filter_so_run() 在 filter_so.h 中内联。
 */
#include "filter_so.h"
#include "mc_math.h"
#include <math.h>  /* sinf/cosf 仅在初始化时用一次，不影响实时性能 */

void filter_so_init(filter_so_t *f)
{
    f->a1 = 0.0f;
    f->a2 = 0.0f;
    f->b0 = 1.0f;
    f->b1 = 0.0f;
    f->b2 = 0.0f;
    f->x1 = 0.0f;
    f->x2 = 0.0f;
    f->y1 = 0.0f;
    f->y2 = 0.0f;
}

void filter_so_set_coeffs(filter_so_t *f, float a1, float a2,
                          float b0, float b1, float b2)
{
    f->a1 = a1;
    f->a2 = a2;
    f->b0 = b0;
    f->b1 = b1;
    f->b2 = b2;
}

/*
 * 二阶 Butterworth 低通系数计算（双线性变换）。
 * fc: 截止频率 (Hz), fs: 采样频率 (Hz)
 *
 * 连续域 Butterworth: H(s) = 1 / (s^2 + sqrt(2)*s + 1)  (归一化)
 * 预畸变: wc = 2*fs*tan(PI*fc/fs)
 * 双线性变换后归一化得到 a1, a2, b0, b1, b2。
 */
void filter_so_set_lowpass(filter_so_t *f, float fc, float fs)
{
    float fr = fc / fs;
    float omega = tanf(MC_PI * fr);
    float omega2 = omega * omega;
    float sqrt2_omega = 1.41421356f * omega;  /* sqrt(2) * omega */
    float denom = 1.0f + sqrt2_omega + omega2;
    float inv_denom = 1.0f / denom;

    f->b0 = omega2 * inv_denom;
    f->b1 = 2.0f * f->b0;
    f->b2 = f->b0;
    f->a1 = 2.0f * (omega2 - 1.0f) * inv_denom;
    f->a2 = (1.0f - sqrt2_omega + omega2) * inv_denom;
}

void filter_so_reset(filter_so_t *f)
{
    f->x1 = 0.0f;
    f->x2 = 0.0f;
    f->y1 = 0.0f;
    f->y2 = 0.0f;
}
