/*
 * filter_fo.c — 一阶 IIR 滤波器的初始化和系数计算。
 * 运行函数 filter_fo_run() 在 filter_fo.h 中内联。
 */
#include "filter_fo.h"
#include "mc_math.h"

void filter_fo_init(filter_fo_t *f)
{
    f->a1 = 0.0f;
    f->b0 = 1.0f;
    f->b1 = 0.0f;
    f->x1 = 0.0f;
    f->y1 = 0.0f;
}

void filter_fo_set_coeffs(filter_fo_t *f, float a1, float b0, float b1)
{
    f->a1 = a1;
    f->b0 = b0;
    f->b1 = b1;
}

/*
 * 一阶低通滤波器系数计算（双线性变换）。
 * fc: 截止频率 (Hz)
 * fs: 采样频率 (Hz)
 *
 * 连续域: H(s) = wc / (s + wc)
 * 双线性变换 s = 2*fs*(1-z^-1)/(1+z^-1) 后:
 *   b0 = wc*T / (2 + wc*T)
 *   b1 = b0
 *   a1 = -(2 - wc*T) / (2 + wc*T)   (注意差分方程里 a1 带负号)
 */
void filter_fo_set_lowpass(filter_fo_t *f, float fc, float fs)
{
    float wc_t = MC_TWO_PI * fc / fs;  /* wc * T */
    float denom = 2.0f + wc_t;

    f->b0 = wc_t / denom;
    f->b1 = f->b0;
    f->a1 = (wc_t - 2.0f) / denom;    /* = -(2-wc_t)/denom */
}

void filter_fo_reset(filter_fo_t *f)
{
    f->x1 = 0.0f;
    f->y1 = 0.0f;
}
