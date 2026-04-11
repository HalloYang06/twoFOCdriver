/*
 * filter_notch.c — 陷波滤波器的初始化和系数计算。
 * 运行函数 filter_notch_run() 在 filter_notch.h 中内联。
 */
#include "filter_notch.h"
#include "mc_math.h"
#include <math.h>  /* cosf/sinf 仅在初始化时用一次 */

void filter_notch_init(filter_notch_t *f)
{
    f->in1  = 0.0f;
    f->in2  = 0.0f;
    f->out1 = 0.0f;
    f->out2 = 0.0f;
}

void filter_notch_coeff_init(filter_notch_coeff_t *c)
{
    /* 直通：输出 = 输入 */
    c->a1 = 0.0f;
    c->a2 = 0.0f;
    c->b0 = 1.0f;
    c->b1 = 0.0f;
    c->b2 = 0.0f;
}

/*
 * 陷波滤波器系数计算。
 * fn: 陷波频率 (Hz)
 * bw: -3dB 带宽 (Hz)
 * fs: 采样频率 (Hz)
 *
 * 设计方法：
 *   w0 = 2*PI*fn/fs
 *   alpha = sin(w0) * sinh(ln(2)/2 * bw/fn * w0/sin(w0))
 *         简化为 alpha = sin(w0) * (bw*PI)/(fn*fs) 近似
 *   更精确: alpha = tan(PI*bw/fs)
 *
 *   b0 =  1
 *   b1 = -2*cos(w0)
 *   b2 =  1
 *   a0 =  1 + alpha
 *   a1 =  2*cos(w0)     (注意 TI 符号约定：直接相加)
 *   a2 = -(1 - alpha)
 *
 *   归一化: 所有系数除以 a0
 */
void filter_notch_set_params(filter_notch_coeff_t *c, float fn, float bw, float fs)
{
    float w0 = MC_TWO_PI * fn / fs;
    float cos_w0 = cosf(w0);
    float alpha = tanf(MC_PI * bw / fs);
    float inv_a0 = 1.0f / (1.0f + alpha);

    c->b0 =  1.0f * inv_a0;
    c->b1 = -2.0f * cos_w0 * inv_a0;
    c->b2 =  1.0f * inv_a0;
    c->a1 =  2.0f * cos_w0 * inv_a0;       /* 反馈，直接相加 */
    c->a2 = -(1.0f - alpha) * inv_a0;
}

void filter_notch_set_coeffs(filter_notch_coeff_t *c,
                             float a1, float a2,
                             float b0, float b1, float b2)
{
    c->a1 = a1;
    c->a2 = a2;
    c->b0 = b0;
    c->b1 = b1;
    c->b2 = b2;
}
