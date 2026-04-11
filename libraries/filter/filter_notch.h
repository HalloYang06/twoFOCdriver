#ifndef H7FOC_LIBRARIES_FILTER_FILTER_NOTCH_H
#define H7FOC_LIBRARIES_FILTER_FILTER_NOTCH_H

/*
 * filter_notch.h — 陷波滤波器
 * 参照 TI C2000 MCSDK FILTER_NOTCH 实现。
 * 系数和状态分离：多通道可共享同一组系数。
 *
 * 传递函数: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 - a1*z^-1 - a2*z^-2)
 * 注意：TI 的陷波滤波器 a1/a2 符号约定与通用 biquad 相反，
 *       差分方程: out = b0*in + b1*in1 + b2*in2 + a1*out1 + a2*out2
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 系数结构体（可多通道共享） */
typedef struct
{
    float a1;   /* 反馈系数 (注意符号：直接相加) */
    float a2;
    float b0;   /* 前馈系数 */
    float b1;
    float b2;
} filter_notch_coeff_t;

/* 状态结构体（每通道独立） */
typedef struct
{
    float in1;   /* x[n-1] */
    float in2;   /* x[n-2] */
    float out1;  /* y[n-1] */
    float out2;  /* y[n-2] */
} filter_notch_t;

/* 初始化状态（清零） */
void filter_notch_init(filter_notch_t *f);

/* 初始化系数（设为直通） */
void filter_notch_coeff_init(filter_notch_coeff_t *c);

/*
 * 根据陷波频率、带宽和采样频率计算系数。
 * fn: 陷波频率 (Hz)
 * bw: 带宽 (Hz)，即 -3dB 点之间的宽度
 * fs: 采样频率 (Hz)
 */
void filter_notch_set_params(filter_notch_coeff_t *c, float fn, float bw, float fs);

/* 直接设置系数 */
void filter_notch_set_coeffs(filter_notch_coeff_t *c,
                             float a1, float a2,
                             float b0, float b1, float b2);

/*
 * 陷波滤波器执行 — 内联。
 * 系数和状态分离，局部变量缓存优化。
 */
static inline float filter_notch_run(filter_notch_t *f,
                                     const filter_notch_coeff_t *c,
                                     float input)
{
    float b0 = c->b0;
    float b1 = c->b1;
    float b2 = c->b2;
    float a1 = c->a1;
    float a2 = c->a2;

    float output = b0 * input + b1 * f->in1 + b2 * f->in2
                 + a1 * f->out1 + a2 * f->out2;

    f->in2  = f->in1;
    f->in1  = input;
    f->out2 = f->out1;
    f->out1 = output;

    return output;
}

#ifdef __cplusplus
}
#endif

#endif
