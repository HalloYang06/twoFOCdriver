#ifndef H7FOC_LIBRARIES_FILTER_FILTER_FO_H
#define H7FOC_LIBRARIES_FILTER_FILTER_FO_H

/*
 * filter_fo.h — 一阶 IIR 滤波器
 * 参照 TI C2000 MCSDK FILTER_FO 实现。
 * 传递函数: H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
 * 差分方程: y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
 *
 * run() 内联在头文件中，init/系数设置在 filter_fo.c 中。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float a1;   /* 分母系数 z^(-1) */
    float b0;   /* 分子系数 z^(0)  */
    float b1;   /* 分子系数 z^(-1) */
    float x1;   /* 上一次输入 x[n-1] */
    float y1;   /* 上一次输出 y[n-1] */
} filter_fo_t;

/* 初始化（清零所有状态和系数） */
void filter_fo_init(filter_fo_t *f);

/* 直接设置系数 */
void filter_fo_set_coeffs(filter_fo_t *f, float a1, float b0, float b1);

/*
 * 根据截止频率和采样频率自动计算低通系数。
 * 使用双线性变换: fc = 截止频率(Hz), fs = 采样频率(Hz)
 */
void filter_fo_set_lowpass(filter_fo_t *f, float fc, float fs);

/* 清零滤波器状态（保留系数） */
void filter_fo_reset(filter_fo_t *f);

/*
 * 滤波器执行 — 内联，20kHz 中断中零调用开销。
 * 局部变量缓存系数，帮助编译器寄存器分配优化。
 */
static inline float filter_fo_run(filter_fo_t *f, float input)
{
    float a1 = f->a1;
    float b0 = f->b0;
    float b1 = f->b1;
    float x1 = f->x1;
    float y1 = f->y1;

    float output = b0 * input + b1 * x1 - a1 * y1;

    f->x1 = input;
    f->y1 = output;

    return output;
}

/* 向后兼容旧接口 */
static inline void filter_fo_set(filter_fo_t *f, float a1, float b0)
{
    filter_fo_set_coeffs(f, a1, b0, 0.0f);
}

#ifdef __cplusplus
}
#endif

#endif
