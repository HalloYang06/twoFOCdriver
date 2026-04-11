#ifndef H7FOC_LIBRARIES_FILTER_FILTER_SO_H
#define H7FOC_LIBRARIES_FILTER_FILTER_SO_H

/*
 * filter_so.h — 二阶 IIR 滤波器 (Biquad / Direct Form I)
 * 参照 TI C2000 MCSDK FILTER_SO 实现。
 * 传递函数: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 * 差分方程: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *
 * run() 内联在头文件中，init/系数设置在 filter_so.c 中。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float a1;   /* 分母系数 z^(-1) */
    float a2;   /* 分母系数 z^(-2) */
    float b0;   /* 分子系数 z^(0)  */
    float b1;   /* 分子系数 z^(-1) */
    float b2;   /* 分子系数 z^(-2) */
    float x1;   /* x[n-1] */
    float x2;   /* x[n-2] */
    float y1;   /* y[n-1] */
    float y2;   /* y[n-2] */
} filter_so_t;

/* 初始化（清零所有状态和系数） */
void filter_so_init(filter_so_t *f);

/* 直接设置系数 */
void filter_so_set_coeffs(filter_so_t *f, float a1, float a2,
                          float b0, float b1, float b2);

/* 根据截止频率和采样频率计算二阶 Butterworth 低通系数 */
void filter_so_set_lowpass(filter_so_t *f, float fc, float fs);

/* 清零滤波器状态（保留系数） */
void filter_so_reset(filter_so_t *f);

/*
 * 滤波器执行 — 内联，局部变量缓存系数优化寄存器分配。
 */
static inline float filter_so_run(filter_so_t *f, float input)
{
    float a1 = f->a1;
    float a2 = f->a2;
    float b0 = f->b0;
    float b1 = f->b1;
    float b2 = f->b2;
    float x1 = f->x1;
    float x2 = f->x2;
    float y1 = f->y1;
    float y2 = f->y2;

    float output = b0 * input + b1 * x1 + b2 * x2
                 - a1 * y1 - a2 * y2;

    f->x2 = x1;
    f->x1 = input;
    f->y2 = y1;
    f->y1 = output;

    return output;
}

/* 向后兼容旧接口 */
static inline void filter_so_set(filter_so_t *f, float a1, float a2,
                                 float b0, float b1, float b2)
{
    filter_so_set_coeffs(f, a1, a2, b0, b1, b2);
}

#ifdef __cplusplus
}
#endif

#endif
