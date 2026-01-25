#ifndef FOC_H
#define FOC_H

#include "main.h"
#include "math.h"
#include "encoder.h"
#include "current_sense.h"
#include "pwm_driver.h"

/* 数学常数 */
#define PI              3.14159265358979f
#define TWO_PI          (2.0f * PI)
#define SQRT3           1.732050808f
#define SQRT3_BY_2      0.866025404f
#define ONE_BY_SQRT3    0.577350269f

/* FOC控制参数 */
#define FOC_PWM_PERIOD          2999        // PWM周期值
#define FOC_VOLTAGE_LIMIT       24.0f       // 电源电压限制 (V)
#define FOC_CURRENT_LIMIT       10.0f       // 电流限制 (A)
#define FOC_VELOCITY_LIMIT      1000.0f     // 速度限制 (rad/s)

/* PID控制器结构体 */
typedef struct {
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数

    float target;         // 目标值
    float output;           // 输出值
    float output_limit;     // 输出限制

    float integral;         // 积分累积
    float prev_error;       // 上次误差
    float integral_limit;   // 积分限制（抗饱和）

    float dt;               // 采样周期 (s)
} PID_TypeDef;

/* 三相电流结构体 */
typedef struct {
    float Ia;
    float Ib;
    float Ic;
} PhaseCurrents_TypeDef;

/* αβ坐标系电流结构体 */
typedef struct {
    float alpha;
    float beta;
} AlphaBeta_TypeDef;

/* dq坐标系电流/电压结构体 */
typedef struct {
    float d;
    float q;
} DQ_TypeDef;

/* SVPWM扇区信息 */
typedef struct {
    uint8_t sector;         // 扇区编号 (1-6)
    float T1;               // 矢量1作用时间
    float T2;               // 矢量2作用时间
    float T0;               // 零矢量作用时间
} SVPWM_TypeDef;

/* FOC控制器结构体 */
typedef struct {
    // PID控制器
    PID_TypeDef pid_id;         // d轴电流环PID
    PID_TypeDef pid_iq;         // q轴电流环PID
    PID_TypeDef pid_velocity;   // 速度环PID

    // 电流
    PhaseCurrents_TypeDef i_abc;    // 三相电流
    AlphaBeta_TypeDef i_alphabeta;  // αβ电流
    DQ_TypeDef i_dq;                // dq电流

    // 电压
    DQ_TypeDef v_dq;                // dq电压（PID输出）
    AlphaBeta_TypeDef v_alphabeta;  // αβ电压

    // 占空比
    float duty_a;
    float duty_b;
    float duty_c;

    // 转子角度和速度
    float theta_elec;           // 电角度 (rad)
    float omega_elec;           // 电角速度 (rad/s)

    // 目标值
    float target_velocity;      // 目标速度 (rad/s)
    float target_id;            // 目标d轴电流 (通常为0)
    float target_iq;            // 目标q轴电流 (由速度环输出)

    // 电机参数
    uint8_t pole_pairs;         // 极对数
    float voltage_supply;       // 供电电压 (V)

    // 控制标志
    uint8_t enabled;            // FOC使能标志
    uint8_t open_loop;          // 开环模式标志

} FOC_TypeDef;

/* 注意: FOC控制器不再使用全局实例，而是作为Motor对象的成员 */

/* ==================== PID控制器函数 ==================== */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float dt, float output_limit);
float PID_Cal(PID_TypeDef *pid, float actual_value);
void PID_Reset(PID_TypeDef *pid);
void PID_SetTarget(PID_TypeDef *pid, float Target);

/* ==================== 坐标变换函数 ==================== */
void Inverse_Park_Transform(DQ_TypeDef *v_dq, float theta, AlphaBeta_TypeDef *v_alphabeta);

/* ==================== SVPWM调制函数 ==================== */
void SVPWM_Calculate(AlphaBeta_TypeDef *v_alphabeta, float v_dc, SVPWM_TypeDef *svpwm);
void SVPWM_GetDutyCycles(SVPWM_TypeDef *svpwm, float *duty_a, float *duty_b, float *duty_c);
void SVPWM_GetDutyCycles2(SVPWM_TypeDef *svpwm, float *duty_a, float *duty_b, float *duty_c);
void SVPWM_Calculate_Simplified(AlphaBeta_TypeDef *v_alphabeta, float v_dc, SVPWM_TypeDef *svpwm);
/* ==================== SPWM调制函数 ==================== */
void SPWM_Calculate(FOC_TypeDef *foc,AlphaBeta_TypeDef *v_alphabeta, float vdc);
/* ==================== FOC主控制函数 ==================== */
void FOC_Init(FOC_TypeDef *foc_ctrl, uint8_t pole_pairs, float voltage_supply);
void FOC_SetVelocity(FOC_TypeDef *foc_ctrl, float target_velocity);
void FOC_SetCurrent(FOC_TypeDef *foc_ctrl, float target_iq);
void FOC_UpdateCurrents(FOC_TypeDef *foc_ctrl, PhaseCurrents_TypeDef *i_abc);
void FOC_UpdateAngle(FOC_TypeDef *foc_ctrl, float mechanical_angle);
void FOC_CalCurrentLoop(FOC_TypeDef *foc_ctrl);
void FOC_CalVelocityLoop(FOC_TypeDef *foc_ctrl);
/* FOC_UpdatePWM 已移除，PWM更新由Motor层的硬件抽象层负责 */
void FOC_Enable(FOC_TypeDef *foc_ctrl);
void FOC_Disable(FOC_TypeDef *foc_ctrl);
void FOC_EmergencyStop(FOC_TypeDef *foc_ctrl);

/* ==================== 工具函数 ==================== */
float _normalizeAngle(float angle);
float _constrainFloat(float value, float min, float max);
float _electricalAngle(float mechanical_angle, uint8_t pole_pairs);

#endif /* FOC_H */
