#include "FOC.h"
#include <string.h>
#include "arm_math.h"
/* 注意: 全局FOC实例已移除，FOC控制器现在作为Motor对象的成员 */

/* ==================== PID控制器函数实现 ==================== */

/**
 * @brief  初始化PID控制器
 * @param  pid: PID结构体指针
 * @param  Kp: 比例系数
 * @param  Ki: 积分系数
 * @param  Kd: 微分系数
 * @param  dt: 采样周期 (s)
 * @param  output_limit: 输出限制
 * @retval None
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float dt, float output_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->output_limit = output_limit;
    pid->integral_limit = output_limit * 0.8f;  // 积分限制为输出限制的80%

    PID_Reset(pid);
}

/**
 * @brief  重置PID控制器
 * @param  pid: PID结构体指针
 * @retval None
 */
void PID_Reset(PID_TypeDef *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  设置PID目标值
 * @param  pid: PID结构体指针
 * @param  Target: 目标值
 * @retval None
 */
void PID_SetTarget(PID_TypeDef *pid, float Target)
{
    pid->target = Target;
}

/**
 * @brief  PID计算
 * @param  pid: PID结构体指针
 * @param  actual_value: 测量值
 * @retval PID输出值
 */
float PID_Cal(PID_TypeDef *pid, float actual_value)
{
    // 计算误差
    float error = pid->target - actual_value;

    // 比例项
    float P_term = pid->Kp * error;

    // 积分项（带抗饱和）
    pid->integral += error * pid->dt;
    pid->integral = _constrainFloat(pid->integral, -pid->integral_limit, pid->integral_limit);
    float I_term = pid->Ki * pid->integral;

    // 微分项
    float derivative = (error - pid->prev_error) / pid->dt;
    float D_term = pid->Kd * derivative;

    // 计算输出
    pid->output = P_term + I_term + D_term;
    pid->output = _constrainFloat(pid->output, -pid->output_limit, pid->output_limit);

    // 保存当前误差
    pid->prev_error = error;

    return pid->output;
}

/* ==================== 坐标变换函数实现 ==================== */

/**
 * @brief  Clarke变换 (abc -> αβ)
 * @param  i_abc: 三相电流
 * @param  i_alphabeta: αβ电流输出
 * @retval None
 */
static inline void Clarke_Transform(PhaseCurrents_TypeDef *i_abc, AlphaBeta_TypeDef *i_alphabeta)
{

    i_alphabeta->alpha = i_abc->Ia;
    i_alphabeta->beta = ONE_BY_SQRT3 * i_abc->Ia + (2.0f * ONE_BY_SQRT3) * i_abc->Ib;
}

/**
 * @brief  Park变换 (αβ -> dq)
 * @param  i_alphabeta: αβ电流
 * @param  theta: 电角度 (rad)
 * @param  i_dq: dq电流输出
 * @retval None
 */
static inline void Park_Transform(AlphaBeta_TypeDef *i_alphabeta, float theta, DQ_TypeDef *i_dq)
{
    float32_t cos_value,sin_value;
    arm_sin_cos_f32(theta*57.2987f,&sin_value,&cos_value);

    i_dq->d = i_alphabeta->alpha * cos_value + i_alphabeta->beta * sin_value;
    i_dq->q = -i_alphabeta->alpha * sin_value + i_alphabeta->beta * cos_value;
}

/**
 * @brief  反Park变换 (dq -> αβ)
 * @param  v_dq: dq电压
 * @param  theta: 电角度 (rad)
 * @param  v_alphabeta: αβ电压输出
 * @retval None
 */
void Inverse_Park_Transform(DQ_TypeDef *v_dq, float theta, AlphaBeta_TypeDef *v_alphabeta)
{
    float32_t cos_value,sin_value;
    arm_sin_cos_f32(theta*57.2987f,&sin_value,&cos_value);

    v_alphabeta->alpha = v_dq->d * cos_value - v_dq->q * sin_value;
    v_alphabeta->beta = v_dq->d * sin_value + v_dq->q * cos_value;
}

/* ==================== SVPWM调制函数实现 ==================== */

/**
 * @brief  SVPWM计算
 * @param  v_alphabeta: αβ电压
 * @param  v_dc: 直流母线电压
 * @param  svpwm: SVPWM结构体输出
 * @retval None
 */
void SVPWM_Calculate(AlphaBeta_TypeDef *v_alphabeta, float v_dc, SVPWM_TypeDef *svpwm)
{
    float Valpha = v_alphabeta->alpha;
    float Vbeta  = v_alphabeta->beta;
    
    // 1. 计算中间变量 (基于归一化，系数调整为 1.732)
    float inv_vdc = SQRT3 / v_dc;
    
    float X = Vbeta * inv_vdc;
    float Y = (Valpha * 1.5f + Vbeta * SQRT3_BY_2) * (1.0f / v_dc) * SQRT3;
    float Z = (-Valpha * 1.5f + Vbeta * SQRT3_BY_2) * (1.0f / v_dc) * SQRT3;

    // 2. 扇区判定 (最原始的逻辑)
    uint8_t sector = 0;
    if (X > 0) sector += 1;
    if (Y > 0) sector += 2;
    if (Z > 0) sector += 4;
    svpwm->sector = sector;

    // 3. 根据扇区分配 T1, T2
    // 如果电机还是震动，很可能是这里的 T1, T2 对应反了
    switch(sector) {
        case 3: svpwm->T1 = Y;  svpwm->T2 = X;  break; // Sector 1
        case 1: svpwm->T1 = X;  svpwm->T2 = -Z; break; // Sector 2
        case 5: svpwm->T1 = -Z; svpwm->T2 = -Y; break; // Sector 3
        case 4: svpwm->T1 = -Y; svpwm->T2 = -X; break; // Sector 4
        case 6: svpwm->T1 = -X; svpwm->T2 = Z;  break; // Sector 5
        case 2: svpwm->T1 = Z;  svpwm->T2 = Y;  break; // Sector 6
        default: svpwm->T1 = 0; svpwm->T2 = 0; break;
    }

    // 4. 过调制处理
    float sum = svpwm->T1 + svpwm->T2;
    if (sum > 1.0f) {
        svpwm->T1 /= sum;
        svpwm->T2 /= sum;
        svpwm->T0 = 0;
    } else {
        svpwm->T0 = 1.0f - sum;
    }
}
/**
 *@brief SPWM计算
 * @param  foc: FOC结构体
 * @param  v_alphabeta :alphabeta结构体
 * @param  vdc:母线电压
 * @retval none
 */
void SPWM_Calculate(FOC_TypeDef *foc,AlphaBeta_TypeDef *v_alphabeta, float vdc) {
    float v_a=v_alphabeta->alpha;
    float v_b=-0.5f*v_alphabeta->alpha+0.8660254*v_alphabeta->beta;
    float v_c=-0.8660254*v_alphabeta->alpha-0.5f*v_alphabeta->beta;
    foc->duty_c = v_c / vdc+0.5;
    foc->duty_a = v_a / vdc+0.5;
    foc->duty_b = v_b / vdc+0.5;
    if (foc->duty_a>1.0f) foc->duty_a=1.0f;if (foc->duty_a<0.0f) foc->duty_a=0.0f;
    if (foc->duty_b>1.0f) foc->duty_b=1.0f;if (foc->duty_b<0.0f) foc->duty_b=0.0f;
    if (foc->duty_c>1.0f) foc->duty_c=1.0f;if (foc->duty_c<0.0f) foc->duty_c=0.0f;
}
/**
 * @brief  根据SVPWM计算三相占空比
 * @param  svpwm: SVPWM结构体
 * @param  duty_a: A相占空比输出
 * @param  duty_b: B相占空比输出
 * @param  duty_c: C相占空比输出
 * @retval None
 */
void SVPWM_GetDutyCycles(SVPWM_TypeDef *svpwm, float *duty_a, float *duty_b, float *duty_c)
{
    float T0_half = svpwm->T0 * 0.25f;
    float T1_half = svpwm->T1 * 0.5f;
    float T2_half = svpwm->T2 * 0.5f;

    // 标准七段式：每个周期中心对称
    switch(svpwm->sector) {
        case 3: // Sector 1
            *duty_a = T0_half + T1_half + T2_half;
            *duty_b = T0_half + T2_half;
            *duty_c = T0_half;
            break;
        case 1: // Sector 2
            *duty_a = T0_half + T1_half;
            *duty_b = T0_half + T1_half + T2_half;
            *duty_c = T0_half;
            break;
        case 5: // Sector 3
            *duty_a = T0_half;
            *duty_b = T0_half + T1_half + T2_half;
            *duty_c = T0_half + T2_half;
            break;
        case 4: // Sector 4
            *duty_a = T0_half;
            *duty_b = T0_half + T1_half;
            *duty_c = T0_half + T1_half + T2_half;
            break;
        case 6: // Sector 5
            *duty_a = T0_half + T2_half;
            *duty_b = T0_half;
            *duty_c = T0_half + T1_half + T2_half;
            break;
        case 2: // Sector 6
            *duty_a = T0_half + T1_half + T2_half;
            *duty_b = T0_half;
            *duty_c = T0_half + T1_half;
            break;
        default:
            *duty_a = 0.5f; *duty_b = 0.5f; *duty_c = 0.5f;
            break;
    }
}

/* ==================== FOC主控制函数实现 ==================== */

/**
 * @brief  初始化FOC控制器
 * @param  foc_ctrl: FOC控制器指针
 * @param  pole_pairs: 电机极对数
 * @param  voltage_supply: 供电电压 (V)
 * @retval None
 */
void FOC_Init(FOC_TypeDef *foc_ctrl, uint8_t pole_pairs, float voltage_supply)
{
    // 清零结构体
    memset(foc_ctrl, 0, sizeof(FOC_TypeDef));

    // 设置电机参数
    foc_ctrl->pole_pairs = pole_pairs;
    foc_ctrl->voltage_supply = voltage_supply;

    // 初始化d轴电流环PID (采样周期假设为50us = 0.00005s)
    // Kp=1.0, Ki=50.0, Kd=0.0
    PID_Init(&foc_ctrl->pid_id, 1.0f, 50.0f, 0.0f, 0.00005f, FOC_VOLTAGE_LIMIT);

    // 初始化q轴电流环PID
    PID_Init(&foc_ctrl->pid_iq, 1.0f, 50.0f, 0.0f, 0.00005f, FOC_VOLTAGE_LIMIT);

    // 初始化速度环PID (采样周期假设为1ms = 0.001s)
    // Kp=0.5, Ki=2.0, Kd=0.01
    PID_Init(&foc_ctrl->pid_velocity, 0.5f, 2.0f, 0.01f, 0.001f, FOC_CURRENT_LIMIT);

    // 设置目标值
    foc_ctrl->target_id = 0.0f;  // d轴电流通常设为0（PMSM最大转矩/电流比控制）
    foc_ctrl->target_iq = 0.0f;
    foc_ctrl->target_velocity = 0.0f;

    // 控制标志
    foc_ctrl->enabled = 0;
    foc_ctrl->open_loop = 0;
}

/**
 * @brief  设置目标速度
 * @param  foc_ctrl: FOC控制器指针
 * @param  target_velocity: 目标速度 (rad/s)
 * @retval None
 */
void FOC_SetVelocity(FOC_TypeDef *foc_ctrl, float target_velocity)
{
    foc_ctrl->target_velocity = _constrainFloat(target_velocity, -FOC_VELOCITY_LIMIT, FOC_VELOCITY_LIMIT);
    PID_SetTarget(&foc_ctrl->pid_velocity, foc_ctrl->target_velocity);
}

/**
 * @brief  设置目标q轴电流（转矩电流）
 * @param  foc_ctrl: FOC控制器指针
 * @param  target_iq: 目标q轴电流 (A)
 * @retval None
 */
void FOC_SetCurrent(FOC_TypeDef *foc_ctrl, float target_iq)
{
    foc_ctrl->target_iq = _constrainFloat(target_iq, -FOC_CURRENT_LIMIT, FOC_CURRENT_LIMIT);
    PID_SetTarget(&foc_ctrl->pid_iq, foc_ctrl->target_iq);
}

/**
 * @brief  更新三相电流
 * @param  foc_ctrl: FOC控制器指针
 * @param  i_abc: 三相电流
 * @retval None
 */
void FOC_UpdateCurrents(FOC_TypeDef *foc_ctrl, PhaseCurrents_TypeDef *i_abc)
{
    // 保存三相电流
    foc_ctrl->i_abc = *i_abc;

    // Clarke变换
    Clarke_Transform(&foc_ctrl->i_abc, &foc_ctrl->i_alphabeta);

    // Park变换
    Park_Transform(&foc_ctrl->i_alphabeta, foc_ctrl->theta_elec, &foc_ctrl->i_dq);
}

/**
 * @brief  更新转子角度
 * @param  foc_ctrl: FOC控制器指针
 * @param  mechanical_angle: 机械角度 (rad)
 * @retval None
 */
void FOC_UpdateAngle(FOC_TypeDef *foc_ctrl, float mechanical_angle)
{
    // 计算电角度
    foc_ctrl->theta_elec = _electricalAngle(mechanical_angle, foc_ctrl->pole_pairs);
}

/**
 * @brief  电流环计算
 * @param  foc_ctrl: FOC控制器指针
 * @retval None
 */
void FOC_CalCurrentLoop(FOC_TypeDef *foc_ctrl)
{
    // d轴电流环
    foc_ctrl->v_dq.d = PID_Cal(&foc_ctrl->pid_id, foc_ctrl->i_dq.d);

    // q轴电流环
    foc_ctrl->v_dq.q = PID_Cal(&foc_ctrl->pid_iq, foc_ctrl->i_dq.q);

    // 电压幅值限制
    float v_magnitude = sqrtf(foc_ctrl->v_dq.d * foc_ctrl->v_dq.d +
                               foc_ctrl->v_dq.q * foc_ctrl->v_dq.q);
    if (v_magnitude > FOC_VOLTAGE_LIMIT) {
        float scale = FOC_VOLTAGE_LIMIT / v_magnitude;
        foc_ctrl->v_dq.d *= scale;
        foc_ctrl->v_dq.q *= scale;
    }
}

/**
 * @brief  速度环计算
 * @param  foc_ctrl: FOC控制器指针
 * @retval None
 */
void FOC_CalVelocityLoop(FOC_TypeDef *foc_ctrl)
{
    // 速度环PID输出q轴电流目标值
    foc_ctrl->target_iq = PID_Cal(&foc_ctrl->pid_velocity, foc_ctrl->omega_elec);
    PID_SetTarget(&foc_ctrl->pid_iq, foc_ctrl->target_iq);
}

/* FOC_UpdatePWM 函数已移除 - PWM更新现在由Motor层的硬件抽象层负责 */

/**
 * @brief  使能FOC控制
 * @param  foc_ctrl: FOC控制器指针
 * @retval None
 */
void FOC_Enable(FOC_TypeDef *foc_ctrl)
{
    foc_ctrl->enabled = 1;
}

/**
 * @brief  禁用FOC控制
 * @param  foc_ctrl: FOC控制器指针
 * @retval None
 */
void FOC_Disable(FOC_TypeDef *foc_ctrl)
{
    foc_ctrl->enabled = 0;

    // 重置PID
    PID_Reset(&foc_ctrl->pid_id);
    PID_Reset(&foc_ctrl->pid_iq);
    PID_Reset(&foc_ctrl->pid_velocity);

    // PWM停止由Motor层的硬件抽象层负责
}

/**
 * @brief  FOC紧急停止
 * @param  foc_ctrl: FOC控制器指针
 * @retval None
 */
void FOC_EmergencyStop(FOC_TypeDef *foc_ctrl)
{
    FOC_Disable(foc_ctrl);
    // PWM紧急停止由Motor层的硬件抽象层负责
}

/* ==================== 工具函数实现 ==================== */

/**
 * @brief  角度归一化到[0, 2π)
 * @param  angle: 输入角度 (rad)
 * @retval 归一化后的角度 (rad)
 */
float _normalizeAngle(float angle)
{
    angle = fmodf(angle, TWO_PI);
    if (angle < 0) {
        angle += TWO_PI;
    }
    return angle;
}

/**
 * @brief  限制浮点数范围
 * @param  value: 输入值
 * @param  min: 最小值
 * @param  max: 最大值
 * @retval 限制后的值
 */
float _constrainFloat(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief  计算电角度
 * @param  mechanical_angle: 机械角度 (rad)
 * @param  pole_pairs: 极对数
 * @retval 电角度 (rad)
 */
float _electricalAngle(float mechanical_angle, uint8_t pole_pairs)
{
    return _normalizeAngle(mechanical_angle * pole_pairs);
}
