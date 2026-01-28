#include "motor.h"
#include <string.h>
#include <stdio.h>

/* ==================== 电机初始化函数实现 ==================== */
/**
 * @brief  初始化电机对象
 * @param  motor: 电机对象指针
 * @param  motor_id: 电机ID (1, 2, ...)
 * @param  name: 电机名称
 * @param  hal_config: 硬件抽象层配置
 * @param  params: 电机参数
 * @retval None
 */
void Motor_Init(Motor_TypeDef *motor,
                uint8_t motor_id,
                const char *name,
                MotorHAL_TypeDef *hal_config,
                MotorParams_TypeDef *params)
{
    // 清零电机结构体
    memset(motor, 0, sizeof(Motor_TypeDef));

    // 设置基本信息
    motor->id = motor_id;
    strncpy(motor->name, name, sizeof(motor->name) - 1);

    // 复制硬件配置
    memcpy(&motor->hal, hal_config, sizeof(MotorHAL_TypeDef));

    // 复制电机参数
    memcpy(&motor->params, params, sizeof(MotorParams_TypeDef));

    // 初始化硬件抽象层
    MotorHAL_Init(&motor->hal);

    // 初始化FOC控制器
    FOC_Init(&motor->foc, params->pole_pairs, params->voltage_supply);

    // 设置电流和速度限制
    motor->foc.pid_iq.output_limit = params->current_limit;
    motor->foc.pid_id.output_limit = params->current_limit;
    motor->foc.pid_velocity.output_limit = params->current_limit;

    // 初始化状态
    motor->state = MOTOR_STATE_IDLE;
    motor->mode = MOTOR_MODE_DISABLED;
    motor->error_flags = MOTOR_ERROR_NONE;
}

/* ==================== 电机控制函数实现 ==================== */

/**
 * @brief  使能电机
 * @param  motor: 电机对象指针
 * @retval None
 */
void Motor_Enable(Motor_TypeDef *motor)
{
    if (motor->state == MOTOR_STATE_ERROR) {
        // 如果处于错误状态，不允许使能
        return;
    }

    // 启动PWM输出
    MotorHAL_PWM_Start(&motor->hal);

    // 使能FOC控制
    FOC_Enable(&motor->foc);

    // 更新状态
    motor->state = MOTOR_STATE_RUNNING;
}

/**
 * @brief  禁用电机
 * @param  motor: 电机对象指针
 * @retval None
 */
void Motor_Disable(Motor_TypeDef *motor)
{
    // 禁用FOC控制
    FOC_Disable(&motor->foc);

    // 停止PWM输出
    MotorHAL_PWM_Stop(&motor->hal);

    // 更新状态
    motor->state = MOTOR_STATE_IDLE;
    motor->mode = MOTOR_MODE_DISABLED;
}

/**
 * @brief  设置电机控制模式
 * @param  motor: 电机对象指针
 * @param  mode: 控制模式
 * @retval None
 */
void Motor_SetMode(Motor_TypeDef *motor, MotorControlMode_TypeDef mode)
{
    motor->mode = mode;

    // 根据模式重置PID
    if (mode == MOTOR_MODE_TORQUE) {
        PID_Reset(&motor->foc.pid_velocity);
    } else if (mode == MOTOR_MODE_VELOCITY) {
        // 速度模式正常运行
    } else if (mode == MOTOR_MODE_OPEN_LOOP) {
        PID_Reset(&motor->foc.pid_id);
        PID_Reset(&motor->foc.pid_iq);
        PID_Reset(&motor->foc.pid_velocity);
    }
}

/**
 * @brief  设置目标力矩（力矩模式）
 * @param  motor: 电机对象指针
 * @param  target_iq: 目标q轴电流 (A)
 * @retval None
 */
void Motor_SetTorque(Motor_TypeDef *motor, float target_iq)
{
    if (motor->mode != MOTOR_MODE_TORQUE) {
        Motor_SetMode(motor, MOTOR_MODE_TORQUE);
    }

    FOC_SetCurrent(&motor->foc, target_iq);
}

/**
 * @brief  设置目标速度（速度模式）
 * @param  motor: 电机对象指针
 * @param  target_velocity: 目标速度 (rad/s)
 * @retval None
 */
void Motor_SetVelocity(Motor_TypeDef *motor, float target_velocity)
{
    if (motor->mode != MOTOR_MODE_VELOCITY) {
        Motor_SetMode(motor, MOTOR_MODE_VELOCITY);
    }

    FOC_SetVelocity(&motor->foc, target_velocity);
}

/**
 * @brief  设置目标位置（位置模式，预留）
 * @param  motor: 电机对象指针
 * @param  target_position: 目标位置 (rad)
 * @retval None
 */
void Motor_SetPosition(Motor_TypeDef *motor, float target_position)
{
    // 位置模式预留，需要增加位置环PID
    (void)motor;
    (void)target_position;
}

/* ==================== 电机更新函数实现 ==================== */

/**
 * @brief  更新传感器数据
 * @param  motor: 电机对象指针
 * @retval None
 * @note   应在高频中断中调用（如20kHz）
 */
void Motor_UpdateSensors(Motor_TypeDef *motor)
{
    // 读取三相电流
    float current_u, current_v, current_w;
    MotorHAL_CurrentSense_Read(&motor->hal, &current_u, &current_v, &current_w);

    // 更新FOC电流
    PhaseCurrents_TypeDef i_abc = {
        .Ia = current_u,
        .Ib = current_v,
        .Ic = current_w
    };
    FOC_UpdateCurrents(&motor->foc, &i_abc);

    // 读取编码器角度
    float angle = MotorHAL_Encoder_GetAngle(&motor->hal);
    motor->current_angle = angle;

    // 更新FOC角度
    FOC_UpdateAngle(&motor->foc, angle);

    // 读取速度
    float velocity = MotorHAL_Encoder_GetVelocity(&motor->hal);
    motor->current_velocity = velocity;
    motor->foc.omega_elec = velocity * motor->params.pole_pairs;

    // 保存当前dq电流
    motor->current_id = motor->foc.i_dq.d;
    motor->current_iq = motor->foc.i_dq.q;
}

/**
 * @brief  更新电流环
 * @param  motor: 电机对象指针
 * @retval None
 * @note

 */
void Motor_UpdateCurrentLoop(Motor_TypeDef *motor)
{
    if (motor->state != MOTOR_STATE_RUNNING) {
        return;
    }

    if (motor->mode == MOTOR_MODE_DISABLED) {
        return;
    }

    // 检查限制
    if (Motor_CheckLimits(motor) != 0) {
        Motor_EmergencyStop(motor);
        return;
    }

    // 执行电流环计算
    FOC_CalCurrentLoop(&motor->foc);

    // 更新PWM输出
    // 反Park变换
    Inverse_Park_Transform(&motor->foc.v_dq, motor->foc.theta_elec, &motor->foc.v_alphabeta);

    // SVPWM调制
    SVPWM_TypeDef svpwm;
    SVPWM_Calculate(&motor->foc.v_alphabeta, motor->foc.voltage_supply, &svpwm);
    SVPWM_GetDutyCycles(&svpwm, &motor->foc.duty_a, &motor->foc.duty_b, &motor->foc.duty_c);

    // 输出PWM
    MotorHAL_PWM_SetDutyCycle(&motor->hal,
                              motor->foc.duty_a,
                              motor->foc.duty_b,
                              motor->foc.duty_c);
}

/**
 * @brief  更新速度环
 * @param  motor: 电机对象指针
 * @retval None
 * @note
 */
void Motor_UpdateVelocityLoop(Motor_TypeDef *motor)
{
    if (motor->state != MOTOR_STATE_RUNNING) {
        return;
    }

    if (motor->mode != MOTOR_MODE_VELOCITY) {
        return;
    }

    // 执行速度环计算
    FOC_CalVelocityLoop(&motor->foc);
}

/* ==================== 电机校准函数实现 ==================== */

/**
 * @brief  校准电流传感器
 * @param  motor: 电机对象指针
 * @retval None
 */
void Motor_CalibrateCurrentSensor(Motor_TypeDef *motor)
{
    motor->state = MOTOR_STATE_CALIBRATING;

    // 确保PWM停止
    MotorHAL_PWM_Stop(&motor->hal);

    // 执行电流传感器校准
    MotorHAL_CurrentSense_Calibrate(&motor->hal);

    motor->state = MOTOR_STATE_IDLE;
}

/**
 * @brief  校准编码器
 * @param  motor: 电机对象指针
 * @retval None
 */
void Motor_CalibrateEncoder(Motor_TypeDef *motor)
{
    motor->state = MOTOR_STATE_CALIBRATING;

    // 编码器零点校准
    MotorHAL_Encoder_Reset(&motor->hal);

    motor->state = MOTOR_STATE_IDLE;
}

/* ==================== 电机状态查询函数实现 ==================== */

/**
 * @brief  获取电机状态
 * @param  motor: 电机对象指针
 * @retval 电机状态
 */
MotorState_TypeDef Motor_GetState(Motor_TypeDef *motor)
{
    return motor->state;
}

/**
 * @brief  获取电机当前角度
 * @param  motor: 电机对象指针
 * @retval 当前机械角度 (rad)
 */
float Motor_GetAngle(Motor_TypeDef *motor)
{
    return motor->current_angle;
}

/**
 * @brief  获取电机当前速度
 * @param  motor: 电机对象指针
 * @retval 当前机械角速度 (rad/s)
 */
float Motor_GetVelocity(Motor_TypeDef *motor)
{
    return motor->current_velocity;
}

/**
 * @brief  获取电机当前电流
 * @param  motor: 电机对象指针
 * @retval 当前q轴电流 (A)
 */
float Motor_GetCurrent(Motor_TypeDef *motor)
{
    return motor->current_iq;
}

/**
 * @brief  获取错误标志
 * @param  motor: 电机对象指针
 * @retval 错误标志位
 */
uint32_t Motor_GetErrorFlags(Motor_TypeDef *motor)
{
    return motor->error_flags;
}

/* ==================== 电机保护函数实现 ==================== */

/**
 * @brief  紧急停止
 * @param  motor: 电机对象指针
 * @retval None
 */
void Motor_EmergencyStop(Motor_TypeDef *motor)
{
    // 立即停止PWM
    MotorHAL_EmergencyStop(&motor->hal);

    // 禁用FOC
    FOC_Disable(&motor->foc);

    // 更新状态
    motor->state = MOTOR_STATE_EMERGENCY_STOP;
}

/**
 * @brief  清除错误标志
 * @param  motor: 电机对象指针
 * @retval None
 */
void Motor_ClearError(Motor_TypeDef *motor)
{
    motor->error_flags = MOTOR_ERROR_NONE;

    if (motor->state == MOTOR_STATE_ERROR || motor->state == MOTOR_STATE_EMERGENCY_STOP) {
        motor->state = MOTOR_STATE_IDLE;
    }
}

/**
 * @brief  检查电机限制
 * @param  motor: 电机对象指针
 * @retval 0: 正常, 非0: 超限
 */
uint8_t Motor_CheckLimits(Motor_TypeDef *motor)
{
    uint8_t error = 0;

    // 检查电流限制
    float current_magnitude = sqrtf(motor->current_id * motor->current_id +
                                    motor->current_iq * motor->current_iq);
    if (current_magnitude > motor->params.current_limit) {
        motor->error_flags |= MOTOR_ERROR_OVERCURRENT;
        error = 1;
    }

    // 检查电压限制
    float voltage_magnitude = sqrtf(motor->foc.v_dq.d * motor->foc.v_dq.d +
                                    motor->foc.v_dq.q * motor->foc.v_dq.q);
    if (voltage_magnitude > motor->params.voltage_supply) {
        motor->error_flags |= MOTOR_ERROR_OVERVOLTAGE;
        error = 1;
    }

    // 检查速度限制
    float velocity_abs = (motor->current_velocity > 0) ? motor->current_velocity : -motor->current_velocity;
    if (velocity_abs > motor->params.velocity_limit) {
        // 速度超限不一定是错误，可以仅记录
    }

    if (error) {
        motor->state = MOTOR_STATE_ERROR;
    }

    return error;
}

/* ==================== 电机PID参数调整函数实现 ==================== */

/**
 * @brief  设置电流环PID参数
 * @param  motor: 电机对象指针
 * @param  Kp: 比例系数
 * @param  Ki: 积分系数
 * @param  Kd: 微分系数
 * @retval None
 */
void Motor_SetCurrentPID(Motor_TypeDef *motor, float Kp, float Ki, float Kd)
{
    motor->foc.pid_id.Kp = Kp;
    motor->foc.pid_id.Ki = Ki;
    motor->foc.pid_id.Kd = Kd;

    motor->foc.pid_iq.Kp = Kp;
    motor->foc.pid_iq.Ki = Ki;
    motor->foc.pid_iq.Kd = Kd;
}

/**
 * @brief  设置速度环PID参数
 * @param  motor: 电机对象指针
 * @param  Kp: 比例系数
 * @param  Ki: 积分系数
 * @param  Kd: 微分系数
 * @retval None
 */
void Motor_SetVelocityPID(Motor_TypeDef *motor, float Kp, float Ki, float Kd)
{
    motor->foc.pid_velocity.Kp = Kp;
    motor->foc.pid_velocity.Ki = Ki;
    motor->foc.pid_velocity.Kd = Kd;
}
