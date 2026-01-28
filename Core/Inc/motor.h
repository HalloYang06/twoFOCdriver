#ifndef MOTOR_H
#define MOTOR_H

#include "FOC.h"
#include "motor_hardware.h"

/* 电机控制模式 */
typedef enum {
    MOTOR_MODE_DISABLED = 0,       // 禁用
    MOTOR_MODE_TORQUE,             // 力矩模式（直接设置Iq电流）
    MOTOR_MODE_VELOCITY,           // 速度模式
    MOTOR_MODE_POSITION,           // 位置模式（预留）
    MOTOR_MODE_OPEN_LOOP           // 开环模式
} MotorControlMode_TypeDef;

/* 电机状态 */
typedef enum {
    MOTOR_STATE_IDLE = 0,          // 空闲
    MOTOR_STATE_CALIBRATING,       // 校准中
    MOTOR_STATE_RUNNING,           // 运行中
    MOTOR_STATE_ERROR,             // 错误
    MOTOR_STATE_EMERGENCY_STOP     // 紧急停止
} MotorState_TypeDef;

/* 电机参数配置 */
typedef struct {
    uint8_t pole_pairs;            // 极对数
    float voltage_supply;          // 供电电压 (V)
    float phase_resistance;        // 相电阻 (Ω)
    float phase_inductance;        // 相电感 (H)
    float current_limit;           // 电流限制 (A)
    float velocity_limit;          // 速度限制 (rad/s)
} MotorParams_TypeDef;

/* 电机对象 */
typedef struct {
    /* 基本信息 */
    uint8_t id;                    // 电机ID
    char name[16];                 // 电机名称

    /* 控制层 */
    FOC_TypeDef foc;               // FOC控制器实例

    /* 硬件抽象层 */
    MotorHAL_TypeDef hal;          // 硬件抽象层实例

    /* 电机参数 */
    MotorParams_TypeDef params;    // 电机参数

    /* 运行状态 */
    MotorState_TypeDef state;      // 电机状态
    MotorControlMode_TypeDef mode; // 控制模式

    /* 反馈数据 */
    float current_angle;           // 当前机械角度 (rad)
    float current_velocity;        // 当前机械角速度 (rad/s)
    float current_iq;              // 当前q轴电流 (A)
    float current_id;              // 当前d轴电流 (A)

    /* 错误标志 */
    uint32_t error_flags;          // 错误标志位

} Motor_TypeDef;

/* 错误标志位定义 */
#define MOTOR_ERROR_NONE            0x00000000
#define MOTOR_ERROR_OVERCURRENT     0x00000001
#define MOTOR_ERROR_OVERVOLTAGE     0x00000002
#define MOTOR_ERROR_OVERTEMP        0x00000004
#define MOTOR_ERROR_ENCODER         0x00000008
#define MOTOR_ERROR_CALIBRATION     0x00000010

/* ==================== 电机初始化函数 ==================== */
void Motor_Init(Motor_TypeDef *motor,
                uint8_t motor_id,
                const char *name,
                MotorHAL_TypeDef *hal_config,
                MotorParams_TypeDef *params);

/* ==================== 电机控制函数 ==================== */
void Motor_Enable(Motor_TypeDef *motor);
void Motor_Disable(Motor_TypeDef *motor);
void Motor_SetMode(Motor_TypeDef *motor, MotorControlMode_TypeDef mode);

/* 设置目标值 */
void Motor_SetTorque(Motor_TypeDef *motor, float target_iq);        // 力矩模式
void Motor_SetVelocity(Motor_TypeDef *motor, float target_velocity);// 速度模式
void Motor_SetPosition(Motor_TypeDef *motor, float target_position);// 位置模式

/* ==================== 电机更新函数 ==================== */
void Motor_UpdateSensors(Motor_TypeDef *motor);       // 更新传感器数据
void Motor_UpdateCurrentLoop(Motor_TypeDef *motor);   // 更新电流环（高频，如20kHz）
void Motor_UpdateVelocityLoop(Motor_TypeDef *motor);  // 更新速度环（低频，如1kHz）

/* ==================== 电机校准函数 ==================== */
void Motor_CalibrateCurrentSensor(Motor_TypeDef *motor);
void Motor_CalibrateEncoder(Motor_TypeDef *motor);

/* ==================== 电机状态查询函数 ==================== */
MotorState_TypeDef Motor_GetState(Motor_TypeDef *motor);
float Motor_GetAngle(Motor_TypeDef *motor);
float Motor_GetVelocity(Motor_TypeDef *motor);
float Motor_GetCurrent(Motor_TypeDef *motor);
uint32_t Motor_GetErrorFlags(Motor_TypeDef *motor);

/* ==================== 电机保护函数 ==================== */
void Motor_EmergencyStop(Motor_TypeDef *motor);
void Motor_ClearError(Motor_TypeDef *motor);
uint8_t Motor_CheckLimits(Motor_TypeDef *motor);

/* ==================== 电机PID参数调整函数 ==================== */
void Motor_SetCurrentPID(Motor_TypeDef *motor, float Kp, float Ki, float Kd);
void Motor_SetVelocityPID(Motor_TypeDef *motor, float Kp, float Ki, float Kd);

#endif /* MOTOR_H */
