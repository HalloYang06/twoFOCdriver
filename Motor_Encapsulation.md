# 电机封装架构说明

## 📐 整体架构

本项目采用分层架构设计，将FOC控制算法、硬件抽象层和电机对象完全解耦，支持多电机独立控制。

```
┌─────────────────────────────────────────┐
│          应用层 (main.c)                │
│    - 双电机控制逻辑                      │
│    - RTOS任务调度                       │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│      电机对象层 (motor.h/.c)            │
│    - Motor_TypeDef 电机实例            │
│    - 统一的控制接口                     │
│    - 状态管理和保护                     │
└─────────────────────────────────────────┘
          ↓                    ↓
┌──────────────────┐  ┌──────────────────┐
│  FOC控制层       │  │  硬件抽象层       │
│  (FOC.h/.c)      │  │  (motor_hardware) │
│  - 纯算法实现    │  │  - PWM驱动       │
│  - PID控制       │  │  - ADC采样       │
│  - 坐标变换      │  │  - 编码器读取    │
│  - SVPWM调制     │  │                  │
└──────────────────┘  └──────────────────┘
                             ↓
                  ┌──────────────────┐
                  │    STM32 HAL层   │
                  │  - TIM, ADC, etc │
                  └──────────────────┘
```

## 🎯 核心设计理念

### 1. **分层解耦**
- **FOC控制层**：纯算法，不依赖任何硬件
- **硬件抽象层**：封装PWM、ADC、编码器等硬件操作
- **电机对象层**：整合控制算法和硬件资源

### 2. **面向对象**
```c
Motor_TypeDef motor1;  // 电机1对象
Motor_TypeDef motor2;  // 电机2对象
```
每个电机对象包含：
- FOC控制器实例
- 硬件资源配置
- 电机参数
- 运行状态

### 3. **硬件抽象**
通过配置结构体绑定硬件资源，无需修改核心代码：
```c
MotorHAL_TypeDef motor1_hal = {
    .pwm = { .timer = &htim1, .channel_u = TIM_CHANNEL_1, ... },
    .current_sense = { .adc = &hadc1, ... },
    .encoder = { .timer = &htim2, ... }
};
```

## 📁 文件结构

```
Core/
├── Inc/
│   ├── FOC.h                    # FOC控制算法接口
│   ├── motor.h                  # 电机对象接口
│   └── motor_hardware.h         # 硬件抽象层接口
└── Src/
    ├── FOC.c                    # FOC控制算法实现
    ├── motor.c                  # 电机对象实现
    ├── motor_hardware.c         # 硬件抽象层实现
    └── dual_motor_example.c     # 双电机使用示例
```

## 🚀 快速开始

### 1. 初始化电机

```c
#include "motor.h"

Motor_TypeDef motor1;

// 配置硬件资源
MotorHAL_TypeDef motor1_hal = {
    .pwm = {
        .timer = &htim1,
        .channel_u = TIM_CHANNEL_1,
        .channel_v = TIM_CHANNEL_2,
        .channel_w = TIM_CHANNEL_3,
        .period = 2999
    },
    .current_sense = {
        .adc = &hadc1,
        .channel_u = ADC_CHANNEL_0,
        .channel_v = ADC_CHANNEL_1,
        .shunt_resistance = 0.01f,      // 10mΩ
        .amplifier_gain = 20.0f,
        .adc_ref_voltage = 3.3f,
        .adc_resolution = 4096
    },
    .encoder = {
        .timer = &htim2,
        .cpr = 1000,                    // 1000线
        .max_count = 65535,
        .direction = 1.0f
    }
};

// 配置电机参数
MotorParams_TypeDef motor1_params = {
    .pole_pairs = 7,
    .voltage_supply = 24.0f,
    .phase_resistance = 0.5f,
    .phase_inductance = 0.001f,
    .current_limit = 10.0f,
    .velocity_limit = 100.0f
};

// 初始化电机
Motor_Init(&motor1, 1, "Motor1", &motor1_hal, &motor1_params);
```

### 2. 使能电机并设置控制模式

```c
// 使能电机
Motor_Enable(&motor1);

// 力矩模式控制
Motor_SetMode(&motor1, MOTOR_MODE_TORQUE);
Motor_SetTorque(&motor1, 2.0f);  // 2A

// 或速度模式控制
Motor_SetMode(&motor1, MOTOR_MODE_VELOCITY);
Motor_SetVelocity(&motor1, 50.0f);  // 50 rad/s
```

### 3. 周期性更新（在中断或RTOS任务中）

```c
// 高频电流环 (20kHz)
void TIM_CurrentLoop_ISR(void)
{
    Motor_UpdateSensors(&motor1);      // 读取传感器
    Motor_UpdateCurrentLoop(&motor1);   // 执行电流环
}

// 低频速度环 (1kHz)
void TIM_VelocityLoop_ISR(void)
{
    Motor_UpdateVelocityLoop(&motor1);
}
```

## 🔄 双电机控制

```c
Motor_TypeDef motor1, motor2;

// 初始化两个电机（配置不同的硬件资源）
Motor_Init(&motor1, 1, "Motor1", &motor1_hal, &motor1_params);
Motor_Init(&motor2, 2, "Motor2", &motor2_hal, &motor2_params);

// 使能并控制
Motor_Enable(&motor1);
Motor_Enable(&motor2);

Motor_SetVelocity(&motor1, 50.0f);
Motor_SetVelocity(&motor2, -50.0f);  // 反向

// 在中断中更新
void TIM_ISR(void)
{
    Motor_UpdateSensors(&motor1);
    Motor_UpdateCurrentLoop(&motor1);

    Motor_UpdateSensors(&motor2);
    Motor_UpdateCurrentLoop(&motor2);
}
```

## 📊 控制模式

### 1. 力矩模式 (Torque Mode)
直接设置q轴电流，适合需要精确力矩控制的场合。

```c
Motor_SetMode(&motor1, MOTOR_MODE_TORQUE);
Motor_SetTorque(&motor1, 3.0f);  // 3A q轴电流
```

### 2. 速度模式 (Velocity Mode)
设置目标速度，速度环PID自动调节。

```c
Motor_SetMode(&motor1, MOTOR_MODE_VELOCITY);
Motor_SetVelocity(&motor1, 100.0f);  // 100 rad/s
```

### 3. 位置模式 (预留)
后续可扩展位置环控制。

## ⚙️ PID参数调整

```c
// 调整电流环PID
Motor_SetCurrentPID(&motor1, 1.5f, 60.0f, 0.0f);

// 调整速度环PID
Motor_SetVelocityPID(&motor1, 0.8f, 3.0f, 0.02f);
```

## 🛡️ 保护和错误处理

```c
// 检查错误
uint32_t errors = Motor_GetErrorFlags(&motor1);

if (errors & MOTOR_ERROR_OVERCURRENT) {
    Motor_EmergencyStop(&motor1);
}

// 清除错误
Motor_ClearError(&motor1);
```

## 📈 状态查询

```c
float angle = Motor_GetAngle(&motor1);        // 当前角度
float velocity = Motor_GetVelocity(&motor1);  // 当前速度
float current = Motor_GetCurrent(&motor1);    // 当前电流
MotorState_TypeDef state = Motor_GetState(&motor1);  // 电机状态
```

## 🔧 硬件适配

### 添加第3个电机
只需创建新的电机对象和硬件配置：

```c
Motor_TypeDef motor3;

MotorHAL_TypeDef motor3_hal = {
    .pwm = { .timer = &htim4, ... },
    .current_sense = { .adc = &hadc3, ... },
    .encoder = { .timer = &htim5, ... }
};

Motor_Init(&motor3, 3, "Motor3", &motor3_hal, &motor3_params);
```

### 更换硬件外设
只需修改配置结构体中的定时器和ADC句柄，无需修改核心代码。

## 📝 注意事项

1. **中断频率**：
   - 电流环建议 20kHz
   - 速度环建议 1kHz

2. **传感器校准**：
   初始化后务必校准电流传感器
   ```c
   Motor_CalibrateCurrentSensor(&motor1);
   ```

3. **硬件资源冲突**：
   确保每个电机使用独立的定时器和ADC通道

4. **堆栈大小**：
   使用FreeRTOS时注意任务堆栈大小，`Motor_TypeDef`结构体较大

## 🎓 扩展建议

- 位置环控制
- CAN总线通信
- 参数自动整定
- 数据记录和分析
- 上位机监控界面

## 📄 相关文件

- `dual_motor_example.c` - 完整的双电机使用示例
- `FOC_Usage.md` - FOC算法使用说明