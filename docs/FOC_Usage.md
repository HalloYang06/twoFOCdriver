# FOC (Field Oriented Control) 使用说明

## 1. 系统架构

### 1.1 控制流程
```
编码器 → 角度/速度 ┐
                   ├→ FOC算法 → SVPWM → PWM输出 → 电机
电流传感器 → 电流 ┘
```

### 1.2 控制环路
- **电流环**: 20kHz (PWM频率)，d/q轴电流PID控制
- **速度环**: 1kHz，速度PID控制，输出q轴电流目标值

## 2. 初始化示例

在`main.c`中添加以下初始化代码：

```c
#include "FOC.h"
#include "encoder.h"
#include "current_sense.h"

/* 全局变量 */
extern FOC_TypeDef foc;
Encoder_TypeDef encoder_M0;
CurrentSense_TypeDef current_sense;

void System_Init(void)
{
    /* 硬件初始化 */
    PWM_Init();

    /* 编码器初始化 */
    Encoder_Init(&encoder_M0, &htim3);
    Encoder_Start(&encoder_M0);

    /* 电流采样初始化 */
    CurrentSense_Init(&current_sense, &hadc1);
    CurrentSense_Calibrate(&current_sense, 100);  // 零点校准
    CurrentSense_Start(&current_sense);

    /* FOC初始化 */
    // 参数: 极对数=7, 供电电压=12V
    FOC_Init(&foc, 7, 12.0f);

    /* 启用FOC */
    FOC_Enable(&foc);
}
```

## 3. 电流环控制 (20kHz)

在`stm32h7xx_it.c`的ADC中断中调用：

```c
/* ADC1转换完成中断 */
void ADC1_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

/* ADC转换完成回调 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc1)
    {
        /* 更新电流采样 */
        CurrentSense_DMA_CpltCallback(&current_sense);

        if (foc.enabled)
        {
            /* 获取三相电流 */
            PhaseCurrents_TypeDef i_abc;
            CurrentSense_GetCurrents(&current_sense, &i_abc.Ia, &i_abc.Ib, &i_abc.Ic);

            /* 更新FOC电流 */
            FOC_UpdateCurrents(&foc, &i_abc);

            /* 计算电流环 */
            FOC_ComputeCurrentLoop(&foc);

            /* 更新PWM输出 */
            FOC_UpdatePWM(&foc);
        }
    }
}
```

## 4. 速度环控制 (1kHz)

在`stm32h7xx_it.c`的TIM7中断中调用：

```c
/* TIM7中断处理 */
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim7);
}

/* TIM7周期中断回调 (1ms周期) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim7)
    {
        /* 更新编码器速度 */
        Encoder_UpdateSpeed(&encoder_M0);

        if (foc.enabled)
        {
            /* 获取机械角度 */
            float mechanical_angle = (float)Encoder_GetTotalCount(&encoder_M0)
                                   / (4.0f * ENCODER_PPR) * 2.0f * PI;

            /* 更新FOC角度 */
            FOC_UpdateAngle(&foc, mechanical_angle);

            /* 获取电角速度 (rad/s) */
            foc.omega_elec = Encoder_GetSpeed_RPS(&encoder_M0) * 2.0f * PI * foc.pole_pairs;

            /* 计算速度环 */
            FOC_ComputeVelocityLoop(&foc);
        }
    }
}
```

## 5. 控制接口

### 5.1 速度控制模式
```c
// 设置目标速度 (rad/s)
FOC_SetVelocity(&foc, 100.0f);  // 100 rad/s

// 启用FOC
FOC_Enable(&foc);
```

### 5.2 转矩控制模式
```c
// 直接设置q轴电流 (A)
FOC_SetCurrent(&foc, 2.0f);  // 2A转矩电流

// 启用FOC
FOC_Enable(&foc);
```

### 5.3 紧急停止
```c
// 立即停止电机
FOC_EmergencyStop(&foc);
```

## 6. PID参数调节

### 6.1 电流环PID（FOC.c:297-300）
```c
// d轴电流环 (采样周期: 50us)
PID_Init(&foc_ctrl->pid_id, 1.0f, 50.0f, 0.0f, 0.00005f, FOC_VOLTAGE_LIMIT);

// q轴电流环 (采样周期: 50us)
PID_Init(&foc_ctrl->pid_iq, 1.0f, 50.0f, 0.0f, 0.00005f, FOC_VOLTAGE_LIMIT);
```

**调节建议**:
- Kp: 从小到大增加，直到响应迅速但无振荡
- Ki: 消除稳态误差，过大会导致超调
- Kd: 通常为0，或设置很小值减少噪声影响

### 6.2 速度环PID（FOC.c:304）
```c
// 速度环 (采样周期: 1ms)
PID_Init(&foc_ctrl->pid_velocity, 0.5f, 2.0f, 0.01f, 0.001f, FOC_CURRENT_LIMIT);
```

**调节建议**:
- Kp: 0.1 ~ 1.0
- Ki: 1.0 ~ 10.0
- Kd: 0.001 ~ 0.1

## 7. 参数配置

### 7.1 FOC参数（FOC.h）
```c
#define FOC_VOLTAGE_LIMIT       12.0f       // 电源电压限制 (V)
#define FOC_CURRENT_LIMIT       10.0f       // 电流限制 (A)
#define FOC_VELOCITY_LIMIT      1000.0f     // 速度限制 (rad/s)
```

### 7.2 电机参数
- **极对数**: 根据电机规格设置（通常为1-8）
- **供电电压**: 实际直流母线电压
- **编码器线数**: encoder.h中的ENCODER_PPR

## 8. 调试技巧

### 8.1 开环测试
```c
// 禁用闭环控制
foc.enabled = 0;

// 手动设置d/q电压
foc.v_dq.d = 0.0f;
foc.v_dq.q = 3.0f;  // 3V

// 手动设置角度（用于开环转动）
foc.theta_elec += 0.01f;  // 每次增加角度

// 更新PWM
FOC_UpdatePWM(&foc);
```

### 8.2 监控变量
- `foc.i_dq.d` / `foc.i_dq.q`: 当前dq轴电流
- `foc.v_dq.d` / `foc.v_dq.q`: 当前dq轴电压
- `foc.omega_elec`: 电角速度
- `foc.theta_elec`: 电角度

## 9. 注意事项

1. **死区时间**: tim.c:98已设置为100 (约417ns)，根据IR2010S规格可能需要调整
2. **电流限制**: 根据电机和驱动器规格设置合理的电流限制
3. **零点校准**: 启动前必须进行电流传感器零点校准
4. **编码器对齐**: 如果使用增量编码器，需要进行初始电角度对齐
5. **过流保护**: 建议添加硬件过流保护电路

## 10. 常见问题

### Q1: 电机不转或抖动
- 检查编码器方向是否正确
- 检查电流采样是否正常
- 检查极对数设置是否正确
- 尝试降低PID增益

### Q2: 电流环振荡
- 降低电流环Kp和Ki
- 检查死区时间设置
- 检查电流采样滤波

### Q3: 速度响应慢
- 增加速度环Kp
- 增加电流限制（如果安全）
- 检查编码器分辨率
