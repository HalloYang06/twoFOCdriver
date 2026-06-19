# FOC调试指南 - 定位电机不转问题

## 当前问题
- 电机不转，没有任何反应
- 校准时电机能动
- 怀疑卡在ADC DMA中断回调

## 调试步骤

### 第1步：验证ADC中断是否正常进入

在 `stm32h7xx_it.c` 的 `HAL_ADC_ConvCpltCallback` 开头添加LED闪烁：

```c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    static uint32_t callback_count = 0;
    callback_count++;

    // 每1000次闪烁LED1 - 验证中断是否进入
    if (callback_count % 1000 == 0) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }

    // ... 后续代码
}
```

**预期结果**：LED1应该闪烁（约1Hz）
- 如果不闪，说明ADC中断没进入 → 检查ADC/TIM1配置
- 如果闪烁，说明中断正常 → 继续下一步

### 第2步：验证FOC是否启用

在 `if (foc.enabled)` 内部添加LED2闪烁：

```c
if (foc.enabled)
{
    static uint32_t foc_count = 0;
    foc_count++;
    if (foc_count % 1000 == 0) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);  // LED2表示FOC正在运行
    }

    // ... FOC控制代码
}
```

**预期结果**：LED2应该闪烁
- 如果不闪，说明 `foc.enabled = 0` → 检查main.c中是否调用FOC_Enable
- 如果闪烁，说明FOC在运行 → 继续下一步

### 第3步：检查PWM输出

在PWM更新后添加验证：

```c
/* 2.5 更新PWM */
PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, (uint32_t)(foc.duty_a * FOC_PWM_PERIOD));
PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, (uint32_t)(foc.duty_b * FOC_PWM_PERIOD));
PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, (uint32_t)(foc.duty_c * FOC_PWM_PERIOD));

// 验证PWM值不为0
static uint32_t pwm_check = 0;
pwm_check++;
if (pwm_check % 5000 == 0) {
    if (foc.duty_a == 0 && foc.duty_b == 0 && foc.duty_c == 0) {
        // 所有PWM都为0，电机当然不转！
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);  // LED2常亮表示PWM全为0
    }
}
```

### 第4步：检查电流采样

添加电流值检查：

```c
/* 2.2 获取三相电流 */
PhaseCurrents_TypeDef i_abc;
CurrentSense_GetCurrents(&current_sense, &i_abc.Ia, &i_abc.Ib, &i_abc.Ic);

// 检查电流是否异常
static uint32_t current_check = 0;
current_check++;
if (current_check % 5000 == 0) {
    if (i_abc.Ia > 5.0f || i_abc.Ib > 5.0f || i_abc.Ic > 5.0f) {
        // 电流过大！可能损坏硬件
        FOC_Disable(&foc);
        while(1) {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            HAL_Delay(100);  // 快速闪烁报警
        }
    }
}
```

## 快速测试代码

我建议你先运行这个简化的测试代码，验证基础功能：

