# FOC 调试指南：速度环、位置环与上位机联调

## 调试路线总览

```
电流环确认 → 速度环调试 → 位置环调试 → 上位机联调（Modbus RTU）
```

每一阶段必须在前一阶段稳定后再进入下一阶段。级联控制的外环依赖内环的正确性。

---

## 第一阶段：确认电流环正常

### 目的

速度环和位置环都建立在电流环之上，电流环不稳一切白搭。

### VOFA+ 通道分配建议

修改 `app/app_comm.c` 中的 `comm_vofa_send_float` 调用：

```c
comm_vofa_send_float(
    g_axis0.foc.theta_elec,        // ch0: 电角度
    g_axis0.foc.i_dq.d,            // ch1: d 轴电流（应趋近 0）
    g_axis0.foc.i_dq.q,            // ch2: q 轴电流（应跟踪目标）
    g_axis0.foc.target_iq,         // ch3: q 轴目标
    g_axis0.foc.v_dq.d,            // ch4: d 轴电压输出
    g_axis0.foc.v_dq.q,            // ch5: q 轴电压输出
    g_axis0.foc.i_abc.ia,          // ch6: A 相原始电流
    g_axis0.speed_mech_rad_s       // ch7: 机械角速度
);
```

### 验证方法

1. 设置模式为 `AXIS_MODE_CURRENT`
2. 给一个小的 `target_iq`（0.05A）
3. 使能电机

### 合格标准

| 指标 | 期望 |
|------|------|
| `i_dq.q` | 快速跟踪 `target_iq`，无振荡 |
| `i_dq.d` | 在 0 附近，幅值 < 0.1A |
| `theta_elec` | 平滑连续，无跳变 |
| `Ia` | 干净正弦波 |

### 电流环 PID 参数（当前默认值）

```
Kp = 1.2
Ki = 60.0
Kd = 0.0
Ts = 50μs (20kHz)
输出限幅 = 24V (FOC_VOLTAGE_LIMIT)
```

### 常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| 电流波形有毛刺 | ADC 采样噪声 | 检查滤波系数，确认零点校准 |
| `i_dq.d` 偏大 | 电角度不准 | 检查编码器零点校准 |
| 电流不跟踪 | PID 参数不合适 | 先降 Ki 到 20，观察响应 |
| HardFault | FPU 未初始化 | 确认 main.c 中 SCB->CPACR 设置 |

---

## 第二阶段：速度环调试

### 控制结构

```
目标速度 → [速度 PID] → target_iq → [电流环] → 电压输出
                ↑
          实际电角速度 (omega_elec)
```

### 调试步骤

1. 设置模式为 `AXIS_MODE_VELOCITY`
2. 给一个小的目标速度（机械角速度 × pole_pairs = 电角速度）
3. 使能电机
4. 用 VOFA+ 观察 `omega_elec` vs `target_velocity`

### 速度环 PID 调参方法

**起步参数：**

```
Kp = 0.3
Ki = 0.5
Kd = 0.0
Ts = 1ms (1kHz)
输出限幅 = 10A (FOC_CURRENT_LIMIT)
```

**调参顺序：**

```
第1步: Ki=0, Kd=0, 只调 Kp
       → 电机能转起来，有稳态误差是正常的
       → 振荡就减 Kp

第2步: 固定 Kp，慢慢加 Ki
       → 消除稳态误差
       → 低频振荡就减 Ki

第3步: 一般不需要 Kd
       → 如果响应需要更快，可以加很小的 Kd (0.001~0.01)
       → 高频噪声就去掉 Kd
```

### VOFA+ 观察重点

```
ch0: target_velocity   — 目标速度（阶跃信号）
ch1: omega_elec        — 实际电角速度
ch2: target_iq         — 速度环输出（电流指令）
ch3: i_dq.q            — 实际 q 轴电流
```

### 合格标准

| 指标 | 期望 |
|------|------|
| 上升时间 | < 50ms |
| 超调量 | < 10% |
| 稳态误差 | < 1% |
| 无持续振荡 | |

---

## 第三阶段：位置环调试

### 控制结构

```
目标位置 → [位置 PID] → 速度指令 → [速度环] → [电流环] → 电压输出
                ↑
          实际机械角度 (position_mech_rad)
```

### 调试步骤

1. 设置模式为 `AXIS_MODE_POSITION`
2. 写目标位置
3. 触发运动（`axis_request_move`）
4. 观察 `position_mech_rad` 跟踪 `target_position`

### 位置环 PID 参数（当前默认值）

```
Kp = 12.0
Ki = 0.01
Kd = 0.0
Ts = 1ms (1kHz)
输出限幅 = FOC_VELOCITY_LIMIT / pole_pairs
```

### 调参方法

```
超调大   → 减 Kp（比如 8.0）
到位慢   → 加 Kp（比如 15.0）
到位后抖 → 减 Kp，检查编码器分辨率
有稳态偏差 → 加 Ki（比如 0.05）
```

### 到位判断

`axis.c` 中的到位条件：

```c
if ((fabsf(position_error) <= 0.01f) && (fabsf(speed_mech_rad_s) <= 0.01f))
{
    move_done = 1;
}
```

位置误差 < 0.01 rad 且速度 < 0.01 rad/s 时判定到位。

---

## 第四阶段：上位机联调（Modbus RTU）

### 通信参数

| 参数 | 值 |
|------|-----|
| 接口 | UART3 (RS485) |
| 波特率 | 与 CubeMX 配置一致 |
| 协议 | Modbus RTU |
| 从站地址 | 0x01 |
| 功能码 | 0x03, 0x04, 0x06, 0x10 |

### 寄存器地图

#### 输入寄存器（只读，功能码 0x04）

| 地址 | 类型 | 说明 |
|------|------|------|
| 0x0001 | uint16 | move_done 标志 |
| 0x0101-0x0102 | float | 实际位置 (rad) |
| 0x0103-0x0104 | float | 实际速度 (rad/s) |
| 0x0105-0x0106 | float | 实际电流 Iq (A) |

#### 保持寄存器（读写，功能码 0x03/0x06/0x10）

| 地址 | 类型 | 说明 |
|------|------|------|
| 0x0201 | uint16 | LED 控制 |
| 0x0202 | uint16 | 电机使能 (1=使能, 0=禁用) |
| 0x0203 | uint16 | 运动触发 (1=开始, 0=停止) |
| 0x0301-0x0302 | float | 目标位置 (rad) |
| 0x0303-0x0304 | float | 目标速度 (rad/s) |
| 0x0311-0x0312 | float | 速度环 Kp |
| 0x0313-0x0314 | float | 速度环 Ki |
| 0x0315-0x0316 | float | 位置环 Kp |
| 0x0317-0x0318 | float | 位置环 Ki |
| 0x0319-0x031A | float | Iq 环 Kp |
| 0x031B-0x031C | float | Iq 环 Ki |
| 0x031D-0x031E | float | Id 环 Kp |
| 0x031F-0x0320 | float | Id 环 Ki |
| 0x0321 | uint16 | 控制模式 (0=禁用,1=电流,2=速度,3=位置) |
| 0x0322 | uint16 | move_flag |
| 0x0323-0x0324 | float | Iq 目标 (A) |

### Float 编码格式

float 占 2 个连续寄存器，大端字节序：

```
寄存器 N   = float 的高 16 位
寄存器 N+1 = float 的低 16 位
```

写 float 使用功能码 0x10（写多个寄存器），quantity = 2。

### 联调操作流程

#### 1. 验证通信链路

```
上位机发: 01 03 03 21 00 01 [CRC]    ← 读控制模式寄存器
固件回:   01 03 02 00 00 [CRC]        ← 返回模式值 0 (DISABLED)
```

能收到正确回复说明通信链路通了。

#### 2. 电流控制

```
步骤1: 写模式 → 0x0321 = 1 (CURRENT)
       01 06 03 21 00 01 [CRC]

步骤2: 写 Iq 目标 → 0x0323 = 0.05 (float)
       01 10 03 23 00 02 04 [4字节float] [CRC]

步骤3: 使能电机 → 0x0202 = 1
       01 06 02 02 00 01 [CRC]

步骤4: 读实际电流 → 0x0105 (input reg)
       01 04 01 05 00 02 [CRC]
```

#### 3. 速度控制

```
步骤1: 写模式 → 0x0321 = 2 (VELOCITY)
步骤2: 写目标速度 → 0x0303 = float(target_speed)
步骤3: 使能电机 → 0x0202 = 1
步骤4: 周期读速度 → 0x0103
```

#### 4. 位置控制

```
步骤1: 写模式 → 0x0321 = 3 (POSITION)
步骤2: 写目标位置 → 0x0301 = float(target_pos)
步骤3: 使能电机 → 0x0202 = 1
步骤4: 触发运动 → 0x0203 = 1
步骤5: 轮询 move_done → 0x0001
步骤6: 读实际位置 → 0x0101
```

#### 5. 在线调 PID

```
写速度环 Kp: 01 10 03 11 00 02 04 [float bytes] [CRC]
写速度环 Ki: 01 10 03 13 00 02 04 [float bytes] [CRC]
写位置环 Kp: 01 10 03 15 00 02 04 [float bytes] [CRC]
写 Iq 环 Kp: 01 10 03 19 00 02 04 [float bytes] [CRC]
```

修改后立即生效，无需重启。

### 注意事项

1. **操作顺序**：先写模式寄存器 `0x0321`，再写目标值，最后使能。写目标值不会自动切模式。
2. **float 字节序**：与 TEST1 一致，高字在前。
3. **CRC**：标准 Modbus RTU 低字节在前，与 TEST1 和 Qt 上位机兼容。
4. **使能前确保校准完成**：电流零点校准在 `App_AxisInit()` 中自动执行（2000 个样本）。

---

## VOFA+ 实时调试

### 连接

- 接口：UART4
- 波特率：与 CubeMX 配置一致
- 协议：JustFloat（8 通道 float + 尾部 `00 00 80 7F`）
- 刷新率：500Hz

### 推荐通道配置

#### 电流环调试

```c
ch0: theta_elec          ch4: v_dq.d
ch1: i_dq.d              ch5: v_dq.q
ch2: i_dq.q              ch6: Ia
ch3: target_iq            ch7: speed_mech_rad_s
```

#### 速度环调试

```c
ch0: target_velocity      ch4: i_dq.q
ch1: omega_elec           ch5: i_dq.d
ch2: target_iq            ch6: position_mech_rad
ch3: speed_mech_rad_s     ch7: theta_elec
```

#### 位置环调试

```c
ch0: target_position      ch4: target_velocity (速度环输出)
ch1: position_mech_rad    ch5: omega_elec
ch2: speed_mech_rad_s     ch6: i_dq.q
ch3: move_done            ch7: target_iq
```

---

## 快速排障

| 现象 | 检查点 |
|------|--------|
| 电机不转 | 1. PWM 输出是否启动 2. FOC enabled 标志 3. ADC 中断是否进入 |
| 电机抖动 | 1. 电角度是否正确 2. 编码器零点 3. 电流环 PID |
| 速度不稳 | 1. 速度反馈是否平滑 2. 速度环 Ki 是否太大 3. 编码器分辨率 |
| 位置超调 | 1. 位置环 Kp 太大 2. 速度环响应太慢 |
| Modbus 无回复 | 1. UART3 波特率 2. RS485 方向引脚 3. DMA 缓存地址 |
| Modbus CRC 错误 | 1. 确认上位机用标准 Modbus RTU CRC 2. 检查帧长度 |
| float 值乱码 | 1. 确认 2 寄存器读写 2. 检查字节序（高字在前） |
