# 多摩川编码器 RS485 通信笔记

## 1. RS485 基础知识

### 1.1 RS485 总线特点
- **差分信号**：使用 A/B 两根线传输，抗干扰能力强
- **半双工**：同一时刻只能发送或接收，需要方向控制
- **多设备**：一条总线可挂多个设备（最多32个节点）
- **长距离**：最远可达1200米

### 1.2 RS485 收发器引脚
```
MCU TX ──→ DI (Driver Input)     ──→ A/B 差分线
MCU RX ←── RO (Receiver Output)  ←── A/B 差分线
MCU GPIO → DE (Driver Enable)    高电平=发送使能
MCU GPIO → RE (Receiver Enable)  低电平=接收使能
```

**DE 和 RE 通常短接**，用一个 GPIO 控制方向：
- GPIO = HIGH → DE=1, RE=1 → 发送模式（接收禁止）
- GPIO = LOW  → DE=0, RE=0 → 接收模式（发送禁止）

### 1.3 STM32 RS485 两种模式

| 特性 | 硬件RS485模式 | 普通UART+手动GPIO |
|------|-------------|------------------|
| 初始化 | `HAL_RS485Ex_Init()` | `HAL_UART_Init()` |
| DE控制 | 硬件自动（需AF引脚） | GPIO手动控制 |
| RE控制 | 需额外GPIO | 同一个GPIO（DE/RE短接） |
| 灵活性 | 低（受限于AF引脚） | 高 |

**重要教训**：如果 DE/RE 短接且只接了一个 GPIO（如PD4），不要用硬件RS485模式。`HAL_RS485Ex_Init` 会使能 USART 内部的 DE 信号控制，即使 DE 引脚（如PA1）没有物理连接，也会影响 USART 的发送时序，导致通信失败。

## 2. 多摩川协议

### 2.1 命令字节
| Data ID | 命令字节 | 功能 | 响应字节数 |
|---------|---------|------|-----------|
| ID0 | 0x02 | 读取单圈位置 | 6 |
| ID1 | 0x8A | 读取多圈数据 | 6 |
| ID2 | 0x92 | 读取编码器ID | 4 |
| ID3 | 0x1A | 读取单圈+多圈+ID+报警 | 11 |
| ID6 | 0x32 | 写EEPROM | 4 |
| ID7 | 0xBA | 重置错误 | 6 |
| ID8 | 0xC2 | 重置圈数 | 6 |
| IDC | 0x62 | 重置圈数与错误 | 6 |
| IDD | 0xEA | 读EEPROM | 4 |

### 2.2 通信流程
```
主机(MCU)                          编码器
   |                                  |
   |-- [方向切TX] --                  |
   |-- [发送命令字节 0x02] --------→  |
   |-- [方向切RX] --                  |
   |                                  |
   |  ←-------- [CF][SF][ABS0][ABS1][ABS2][CRC]
   |                                  |
   |-- [解析数据] --                  |
```

### 2.3 DATA_ID_0 响应格式（6字节，最常用）
```
Byte 0: CF   - 控制帧
Byte 1: SF   - 状态帧
Byte 2: ABS0 - 位置低8位
Byte 3: ABS1 - 位置中8位
Byte 4: ABS2 - 位置高8位（17bit编码器只用低1位）
Byte 5: CRC  - 校验
```

单圈位置 = `ABS0 | (ABS1 << 8) | (ABS2 << 16)`，范围 0 ~ 131071（17bit）

### 2.4 角度计算
```c
// 机械角度 [0, 2π)
angle_mech = (position / 131072.0f) * 2π

// 电角度 = 机械角度 × 极对数 - 零点偏移
angle_elec = fmodf(angle_mech * pole_pairs, 2π) - zero_offset
```

## 3. STM32H7 配置要点

### 3.1 USART 配置
```c
huart2.Instance = USART2;
huart2.Init.BaudRate = 2500000;        // 多摩川标准 2.5Mbps
huart2.Init.WordLength = UART_WORDLENGTH_8B;
huart2.Init.StopBits = UART_STOPBITS_1;
huart2.Init.Parity = UART_PARITY_NONE;
huart2.Init.Mode = UART_MODE_TX_RX;
huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;  // 不用硬件流控
HAL_UART_Init(&huart2);               // 用普通UART，不用RS485Ex
```

### 3.2 DMA 配置关键
```
DMA RX Mode = DMA_NORMAL    ← 必须！CIRCULAR模式下ReceiveToIdle不触发回调
DMA Priority = DMA_PRIORITY_HIGH
不要配置 DMAMUX Sync（会干扰DMA请求）
```

### 3.3 GPIO 方向控制
```c
// PD4 控制 RS485 方向（DE/RE短接）
#define TAMA_RS485_TX()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET)
#define TAMA_RS485_RX()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET)

// 方向切换前后必须加延时（约50个NOP）
static void Tamagawa_Delay(void) {
    for (volatile uint8_t i = 0; i < 50; i++) { __NOP(); }
}
```

### 3.4 DMA 缓冲区内存放置
STM32H7 的 DMA 不能访问 DTCM RAM（0x20000000），缓冲区必须放在 D2 SRAM：
```c
// Keil
__attribute__((section(".ARM.__at_0x30020200"))) uint8_t tama_tx_buf[4];
__attribute__((section(".ARM.__at_0x30020220"))) uint8_t tama_rx_buf[16];

// GCC
__attribute__((section(".RAM_D2"))) uint8_t tama_tx_buf[4];
__attribute__((section(".RAM_D2"))) uint8_t tama_rx_buf[16];
```

注意不要和其他 DMA 缓冲区（如 CurrentSense）地址冲突。

## 4. 收发实现

### 4.1 发送方式
参考实现采用**逐字节DMA发送**，发完立刻切回接收方向：
```c
void Tamagawa_RequestData(Tamagawa_TypeDef *tama, uint8_t data_id) {
    tama_tx_buf[0] = tx_cmd;

    Tamagawa_Delay();
    TAMA_RS485_TX();           // 切发送
    Tamagawa_Delay();

    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_tx_buf, 4);
    HAL_UART_Transmit_DMA(tama->huart, tama_tx_buf, 1);

    Tamagawa_Delay();
    TAMA_RS485_RX();           // 立刻切回接收
    Tamagawa_Delay();

    tama->rx_flag = 2;         // 标记等待接收
}
```

### 4.2 接收方式
使用 `HAL_UARTEx_ReceiveToIdle_DMA`（IDLE空闲检测），不是 `HAL_UART_Receive_DMA`：
```c
// 初始化时启动持续监听
HAL_UARTEx_ReceiveToIdle_DMA(huart, tama_rx_buf, TAMA_RX_BUF_SIZE);

// 回调函数（IDLE检测到一帧结束时触发）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        // 解析数据
        rx_parse(tama_rx_buf);
        tama->rx_flag = 1;
        // 重新启动监听
        HAL_UARTEx_ReceiveToIdle_DMA(huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
    }
}
```

**为什么用 ReceiveToIdle 而不是 Receive_DMA？**
- `HAL_UART_Receive_DMA` 必须收满指定字节数才触发回调，如果编码器回复字节数不够就永远不触发
- `HAL_UARTEx_ReceiveToIdle_DMA` 检测到总线空闲（IDLE）就触发回调，自动适应不同长度的响应

### 4.3 状态机（20kHz ADC中断中调用）
```
rx_flag=0 (空闲)  → 发送请求 → rx_flag=2 (等待)
rx_flag=2 (等待)  → IDLE回调触发 → rx_flag=1 (收到)
rx_flag=1 (收到)  → 解析角度 → rx_flag=0 (空闲)
```

超时保护：rx_flag=2 持续超过1秒，中止DMA并重置。

## 5. 踩坑记录

### 5.1 HardFault
- **原因**：在20kHz ADC中断里调用DMA发送，USART2的DMA配置不正确（CIRCULAR模式+DMAMUX Sync）
- **解决**：DMA改为NORMAL模式，去掉DMAMUX Sync配置

### 5.2 收不到编码器回复
- **原因**：使用了 `HAL_RS485Ex_Init` 硬件RS485模式，PA1作为DE引脚由硬件控制，但实际硬件DE/RE只接了PD4
- **解决**：改用 `HAL_UART_Init` 普通UART模式，PD4手动控制方向

### 5.3 ReceiveToIdle 回调不触发
- **原因**：DMA RX 配置为 `DMA_CIRCULAR` 模式
- **解决**：改为 `DMA_NORMAL` 模式，IDLE中断才能正确触发 `HAL_UARTEx_RxEventCallback`

### 5.4 DMA传输乱码
- **原因**：STM32H7 DMA 无法访问 DTCM RAM（0x20000000）
- **解决**：DMA缓冲区放在 D2 SRAM（0x30000000 区域），操作前后注意 DCache 清除/无效化

### 5.5 角度波形有阶梯
- **原因**：Tamagawa_Update 在1kHz定时器中调用，通信周期需要2-3ms，角度更新率只有300-500Hz
- **解决**：移到20kHz ADC中断中调用，2.5Mbps下一次通信约40-50us，更新率可达7-10kHz

## 6. 电角度零点校准流程
```
1. 开环施加 d 轴电压（Vd=3V, Vq=0），角度=0
2. 等待转子物理对齐到 d 轴（1.5秒）
3. 读取多摩川编码器当前位置
4. 计算原始电角度 = fmodf(机械角度 × 极对数, 2π)
5. 将此角度记录为零点偏移 elec_zero_offset
6. 后续电角度 = 原始电角度 - elec_zero_offset
```

## 7. ReceiveToIdle + DMA HT �ж����⸴�̣����ιؼ���

### 7.1 ����
- �Ƕ� I0 ��ʱ��Ϊ 0
- rx_flag ��ʱ��ͣ�� 2���ȴ����գ�
- �����뷢�ͼ�����������������ʱ������������
- �ϲ㿴�������лص�����û��Ч֡��

### 7.2 ����
- USART2 ʹ�� HAL_UARTEx_ReceiveToIdle_DMA(...) �հ�
- DMA �İ봫���жϣ�HT��Ҳ�ڴ����ж���·
- �� 11 �ֽڻ����£�HT �����ڵ� 5 �ֽڸ��������������¼���
- �ص��õ����ǲ�����֡������ ID0 ���� 6 �ֽڣ�����������������
- �ص��������� DMA��������һ֡��������ǰ��ϣ��γɡ���Զ�ղ�������֡���ıջ�

### 7.3 �޸�����
��ÿ������ ReceiveToIdle DMA ��ر� HT �жϣ���������������/�����¼���

```c
HAL_UARTEx_ReceiveToIdle_DMA(huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
if (huart->hdmarx != NULL)
{
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
```

���� RxEvent �����Ӷ�֡������
- �� Size < rx_size����������ֱ�ӻص���������

### 7.4 ���֪ʶ��
- ReceiveToIdle �����Ŀ���ǡ��� IDLE �߽���֡���������ǰ� HT ��Ƭ��֡
- �Թ̶�����Э��֡���� Tamagawa ID0=6 �ֽڣ���HT �����İ�֡�¼�ͨ��������
- �ڰ�˫�� RS485 �����������¼�Դ��Ŵ�״̬������������ΪƵ����ʱ

### 7.5 �����ۣ��ɸ��ã�
1. �ȿ�״̬���Ƿ��ڵȴ�̬��rx_flag=2��
2. �ٷֽ⡰����ȥ���� / �յ��¼����� / �����ɹ��������μ���
3. ������� RxEvent ������Ч֡�������ȼ�飺
   - DMA ģʽ�Ƿ� NORMAL
   - �Ƿ���������� HT �ж�
   - �ص����Ƿ�������� DMA
   - ֡���ж��Ƿ��ϸ�ƥ��Э��
4. ����ٿ������㣨�����ʡ�RS485�����л����ն˵��衢����

### 7.6 �� FOC �ֲ�Ĺ�ϵ
- ��������20kHz��ֻʹ�����½Ƕȣ�foc.theta_elec = tamagawa.angle_elec_rad��
- ��Ħ��ͨ��״̬�����ڵ�Ƶ���ȣ��� TIM7 1kHz��
- �����ȱ����ڸ�Ƶ�ж����� UART DMA���ֱ��ֿ��ƻ�ʵʱ�Ժ��ȶ���
