# STM32H7 上电即 HardFault 复盘（多摩川 + FOC 工程）

## 1. 现象与寄存器信息

- 上电后立即进入 `HardFault`
- 打印信息：
  - `PC=0x00000000`
  - `LR=0x00000000`
  - `CFSR=0x00000400`
  - `HFSR=0x40000000`
  - `BFAR=0x00000000`

## 2. 根因结论（前因链）

本次故障的主链路是：

1. 在多摩川初始化中手动开启了 `UART_IT_RXNE`
2. 但当前接收模式实际走的是 `HAL_UARTEx_ReceiveToIdle_DMA(...)`
3. `RXNE` 中断进入 `HAL_UART_IRQHandler()` 时，HAL 期望 `huart->RxISR` 已按接收流程正确配置
4. 该路径下 `RxISR` 可能为空/非法，触发函数指针跳转异常
5. 结果表现为 `PC=0x00000000`，并触发 `INVSTATE`（`CFSR bit10`）

简化理解：**中断源与 HAL 状态机不一致，导致 ISR 间接调用了空指针。**

## 3. 背后的单片机知识点

## 3.1 `CFSR=0x00000400` 代表什么

- `0x00000400` 对应 UsageFault 的 `INVSTATE`
- 常见触发场景：
  - 试图执行非法状态指令（ARM/Thumb 状态不对）
  - 异常返回状态损坏
  - 跳转到了非法地址（常见是函数指针为 `NULL`）

结合 `PC=0`/`LR=0`，本案最吻合的是**空函数指针调用**。

## 3.2 STM32 HAL UART 的中断状态机要点

- `HAL_UART_IRQHandler()` 不是“裸寄存器中断处理”，它依赖 `UART_HandleTypeDef` 内部状态机
- 不同 API（阻塞、IT、DMA、ReceiveToIdle）会设置不同的内部回调入口
- 如果手动开中断位（如 `RXNEIE`），却不走对应 HAL 接收启动流程，就可能出现“中断来了但回调入口未准备好”的崩溃

## 3.3 Cortex-M7 + Cache + DMA 注意点

- DMA 缓冲区建议放在 DMA 可访问 SRAM（你当前放 D2 是正确方向）
- 需要做 DCache 维护（`Invalidate/CleanInvalidate`），避免 CPU 读到旧数据
- 回调中解析协议时必须有长度检查，否则很容易出现越界读写，导致随机异常

## 3.4 嵌入式内存管理核心知识（结合本工程）

### 3.4.1 栈（Stack）管理

- 栈用于函数局部变量、函数调用现场、异常现场压栈
- 栈溢出常导致“看起来随机”的 HardFault，常伴随回溯失败
- 中断中再调用重函数（如格式化打印）会放大栈压力

结合本工程：

- `HardFault_Handler` 中使用 `char buf[128] + snprintf`，若栈已损坏可能二次异常
- `Tamagawa_CRC` 之前的 `uint8_t buf[12] + memcpy(len)` 属于典型“局部数组越界”，会直接破坏栈

### 3.4.2 堆（Heap）管理

- 堆由 `malloc/free` 或 RTOS 内存管理器维护
- 风险：
  - 内存碎片（长期运行后分配失败）
  - 双重释放/野指针
  - 多线程并发分配导致时序问题
- 电机控制类实时系统通常优先“静态分配”，减少堆不确定性

结合本工程：

- 目前主要用静态/全局对象（这是好事）
- 但项目包含 FreeRTOS，若后续启用任务并大量动态创建对象，应优先固定池化或启动期一次性分配

### 3.4.3 全局/静态区（`.bss/.data`）管理

- 生命周期贯穿全程，适合驱动状态机、DMA缓冲、控制环状态变量
- 需要关注：
  - 占用是否超过目标 RAM 区域
  - 链接地址是否与外设访问能力匹配

结合本工程（从 map 可见）：

- `current_sense` 位于 `0x30020000`
- `tama_tx_buf` 位于 `0x30020200`
- `tama_rx_buf` 位于 `0x30020220`

这说明你已在按“DMA 可访问 RAM”放置关键缓冲，方向正确。

### 3.4.4 MPU / 内存保护

- MPU 可把非法访问直接转成 fault，帮助更早暴露 bug
- 你当前配置了“背景拒绝区”（no-access），这是有效防御
- 但 MPU 只是防护网，不能替代边界检查；越界写仍可能先破坏同区数据再晚些崩溃

### 3.4.5 Cache 一致性（M7 特有重点）

- DMA 不走 CPU DCache，CPU 可能读到旧缓存
- 典型规则：
  - DMA写内存、CPU读前：`Invalidate`
  - CPU写内存、DMA读前：`Clean` 或 `CleanInvalidate`
- 地址与长度按 cache line 处理，避免遗漏

你在多摩川 DMA 缓冲上做了 cache 维护，这是正确操作。

### 3.4.6 中断与内存安全

- ISR 中应避免：
  - 不受控 `memcpy`
  - 未检查长度直接解析帧
  - 与主循环并发访问共享变量且无一致性策略
- 对共享状态（如 `rx_flag`、角度值）至少保证单写单读路径清晰，必要时用 `volatile` 或临界区

### 3.4.7 实时系统内存管理实践建议

1. 控制环、驱动环、DMA缓冲全部静态化  
2. 协议栈统一“长度先行”模板：先判长再解包  
3. 禁止在高频ISR里做复杂格式化和大对象栈分配  
4. 对关键缓冲增加编译期检查（大小、对齐）  
5. 周期性监测任务/中断栈水位（若启用RTOS）  
6. 故障处理保留最小现场，减少二次故障概率

## 4. 本次修复点

## 4.1 去掉 RXNE 手动使能（核心修复）

文件：`Core/Src/tamagawa.c`

### 修复前

```c
/* 使能RXNE中断 */
__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

/* 启动ReceiveToIdle DMA持续监听 */
memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
HAL_UARTEx_ReceiveToIdle_DMA(huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
```

### 修复后

```c
/* 使用 ReceiveToIdle DMA 持续监听，不手动开启 RXNE 中断。
 * 手动开 RXNE 但未配置 huart->RxISR 会在 HAL_UART_IRQHandler 中触发空指针调用。 */
memset(tama_rx_buf, 0, TAMA_RX_BUF_SIZE);
SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tama_rx_buf, TAMA_RX_BUF_SIZE);
HAL_UARTEx_ReceiveToIdle_DMA(huart, tama_rx_buf, TAMA_RX_BUF_SIZE);
```

## 4.2 接收解析长度保护（内存安全）

文件：`Core/Src/tamagawa.c`

### 修复前

```c
if (Size > 0)
{
    Tamagawa_RxParse(tama, tama_rx_buf);
    tama->rx_flag = 1;
}
```

### 修复后

```c
if (Size >= tama->rx_size && tama->rx_size > 0)
{
    Tamagawa_RxParse(tama, tama_rx_buf);
    tama->rx_flag = 1;
}
```

## 4.3 CRC 计算去除潜在栈溢出

文件：`Core/Src/tamagawa.c`

### 修复前

```c
uint8_t buf[12];
memcpy(buf, data, len);
...
uint8_t val = (buf[i] >> 7) ^ (crc >> 7);
buf[i] <<= 1;
```

### 修复后

```c
for (i = 0; i < len; i++)
{
    uint8_t in = data[i];
    for (j = 0; j < 8; j++)
    {
        uint8_t val = (in >> 7) ^ (crc >> 7);
        crc <<= 1;
        in <<= 1;
        crc |= val;
    }
}
```

收益：不再依赖固定长度临时数组，避免 `len` 变化导致越界。

## 4.4 串口数据包解析修复非法指针与边界越界

文件：`Core/Src/usart.c`

### 修复前

```c
if (buf[i]==0xAA && buf[i+1]==0x55) {
    Data_frame_t *package=(Data_frame_t*)buf[i];
    ...
}
```

问题：

- `buf[i+1]` 可能越界
- `(Data_frame_t*)buf[i]` 是把“单字节数值”当地址，属于严重非法指针

### 修复后

```c
if ((i + 1) < len && buf[i]==0xAA && buf[i+1]==0x55) {
    if ((i + (int)sizeof(Data_frame_t)) > len) {
        break;
    }
    Data_frame_t *package=(Data_frame_t*)&buf[i];
    ...
}
```

## 5. 为什么现在上电不再 HardFault

- 移除了 `RXNE` 与 `ReceiveToIdle DMA` 的冲突配置
- 避免了 HAL UART 中断路径进入未准备好的接收回调入口
- 同时修复了协议解析和 CRC 的内存风险点，降低了随机崩溃概率

## 6. 建议的长期防护

1. 保持“单一路径驱动 UART 接收”：要么 IT，要么 DMA+IDLE，不要混开关键中断位  
2. 所有协议解析函数统一做长度前置检查  
3. HardFault 中尽量减少复杂库函数调用（如 `snprintf`），防止二次故障  
4. 给关键 ISR 路径加断言/状态检查（例如 `huart->RxState`）  
5. 持续用 `PC/LR/CFSR/HFSR` 建立故障模式库，后续定位会更快

---

如果后面你希望，我可以再补一版“带时序图”的文档，把 `USART2 IRQ -> HAL_UART_IRQHandler -> RxISR` 的调用链画出来，方便新人快速理解。 
