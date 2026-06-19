# EtherCAT 初学者开发文档草稿（触发版）

## 0. 背景与触发

- 触发时间（UTC）：`2026-04-25T09:17:58.536Z`
- 触发时间（Asia/Shanghai）：`2026-04-25 17:17:58`
- 触发依据：
  - 检测到仓库根目录存在 `DOC_READY.flag`
  - 最近相关提交中包含 `doc-ready` / `完成` 关键词

最近相关提交（按时间倒序）：
1. `003f38c` `2026-04-17 15:08:29 +0800` `fix(modbus): restore TEST1/DriverStudio register compatibility`
2. `3037770` `2026-04-17 12:53:20 +0800` `doc-ready 完成信号: 代码收尾已就绪`
3. `ccadb17` `2026-04-17 12:56:56 +0800` `fix(ecat): unify recovery state and reduce sync0 ISR load`

## 1. 初学者先建立的整体图景

你可以先把当前工程理解为三层：

1. 协议层
`comm_modbus.c` 与 EtherCAT SSC 负责收发、状态机、协议合法性处理。

2. 对象映射层
`comm_od.c`/`comm_od_core.c` 把“寄存器/对象字典地址”翻译成“控制对象字段”。

3. 控制执行层
`axis`/FOC 完成真正的电机控制与反馈更新。

学习主线建议：先看“数据怎么流”，再看“状态怎么转”，最后看“实时性怎么保证”。

## 2. EtherCAT 最小必学知识（面向移植）

1. ESM 状态机：`INIT -> PRE-OP -> SAFE-OP -> OP`
2. CoE：通过对象字典（Index/SubIndex）访问变量
3. PDO：周期实时数据通道（命令与反馈）
4. SDO：参数配置通道（非实时）
5. DC/SYNC0：做多轴同步和周期对齐

一句话记忆：**PDO 跑实时控制，SDO 跑参数管理，DC 保证同步节拍。**

## 3. 当前代码阅读顺序（新手友好）

1. `Core/Src/main.c`：看系统初始化与循环入口
2. `app/app_comm.c`：看 Modbus 与 EtherCAT 周期调度
3. `comm/protocol/comm_ecat_if.c`：看 EtherCAT 桥接逻辑
4. `comm/protocol/comm_od.c`：看寄存器与控制变量映射
5. `comm/protocol/comm_modbus.c`：看 RTU 帧解析、CRC、异常响应
6. `comm/Eth_stack/*`：最后看 SSC 细节（先宏观后细节）

## 4. 对象字典引入的设计思路（核心）

对象字典不是“表格而已”，它是协议层与控制层之间的隔离层：

1. 协议可以换（Modbus/EtherCAT），控制逻辑不用重写
2. 地址稳定（寄存器或Index稳定），实现可迭代
3. 可做权限与边界控制（只读、读写、非法地址异常）

你后续移植时要坚持：
**协议层只编解码，业务层只控制，映射层负责翻译。**

## 5. 下一步文档扩展计划

接下来按以下结构继续扩写：

1. 逐函数讲解 `comm_ecat_if.c`（含时序图）
2. 逐寄存器讲解 `comm_od.c`（含 Modbus/EtherCAT 对照表）
3. 典型联调故障树（进不了 OP、OP 掉线、PDO 有流量但轴不动）
4. 移植检查清单（硬件、时钟、SPI、中断、同步、对象映射）
5. 回归测试矩阵（功能、异常、实时性、恢复能力）
