# h7FOC 文档索引

本目录集中放置 h7FOC 的调试记录、联调手册和协议移植说明。根目录 README 负责介绍项目，本目录提供各模块的详细技术文档。

## 推荐阅读顺序

1. [根目录 README](../README.md)：先看项目定位、硬件组成、代码架构和实时控制流程。
2. [FOC 使用说明](FOC_Usage.md)：理解 20 kHz 电流环、1 kHz 速度环和 PID 调参入口。
3. [电机封装架构说明](Motor_Encapsulation.md)：看 `axis_t` / `foc_t` / BSP 的分层抽象思路。
4. [电机不转排查指南](DEBUG_INSTRUCTIONS.md)：了解 ADC 中断、FOC 使能、PWM 和电流采样的排查方法。
5. [HardFault 复盘](HardFault_问题复盘.md)：了解 Cortex-M7、HAL 状态机、DMA/Cache 和内存安全问题。
6. [Modbus 联调手册](Modbus_TEST1一致性核对与联调手册.md) 和 [EtherCAT 移植手册](EtherCAT_开发移植学习手册.md)：了解工业通信协议的联调过程。

## FOC 与控制架构

| 文档 | 内容 |
| --- | --- |
| [FOC_Usage.md](FOC_Usage.md) | FOC 初始化、电流环、速度环、控制接口、PID 调参和常见问题 |
| [Motor_Encapsulation.md](Motor_Encapsulation.md) | 电机对象封装、硬件抽象、双电机/多电机扩展思路 |
| [FOC_DEBUG_GUIDE.md](FOC_DEBUG_GUIDE.md) | FOC 调试过程、关键变量和观测方式 |

## 调试与故障复盘

| 文档 | 内容 |
| --- | --- |
| [DEBUG_INSTRUCTIONS.md](DEBUG_INSTRUCTIONS.md) | 电机不转时，从 ADC 中断、FOC enable、PWM、电流采样逐步定位 |
| [HardFault_问题复盘.md](HardFault_问题复盘.md) | 上电 HardFault 的寄存器分析、UART DMA 根因链、内存和 Cache 知识点 |
| [修改记录.md](修改记录.md) | 项目阶段性修改记录 |

## 通信与联调

| 文档 | 内容 |
| --- | --- |
| [Modbus_TEST1一致性核对与联调手册.md](Modbus_TEST1一致性核对与联调手册.md) | Modbus TEST1/DriverStudio 兼容性和寄存器一致性核对 |
| [Modbus_串口逐帧验收单.md](Modbus_串口逐帧验收单.md) | Modbus 串口逐帧验收记录 |
| [EtherCAT_开发移植学习手册.md](EtherCAT_开发移植学习手册.md) | LAN9252 EtherCAT 从站移植、CiA402、PDO/SDO 联调路线 |
| [EtherCAT_初学者开发文档草稿_触发版.md](EtherCAT_初学者开发文档草稿_触发版.md) | 面向初学者的 EtherCAT 开发笔记草稿 |

## 文档维护约定

- 调试、复盘和联调资料统一放在 `docs/`。
- 根目录只保留 `README.md`、构建配置、工程配置和源码目录。
- 新增调试文档时同步更新本索引和根目录 README 的“项目文档导航”区域。
