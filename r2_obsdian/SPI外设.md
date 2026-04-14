# SPI外设

[[00-项目总览]] | [[DT35模拟传感器]] | [[AS5047P磁编码器]]

---

## 模块概述

STM32H723 的 SPI 外设由 CubeMX 生成初始化代码，工程使用两路 SPI：
- **SPI1**：AS5047P 磁编码器
- **SPI3**：DT35 激光位移传感器（ADS8688 ADC）

## SPI1 — AS5047P 磁编码器

| 参数 | 值 |
|------|----|
| 模式 | Master，全双工 |
| 帧格式 | Motorola，16位 |
| CPOL/CPHA | 1/1（Mode 3） |
| 片选 | GPIOE_15（软件控制） |
| 数据线 | MOSI/MISO/SCK |

SPI 每次传输 16 位，采用流水线读取（发命令，下帧读结果）。

## SPI3 — ADS8688 (DT35)

| 参数 | 值 |
|------|----|
| 模式 | Master，全双工 |
| 帧格式 | Motorola，16位 |
| CPOL/CPHA | 0/0（Mode 0） |
| 片选（CS） | GPIOE_14（软件控制） |
| 复位（RST） | GPIOB_8（软件控制） |

ADS8688 命令帧 16 位，数据帧 16 位（ADC结果），同样流水线操作。

## 注意事项

- 两个外设共享 SPI 硬件不同通道，互不影响
- CS 引脚均为软件控制（GPIO），每次事务前手动拉低/拉高
- AS5047P 需要在 CS 拉低期间完整传输 16 位；CS 边沿之间有最小建立时间要求
- ADS8688 复位引脚（RST）在 `DT35::ADS8688_Init()` 中先拉低后拉高，完成硬复位

## 相关文件

- `R2_Lift_auto/Core/Src/spi.c`
- `R2_Lift_auto/Core/Inc/spi.h`
- `MDK-ARM/Device/AS5047P/AS5047.cpp` — SPI1 使用
- `MDK-ARM/Device/DT35/DT35.cpp` — SPI3 使用

## 参见

- [[DT35模拟传感器]] — SPI3 + ADS8688 应用
- [[AS5047P磁编码器]] — SPI1 应用
