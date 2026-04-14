# usart_task - 串口调试任务

[[00-项目总览]] | [[DT35模拟传感器]] | [[AS5047P磁编码器]]

---

## 任务概述

串口任务（`usart_task.cpp`）以 **1ms 周期**运行，负责：
- 初始化 AS5047P 磁编码器（SPI1）
- 初始化 DT35 距离传感器（SPI3+ADS8688）
- 每帧通过 LoRa（USART10）发送调试数据到上位机

## 初始化

```cpp
as5047.init(&hspi1);   // 磁编码器挂载 SPI1
dt35.init(&hspi3);     // DT35/ADS8688 挂载 SPI3
```

## 主循环（每 1ms）

```
组装调试帧：
  data[0] = dt35.ch0.voltage_V     // CH0 电压
  data[1] = dt35.ch1.voltage_V     // CH1 电压
  data[2] = dt35.ch0.distance_mm   // CH0 距离
  data[3] = dt35.ch1.distance_mm   // CH1 距离

SendCurveArray_Float_LORA(data, 4)  // DMA 发送至 huart10（LoRa）
as5047.updata()                      // 刷新编码器角度
dt35.update()                        // 刷新 DT35 双通道
```

## 调试帧格式（LoRa）

```
帧结构：[float×N 字节] [END_0 END_1 END_2 END_3]
        float 按原始 4 字节发送，小端序
        结束标志：0x00 0x00 0x80 0x7F（特殊 float NaN 标记）

发送通道：HAL_UART_Transmit_DMA(&huart10, ...) — 非阻塞 DMA
```

## 串口帧解析（上位机→STM32 方向）

### 通用 S/E 帧格式（`parse_SE_simple`）
```
帧格式：S<f0>,<f1>,...,<fn>E
字段：逗号分隔的 ASCII 浮点数
用途：通用调试指令帧
```

### 视觉帧格式（`parse_vision_frame_computer`）
```
帧格式：S,<x_diff>,<y_diff>,<angle_x>E
字段：
  x_diff   — 目标 X 方向偏差
  y_diff   — 目标 Y 方向偏差
  angle_x  — 目标角度偏差
用途：视觉对接时接收上位机识别结果
```

## 模块功能汇总

| 功能 | 接口 | 通道 |
|------|------|------|
| 调试数据发送 | `SendCurveArray_Float_LORA` | USART10（LoRa）DMA |
| 上位机数据发送 | `SendCurveArray_Float` | USB CDC（MiniPC） |
| 通用串口帧解析 | `parse_SE_simple` | 任意串口缓冲区 |
| 视觉帧解析 | `parse_vision_frame_computer` | 任意串口缓冲区 |
| 磁编码器刷新 | `as5047.updata()` | SPI1 |
| DT35 刷新 | `dt35.update()` | SPI3 |

## 相关文件

- `MDK-ARM/TASK/usart_task.cpp`
- `MDK-ARM/TASK/usart_task.h`

## 参见

- [[DT35模拟传感器]] — ADS8688 SPI采样与距离换算
- [[AS5047P磁编码器]] — 14位磁编码器SPI读取
- [[SPI外设]] — SPI1 / SPI3 外设配置
