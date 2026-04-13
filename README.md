  # yezi_R2

  基于 STM32H723VGTx + FreeRTOS，主要用于 R2 底盘、升
  降机构、传感器采集、CAN 电机驱动、串口通信和路线状态机控制。

  ## 项目概览

  本工程由 STM32CubeMX 生成底层外设初始化代码，使用 Keil MDK / EIDE 进行开发。核
  心业务代码集中在 `R2_Lift_auto/MDK-ARM` 下，底层 HAL、CMSIS、FreeRTOS、USB
  Device 等依赖已随工程一起保存。

  当前主要包含：

  - 四轮全向底盘运动学、动力学解算与 PID 闭环控制
  - DJI M3508 / M2006 电机 CAN 控制
  - 云深 J60 电机控制
  - VESC 电调 FDCAN 扩展帧驱动
  - 升降机构高度轨迹控制与半自动上台阶逻辑
  - DT35 激光测距传感器 + ADS8688 ADC 采集
  - AS5047P 磁编码器 SPI 读取
  - SBUS 遥控器、DM IMU、激光测距串口 DMA 接收
  - USB CDC / MiniPC / LoRa 调试通信
  - R2 路线规划与动作状态机框架

  ## 硬件平台

  - MCU：STM32H723VGTx
  - RTOS：FreeRTOS CMSIS-RTOS V2
  - 主频：480 MHz
  - CAN：FDCAN1 / FDCAN2 / FDCAN3
  - SPI：SPI1、SPI3
  - UART：UART5、UART7、UART8、USART1、USART2、USART3、USART10
  - USB：USB Device CDC

  ## 目录结构

  ```text
  yezi_R2/
  ├── README.md
  ├── R2_Lift_auto/
  │   ├── My_Princess_cpp.ioc          # STM32CubeMX 工程
  │   ├── Core/                        # CubeMX 生成的主程序、外设初始化、
  FreeRTOS 入口
  │   ├── Drivers/                     # STM32 HAL / CMSIS
  │   ├── Middlewares/                 # FreeRTOS、USB Device、CMSIS-DSP
  │   ├── USB_DEVICE/                  # USB CDC 设备层
  │   ├── MDK-ARM/
  │   │   ├── My_Princess_cpp.uvprojx  # Keil MDK 工程
  │   │   ├── Bsp/                     # CAN、USART、DWT 等板级支持
  │   │   ├── Control/                 # PID、全向底盘、路线规划
  │   │   ├── Device/                  # 电机、传感器、遥控器、USB/MiniPC 驱动
  │   │   └── TASK/                    # FreeRTOS 任务实现
  │   └── EIDE_Project/                # EIDE 工程配置
  └── 说明文档/
      ├── DT35_ADS8688_使用文档.md
      └── VescMotor 驱动使用文档.md

  ## 主要任务

  工程中 FreeRTOS 创建了 4 个主要任务：

  | 任务 | 周期/用途 |
  |------|-----------|
  | CHASSIS_TASK | 底盘控制、遥控器指令更新、底盘运动学解算、电机电流发送 |
  | USART_TASK | DT35 / AS5047P 数据更新，LoRa 调试数据发送 |
  | LIFT_TASK | 升降机构高度控制、2006 升降轮速度控制、半自动上台阶状态机 |
  | PLAN_ROUTE | R2 路线规划与动作状态机周期更新 |

  ## 核心模块

  ### 底盘控制

  底盘控制代码位于：

  R2_Lift_auto/MDK-ARM/TASK/chassis_task.cpp
  R2_Lift_auto/MDK-ARM/Control/omni_chassis.cpp

  功能包括：

  - 读取遥控器底盘速度指令
  - 四轮全向底盘正 / 逆运动学解算
  - x/y 方向速度 PID
  - 四个底盘电机转速 PID
  - DJI 电机 CAN 电流控制
  - VESC 电调转速控制接口预留

  ### 升降与上台阶

  相关代码位于：

  R2_Lift_auto/MDK-ARM/TASK/lift_class.cpp
  R2_Lift_auto/MDK-ARM/TASK/lift_auto.cpp

  功能包括：

  - 云深 J60 电机高度控制
  - DJI M2006 升降轮速度闭环
  - 升降高度线性轨迹生成
  - 基于遥控器拨杆的手动 / 半自动切换
  - 基于激光距离的靠近、抬升、爬坡、完成状态机

  ### 路线规划状态机

  相关代码位于：

  R2_Lift_auto/MDK-ARM/Control/plan_route.cpp
  R2_Lift_auto/MDK-ARM/TASK/route_task.cpp

  当前实现了 R2 路线执行框架：

  - 根据 target_id 生成路线步骤
  - 全场导航阶段
  - 入口 DT35 精定位阶段
  - 局部路线执行阶段
  - 上台阶后强制重新定位
  - 吸取失败重试
  - 动作 DONE / FAIL / UNKNOWN 结果处理
  - 超时与故障状态管理

  注意：当前 dispatchMove()、dispatchClimbUp()、dispatchSuckBlock() 等具体执行函
  数仍是占位，需要后续接入真实底盘、机械臂、上台阶执行层。

  ### 通信与传感器

  - Bsp/bsp_can.cpp：三路 FDCAN 初始化、发送、接收中断分发
  - Bsp/bsp_usart.cpp：USART DMA 双缓冲接收、空闲中断分发
  - Device/DT35：DT35 + ADS8688 双通道模拟采集
  - Device/AS5047P：AS5047P 磁编码器 SPI 读取
  - Device/Motor：DJI、DM、Yun J60、VESC 电机驱动
  - Device/Remote_control：SBUS 遥控器解析
  - Device/USB：MiniPC / USB CDC 通信接口

  ## 构建与烧录

  ### Keil MDK

  1. 打开工程：

  R2_Lift_auto/MDK-ARM/My_Princess_cpp.uvprojx

  2. 确认芯片型号为：

  STM32H723VGTx

  3. 编译工程。
  4. 连接调试器后下载到目标板。

  ### STM32CubeMX

  如需修改外设配置，可打开：

  R2_Lift_auto/My_Princess_cpp.ioc

  修改后重新生成代码，再检查 MDK-ARM 下的用户代码是否仍在工程中。

  ## 调试说明

  - LoRa 调试数据通过 USART10 DMA 发送。
  - 当前 USART_TASK 会发送 DT35 CH0 / CH1 的电压和距离数据。
  - 遥控器通过 UART5 接收 SBUS。
  - DM IMU 通过 USART2 接收。
  - 激光测距串口通过 UART8 接收。
  - CAN 接收由 bsp_can.cpp 统一分发到对应电机驱动。

  ## 说明文档

  仓库中附带了两个模块文档：

  说明文档/DT35_ADS8688_使用文档.md
  说明文档/VescMotor 驱动使用文档.md

  分别说明 DT35 + ADS8688 采集链路和 VESC 电调 CAN 驱动的使用方法。

  ## 当前状态

  本仓库保存的是 R2 机器人嵌入式控制代码。底盘、升降、电机、传感器和通信模块已有
  实现；路线规划部分已经搭好状态机框架，但具体动作执行层还需要继续接入和联调。

