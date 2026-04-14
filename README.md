# yezi_R2 Embedded Control

面向 R2 机器人的 STM32H723VGTx 嵌入式控制工程。项目基于 STM32CubeMX、HAL、FreeRTOS CMSIS-RTOS V2 和 Keil MDK，覆盖全向底盘、升降机构、CAN 电机、传感器采集、遥控器输入、USB/MiniPC 通信以及自动上台阶控制链路。

> 当前仓库保存的是机器人下位机控制代码。底盘、升降、电机、传感器和通信链路已有实现；路线规划任务已经在 RTOS 层预留，具体动作执行层仍需要继续接入和联调。

## Highlights

- STM32H723VGTx Cortex-M7，480 MHz 主频，FreeRTOS 静态任务栈
- 三路 FDCAN 总线，统一收发与设备分发
- 四轮全向底盘运动学/动力学解算，速度环 PID 与电机电流控制
- DJI M3508 / M2006、云深 J60、达妙电机、VESC 扩展帧驱动接入
- DT35 激光测距 + ADS8688 模拟采样链路
- AS5047P 磁编码器 SPI 采集
- SBUS 遥控器、DM IMU、激光测距串口 DMA 接收
- USB CDC、MiniPC、LoRa 调试数据通道
- 升降高度轨迹生成与半自动上台阶状态机

## Platform

| Category | Configuration |
| --- | --- |
| MCU | STM32H723VGTx |
| Core | ARM Cortex-M7 |
| Clock | 480 MHz |
| RTOS | FreeRTOS + CMSIS-RTOS V2 |
| IDE | Keil MDK-ARM / EIDE / STM32CubeMX |
| CAN | FDCAN1 / FDCAN2 / FDCAN3 |
| SPI | SPI1 / SPI3 |
| UART | UART5 / UART7 / UART8 / USART1 / USART2 / USART3 / USART10 |
| USB | USB Device CDC |

## Repository Layout

```text
yezi_R2/
├── README.md
├── R2_Lift_auto/
│   ├── My_Princess_cpp.ioc              # STM32CubeMX 工程配置
│   ├── Core/                            # CubeMX 生成的启动、外设、RTOS 入口
│   ├── Drivers/                         # STM32 HAL / CMSIS
│   ├── Middlewares/                     # FreeRTOS、USB Device、CMSIS-DSP
│   ├── USB_DEVICE/                      # USB CDC 设备层
│   ├── EIDE_Project/                    # EIDE 工程配置
│   └── MDK-ARM/
│       ├── My_Princess_cpp.uvprojx      # Keil MDK 工程
│       ├── Bsp/                         # CAN、USART、DWT、IMU 等板级支持
│       ├── Control/                     # PID、全向底盘、路线规划接口
│       ├── Device/                      # 电机、传感器、遥控器、USB/MiniPC 驱动
│       └── TASK/                        # FreeRTOS 任务实现
└── 说明文档/
    ├── DT35_ADS8688_使用文档.md
    └── VescMotor 驱动使用文档.md
```

## Runtime Architecture

```text
Remote / MiniPC / Sensors
          │
          ▼
  BSP_CAN / BSP_USART / SPI / USB CDC
          │
          ▼
Device drivers: DJI / J60 / DM / VESC / DT35 / AS5047P / SBUS / IMU
          │
          ▼
Control layer: PID / omni chassis / lift trajectory / lift auto
          │
          ▼
FreeRTOS tasks: CHASSIS_TASK / USART_TASK / LIFT_TASK / PLAN_ROUTE
          │
          ▼
Actuators: chassis motors / lift motors / lift wheels / external ESC
```

## FreeRTOS Tasks

任务创建入口位于 `R2_Lift_auto/Core/Src/freertos.c`。当前使用静态任务栈，每个任务栈为 512 words。

| Task | Priority | Period | Responsibility |
| --- | --- | --- | --- |
| `CHASSIS_TASK` | `osPriorityNormal` | 1 ms | 初始化 CAN/USART/DWT，读取遥控器指令，更新底盘运动学，计算 PID，发送底盘电机控制帧 |
| `USART_TASK` | `osPriorityNormal` | 1 ms | 更新 AS5047P、DT35 数据，通过 LoRa/MiniPC 通道发送调试曲线数据 |
| `LIFT_TASK` | `osPriorityNormal` | 1 ms | 升降高度轨迹、J60 角度控制、M2006 升降轮速度控制、自动上台阶状态机 |
| `PLAN_ROUTE` | `osPriorityNormal2` | 1 ms | 路线规划任务入口已预留，等待动作执行层和状态机继续接入 |

## Core Modules

### Chassis Control

主要文件：

- `R2_Lift_auto/MDK-ARM/TASK/chassis_task.cpp`
- `R2_Lift_auto/MDK-ARM/Control/omni_chassis.cpp`
- `R2_Lift_auto/MDK-ARM/Device/Motor/dji_motor.cpp`
- `R2_Lift_auto/MDK-ARM/Device/Motor/VescMotor.cpp`

控制链路：

1. `remove_dji.monitor()` 监控遥控器在线状态。
2. `remove_dji.updateChassosCommand()` 刷新底盘速度目标。
3. `omni_chassis.forwardKinematics()` 根据四轮反馈转速反解底盘当前状态。
4. `lift_auto.getChassisVyTarget()` 在自动上台阶时接管底盘 Y 轴速度。
5. `pid_F_chassis_linear_x/y` 计算底盘 x/y 方向驱动力。
6. `omni_chassis.dynamicsInverse()` 和 `inverseKinematics()` 完成力/速度到轮速的分配。
7. 四个 3508 速度环 PID 输出叠加前馈，最终通过 FDCAN 发送电流指令。

### Lift And Climb Automation

主要文件：

- `R2_Lift_auto/MDK-ARM/TASK/lift_class.cpp`
- `R2_Lift_auto/MDK-ARM/TASK/lift_auto.cpp`
- `R2_Lift_auto/MDK-ARM/Device/Motor/yun_J60.cpp`
- `R2_Lift_auto/MDK-ARM/Device/Motor/dji_motor.cpp`

升降控制包含两条链路：

- 高度链路：J60 电机位置反馈 -> 高度换算 -> 线性高度轨迹 -> 左右高度 PID -> J60 力矩/位置控制
- 升降轮链路：M2006 反馈转速 -> 线速度换算 -> 目标线速度反算转速 -> 速度环 PID -> CAN 电流控制

半自动上台阶状态机位于 `LiftAuto`，由左右拨杆同时处于指定位置触发，核心状态为：

| State | Behavior |
| --- | --- |
| `STEP_IDLE` | 空闲，手动指令透传 |
| `STEP_APPROACH_Y` | 接管底盘 Y 轴，依据激光距离靠近目标 |
| `STEP_WAIT_NEW_HEIGHT` | 停车并等待升降高度稳定 |
| `STEP_CLIMB_FORWARD` | 输出升降线速度，执行爬坡 |
| `STEP_FINISHED` | 释放底盘接管，等待人工操作 |

### Communication And Sensors

| Module | Path | Notes |
| --- | --- | --- |
| CAN BSP | `R2_Lift_auto/MDK-ARM/Bsp/bsp_can.cpp` | 三路 FDCAN 初始化、发送、接收中断分发 |
| USART BSP | `R2_Lift_auto/MDK-ARM/Bsp/bsp_usart.cpp` | DMA 接收、空闲中断、串口设备分发 |
| DT35 + ADS8688 | `R2_Lift_auto/MDK-ARM/Device/DT35/` | 双通道电压与距离采集 |
| AS5047P | `R2_Lift_auto/MDK-ARM/Device/AS5047P/` | SPI 磁编码器读取 |
| Remote | `R2_Lift_auto/MDK-ARM/Device/Remote_control/` | SBUS 遥控器解析 |
| DM IMU | `R2_Lift_auto/MDK-ARM/Bsp/dm_imu.cpp` | IMU 串口数据解析 |
| Laser UART | `R2_Lift_auto/MDK-ARM/Device/laser_distance/` | 串口激光测距 |
| USB/MiniPC | `R2_Lift_auto/MDK-ARM/Device/USB/` | USB CDC / MiniPC 数据接口 |

## Build And Flash

### Keil MDK

1. 打开工程：

   ```text
   R2_Lift_auto/MDK-ARM/My_Princess_cpp.uvprojx
   ```

2. 确认目标芯片为 `STM32H723VGTx`。
3. 编译 `My_Princess_cpp` target。
4. 连接 ST-LINK/J-LINK 或团队使用的调试器。
5. 下载到目标板并复位运行。

### STM32CubeMX

外设配置入口：

```text
R2_Lift_auto/My_Princess_cpp.ioc
```

修改 `.ioc` 后重新生成代码时，需要重点检查：

- `Core/Src/freertos.c` 中任务入口是否仍指向业务实现
- `Core/Inc/*` 和 `Core/Src/*` 的 USER CODE 区是否被保留
- `MDK-ARM` 工程是否仍包含新增的 `.cpp/.h` 文件
- FDCAN、USART DMA、SPI、USB CDC 配置是否与硬件接线一致

## Debugging Notes

- LoRa 调试数据通过 `USART10` DMA 发送。
- `USART_TASK` 当前发送 DT35 CH0/CH1 的电压与距离数据。
- SBUS 遥控器通过 `UART5` 接收。
- DM IMU 通过 `USART2` 接收。
- 串口激光测距通过 `UART8` 接收。
- CAN 接收由 `bsp_can.cpp` 统一分发到电机驱动。
- 高精度时间基由 `Bsp/bsp_dwt.*` 提供，初始化时使用 480 MHz 参数。

## Development Rules

- CubeMX 生成区只在 `USER CODE BEGIN/END` 内写业务代码。
- 新增 C++ 源文件后同步检查 Keil 工程文件是否已经收录。
- 控制参数集中放在对应模块头文件或配置文件中，避免散落在任务循环中。
- 1 ms 周期任务内避免阻塞式串口发送和长时间计算。
- CAN/USART 中断回调只做分发、缓存和轻量解析，复杂逻辑放回任务上下文。
- 修改电机方向、零点、行程限位前先断开负载或抬高机器人做空载验证。

## Reference Documents

- `说明文档/DT35_ADS8688_使用文档.md`
- `说明文档/VescMotor 驱动使用文档.md`

## Current Status

| Area | Status |
| --- | --- |
| Chassis | 已实现基础闭环、四轮解算和 DJI 电机电流控制 |
| Lift | 已实现高度轨迹、J60 控制、M2006 升降轮速度控制 |
| Auto climb | 已实现基于遥控器触发和激光距离判断的半自动状态机 |
| Sensors | DT35、AS5047P、DM IMU、串口激光测距链路已接入 |
| Communication | CAN、USART DMA、USB CDC、MiniPC/LoRa 调试链路已接入 |
| Route planning | RTOS 任务入口已预留，具体路线状态机与动作派发仍待补齐 |

## Roadmap

- 补齐 `PLAN_ROUTE` 的目标选择、步骤生成、动作派发和失败恢复逻辑
- 将底盘、升降、吸取机构抽象为统一动作执行接口
- 为关键传感器增加离线检测、数据有效性窗口和降级策略
- 整理 PID 参数表和现场标定流程
- 增加上位机调试协议说明，统一 LoRa、USB、MiniPC 数据帧格式
