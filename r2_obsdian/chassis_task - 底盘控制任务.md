# chassis_task - 底盘控制任务

[[00-项目总览]] | [[全向底盘运动学]] | [[PID控制器]] | [[DJI电机驱动]] | [[VESC电机驱动]]

---

## 任务概述

底盘控制任务（`chassis_task.cpp`）是系统主控任务之一，以 **1ms 周期**运行。负责四轮全向底盘的完整运动控制闭环：从遥控器输入到电机电流输出。

## 初始化流程

```
1. BSP_CAN::Init()       — 初始化三路 FDCAN 总线
2. BSP_USART::Init()     — 初始化串口
3. DWT_.init(480)        — 高精度计时（480MHz 系统时钟）
4. VescMotors[0].init(&hfdcan2, 80)  — VESC 电机绑定 FDCAN2，节点ID=80
5. GPIO 配置              — GPIOC 13/14/15 置高，GPIOE 14 置低（外设使能）
6. chassis_pid_init()    — 初始化全部 PID 控制器
```

## 主循环控制流程（每 1ms）

```
步骤1  remove_dji.monitor()          — 检测遥控器是否在线，离线则保护
步骤2  remove_dji.updateChassosCommand() — 刷新遥控器速度指令
步骤3  读取 4 个底盘电机当前 RPM     — chassis_motor.Chassis_Motor[i].Data.Rpm
步骤4  omni_chassis.forwardKinematics() — 正运动学：轮速 → 底盘速度
步骤5  lift_auto.getChassisVyTarget()   — 获取 Y 轴速度目标（可被自动上台阶接管）
步骤6  omni_chassis.setRemote(Vx, Vy, Vz) — 写入目标速度
步骤7  pid_F_chassis_linear_x/y.PID_Calculate() — X/Y 速度 PID → 合力 Fx/Fy
步骤8  omni_chassis.dynamicsInverse(Fx, Fy, 0) — 动力学逆解：力 → 各轮输出
步骤9  omni_chassis.inverseKinematics()         — 逆运动学：速度 → 目标 RPM
步骤10 pid_chassis_0~3.PID_Calculate()          — 4个电机转速 PID 闭环
步骤11 motor_input = PID输出 + 前馈(Current_rpm) — 叠加前馈电流
步骤12 chassis_motor.Send_CurrentCommand(FDCAN3, 0x200, ...) — CAN 发送电流
步骤13 VescMotors[0].setRpm(rpm)                — VESC 电机转速控制
```

## PID 参数一览

| 控制器 | 用途 | KP | KI | KD | 输出上限 |
|--------|------|----|----|----|---------|
| pid_chassis_0~3 | 4个底盘轮转速环 | 15 | 0.8 | 0 | 16384 |
| pid_F_chassis_linear_x | X方向速度→力 | 20 | 0 | 0 | 6 |
| pid_F_chassis_linear_y | Y方向速度→力 | 20 | 0 | 0 | 6 |
| pid_F_chassis_angle | 角度环（预留） | 10 | 0 | 1 | 16384 |
| pid_yaw | 航向角保持 | 2 | 0 | 0 | 2 |

## 与其他模块的联动

- 调用 `lift_auto.getChassisVyTarget()` —— 半自动上台阶时，Y轴速度由状态机接管
- 使用 `omni_chassis` 做全部运动学/动力学解算
- 底盘电机通过 FDCAN3 总线 `0x200` 地址发送电流指令

## 相关文件

- `MDK-ARM/TASK/chassis_task.cpp`
- `MDK-ARM/TASK/chassis_task.h`（含 PID 宏定义）

## 参见

- [[全向底盘运动学]] — 运动学与动力学算法细节
- [[lift_auto - 半自动上台阶]] — Y轴速度接管逻辑
- [[DJI电机驱动]] — 底盘电机驱动
- [[VESC电机驱动]] — VESC 辅助电机
- [[FDCAN总线]] — CAN 通信层
