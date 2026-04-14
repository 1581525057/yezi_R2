# lift_task - 升降控制任务

[[00-项目总览]] | [[lift_auto - 半自动上台阶]] | [[云深J60电机]] | [[DJI电机驱动]] | [[PID控制器]]

---

## 任务概述

升降控制任务（`lift_class.cpp`）以 **1ms 周期**运行，负责机器人升降机构的完整控制，包括：
- 双侧云深J60电机的高度位置控制（角度环）
- 双侧大疆2006电机的升降轮速度控制（速度环）
- 线性高度轨迹规划

## 机械参数

| 参数 | 值 |
|------|-----|
| 抬升卷轮直径 | 40.59mm |
| 从动轮直径 | 30.0mm |
| 直驱轮齿数 | 24 |
| 从动轮齿数 | 32 |
| 减速比（齿轮） | 24/32 = 0.75 |
| 高度范围 | -220mm ~ +120mm |

## 初始化流程

```
1. yun_j60_motor.EnableMotor(0x02)  — 使能左侧J60电机（ID=2）
2. yun_j60_motor.EnableMotor(0x03)  — 使能右侧J60电机（ID=3）
3. lift_class_pid_init()             — 初始化全部PID
4. lift_height_set_target(0.0f, 2.0f) — 上电归零，2秒线性到0高度
```

## 主循环控制流程（每 1ms）

```
步骤1  lift_cauclate_height()       — J60电机角度 → 左右侧当前高度(mm)
步骤2  calc_linear_speed_from_motor_rpm() — 2006电机RPM → 升降轮线速度(m/s)
步骤3  lift_auto.update()           — 更新半自动状态机
步骤4  lift_auto.getLiftLinearSpeedTarget() — 获取目标线速度（自动/手动）
步骤5  calc_motor_rpm_from_linear_speed_target() — 目标线速度 → 2006目标RPM
步骤6  lift_auto.getLiftSwitch()    — 读取当前档位（自动/手动）
步骤7  档位变化检测 → 重新设置目标高度
       档位3 → 0mm, 档位1 → 50mm, 档位2 → -205mm
步骤8  lift_height_set_target()     — 生成0.7s线性高度轨迹
步骤9  lift_height_input()          — 推进轨迹，得到当前跟踪高度
步骤10 lift_position_input()        — 目标高度 → J60目标角度
步骤11 lift_class_pid_calculate()   — 高度环PID + 速度环PID计算
步骤12 yun_j60_motor.SendControl()  — 发送J60控制指令
步骤13 lift_motor.Send_CurrentCommand(FDCAN3, 0x1FF, ...) — 发送2006电流
```

## 高度轨迹（LiftHeight_t）

采用**线性插值**的方式规划高度轨迹：

```
current_height = start_height + (target - start) * (t / total_time)
```

- 轨迹完成后锁定在目标高度
- 支持立即到达（total_time <= 0 时直接跳到终点）
- 基于 DWT 高精度计时器计时

## 高度换算公式

```
J60角度(rad) → 高度(mm) = angle × HEIGHT_DIAMETER / 2.0
高度(mm) → J60角度(rad) = height × 2.0 / HEIGHT_DIAMETER
```

左侧电机角度取负号（安装方向相反）。

## PID 参数一览

| 控制器 | 用途 | KP | KI | KD | 输出上限 |
|--------|------|----|----|----|---------|
| pid_lift_left/right | J60高度角度环 | 0.5 | 0 | 0 | 3 |
| pid_2006_l/r | 2006升降轮速度环 | 12 | 0.8 | 0 | 10000 |

## 数据结构

```cpp
Lift_Class {
    left/right: { height(mm), angle(rad) }  // 左右侧高度状态
    now:   { rpm_left/right, vel_2006_left/right }  // 当前速度
    target:{ rpm_left/right, vel_2006_left/right }  // 目标速度
}

LiftHeight_t {
    start_height, target_height, current_height
    total_time, start_time
    started(flag), finished(flag)
}
```

## 相关文件

- `MDK-ARM/TASK/lift_class.cpp`
- `MDK-ARM/TASK/lift_class.h`

## 参见

- [[lift_auto - 半自动上台阶]] — 状态机接管升降控制
- [[云深J60电机]] — J60电机驱动与通信
- [[DJI电机驱动]] — 2006电机驱动
- [[PID控制器]] — PID算法实现
