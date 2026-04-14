# lift_auto - 半自动上台阶

[[00-项目总览]] | [[lift_task - 升降控制任务]] | [[chassis_task - 底盘控制任务]] | [[激光测距模块]]

---

## 模块概述

`LiftAuto`（`lift_auto.cpp`）实现机器人**半自动上台阶**功能：
- 遥控器左右拨杆同时拨到"上"位时触发自动流程
- 利用激光测距仪感知距离，驱动5段状态机完成爬坡
- 自动接管底盘Y轴速度和升降档位，完成后归还手动控制

## 触发条件

```
left_sw == 1 (上) AND right_sw == 1 (上)
→ 自动流程激活

任意拨杆离开"上"位 → 立即 reset()，回到 IDLE
```

## 状态机流程

```
STEP_IDLE
    │ 双杆均在"上"位
    ▼
STEP_APPROACH_Y  ←─ 底盘Y轴以 -0.45m/s 向台阶靠近
    │                升降档位=1（待机位）
    │ 激光距离 ≤ 110mm
    ▼
STEP_WAIT_NEW_HEIGHT ←─ 底盘停止，升降档位=2（抬升）
    │                   等待激光高度稳定在 1120mm ± 30mm
    │                   连续10帧在范围内 → 确认到位
    ▼
STEP_CLIMB_FORWARD ←─ 升降机构以 0.27m/s 向前爬坡
    │                  底盘停止
    │ 激光距离 ≤ 490mm → 爬坡完成
    ▼
STEP_FINISHED  ←─ 停止，升降收起（档位=1）
                   释放底盘Y轴接管，还给手动控制
```

## 关键阈值参数

| 参数 | 值 | 说明 |
|------|----|------|
| LIFT_AUTO_APPROACH_VY_MPS | -0.45 m/s | 靠近速度 |
| LIFT_AUTO_CLIMB_SPEED_MPS | 0.27 m/s | 爬坡速度 |
| LIFT_AUTO_PREPARE_MM | 110 mm | 停止靠近的距离 |
| LIFT_AUTO_NEW_HEIGHT_MM | 1120 mm | 抬升目标激光高度 |
| LIFT_AUTO_HEIGHT_TOL_MM | 30 mm | 高度判断容差 |
| LIFT_AUTO_STABLE_COUNT | 10 帧 | 稳定确认帧数 |
| LIFT_AUTO_FINISH_MM | 490 mm | 爬坡完成判断距离 |

## 对外接口

```cpp
lift_auto.update();                         // 每帧驱动状态机（在lift_task中调用）
lift_auto.getLiftSwitch(manual_sw);         // 获取档位（IDLE时透传手动，否则返回自动档位）
lift_auto.getLiftLinearSpeedTarget(manual); // 获取升降线速度（IDLE时透传手动）
lift_auto.getChassisVyTarget(manual);       // 获取底盘Vy（接管时返回自动，否则透传手动）
```

## 接管机制（透传 vs 接管）

```
IDLE状态：
  所有接口直接透传手动输入 → 完全手动控制

非IDLE状态：
  getLiftSwitch → 返回 lift_switch_target_
  getLiftLinearSpeedTarget → 返回 lift_linear_speed_target_
  chassis_vy_override_=1 → getChassisVyTarget 返回 chassis_vy_target_
```

## 数据流图

```
遥控器拨杆 ──┐
             ├──→ LiftAuto::update()
激光测距 ────┘         │
                       ├──→ getLiftSwitch() ──→ lift_task（档位→高度目标）
                       ├──→ getLiftLinearSpeedTarget() ──→ lift_task（速度环）
                       └──→ getChassisVyTarget() ──→ chassis_task（底盘Vy）
```

## 相关文件

- `MDK-ARM/TASK/lift_auto.cpp`
- `MDK-ARM/TASK/lift_auto.h`

## 参见

- [[激光测距模块]] — 距离感知来源
- [[lift_task - 升降控制任务]] — 接收档位和速度指令
- [[chassis_task - 底盘控制任务]] — 接收Vy接管指令
- [[遥控器解析]] — 拨杆状态来源
