# PID控制器

[[00-项目总览]] | [[chassis_task - 底盘控制任务]] | [[lift_task - 升降控制任务]]

---

## 模块概述

`PID`（`PID.cpp`）是工程统一的 PID 控制器实现，支持积分限幅和输出低通滤波两种增强功能。

## 接口

```cpp
void Init(float max_out, float intergral_limit, float deadband,
          float Kp, float Ki, float Kd,
          float output_fiter_factor, uint8_t improve);

float PID_Calculate(float measure, float ref);
// measure: 当前测量值
// ref:     目标参考值
// return:  本次PID输出
```

## 增强功能枚举

```cpp
PID_IMPROVE_NONE          = 0x00  // 无增强
PID_IMPROVE_OUTPUT_FILTER = 0x01  // 输出低通滤波
PID_IMPROVE_INT_LIMIT     = 0x02  // 积分限幅
```

## 内部计算结构

```
Err = Ref - Measure
Pout = Kp × Err
ITerm = Ki × Err × dt
Iout = Iout + ITerm             （可选：积分限幅在 ±IntegralLimit）
Dout = Kd × (Err - Last_Err) / dt
Output = Pout + Iout + Dout     （限幅在 ±MaxOut，死区 Deadband 内置零）
（可选：Output = α×Output + (1-α)×Last_Output  输出低通滤波）
```

## 工程中各 PID 实例

| 实例 | 所在任务 | 用途 |
|------|---------|------|
| pid_chassis_0~3 | chassis_task | 4个底盘轮转速环 |
| pid_F_chassis_linear_x/y | chassis_task | 底盘X/Y速度→力 |
| pid_F_chassis_angle | chassis_task | 底盘角度（预留） |
| pid_yaw | chassis_task | 航向角保持 |
| pid_lift_left/right | lift_task | J60高度角度环 |
| pid_2006_l/r | lift_task | 2006升降轮速度环 |

## 相关文件

- `MDK-ARM/Control/PID.cpp`
- `MDK-ARM/Control/PID.h`

## 参见

- [[chassis_task - 底盘控制任务]] — PID应用场景
- [[lift_task - 升降控制任务]] — PID应用场景
