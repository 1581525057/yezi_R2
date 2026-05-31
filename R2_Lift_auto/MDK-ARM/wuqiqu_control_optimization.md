# 武器区跑点控制优化方案

## 目标

武器区跑点目标是视觉置零后的绝对坐标，例如：

```cpp
{ 0.77f, 0.62f, 0.0f }
```

这个点表示夹爪和武器头的最佳抓取位姿。底盘允许通过导轮轻微抵住木架，但不能高速撞上木架。

第一版优化只改 `wuqiqu` 自身的跑点控制逻辑，暂时不改：

- `MDK-ARM/TASK/chassis_task.cpp`
- `MDK-ARM/TASK/FTMTask.cpp`

已知但暂时不处理的两个集成问题：

- `chassis_task.cpp` 后续应把 `target_vz` 传给 `omni_chassis.setRemote(...)`，否则武器区 yaw 修正不会真正下发。
- `FTMTask.cpp` 后续应恢复 `FTM_STATE_WUQIQU_ZERO -> FTM_STATE_WUQIQU_ROUTE` 的状态切换，否则置零后不会自动开始跑点。

## 需要修改的文件

### `MDK-ARM/Route_Plan/wuqiqu.h`

修改 `WuqiquPathPlanner` 的状态和参数定义。

建议改动：

- 用新的跑点状态替换旧的 `STATE_MOVING` / `STATE_YAW_CORRECTING`。
- 增加分段速度上限、位置 PD 参数、yaw 控制参数、软贴靠计时和完成阈值。
- 增加软贴靠阶段的计时字段和稳定计数字段。

### `MDK-ARM/Route_Plan/wuqiqu.cpp`

实现新的跑点控制逻辑。

建议改动：

- 用位置 PD 生成 XY 目标速度。
- 按不同状态限制速度。
- 增加刹车速度约束。
- yaw 在所有跑点阶段同时修正。
- yaw 误差大时降低 XY 速度。
- 目标附近进入低速软贴靠。
- 支持正常完成和超时完成。

### `MDK-ARM/TASK/wuqiqu_task.cpp`

保留为任务接入层。

建议职责：

- 把 `vision.x_diff`、`vision.y_diff`、`vision.angle_x` 转成 planner 使用的 `Pose`。
- 调用 `wuqiqu.follow(pose)`。
- 把 planner 输出转成底盘目标速度。
- 保留最后一层安全限幅和斜率限制。

尽量不要把跑点控制策略放到这个文件里，避免后续调参时逻辑分散。

## 当前问题

### 速度过激

当前参数包括：

```cpp
pos_kp_ = 5.0f;
kMaxLinearSpeedMps = 2.0f;
```

目标点距离约为：

```text
sqrt(0.77^2 + 0.62^2) ~= 0.99m
```

初始 P 控制理论速度约为：

```text
5.0 * 0.99 ~= 4.95m/s
```

随后被限幅到 `2.0m/s`。这意味着车在距离目标约：

```text
2.0 / 5.0 = 0.4m
```

之前，基本都在顶着最大速度跑。距离木架只剩 40 cm 时，底盘仍可能保持很高速度，因此过冲和猛烈撞击是当前参数的必然结果。

### 旧状态机不适合取武器

当前逻辑大致是：

```text
MOVING
  先追 XY 点位

YAW_CORRECTING
  XY 到点后停止平移，再单独修 yaw

FINISHED
```

对于贴近木架取武器头，这种逻辑不理想。更合理的是边接近边修 yaw，最后进入低速软贴靠，而不是把目标点当作普通数学点直接冲过去。

## 新状态机

建议替换为：

```text
APPROACH
SLOW_APPROACH
SOFT_CONTACT
FINISHED
```

### APPROACH

进入条件：

```text
distance > slow_dist
```

作用：

- 以中等速度接近目标。
- 同时修正 yaw。
- yaw 误差较大时降低 XY 速度。

### SLOW_APPROACH

进入条件：

```text
contact_dist < distance <= slow_dist
```

作用：

- 进入木架附近时明显降速。
- 使用更保守的位置 PD 参数。
- 继续修 yaw。

### SOFT_CONTACT

进入条件：

```text
distance <= contact_dist
```

作用：

- 继续朝目标点方向低速靠近。
- 允许导轮轻微贴住木架。
- 保持 XY 和 yaw 微调，但严格限制速度。
- 达到最佳抓取位姿并稳定后正常完成。
- 如果一直无法收敛到 2 cm 内，则超时完成，避免持续顶木架。

### FINISHED

输出零速度，并向上层报告跑点完成。

## 第一版参数

建议用下面这组参数作为第一版试车基准：

```cpp
approach_v_max = 0.55f;      // m/s
slow_v_max = 0.20f;          // m/s
contact_v_max = 0.04f;       // m/s
finish_v_max = 0.02f;        // m/s

slow_dist = 0.25f;           // m
contact_dist = 0.08f;        // m
finish_dist = 0.02f;         // m

decel = 0.45f;               // m/s^2

kp_approach = 1.2f;
kd_approach = 0.15f;
kp_slow = 0.8f;
kd_slow = 0.20f;
kp_contact = 0.5f;
kd_contact = 0.10f;

yaw_sign = -1.0f;            // 右手坐标系默认假设，必须实车确认
yaw_kp = 0.8f;
moving_wz_max = 0.30f;       // rad/s
settle_wz_max = 0.15f;       // rad/s
yaw_tol = 3.0f;              // deg

stable_cycles = 120U;        // 第一版先按任务循环计数，不按视觉新帧计数
contact_hold_ms = 500U;
contact_timeout_ms = 1500U;
```

## XY 控制

使用位置 PD 生成目标速度：

```cpp
err_x = target_x - pose.x;
err_y = target_y - pose.y;

vx = kp * err_x - kd * pose.car_speed_x;
vy = kp * err_y - kd * pose.car_speed_y;
```

然后按下面顺序施加约束：

```text
1. 当前状态的二维速度限幅
2. 刹车速度约束
3. yaw 误差导致的 XY 降速
4. 软贴靠阶段限速
5. 斜率限制
```

### 二维速度限幅

限制二维速度大小，但保持方向不变：

```cpp
speed = sqrtf(vx * vx + vy * vy);
if (speed > limit && speed > 0.000001f) {
    scale = limit / speed;
    vx *= scale;
    vy *= scale;
}
```

### 刹车速度约束

用刹车速度约束让车提前减速：

```cpp
brake_v_max = sqrtf(2.0f * decel * max(distance - finish_dist, 0.0f));
limitVector(vx, vy, brake_v_max);
```

这是防止高速撞木架的核心改动。

### 软贴靠限速

当：

```text
distance <= contact_dist
```

二维速度限制到：

```cpp
contact_v_max = 0.04f;
```

当：

```text
distance <= finish_dist
```

进一步限制到：

```cpp
finish_v_max = 0.02f;
```

这样最后阶段会轻轻贴近木架，而不是猛撞。

## yaw 控制

yaw 目标是 `0 deg`。

第一版公式：

```cpp
wz = yaw_sign * yaw_kp * pose.yaw_360 * DEG_TO_RAD;
```

默认：

```cpp
yaw_sign = -1.0f;
```

这是假设视觉 yaw 和底盘 `Vz` 都符合一致的右手坐标系。这个符号必须实车确认。

如果实车发现给正 `Vz` 会让 `vision.angle_x` 变小，则应改为：

```cpp
yaw_sign = 1.0f;
```

### yaw 限幅

远处接近时允许较大 yaw 速度：

```cpp
moving_wz_max = 0.30f;
```

靠近木架后降低 yaw 速度：

```cpp
settle_wz_max = 0.15f;
```

### yaw 误差压低 XY 速度

如果 yaw 误差较大，降低 XY 速度：

```cpp
if (fabsf(pose.yaw_360) > 20.0f) {
    xy_scale = 0.25f;
} else if (fabsf(pose.yaw_360) > 10.0f) {
    xy_scale = 0.5f;
} else {
    xy_scale = 1.0f;
}

vx *= xy_scale;
vy *= xy_scale;
```

这样可以避免车身角度还没对正时快速斜着撞向木架。

## 完成条件

正常完成：

```text
state == SOFT_CONTACT
distance <= finish_dist
fabs(yaw_error_deg) <= yaw_tol
stable_cycles >= 120
time_in_soft_contact >= 500ms
```

超时完成：

```text
state == SOFT_CONTACT
time_in_soft_contact >= 1500ms
```

超时完成后也允许进入抓取动作。这样可以避免视觉目标和物理贴靠点存在偏差时，底盘一直低速顶着木架。

## 建议实现结构

让 `WuqiquPathPlanner::follow()` 成为跑点控制核心：

```text
输入 Pose
内部维护目标点、参数和状态机
输出 vx / vy / wz
```

让 `wuqiqu_task.cpp` 继续作为接入层：

```text
vision / chassis 全局状态 -> Pose
planner 输出 -> 底盘目标速度
```

这样调参主要看 `wuqiqu.cpp`，不会在任务层和底盘层之间反复跳转。

## 试车和整定流程

### 第一步：确认 XY 方向

先确认坐标方向：

```text
目标 x/y 为正时，车应该朝视觉目标靠近。
如果车远离目标，先修坐标符号，不要调参数。
```

### 第二步：确认 yaw 符号

给一个很小的正 `Vz`，例如：

```text
+0.1 rad/s
```

观察 `vision.angle_x`：

```text
如果 angle_x 变大，yaw_sign = -1.0f 大概率正确。
如果 angle_x 变小，yaw_sign = 1.0f 大概率正确。
```

### 第三步：第一轮安全试跑

使用上面的第一版参数。期望现象：

```text
远离木架时：中等速度接近，不再猛冲。
进入 25 cm 内：明显减速。
进入 8 cm 内：低速软贴靠。
进入 2 cm 内：稳定后完成。
```

### 如果仍然太快

按下面顺序改：

```text
approach_v_max: 0.55 -> 0.45
slow_v_max: 0.20 -> 0.15
decel: 0.45 -> 0.35
```

### 如果太慢

按下面顺序改：

```text
approach_v_max: 0.55 -> 0.65
kp_approach: 1.2 -> 1.4
slow_v_max: 0.20 -> 0.25
```

不要过早提高 `contact_v_max`，最后贴木架阶段的速度必须保持很低。

### 如果木架附近来回振荡

按下面顺序改：

```text
kd_contact: 0.10 -> 0.15
kp_contact: 0.5 -> 0.35
finish_dist: 0.02 -> 0.03
stable_cycles: 120 -> 180
```

### 如果停得太远

按下面顺序改：

```text
contact_timeout_ms: 1500 -> 2000
contact_v_max: 0.04 -> 0.05
finish_v_max: 0.02 -> 0.025
```

## 后续可继续优化

这些不属于第一版改动：

- 用视觉新帧序号统计稳定帧，而不是用 1 ms 任务循环计数。
- 增加视觉数据超时保护。
- 根据木架几何，把最后接近速度拆成法向和切向。
- 如果后续有接触传感器或电流判断，可加入接触检测。
- 修复 `chassis_task.cpp` 中 `target_vz` 没下发的问题。
- 恢复 `FTMTask.cpp` 中置零后进入武器区跑点的状态切换。
