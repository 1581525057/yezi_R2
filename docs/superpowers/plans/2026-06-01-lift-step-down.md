# 自动下台阶类 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在两个空文件中实现独立的 `LiftStepDown` 类，为后续公共速度接口接线提供基于雷达坐标的自动下台阶状态机。

**Architecture:** `LiftStepDown` 维护独立状态和独立输出缓存，不修改现有 `LiftAuto`。调用方配置四个雷达目标值并周期调用 `update()`；空闲时 getter 透传上游输入，运行时 getter 输出当前阶段目标。

**Tech Stack:** STM32 C++、Keil MDK-ARM、CMSIS 工程、`VisionData_t` 雷达坐标、`math.h`

---

## 文件结构

- 创建或补齐：`R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.h`
  - 声明独立下台阶类、四阶段状态机、输出 getter 和雷达目标 setter。
- 创建或补齐：`R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.cpp`
  - 实现状态机、梯形速度函数、稳定计数和全局实例。
- 不修改：`R2_Lift_auto/MDK-ARM/TASK/route_task.cpp`
  - 用户自行接入视觉命令 `case 5` 和 `vision_block_pop()`。
- 不修改：`R2_Lift_auto/MDK-ARM/TASK/lift_class.cpp`
  - 用户自行串接升降轮公共速度接口。
- 不修改：`R2_Lift_auto/MDK-ARM/TASK/chassis_task.cpp`
  - 用户自行串接底盘公共速度接口。

当前工程未发现现成单元测试框架。本计划不引入宿主机测试基础设施，使用静态检查和 Keil 工程编译作为验证手段。

### Task 1: 声明独立下台阶类

**Files:**
- Create: `R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.h`

- [ ] **Step 1: 确认头文件目前为空**

Run:

```powershell
Get-Item R2_Lift_auto\MDK-ARM\Route_Plan\lift_step_down.h | Select-Object Length
```

Expected: `Length` 为 `0`。

- [ ] **Step 2: 写入类声明**

写入以下内容：

```cpp
#ifndef LIFT_STEP_DOWN_H
#define LIFT_STEP_DOWN_H

#include "main.h"
#include <stdint.h>

class LiftStepDown
{
public:
    LiftStepDown();

    void startStepDown(void);
    void stopStepDown(void);
    uint8_t isStepDownFinished(void) const;
    void update(void);

    uint8_t getLiftSwitch(uint8_t manual_switch) const;
    float getLiftLinearSpeedTarget(float manual_target) const;
    float getChassisVxTarget(float manual_target) const;
    float getChassisVyTarget(float manual_target) const;

    void setStepDownRadarTarget(float x_ref_prepare,
                                float x_ref_descend,
                                float x_ref_finish,
                                float y_ref_finish);
    void setStepDownBlockNum(int num);

private:
    enum StepDownState {
        STEP_DOWN_IDLE = 0,
        STEP_DOWN_MOVE_TO_PREPARE,
        STEP_DOWN_DESCEND,
        STEP_DOWN_MOVE_TO_FINISH,
        STEP_DOWN_FINISHED
    };

    void resetStepDown(void);
    float speed_limit(float speed, float max);
    float trapezoid_speed(float error, float acc, float max);
    uint8_t step_down_stable_confirm(uint8_t condition);

    uint8_t step_down_started_;
    StepDownState step_down_state_;

    uint8_t lift_switch_target_;
    float lift_linear_speed_target_;
    float chassis_vx_target_;
    float chassis_vy_target_;

    uint8_t step_down_stable_count_;
    int step_down_block_num_;
    float step_down_radar_x_ref_prepare_;
    float step_down_radar_x_ref_descend_;
    float step_down_radar_x_ref_finish_;
    float step_down_radar_y_ref_finish_;
};

extern LiftStepDown lift_step_down;

#endif
```

- [ ] **Step 3: 检查头文件只有下台阶类接口**

Run:

```powershell
rg -n "LiftStepDown|LiftAuto|vision_block_pop" R2_Lift_auto\MDK-ARM\Route_Plan\lift_step_down.h
```

Expected:

- 存在 `LiftStepDown`。
- 不存在 `LiftAuto`。
- 不存在 `vision_block_pop`。

### Task 2: 实现四阶段状态机

**Files:**
- Create: `R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.cpp`

- [ ] **Step 1: 确认实现文件目前为空**

Run:

```powershell
Get-Item R2_Lift_auto\MDK-ARM\Route_Plan\lift_step_down.cpp | Select-Object Length
```

Expected: `Length` 为 `0`。

- [ ] **Step 2: 写入完整实现**

写入以下内容：

```cpp
#include "lift_step_down.h"
#include "usart_task.h"
#include <math.h>

extern VisionData_t vision;

// 底盘移动阶段最大速度
float STEP_DOWN_AUTO_CHASSIS_SPEED_MPS = 0.8f;
// 升降轮移动阶段最大速度
float STEP_DOWN_AUTO_LIFT_SPEED_MPS = 0.65f;
// 底盘移动阶段最大加速度
float STEP_DOWN_CHASSIS_ACC_SPEED = 0.8f;
// 升降轮移动阶段最大加速度
float STEP_DOWN_LIFT_ACC_SPEED = 0.4f;
// 雷达坐标连续满足目标条件的确认次数
uint8_t STEP_DOWN_AUTO_STABLE_COUNT = 10U;

LiftStepDown lift_step_down;

LiftStepDown::LiftStepDown()
{
    resetStepDown();
}

float LiftStepDown::speed_limit(float speed, float max)
{
    if (speed > max) {
        speed = max;
    }
    if (speed < -max) {
        speed = -max;
    }
    return speed;
}

float LiftStepDown::trapezoid_speed(float error, float acc, float max)
{
    if (error == 0.0f || acc <= 0.0f || max <= 0.0f) {
        return 0.0f;
    }

    float speed = sqrtf(2.0f * fabsf(error) * acc);
    if (error < 0.0f) {
        speed = -speed;
    }
    return speed_limit(speed, max);
}

uint8_t LiftStepDown::step_down_stable_confirm(uint8_t condition)
{
    if (condition == 0U) {
        step_down_stable_count_ = 0U;
        return 0U;
    }

    if (step_down_stable_count_ < STEP_DOWN_AUTO_STABLE_COUNT) {
        step_down_stable_count_++;
    }

    return (step_down_stable_count_ >= STEP_DOWN_AUTO_STABLE_COUNT) ? 1U : 0U;
}

void LiftStepDown::resetStepDown(void)
{
    step_down_started_             = 0U;
    step_down_state_               = STEP_DOWN_IDLE;
    lift_switch_target_            = 0U;
    lift_linear_speed_target_      = 0.0f;
    chassis_vx_target_             = 0.0f;
    chassis_vy_target_             = 0.0f;
    step_down_stable_count_        = 0U;
    step_down_block_num_           = 0;
    step_down_radar_x_ref_prepare_ = 0.0f;
    step_down_radar_x_ref_descend_ = 0.0f;
    step_down_radar_x_ref_finish_  = 0.0f;
    step_down_radar_y_ref_finish_  = 0.0f;
}

void LiftStepDown::startStepDown(void)
{
    step_down_started_ = 1U;
}

void LiftStepDown::stopStepDown(void)
{
    resetStepDown();
}

uint8_t LiftStepDown::isStepDownFinished(void) const
{
    return (step_down_state_ == STEP_DOWN_FINISHED) ? 1U : 0U;
}

void LiftStepDown::update(void)
{
    if (step_down_started_ == 0U) {
        resetStepDown();
        return;
    }

    if (step_down_state_ == STEP_DOWN_IDLE) {
        step_down_state_ = STEP_DOWN_MOVE_TO_PREPARE;
    }

    switch (step_down_state_) {
        case STEP_DOWN_MOVE_TO_PREPARE: {
            lift_switch_target_        = 1U;
            lift_linear_speed_target_  = 0.0f;
            chassis_vy_target_         = 0.0f;
            const float x_err          = step_down_radar_x_ref_prepare_ - vision.x_diff;
            chassis_vx_target_         = trapezoid_speed(x_err, STEP_DOWN_CHASSIS_ACC_SPEED, STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);

            if (step_down_stable_confirm((fabsf(x_err) < 0.050f) ? 1U : 0U) != 0U) {
                chassis_vx_target_        = 0.0f;
                step_down_stable_count_   = 0U;
                step_down_state_          = STEP_DOWN_DESCEND;
            }
            break;
        }

        case STEP_DOWN_DESCEND: {
            lift_switch_target_        = 2U;
            chassis_vx_target_         = 0.0f;
            chassis_vy_target_         = 0.0f;
            const float x_err          = step_down_radar_x_ref_descend_ - vision.x_diff;
            lift_linear_speed_target_  = trapezoid_speed(x_err, STEP_DOWN_LIFT_ACC_SPEED, STEP_DOWN_AUTO_LIFT_SPEED_MPS);

            if (step_down_stable_confirm((fabsf(x_err) < 0.050f) ? 1U : 0U) != 0U) {
                lift_linear_speed_target_ = 0.0f;
                step_down_stable_count_   = 0U;
                step_down_state_          = STEP_DOWN_MOVE_TO_FINISH;
            }
            break;
        }

        case STEP_DOWN_MOVE_TO_FINISH: {
            lift_switch_target_        = 1U;
            lift_linear_speed_target_  = 0.0f;
            const float x_err          = step_down_radar_x_ref_finish_ - vision.x_diff;
            const float y_err          = step_down_radar_y_ref_finish_ - vision.y_diff;
            chassis_vx_target_         = trapezoid_speed(x_err, STEP_DOWN_CHASSIS_ACC_SPEED, STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);
            chassis_vy_target_         = trapezoid_speed(y_err, STEP_DOWN_CHASSIS_ACC_SPEED, STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);

            if (step_down_stable_confirm((fabsf(x_err) < 0.050f && fabsf(y_err) < 0.050f) ? 1U : 0U) != 0U) {
                chassis_vx_target_        = 0.0f;
                chassis_vy_target_        = 0.0f;
                step_down_stable_count_   = 0U;
                step_down_state_          = STEP_DOWN_FINISHED;
            }
            break;
        }

        case STEP_DOWN_FINISHED:
            lift_switch_target_        = 1U;
            lift_linear_speed_target_  = 0.0f;
            chassis_vx_target_         = 0.0f;
            chassis_vy_target_         = 0.0f;
            break;

        default:
            resetStepDown();
            break;
    }
}

uint8_t LiftStepDown::getLiftSwitch(uint8_t manual_switch) const
{
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_switch;
    }
    return lift_switch_target_;
}

float LiftStepDown::getLiftLinearSpeedTarget(float manual_target) const
{
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_target;
    }
    return lift_linear_speed_target_;
}

float LiftStepDown::getChassisVxTarget(float manual_target) const
{
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_target;
    }
    return chassis_vx_target_;
}

float LiftStepDown::getChassisVyTarget(float manual_target) const
{
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_target;
    }
    return chassis_vy_target_;
}

void LiftStepDown::setStepDownRadarTarget(float x_ref_prepare,
                                          float x_ref_descend,
                                          float x_ref_finish,
                                          float y_ref_finish)
{
    step_down_radar_x_ref_prepare_ = x_ref_prepare;
    step_down_radar_x_ref_descend_ = x_ref_descend;
    step_down_radar_x_ref_finish_  = x_ref_finish;
    step_down_radar_y_ref_finish_  = y_ref_finish;
}

void LiftStepDown::setStepDownBlockNum(int num)
{
    step_down_block_num_ = num;
}
```

### Task 3: 执行静态检查和编译探测

**Files:**
- Verify: `R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.h`
- Verify: `R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.cpp`

- [ ] **Step 1: 检查实现范围**

Run:

```powershell
git diff -- R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.h R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.cpp
```

Expected: 只包含独立下台阶类，不包含任务接线或坐标表。

- [ ] **Step 2: 检查空白错误**

Run:

```powershell
git diff --check -- R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.h R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.cpp
```

Expected: 无新增空白错误。

- [ ] **Step 3: 检查状态和接口齐全**

Run:

```powershell
rg -n "STEP_DOWN_MOVE_TO_PREPARE|STEP_DOWN_DESCEND|STEP_DOWN_MOVE_TO_FINISH|STEP_DOWN_FINISHED|getLiftSwitch|getLiftLinearSpeedTarget|getChassisVxTarget|getChassisVyTarget|setStepDownRadarTarget|setStepDownBlockNum" R2_Lift_auto\MDK-ARM\Route_Plan\lift_step_down.h R2_Lift_auto\MDK-ARM\Route_Plan\lift_step_down.cpp
```

Expected: 每个状态和接口均有声明或实现。

- [ ] **Step 4: 探测本机 Keil 构建工具**

Run:

```powershell
$uv4 = Get-Command UV4.exe -ErrorAction SilentlyContinue
if ($uv4) { $uv4.Source } else { 'UV4_NOT_FOUND' }
```

Expected:

- 若找到 `UV4.exe`，执行工程编译并检查新增文件是否有编译错误。
- 若返回 `UV4_NOT_FOUND`，记录本机无法执行 Keil 编译，不伪造编译成功。

- [ ] **Step 5: 保留用户工作区改动**

Run:

```powershell
git status --short
```

Expected: 只确认状态，不提交、不回退用户已有拆分改动。

## 自检结果

- 规格覆盖：四阶段状态机、雷达坐标、稳定计数、结束保持、空闲透传、独立实例均有对应实施步骤。
- 范围检查：计划不修改公共速度接口接线、视觉命令接线、坐标表。
- 完整性扫描：计划没有未展开的实现项。
- 类型一致性：类名固定为 `LiftStepDown`，全局实例固定为 `lift_step_down`，getter 和 setter 命名与规格一致。
