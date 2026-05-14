# LiftAuto Vx 控制 + 雷达模式 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 STEP_APPROACH_MIDDLE 增加 Vx 横向控制，支持激光/雷达两种传感器模式

**Architecture:** 扩展 LiftAuto 类，新增 chassis_vx_target_ 控制量和 use_radar_ 模式标志。进入 STEP_APPROACH_Y 时从 vision_block_queue 读取方块编号决定模式。激光模式用 DT35 ch1/ch2 控 Vx，雷达模式用 vision.x_diff/y_diff 控 Vx+Vy。

**Tech Stack:** C++ (嵌入式 STM32H7), FreeRTOS, DT35 激光传感器, USB CDC 视觉通信

**Spec:** `docs/superpowers/specs/2026-05-08-lift-auto-vx-radar-design.md`

---

### Task 1: lift_auto.h — 新增成员和方法声明

**Files:**
- Modify: `MDK-ARM/Route_Plan/lift_auto.h:34-55`

在 LiftAuto 类中新增 Vx 控制量、模式标志、雷达目标坐标和方法声明。

- [ ] **Step 1: 在 private 成员区新增变量**

在 `chassis_vy_target_` 后面添加：

```cpp
    float chassis_vx_target_;        // 底盘Vx输出（横向速度）
```

在 `climbed_` 后面添加：

```cpp
    // 雷达/激光模式选择
    uint8_t use_radar_;              // 1=雷达模式, 0=激光模式

    // 雷达目标坐标
    float radar_x_ref_;              // 雷达目标X
    float radar_y_ref_climb_;        // STEP_CLIMB_FORWARD 雷达目标Y
    float radar_y_ref_middle_;       // STEP_APPROACH_MIDDLE 雷达目标Y

    // 激光模式参数
    float lateral_ref_mm_;           // 横向目标参考值 = 222.0mm
    uint32_t laser_max_mm_;          // 激光有效阈值 = 1500mm
```

- [ ] **Step 2: 在 public 区新增方法声明**

在 `getChassisVyTarget` 后面添加：

```cpp
    float getChassisVxTarget(float manual_target) const;
    void setRadarTarget(float x_ref, float y_ref_climb, float y_ref_middle);
```

- [ ] **Step 3: 编译验证**

在 Keil 或 EIDE 中编译，确认无语法错误。

---

### Task 2: lift_auto.cpp — 新增 extern 声明和全局变量

**Files:**
- Modify: `MDK-ARM/Route_Plan/lift_auto.cpp:1-21`

在文件顶部添加 vision 相关的 extern 声明和新的全局配置变量。

- [ ] **Step 1: 添加 extern 声明**

在 `#include "mieling.h"` 后面添加：

```cpp
#include "usart_task.h"  // vision_block_pop, VisionData_t

extern VisionData_t vision;
extern uint8_t vision_block_pop(int *out);
```

- [ ] **Step 2: 添加激光模式配置变量**

在 `LIFT_AUTO_MIDDLE_MM` 定义后面添加：

```cpp
// 横向目标参考值 (mm)
float LIFT_AUTO_LATERAL_REF = 222.0f;
// 激光有效阈值 (mm)，超过此距离认为激光不可用
uint32_t LIFT_AUTO_LASER_MAX_MM = 1500U;
```

- [ ] **Step 3: 编译验证**

---

### Task 3: lift_auto.cpp — 实现新方法

**Files:**
- Modify: `MDK-ARM/Route_Plan/lift_auto.cpp` (在 getChassisVyTarget 之后)

实现 getChassisVxTarget() 和 setRadarTarget() 方法。

- [ ] **Step 1: 实现 getChassisVxTarget()**

在 `getChassisVyTarget()` 方法之后添加：

```cpp
// 未接管底盘时透传手动Vx，否则返回自动Vx
float LiftAuto::getChassisVxTarget(float manual_target) const
{
    if (chassis_vy_override_ == 0U) {
        return manual_target;
    }

    return chassis_vx_target_;
}
```

- [ ] **Step 2: 实现 setRadarTarget()**

```cpp
// 配置雷达模式的目标坐标
void LiftAuto::setRadarTarget(float x_ref, float y_ref_climb, float y_ref_middle)
{
    radar_x_ref_        = x_ref;
    radar_y_ref_climb_  = y_ref_climb;
    radar_y_ref_middle_ = y_ref_middle;
}
```

- [ ] **Step 3: 编译验证**

---

### Task 4: lift_auto.cpp — reset() 清零新增成员

**Files:**
- Modify: `MDK-ARM/Route_Plan/lift_auto.cpp:47-57` (reset 方法)

在 reset() 中清零所有新增成员。

- [ ] **Step 1: 在 reset() 末尾添加清零代码**

在 `climbed_ = 0U;` 之后添加：

```cpp
    chassis_vx_target_ = 0.0f;
    use_radar_         = 0U;
    radar_x_ref_       = 0.0f;
    radar_y_ref_climb_ = 0.0f;
    radar_y_ref_middle_ = 0.0f;
    lateral_ref_mm_    = LIFT_AUTO_LATERAL_REF;
    laser_max_mm_      = LIFT_AUTO_LASER_MAX_MM;
```

- [ ] **Step 2: 编译验证**

---

### Task 5: lift_auto.cpp — STEP_APPROACH_Y 读取方块编号

**Files:**
- Modify: `MDK-ARM/Route_Plan/lift_auto.cpp:72-74` (STEP_IDLE → STEP_APPROACH_Y 转换)

在首次进入 STEP_APPROACH_Y 时从 vision_block_queue 读取方块编号，决定使用雷达还是激光模式。

- [ ] **Step 1: 修改状态转换逻辑**

将现有的：

```cpp
    // 首次进入自动流程
    if (state_ == STEP_IDLE) {
        state_ = STEP_APPROACH_Y;
    }
```

改为：

```cpp
    // 首次进入自动流程
    if (state_ == STEP_IDLE) {
        // 从方块队列读取编号，决定传感器模式
        int block_num = 0;
        vision_block_pop(&block_num);
        if (block_num == 6 || block_num == 8) {
            use_radar_ = 1U;
        } else {
            use_radar_ = 0U;
        }
        state_ = STEP_APPROACH_Y;
    }
```

- [ ] **Step 2: 编译验证**

---

### Task 6: lift_auto.cpp — STEP_CLIMB_FORWARD 雷达模式

**Files:**
- Modify: `MDK-ARM/Route_Plan/lift_auto.cpp:111-146` (STEP_CLIMB_FORWARD case)

在 STEP_CLIMB_FORWARD 中增加雷达模式分支。

- [ ] **Step 1: 在 STEP_CLIMB_FORWARD 开头添加模式分支**

将现有的 STEP_CLIMB_FORWARD case 改为：

```cpp
        case STEP_CLIMB_FORWARD:
            chassis_vy_override_ = 1U;
            chassis_vy_target_   = 0.0f;
            lift_switch_target_  = 2U;

            if (use_radar_ != 0U) {
                // 雷达模式：用 vision.y_diff 走到目标 y 点
                float y_err = radar_y_ref_climb_ - vision.y_diff;
                float dist  = fabsf(y_err) * 0.001f;
                float speed = sqrtf(2.0f * LIFT_ACC_SPEED * dist);
                if (speed > LIFT_AUTO_CLIMB_SPEED_MPS) {
                    speed = LIFT_AUTO_CLIMB_SPEED_MPS;
                }
                // y_err > 0 表示还没到，需要前进（Vy 正方向需实车确认）
                lift_linear_speed_target_ = (y_err > 0.0f) ? speed : -speed;

                // 到位判定
                if (fabsf(y_err) < 10.0f) {
                    if (stable_count_ < LIFT_AUTO_STABLE_COUNT) {
                        stable_count_++;
                    }
                } else {
                    stable_count_ = 0U;
                }

                if (stable_count_ >= LIFT_AUTO_STABLE_COUNT) {
                    lift_switch_target_       = 1U;
                    lift_linear_speed_target_ = 0.0f;
                    stable_count_             = 0U;
                    state_                    = STEP_APPROACH_MIDDLE;
                }
            } else {
                // 激光模式：原有逻辑不变
                // 【新增】激光读数先升到高处（超过FINISH_MM），标记已爬升
                if (laser_valid != 0U && laser_mm > LIFT_AUTO_FINISH_MM && climbed_ != 1) {
                    climbed_ = 1;
                }

                // 梯形速度曲线升降
                if (laser_valid != 0U && laser_mm >= LIFT_AUTO_FINISH_MM && climbed_ == 1U) {
                    float dist                = (float)(laser_mm - LIFT_AUTO_FINISH_MM) * 0.001f;
                    float linear_speed        = sqrtf(2.0f * LIFT_ACC_SPEED * dist);
                    lift_linear_speed_target_ = linear_speed > LIFT_AUTO_CLIMB_SPEED_MPS ? LIFT_AUTO_CLIMB_SPEED_MPS : linear_speed;
                } else {
                    lift_linear_speed_target_ = 0.0f;
                }

                // 必须先爬升到高处，再降回FINISH_MM以下才算完成
                if (climbed_ != 0U && laser_valid != 0U && laser_mm <= LIFT_AUTO_FINISH_MM) {
                    stable_count_++;
                } else {
                    stable_count_ = 0;
                }

                if (stable_count_ >= LIFT_AUTO_STABLE_COUNT) {
                    lift_switch_target_       = 1U;
                    lift_linear_speed_target_ = 0.0f;
                    stable_count_             = 0U;
                    state_                    = STEP_APPROACH_MIDDLE;
                }
            }
            break;
```

- [ ] **Step 2: 编译验证**

---

### Task 7: lift_auto.cpp — STEP_APPROACH_MIDDLE 增加 Vx（激光模式）

**Files:**
- Modify: `MDK-ARM/Route_Plan/lift_auto.cpp:148-176` (STEP_APPROACH_MIDDLE case)

在激光模式下增加 Vx 横向控制。

- [ ] **Step 1: 重写 STEP_APPROACH_MIDDLE case**

将整个 `case STEP_APPROACH_MIDDLE` 替换为：

```cpp
        case STEP_APPROACH_MIDDLE: {
            chassis_vy_override_      = 1U;
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;

            if (use_radar_ != 0U) {
                // ========== 雷达模式 ==========
                // Vx: vision.x_diff 走到目标 x 点
                float x_err = radar_x_ref_ - vision.x_diff;
                float x_dist = fabsf(x_err) * 0.001f;
                float x_speed = sqrtf(2.0f * LIFT_CHASSIS_SPEED * x_dist);
                if (x_speed > LIFT_AUTO_APPROACH_VY_MPS) {
                    x_speed = LIFT_AUTO_APPROACH_VY_MPS;
                }
                chassis_vx_target_ = (x_err > 0.0f) ? x_speed : -x_speed;

                // Vy: vision.y_diff 走到目标 y 点
                float y_err = radar_y_ref_middle_ - vision.y_diff;
                float y_dist = fabsf(y_err) * 0.001f;
                float y_speed = sqrtf(2.0f * LIFT_CHASSIS_SPEED * y_dist);
                if (y_speed > LIFT_AUTO_APPROACH_VY_MPS) {
                    y_speed = LIFT_AUTO_APPROACH_VY_MPS;
                }
                chassis_vy_target_ = (y_err > 0.0f) ? y_speed : -y_speed;

                // 到位判定：x 和 y 误差都在容差内
                if (fabsf(x_err) < 10.0f && fabsf(y_err) < 10.0f
                    && fabsf(chassis_vx_target_) < 0.1f && fabsf(chassis_vy_target_) < 0.1f) {
                    if (stable_count_ < LIFT_AUTO_STABLE_COUNT) {
                        stable_count_++;
                    }
                } else {
                    stable_count_ = 0U;
                }

                if (stable_count_ >= LIFT_AUTO_STABLE_COUNT) {
                    chassis_vx_target_ = 0.0f;
                    chassis_vy_target_ = 0.0f;
                    stable_count_      = 0U;
                    state_             = STEP_FINISHED;
                }
            } else {
                // ========== 激光模式 ==========
                // Vy: 前激光 ch0 控制（现有逻辑）
                if (laser_valid != 0U) {
                    float dist         = (float)(laser_mm - LIFT_AUTO_MIDDLE_MM) * 0.001f;
                    float speed        = sqrtf(2.0f * dist * LIFT_CHASSIS_SPEED);
                    chassis_vy_target_ = speed > LIFT_AUTO_APPROACH_VY_MPS ? LIFT_AUTO_APPROACH_VY_MPS : speed;
                } else {
                    chassis_vy_target_ = 0.0f;
                }

                // Vx: 左右激光横向修正（新增）
                float lateral_mm   = 0.0f;
                uint8_t lateral_ok = 0U;

                // 优先左激光 ch1
                if (dt35.ch1.valid != 0U && dt35.ch1.distance_filtered < (float)laser_max_mm_) {
                    lateral_mm = dt35.ch1.distance_filtered;
                    lateral_ok = 1U;
                }
                // 左激光超限，尝试右激光 ch2
                else if (dt35.ch2.valid != 0U && dt35.ch2.distance_filtered < (float)laser_max_mm_) {
                    lateral_mm = dt35.ch2.distance_filtered;
                    lateral_ok = 1U;
                }

                if (lateral_ok != 0U) {
                    float lat_err  = lateral_mm - lateral_ref_mm_;
                    float lat_dist = fabsf(lat_err) * 0.001f;
                    float lat_speed = sqrtf(2.0f * LIFT_CHASSIS_SPEED * lat_dist);
                    if (lat_speed > LIFT_AUTO_APPROACH_VY_MPS) {
                        lat_speed = LIFT_AUTO_APPROACH_VY_MPS;
                    }
                    // lat_err > 0 表示车偏右，需要往左修正（Vx 负方向，需实车确认）
                    chassis_vx_target_ = (lat_err > 0.0f) ? -lat_speed : lat_speed;
                } else {
                    chassis_vx_target_ = 0.0f;
                }

                // 到位判定：Vy 到达中间距离 + Vx 横向收敛
                if (laser_valid != 0U && laser_mm <= (LIFT_AUTO_MIDDLE_MM + 10) && laser_mm >= (LIFT_AUTO_MIDDLE_MM - 10)
                    && fabsf(chassis_vy_target_) < 0.1f && fabsf(chassis_vx_target_) < 0.1f) {
                    if (stable_count_ < LIFT_AUTO_STABLE_COUNT) {
                        stable_count_++;
                    }
                } else {
                    stable_count_ = 0U;
                }

                if (stable_count_ >= LIFT_AUTO_STABLE_COUNT) {
                    chassis_vx_target_ = 0.0f;
                    chassis_vy_target_ = 0.0f;
                    stable_count_      = 0U;
                    state_             = STEP_FINISHED;
                }
            }

            break;
        }
```

- [ ] **Step 2: 编译验证**

---

### Task 8: lift_auto.cpp — STEP_FINISHED 清理新增状态

**Files:**
- Modify: `MDK-ARM/Route_Plan/lift_auto.cpp:178-184` (STEP_FINISHED case)

在 STEP_FINISHED 中清零 Vx 和 use_radar_。

- [ ] **Step 1: 在 STEP_FINISHED 中添加清理代码**

在现有代码后追加 `chassis_vx_target_ = 0.0f;` 和 `use_radar_ = 0U;`：

```cpp
        case STEP_FINISHED:
            // 释放底盘控制权，升降回1档，交还手动
            chassis_vy_override_      = 0U;
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;
            chassis_vx_target_        = 0.0f;
            use_radar_                = 0U;
            climbed_                  = 0;
            break;
```

- [ ] **Step 2: 编译验证**

---

### Task 9: chassis_task.cpp — 集成 Vx 控制

**Files:**
- Modify: `MDK-ARM/TASK/chassis_task.cpp:120-121`

在 lift_auto.getChassisVyTarget 旁边增加 Vx 接管。

- [ ] **Step 1: 添加 Vx 接管**

将现有：

```cpp
        target_vy = lift_auto.getChassisVyTarget(target_vy);
        omni_chassis.setRemote(target_vx, target_vy,remove_dji.chassis_.Vz);
```

改为：

```cpp
        target_vy = lift_auto.getChassisVyTarget(target_vy);
        target_vx = lift_auto.getChassisVxTarget(target_vx);
        omni_chassis.setRemote(target_vx, target_vy,remove_dji.chassis_.Vz);
```

- [ ] **Step 2: 编译验证**

---

### Task 10: 全量编译验证

**Files:**
- 无修改，仅编译

- [ ] **Step 1: Keil 全量编译**

在 Keil MDK 中执行 Rebuild All，确认零错误零警告（或仅有已知警告）。

- [ ] **Step 2: 检查 .map 文件无符号冲突**

确认 `getChassisVxTarget`、`setRadarTarget` 等新符号正确出现在 map 文件中。
