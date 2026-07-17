# KFS 激光 Y 向精定位实施计划

> **执行说明：** 项目要求不加载 superpower skills，因此本计划由当前会话直接逐项执行，不调用 superpower 执行技能或子代理。

**目标：** 在两个 KFS 视觉粗定位点之后，分别使用蓝方右激光、红方左激光完成 Y 向 30 mm 精定位。

**架构：** 保留现有两个跑点状态，增加一个可复位的激光子阶段标志。两个状态共用一个私有激光对齐函数，集中处理阵营传感器选择、目标索引、误差方向、稳定确认和纯世界系 Y 速度输出。

**技术栈：** C++11、STM32H723、Keil MDK/ARM Compiler 6、现有 `LaserDistance` 和 `PathFollower` 接口。

---

### 任务 1：增加激光参数与子阶段状态

**文件：**

- 修改：`R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp`
- 修改：`R2_Lift_auto/MDK-ARM/TASK/conbat_task.h`

- [x] **步骤 1：确认修改前差异**

运行：

```powershell
git diff -- R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp R2_Lift_auto/MDK-ARM/TASK/conbat_task.h
```

预期：`conbat_task.cpp` 中仅显示用户已有修改；`conbat_task.h` 无本任务修改。

- [x] **步骤 2：增加激光依赖和可填写目标表**

在 `conbat_task.cpp` 直接包含激光头文件，并在 KFS 参数区加入：

```cpp
#include "laser_distance.h"

float CONBAT_PICK_KFS_LASER_TARGET_MM[2][2] = {
    {0.0f, 0.0f}, // 蓝方：第一个、第二个 KFS 位置。
    {0.0f, 0.0f}, // 红方：第一个、第二个 KFS 位置。
};
static const float CONBAT_PICK_KFS_LASER_Y_TOL_MM = 30.0f;
```

- [x] **步骤 3：增加并复位子阶段成员**

在 `conbat_task.h` 的 KFS 成员区加入：

```cpp
uint8_t pick_kfs_laser_align_active_;
```

在 `reset()` 和 `handleStateChanged()` 中都将其设为 `0U`，保证重新启动流程时从视觉粗定位开始。

### 任务 2：实现共用激光对齐控制

**文件：**

- 修改：`R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp`
- 修改：`R2_Lift_auto/MDK-ARM/TASK/conbat_task.h`

- [x] **步骤 1：声明私有对齐函数**

在 `conbat_task.h` 中加入：

```cpp
uint8_t runPickKfsLaserAlign(uint8_t field_side_index,
                             uint8_t goal_index,
                             uint8_t stable_target_count);
```

- [x] **步骤 2：实现传感器选择和安全停车**

函数按 `field_side_index` 选择传感器：蓝方索引 `0U` 使用 `laser_right`，红方索引 `1U` 使用 `laser_left`。当目标值不大于零或 `laser.data.valid == 0U` 时调用 `clearPathOutput()`、清零稳定计数并返回未完成。

- [x] **步骤 3：实现误差方向和到位判定**

核心计算为：

```cpp
const float measured_mm = static_cast<float>(laser.data.distance_mm);
const float laser_y_err_mm = (field_side_index == 0U)
                                 ? (target_mm - measured_mm)
                                 : (measured_mm - target_mm);
```

对 `fabsf(laser_y_err_mm) < CONBAT_PICK_KFS_LASER_Y_TOL_MM` 使用现有 `conbat_stable_confirm()`；稳定后停车、清零计数和子阶段标志，并返回完成。

- [x] **步骤 4：实现纯世界系 Y 闭环**

沿用现有 P 闭环参数和限加速度逻辑：X 误差传 `0.0f`，Y 误差传 `laser_y_err_mm * 0.001f`；将世界系速度通过 `PathFollower::worldToBody()` 转为底盘速度，最后设置 `path_wz_target_ = 0.0f` 和 `path_active_ = 1U`。

### 任务 3：接入两个 KFS 跑点状态并验证

**文件：**

- 修改：`R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp`

- [x] **步骤 1：接入第一个 KFS 位置**

视觉稳定后不再直接进入 `PICK_KFS_FIRST_WAIT_READY`，改为停车、清零计数、设置 `pick_kfs_laser_align_active_ = 1U`。子阶段激活时调用：

```cpp
if (runPickKfsLaserAlign(field_side_index, 0U, 50U) != 0U)
{
    pick_kfs_step_ = PICK_KFS_FIRST_WAIT_READY;
}
return 0U;
```

- [x] **步骤 2：接入第二个 KFS 位置**

第二个视觉跑点以同样方式切换子阶段，并调用：

```cpp
if (runPickKfsLaserAlign(field_side_index, 1U, 10U) != 0U)
{
    pick_kfs_step_ = PICK_KFS_SECOND_WAIT_READY;
}
return 0U;
```

进入 `PICK_KFS_FIRST_ACTION` 和 `PICK_KFS_SECOND_ACTION` 时均重置子阶段标志。

- [x] **步骤 3：执行静态检查**

运行：

```powershell
git diff --check
rg -n "CONBAT_PICK_KFS_LASER_TARGET_MM|laser_right|laser_left|runPickKfsLaserAlign|pick_kfs_laser_align_active_" R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp R2_Lift_auto/MDK-ARM/TASK/conbat_task.h
```

预期：无空白错误；目标索引 0/1 均有调用；蓝方分支使用右激光，红方分支使用左激光。

- [x] **步骤 4：执行固件编译**

运行 Keil 命令行全量编译：

```powershell
& 'D:\Keil5\UV4\UV4.exe' -b 'R2_Lift_auto\MDK-ARM\My_Princess_cpp.uvprojx' -j0 -o 'R2_Lift_auto\EIDE_Project\build\codex_uv4_build_kfs_laser_align_20260713.log'
```

预期：日志末尾显示 `0 Error(s)`；若工程既有警告，单独报告且不扩大修改范围。

- [x] **步骤 5：复核最终差异**

运行：

```powershell
git diff -- R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp R2_Lift_auto/MDK-ARM/TASK/conbat_task.h
```

预期：所有新增行都直接对应激光目标表、阶段复位、激光对齐控制或两个跑点接入，没有无关格式化和重构。
