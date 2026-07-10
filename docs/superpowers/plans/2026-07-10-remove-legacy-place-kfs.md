# Remove Legacy KFS Placement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 删除旧版 `CONBAT_PLACE_KFS` 状态机及其专用代码，只保留新版 KFS 放置流程。

**Architecture:** 在现有 `CONBAT_TASK` 内做依赖闭包式清理：先删除头文件中的旧接口和状态数据，再删除实现文件中的入口、复位、参数和函数实现。新版流程继续复用现有放置点选择与停稳判断，不引入新抽象。

**Tech Stack:** C++11、STM32H723、Arm Compiler 6、EIDE

---

### Task 1: 删除旧版状态声明和专用数据

**Files:**
- Modify: `R2_Lift_auto/MDK-ARM/TASK/conbat_task.h:12-155`

- [ ] **Step 1: 删除旧版顶层状态**

从 `ConbatState` 删除 `CONBAT_PLACE_KFS`，使枚举保留以下符号顺序：

```cpp
typedef enum
{
    CONBAT_IDLE = 0,
    CONBAT_RAMP_UP,
    CONBAT_PICK_KFS,
    CONBAT_SELECT_KFS_PLACE,
    CONBAT_RELOAD_COMBINE,
    CONBAT_COMBINE,
    CONBAT_AVOID,
    CONBAT_PLACE_KFS_NEW
} ConbatState;
```

保留原有中文注释，只删除旧状态所在行。

- [ ] **Step 2: 删除旧版子状态和成员**

完整删除 `enum PlaceKfsStep`，并删除以下成员：

```cpp
PlaceKfsStep place_kfs_step_;
uint8_t place_kfs_pick_precision_active_;
float place_kfs_back_start_x_m_;
```

保留 `PlaceKfsNewStep`、`place_kfs_new_step_`、`place_kfs_forward_start_x_m_`、
`place_kfs_forward_start_y_m_` 和 `place_kfs_forward_start_yaw_deg_`，因为新版流程仍使用它们。

- [ ] **Step 3: 删除旧函数声明并检查头文件**

删除：

```cpp
uint8_t runPlaceKfs(void);
```

运行：

```powershell
rg -n "CONBAT_PLACE_KFS,|PlaceKfsStep|place_kfs_step_|place_kfs_pick_precision_active_|place_kfs_back_start_x_m_|runPlaceKfs\(void\)" R2_Lift_auto/MDK-ARM/TASK/conbat_task.h
```

预期：无输出，退出码为 1。

### Task 2: 删除旧版状态实现

**Files:**
- Modify: `R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp:261-1476`

- [ ] **Step 1: 删除旧版复位代码**

从 `CONBAT_TASK::reset()` 和 `CONBAT_TASK::handleStateChanged()` 删除：

```cpp
place_kfs_step_ = PLACE_KFS_LOWER_ACTION;
place_kfs_pick_precision_active_ = 0U;
```

并从 `CONBAT_TASK::reset()` 删除：

```cpp
place_kfs_back_start_x_m_ = 0.0f;
```

- [ ] **Step 2: 删除旧版主状态分支**

从 `CONBAT_TASK::runOnce()` 删除完整的 `case CONBAT_PLACE_KFS` 分支，保留紧随其后的
`case CONBAT_PLACE_KFS_NEW` 分支不变。

- [ ] **Step 3: 删除旧版函数和专用参数**

完整删除 `CONBAT_TASK::runPlaceKfs()` 函数，并删除仅由该函数使用的参数：

```cpp
CONBAT_PLACE_KFS_PICK_PATH_MAX_VEL_M_S
CONBAT_PLACE_KFS_PICK_PATH_MAX_ACC_M_S2
CONBAT_KFS_PLACE_YAW_TOL_DEG
CONBAT_PLACE_KFS_FORWARD_DISTANCE_M
```

保留新版仍使用的 `CONBAT_KFS_PLACE_STOP_SPEED_LIMIT`、
`CONBAT_KFS_PLACE_STOP_STABLE_COUNT` 和 `CONBAT_PLACE_KFS_NEW_FORWARD_DISTANCE_M`。

- [ ] **Step 4: 消除共享速度参数中的旧版命名**

把定义和新版调用中的：

```cpp
CONBAT_PLACE_KFS_FORWARD_SPEED_MPS
```

统一改为：

```cpp
CONBAT_PLACE_KFS_NEW_FORWARD_SPEED_MPS
```

- [ ] **Step 5: 检查旧版符号已全部消失**

运行：

```powershell
rg -n --pcre2 "CONBAT_PLACE_KFS(?!_NEW)|runPlaceKfs\(|PlaceKfsStep|PLACE_KFS_(?!NEW)|place_kfs_step_|place_kfs_pick_precision_active_|place_kfs_back_start_x_m_" R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp R2_Lift_auto/MDK-ARM/TASK/conbat_task.h
```

预期：无输出，退出码为 1。

### Task 3: 编译和差异验证

**Files:**
- Verify: `R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp`
- Verify: `R2_Lift_auto/MDK-ARM/TASK/conbat_task.h`

- [ ] **Step 1: 单独编译战斗任务源文件**

从 EIDE 已生成的编译数据库读取并执行 `conbat_task.cpp` 的原始 Arm Compiler 6 命令：

```powershell
$db = Get-Content -Raw build/My_Princess_cpp/compile_commands.json | ConvertFrom-Json
$entry = $db | Where-Object { $_.file -like '*conbat_task.cpp' }
Invoke-Expression $entry.command
```

工作目录：`R2_Lift_auto/EIDE_Project`。预期：退出码为 0，无编译错误。

- [ ] **Step 2: 检查格式和修改范围**

运行：

```powershell
git diff --check -- R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp R2_Lift_auto/MDK-ARM/TASK/conbat_task.h
git diff --stat
git status --short
```

预期：`diff --check` 退出码为 0；源码修改仅新增 `conbat_task.cpp/.h`，既有
`route_task.cpp/.h` 修改保持原样。

- [ ] **Step 3: 提交实现文件**

运行：

```powershell
git add R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp R2_Lift_auto/MDK-ARM/TASK/conbat_task.h
git commit -m "refactor: 删除旧版 KFS 放置流程"
```

预期：提交只包含两个 `conbat_task` 文件，不包含 `route_task.cpp/.h`。
