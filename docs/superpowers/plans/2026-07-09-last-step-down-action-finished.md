# Last Step-Down Action Finished Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 当动作列表最后一项为下 200 mm 或下 400 mm 时，在该动作实际完成后置位公开的动作完成标志。

**Architecture:** 路由任务在弹出命令后立即判断动作队列是否为空，并把“当前下台阶动作是否为末项”锁存在私有成员中。下台阶状态机报告完成时再读取该锁存值并置位公开标志，从而区分“命令已取出”和“动作已完成”。

**Tech Stack:** STM32H7、C++、FreeRTOS、EIDE/Arm Compiler 6

---

### Task 1: 锁存末项并在下台阶完成后置位

**Files:**
- Modify: `R2_Lift_auto/MDK-ARM/TASK/route_task.h`
- Modify: `R2_Lift_auto/MDK-ARM/TASK/route_task.cpp`

- [ ] **Step 1: 在头文件中定义状态**

在 `ROUTE_TASK` 私有成员中加入：

```cpp
uint8_t last_step_down_is_final_action_; // 当前下台阶动作是否为动作列表最后一项。
```

在公开成员中加入：

```cpp
uint8_t flag_action_finished; // 动作列表最后一个下台阶动作已完成。
```

- [ ] **Step 2: 在路线复位时清零状态**

在 `ROUTE_TASK::route_reset()` 的流程标志初始化区域加入：

```cpp
flag_action_finished = 0U;
last_step_down_is_final_action_ = 0U;
```

- [ ] **Step 3: 弹出动作后锁存末项状态**

在 `ROUTE_TASK::vision_choice()` 成功弹出 `cmd` 后，先清除上一列表的完成状态，并仅为末项下台阶命令设置锁存：

```cpp
flag_action_finished = 0U;
last_step_down_is_final_action_ =
    ((cmd == 5 || cmd == 6) && vision_command_has_pending() == 0U) ? 1U : 0U;
```

这段逻辑必须位于 `switch (cmd)` 之前，保证命令 5/6 开始执行时已经保存队列状态。

- [ ] **Step 4: 下台阶实际完成后置位**

在 `PHASE_STEP_DOWN` 的完成分支中，停止下台阶流程后加入：

```cpp
if (last_step_down_is_final_action_ != 0U)
{
    flag_action_finished = 1U;
}
last_step_down_is_final_action_ = 0U;
```

保留原有 `state = PHASE_VISION;` 状态跳转。

- [ ] **Step 5: 检查差异和格式**

Run:

```powershell
git diff --check -- R2_Lift_auto/MDK-ARM/TASK/route_task.h R2_Lift_auto/MDK-ARM/TASK/route_task.cpp
git diff -- R2_Lift_auto/MDK-ARM/TASK/route_task.h R2_Lift_auto/MDK-ARM/TASK/route_task.cpp
```

Expected: `git diff --check` 无输出且退出码为 0；差异只包含上述两个状态成员、复位、末项锁存和完成置位。

- [ ] **Step 6: 验证三个关键分支**

逐项检查源代码满足：

```text
[5] 或 [6]：弹出后队列为空，完成下台阶后 flag_action_finished == 1
[5, 7] 或 [6, 7]：弹出 5/6 后队列非空，完成下台阶后 flag_action_finished == 0
新动作开始或 route_reset()：flag_action_finished == 0
```

若本机 EIDE/Arm Compiler 6 构建命令可用，执行固件全量构建；若命令行构建入口不可用，则在 EIDE 中执行 Build，并确认 `route_task.cpp` 无编译错误。

- [ ] **Step 7: 提交实现**

```powershell
git add -- R2_Lift_auto/MDK-ARM/TASK/route_task.h R2_Lift_auto/MDK-ARM/TASK/route_task.cpp
git commit -m "feat: 标记末尾下台阶动作完成"
```
