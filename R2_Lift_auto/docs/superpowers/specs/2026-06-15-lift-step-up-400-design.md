# 上 400 台阶流程设计

## 背景

当前 `LiftAuto` 的上台阶流程只覆盖上 200。视觉命令 `case 3` 触发上 200，`case 4` 预留给上 400，但尚未实现。`lift_class.cpp` 已经支持 `now_sw == 3U` 时生成 `+200mm` 高度目标。

## 目标

在不影响上 200 行为的前提下，为 `case 4` 增加上 400 台阶流程。上 400 的第一步、等待 `-200`、爬升前进和回中点逻辑尽量复用上 200 的现有状态。

## 流程

1. `case 4` 与 `case 3` 一样读取方块编号、雷达中点、爬升方向，然后设置 `LiftAuto` 为上 400 模式并进入 `PHASE_STEP_UP`。
2. `STEP_UP_APPROACH_Y` 保持与上 200 完全一致。靠近稳定后，上 400 先进入新增的等待 `+200` 状态。
3. 新增状态输出 `lift_switch_target_ = 3U`，底盘和升降轮保持不动。等待 `lift_calulate.command_seq` 变化且 `lift_calulate.finished == 1U` 后，执行打开气缸宏，再进入 `STEP_UP_WAIT_CLIMB_HEIGHT`。
4. `STEP_UP_WAIT_CLIMB_HEIGHT` 保持与上 200 一致，输出 `lift_switch_target_ = 2U`，等待 `-200` 高度完成。
5. `STEP_UP_CLIMB_FORWARD` 保持爬升判断和运动逻辑一致。爬升完成时，执行收气缸宏，同时输出 `lift_switch_target_ = 1U`，记录回收高度命令序号并进入 `STEP_UP_WAIT_NEW_HEIGHT`。
6. `STEP_UP_WAIT_NEW_HEIGHT` 等待 1 档回收完成，再进入 `STEP_UP_APPROACH_MIDDLE`。
7. `STEP_UP_APPROACH_MIDDLE` 保持与上 200 一致，回到中点后完成流程。

## 气缸接口

在 `lift_step_up.cpp` 中新增空宏，便于后续填实际 IO：

```cpp
#ifndef STEP_UP_CYLINDER_OPEN
#define STEP_UP_CYLINDER_OPEN() do {} while (0)
#endif

#ifndef STEP_UP_CYLINDER_CLOSE
#define STEP_UP_CYLINDER_CLOSE() do {} while (0)
#endif
```

## 验证

新增或扩展 `lift_step_up_test.cpp`：

1. 上 400 靠近完成后先输出 3 档，且底盘和升降轮停止。
2. 3 档未完成前不进入 2 档。
3. 3 档完成后进入 2 档等待 `-200`。
4. 爬升完成后切到 1 档并等待回收高度完成。
5. 现有上 200 测试保持通过，证明旧流程未被破坏。

## 范围

只修改 `lift_step_up.h`、`lift_step_up.cpp`、`route_task.cpp` 和对应测试。`lift_class.cpp` 已具备 3 档目标高度，不做额外改动。
