# 1 区跑点底盘接管设计

## 背景

`ROUTE_TASK::one_two_start()` 已经能根据视觉位姿调用 `path_follow.follow()`，并把路径跟随器输出的世界系速度转换为底盘车体系速度。但当前函数只生成局部变量 `chassis_vx/chassis_vy/chassis_wz`，没有把速度交给 `chassis_task`。

`chassis_task` 目前是底盘速度的统一入口：先取遥控速度，再通过自动来源仲裁选择梅林区或五曲区速度，最后继续经过抬升和机械臂相关限速。新跑点速度应接入这个入口，避免直接写 `omni_chassis` 造成控制权冲突。

## 目标

在 `vision.exec == 1` 时，让 1 区跑点优雅接管底盘平移和旋转速度；路径完成、偏离或视觉不再执行时，自动释放控制权并清零输出。

## 方案

采用 `ROUTE_TASK` 发布速度、`chassis_task` 统一仲裁的方式：

1. `ROUTE_TASK` 内部保存跑点接管状态和目标速度：`path_active_`、`path_vx_target_`、`path_vy_target_`、`path_wz_target_`。
2. `one_two_start()` 每周期只负责更新这些输出，不直接调用 `omni_chassis.setRemote()`。
3. `ROUTE_TASK` 对外提供只读接口：是否接管、读取 X/Y/Z 目标速度。未接管时透传传入的手动或上层速度。
4. `chassis_task` 在 `ChassisAuto_SelectSource()` 中增加路线跑点来源，与梅林区、五曲区互斥。多个自动来源同时有效时进入冲突停车。
5. `chassis_task` 最终下发速度时使用仲裁后的 `target_vz`，否则自动来源产生的旋转速度不会生效。

## 数据流

`plan_route` 周期调用 `route_t.one_two_start()`，路线任务读取 `vision` 并运行 `path_follow`，把输出缓存为车体系 `m/s` 和 `rad/s`。

`chassis_task` 周期读取 `RouteTask_IsPathActive()`。当只有路线跑点有效时，使用 `RouteTask_GetPathChassisVxTarget()`、`RouteTask_GetPathChassisVyTarget()`、`RouteTask_GetPathChassisVzTarget()` 覆盖遥控速度，然后继续经过已有的抬升和机械臂速度链路。

## 释放条件

以下情况清空路线速度并释放接管：

1. `vision.exec != 1`。
2. `path_follow.follow()` 返回 `STATE_FINISHED`。
3. `path_follow.follow()` 返回 `STATE_DEVIATED`。

释放后 `chassis_task` 回到遥控或其他自动任务的仲裁结果。

## 冲突处理

路线跑点、梅林区、五曲区属于同一层自动速度来源，只允许一个来源有效。若任意两个或三个同时有效，`chassis_task` 设置 `vx/vy/wz` 为 0，避免多任务抢速度。

抬升流程和机械臂流程仍保留在自动来源之后，对平移速度继续做已有限制，不在本次改动中重排优先级。

## 验证

新增或扩展窄测试：

1. `vision.exec != 1` 时路线跑点不接管，速度接口透传输入值。
2. `vision.exec == 1` 且路径输出有效时，路线跑点处于 active，并返回车体系速度。
3. 路径完成或偏离后，路线跑点释放并清零输出。
4. 底盘自动来源仲裁中，路线跑点与梅林区或五曲区同时 active 时返回冲突停车。

现有路径跟随器测试保持通过，证明 `PathFollower` 本身行为未被改动。

## 范围

只修改 `route_task.h`、`route_task.cpp`、`chassis_task.cpp` 和对应测试。暂不抽通用速度仲裁器，也不调整梅林、五曲、抬升、机械臂既有速度策略。
