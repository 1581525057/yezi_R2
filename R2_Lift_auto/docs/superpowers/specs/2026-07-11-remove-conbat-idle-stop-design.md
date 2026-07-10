# 删除 CONBAT 空闲停车接管

## 目标

删除 `CONBAT_IDLE_CHASSIS_STOP_ENABLE` 及其对应的空闲停车分支，使 `CONBAT_IDLE` 状态始终透传遥控底盘目标，不再占用底盘自动控制源。

## 设计

- 删除 `CONBAT_IDLE_CHASSIS_STOP_ENABLE` 全局变量。
- 删除 `CONBAT_TASK::getChassisTarget()` 中将空闲状态三轴目标清零并返回 `1U` 的分支。
- 保留函数开头对 `manual_vx`、`manual_vy`、`manual_wz` 的赋值。
- 空闲状态通过现有 `isActive() == 0U` 分支返回 `0U`，由底盘仲裁继续使用遥控目标。
- 非空闲状态的路径速度接管和安全停车逻辑保持不变。

## 验证

- 搜索确认源码中不再存在 `CONBAT_IDLE_CHASSIS_STOP_ENABLE`。
- 验证空闲调用路径不会改写手动目标，并返回未接管。
- 执行项目可用的编译或静态检查，确认删除后没有引用错误。
