# 1 区跑点后接精确定位设计

## 背景

当前 `one_two_start()` 负责 1 区路径跟随，并通过 `path_active_` 接管底盘速度。`FIRST_RELOCATION` 属于梅林精确定位阶段，会通过 `RouteTask_IsMeilingAreaActive()` 让梅林定位接管底盘速度。

如果在 `FIRST_RELOCATION` 内继续调用 `one_two_start()`，跑点和梅林会同时申请底盘速度，`chassis_task` 会判定自动来源冲突并停车。

## 目标

让 1 区跑点只运行一次流程段：跑点未完成时由跑点速度接管；跑点完成后释放跑点速度，再进入 `FIRST_RELOCATION` 做精确定位。

## 方案

1. 新增一个独立阶段，例如 `PHASE_FIRST_PATH`，专门运行 1 区路径跟随。
2. `vision_choice()` 收到 1 区启动命令时进入 `PHASE_FIRST_PATH`，不直接进入 `FIRST_RELOCATION`。
3. `one_two_start()` 从 `void` 改为返回状态：
   - `0U`：跑点仍在运行。
   - `1U`：跑点完成。
   - `2U`：跑点偏离或异常。
4. `PHASE_FIRST_PATH` 中调用 `one_two_start()`。返回 `1U` 后清空跑点输出，重置 `relocation_number`，切到 `FIRST_RELOCATION`。
5. `FIRST_RELOCATION` 中只运行精确定位，不再调用 `one_two_start()`。
6. `FIRST_RELOCATION` 启动 `first_relocation` 目标，不使用 `second_relocation`。

## 速度接管

`PHASE_FIRST_PATH` 时，`path_active_ = 1U`，`chassis_task` 使用跑点速度。

跑点完成后，`one_two_start()` 清空 `path_active_`，`PHASE_FIRST_PATH` 切到 `FIRST_RELOCATION`。下一周期 `RouteTask_IsMeilingAreaActive()` 识别到 `FIRST_RELOCATION`，梅林定位接管速度。

这样两个自动来源不会同时有效。

## 验证

新增或扩展路线测试：

1. 跑点未完成时保持 `PHASE_FIRST_PATH`，跑点速度 active。
2. 跑点完成后进入 `FIRST_RELOCATION`，跑点速度释放。
3. `FIRST_RELOCATION` 首次运行启动 `first_relocation`，不再调用路径跟随。
4. 现有路径跟随和路线状态机测试保持通过。

## 范围

只修改 `route_task.h`、`route_task.cpp` 和对应测试。`chassis_task` 的仲裁规则保持不变。
