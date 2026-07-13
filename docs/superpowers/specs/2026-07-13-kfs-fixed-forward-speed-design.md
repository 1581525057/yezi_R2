# KFS 等待阶段固定前进速度设计

## 目标

第一个和第二个 KFS 边走边吸取阶段不再使用 DT35 激光距离控制 X 轴速度。阶段启动后，底盘沿车体 X 轴以固定 `0.3 m/s` 前进；机械臂回传 `event == 1U` 时立即停止，并进入现有下一状态。

## 状态机行为

- `PICK_KFS_FIRST_WAIT_DONE`：未收到 `event == 1U` 时输出 `path_active_ = 1U`、`path_vx_target_ = 0.3f`、`path_vy_target_ = 0.0f`、`path_wz_target_ = 0.0f`。收到事件后保持现有逻辑，清零路径输出并进入 `PICK_KFS_FIRST_BACKWARD`。
- `PICK_KFS_SECOND_WAIT_READY`：保留现有 `event == 5U` 启动门槛。启动后未收到 `event == 1U` 时输出相同的固定速度；收到事件后保持现有逻辑，清零路径输出并进入 `PICK_KFS_SECOND_BACKWARD`。

## 代码边界

- 删除上述两处对 `dt35.ch2.valid`、`dt35.ch2.distance_filtered` 和梯形速度函数的依赖。
- 删除只服务于这两处逻辑、修改后不再使用的两个 DT35 目标距离参数。
- 保留其他状态使用的梯形速度函数和 KFS 等待阶段速度/加速度参数，不进行无关重构。
- 新增或修改的注释全部使用中文。

## 验证标准

- DT35 通道无效时，两个已启动的边走边吸取阶段仍输出 `vx = 0.3f`。
- `event == 1U` 时路径速度立即清零，并进入各自现有的后退状态。
- 第二阶段在收到 `event == 5U` 前保持停车。
- 相关测试、编译检查通过，差异仅包含本次需求直接涉及的代码和测试。
