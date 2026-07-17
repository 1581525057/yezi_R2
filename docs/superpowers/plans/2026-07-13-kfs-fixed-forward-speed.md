# KFS 固定前进速度实现计划

**目标：** 两个 KFS 边走边吸取阶段脱离 DT35，以车体 X 轴固定 `0.3 m/s` 前进，并在 `event == 1U` 时保持现有停车与状态跳转。

**实现方式：** 仅修改 `conbat_task.cpp` 中两个等待分支及其专用 DT35 参数。主分支当前没有可直接运行的 `CONBAT_TASK` 单元测试框架，因此增加一个聚焦的源码契约测试，先证明旧代码不满足固定速度约束，再完成最小实现并用固件编译验证语法和链接。

**约束：** 按仓库指令在当前会话内执行，不加载 superpower skill，不使用子代理。

---

### 任务 1：建立失败的回归测试

**文件：**

- 新建：`R2_Lift_auto/MDK-ARM/host_tests/test_conbat_kfs_fixed_forward.py`
- 检查：`R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp`

- [x] 编写测试，分别截取 `PICK_KFS_FIRST_WAIT_DONE` 和 `PICK_KFS_SECOND_WAIT_READY` 分支，断言启动后的路径输出为 `path_vx_target_ = 0.3f`，且分支不再读取 `dt35.ch2`。
- [x] 断言两个专用 DT35 目标参数已删除，event=1 对应的后退状态跳转仍存在。
- [x] 运行 `python R2_Lift_auto/MDK-ARM/host_tests/test_conbat_kfs_fixed_forward.py -v`，确认旧代码因仍使用 DT35 且没有 `0.3f` 固定输出而失败。

### 任务 2：完成最小状态机修改

**文件：**

- 修改：`R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp`

- [x] 删除两个 DT35 目标参数，以及修改后不再使用的等待阶段梯形速度参数。
- [x] 在 `PICK_KFS_FIRST_WAIT_DONE` 中保留 event=1 的优先停车与跳转，然后直接输出 `path_active_ = 1U`、`path_vx_target_ = 0.3f`、Y 轴与角速度为零。
- [x] 在 `PICK_KFS_SECOND_WAIT_READY` 中保留 event=5 启动门槛和 event=1 的优先停车与跳转，启动后直接输出相同固定速度。
- [x] 修改涉及的阶段注释为中文且与新行为一致。
- [x] 重新运行聚焦测试，确认全部通过。

### 任务 3：整体校验

**文件：**

- 检查：`R2_Lift_auto/MDK-ARM/TASK/conbat_task.cpp`
- 检查：`R2_Lift_auto/MDK-ARM/host_tests/test_conbat_kfs_fixed_forward.py`

- [x] 运行项目现有固件构建命令，确认修改可以编译和链接。
- [x] 对本次涉及文件运行 `git diff --check`，确认没有空白错误；全仓检查仅保留原有 `My_Princess_cpp.uvprojx` 末尾空行告警。
- [x] 检查最终差异，确认没有覆盖工作区中原有的其他修改。
