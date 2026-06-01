# 自动下台阶类设计

## 目标

在 `R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.h` 和
`R2_Lift_auto/MDK-ARM/Route_Plan/lift_step_down.cpp` 中新增独立的
`LiftStepDown` 类，实现基于雷达坐标的自动下台阶状态机。

本次只实现下台阶类本身。公共速度接口的串接、`route_task.cpp` 中视觉命令
`case 5` 的接线、`vision_block_pop()` 的消费逻辑，以及坐标表均由用户处理。

## 已确认约束

- 下台阶流程与现有上台阶 `LiftAuto` 类隔离，使用独立全局实例
  `lift_step_down`。
- 坐标全部来自雷达数据 `vision.x_diff` 和 `vision.y_diff`。
- 外部接线层负责消费一次 `vision_block_pop(&block_num)`。
- 合法方块编号范围为 `1..12`。
- 下台阶类不负责坐标查表，仅接收外部写入的目标坐标。
- 所有注释使用中文。

## 对外接口

```cpp
class LiftStepDown
{
public:
    LiftStepDown();

    void startStepDown(void);
    void stopStepDown(void);
    uint8_t isStepDownFinished(void) const;
    void update(void);

    uint8_t getLiftSwitch(uint8_t manual_switch) const;
    float getLiftLinearSpeedTarget(float manual_target) const;
    float getChassisVxTarget(float manual_target) const;
    float getChassisVyTarget(float manual_target) const;

    void setStepDownRadarTarget(float x_ref_prepare,
                                float x_ref_descend,
                                float x_ref_finish,
                                float y_ref_finish);
    void setStepDownBlockNum(int num);
};

extern LiftStepDown lift_step_down;
```

`setStepDownBlockNum()` 只保存编号，方便调试和后续接线，不负责查表。

## 状态机

### 1. 移动到下台阶准备点

- 状态名：`STEP_DOWN_MOVE_TO_PREPARE`
- 档位保持为 `1` 档，对应目标高度 `100.0f`。
- 升降轮线速度为 `0.0f`。
- 底盘只沿雷达 `x` 轴移动到 `x_ref_prepare`，`Vy` 为 `0.0f`。

### 2. 升降轮带动车辆离开台阶

- 状态名：`STEP_DOWN_DESCEND`
- 档位切换为 `2` 档，对应目标高度 `-230.0f`。
- 此时底盘悬空，因此底盘 `Vx` 和 `Vy` 均为 `0.0f`。
- 升降轮使用梯形速度曲线，按照雷达 `x` 坐标移动到
  `x_ref_descend`。

### 3. 底盘移动到终点

- 状态名：`STEP_DOWN_MOVE_TO_FINISH`
- 档位切回 `1` 档，对应目标高度 `100.0f`。
- 不等待高度轨迹完成。
- 升降轮线速度归零。
- 底盘立即按雷达坐标移动到终点 `x_ref_finish, y_ref_finish`。

### 4. 流程结束

- 状态名：`STEP_DOWN_FINISHED`
- 底盘速度和升降轮线速度均归零。
- 保持自动输出，直到外部调用 `stopStepDown()`，避免结束瞬间恢复手动输入造成误动作。
- `stopStepDown()` 复位后，所有 getter 恢复透传输入值。

## 速度和到位判定

- 使用与上台阶相同形式的梯形速度函数：
  `sqrtf(2.0f * fabsf(error) * acc)`，并进行最大速度限幅。
- 每个目标点的到位误差阈值为 `0.05f`。
- 目标条件连续满足 `10` 个更新周期后才切换状态，避免雷达数据抖动造成误触发。
- 第 1 阶段和第 3 阶段输出底盘速度。
- 第 2 阶段只输出升降轮线速度。

## 透传规则

- 流程处于空闲状态时，所有 getter 原样返回传入的手动或上游自动目标。
- 流程开始后，getter 返回当前下台阶状态机生成的目标。
- 流程结束但尚未调用 `stopStepDown()` 时，getter 返回零速度和 `1` 档。

这使调用方可以谨慎地将多个自动流程串联起来，并由接线层决定调用顺序。

## 外部接线约定

以下内容不在本次实现范围内，但接线时需要满足：

- 视觉动作命令 `case 5` 触发下台阶。
- 接线层消费一次 `vision_block_pop(&block_num)`。
- 仅接受 `1..12` 范围内的方块编号。
- 接线层查找四个目标值：
  `x_ref_prepare`、`x_ref_descend`、`x_ref_finish`、`y_ref_finish`。
- 接线层调用 `setStepDownBlockNum()` 和 `setStepDownRadarTarget()`。
- 下台阶运行期间，接线层周期调用 `update()`。

## 验证

- 检查新增头文件和实现文件的接口一致性。
- 检查四个状态的输出目标和切换条件。
- 检查空闲状态和结束状态的透传规则。
- 在工程可用的情况下执行编译检查。

当前工程未发现现成单元测试框架。本次不引入新的测试基础设施。
