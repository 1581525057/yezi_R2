# LiftAuto Vx 控制 + 雷达模式设计

## 背景

当前 `STEP_APPROACH_MIDDLE` 只控制底盘 Vy（前后方向），没有 Vx（横向）控制，导致车在台阶上可能横向偏移。左右激光传感器（DT35 ch1/ch2）有时候不可用，需要根据台阶编号选择不同传感器源。

## 需求

1. STEP_APPROACH_MIDDLE 增加 Vx 横向控制
2. 根据 `vision_block_queue` 的方块编号选择传感器模式
3. block 6、8 使用雷达模式（vision 坐标），其他使用激光模式
4. 雷达模式下 STEP_CLIMB_FORWARD 也用 vision.y_diff 替代激光判断升降完成

## 两种模式

### 激光模式（非 block 6, 8）

| 步骤 | Vy 控制 | Vx 控制 |
|------|---------|---------|
| STEP_APPROACH_Y | ch0 前激光（现有） | 无 |
| STEP_CLIMB_FORWARD | ch0 前激光 + climbed_（现有） | 无 |
| STEP_APPROACH_MIDDLE | ch0 前激光（现有） | ch1/ch2 左右激光（新增） |

**Vx 传感器选择逻辑：**
1. 左激光 ch1 < 1500mm → 用 ch1
2. ch1 超限 → 检查右激光 ch2 < 1500mm → 用 ch2
3. 都超限 → Vx = 0（不修正横向）

**Vx 速度曲线：** 梯形速度曲线，与 Vy 相同逻辑
- 横向误差 = 测量值 - 222.0mm（目标参考值）
- 使用 `LIFT_CHASSIS_SPEED` 加速度和 `LIFT_AUTO_APPROACH_VY_MPS` 最大速度

### 雷达模式（block 6, 8）

| 步骤 | Vy 控制 | Vx 控制 |
|------|---------|---------|
| STEP_APPROACH_Y | ch0 前激光（不变） | 无 |
| STEP_CLIMB_FORWARD | vision.y_diff 到目标 y 点（新增） | 无 |
| STEP_APPROACH_MIDDLE | vision.y_diff 到目标 y 点（新增） | vision.x_diff 到目标 x 点（新增） |

**雷达目标坐标：** 每个方块编号独立配置，由用户写死
- block 6: (x_ref_6, y_ref_climb_6, y_ref_middle_6)
- block 8: (x_ref_8, y_ref_climb_8, y_ref_middle_8)
- 红蓝方由用户写两套代码处理

**雷达模式 STEP_CLIMB_FORWARD：**
- 不使用 climbed_ 逻辑（雷达坐标不受台阶面干扰）
- 完成条件：vision.y_diff 到达目标 y 点 ± 容差 + 稳定计数（复用 `LIFT_AUTO_STABLE_COUNT` = 10 次）

**雷达模式 STEP_APPROACH_MIDDLE：**
- Vx: vision.x_diff 与目标 x_ref 的误差 → 梯形曲线
- Vy: vision.y_diff 与目标 y_ref 的误差 → 梯形曲线

## API 变更

### LiftAuto 新增成员

```cpp
// 新增控制量
float chassis_vx_target_;         // 底盘Vx输出

// 模式标志
uint8_t use_radar_;               // 1=雷达模式, 0=激光模式
int block_num_;                   // 当前方块编号

// 雷达目标坐标（按方块编号配置）
float radar_x_ref_;               // 雷达目标X
float radar_y_ref_climb_;         // STEP_CLIMB_FORWARD 雷达目标Y
float radar_y_ref_middle_;        // STEP_APPROACH_MIDDLE 雷达目标Y

// 激光模式参数
float LIFT_AUTO_LATERAL_REF;     // 横向目标参考值 = 222.0mm
uint32_t LIFT_AUTO_LASER_MAX_MM; // 激光有效阈值 = 1500mm
```

### LiftAuto 新增/修改方法

```cpp
// 新增：获取底盘Vx
float getChassisVxTarget(float manual_target) const;

// 新增：配置雷达目标
void setRadarTarget(float x_ref, float y_ref_climb, float y_ref_middle);

// 修改：update() 中 STEP_APPROACH_Y 进入时 pop 方块编号决定模式
```

### chassis_task.cpp 变更

```cpp
// 新增：lift_auto 接管 Vx
target_vx = lift_auto.getChassisVxTarget(target_vx);
```

## 模式初始化（进入 STEP_APPROACH_Y 时）

在 STEP_APPROACH_Y 首次进入时读取方块编号，设 `use_radar_` 标志，后续所有步骤复用：

```
1. vision_block_pop(&block_num)
2. if (block_num == 6 || block_num == 8)
       use_radar_ = 1
       // 雷达目标坐标已通过 setRadarTarget() 预设
   else
       use_radar_ = 0
```

这样 STEP_CLIMB_FORWARD 和 STEP_APPROACH_MIDDLE 都能根据 `use_radar_` 选择对应逻辑。

## 方向约定

- 激光模式：横向误差 = 测量值 - 222mm，正值表示车偏右，需要往左修正（Vx 负方向）
- 雷达模式：vision.x_diff 正负方向由用户在 setRadarTarget 中协调
- 具体正负号在实现阶段根据实车调试确定
