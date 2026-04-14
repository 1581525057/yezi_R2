# DJI电机驱动

[[00-项目总览]] | [[chassis_task - 底盘控制任务]] | [[lift_task - 升降控制任务]] | [[FDCAN总线]]

---

## 模块概述

`DJI_Motor_Class`（`dji_motor.h/.cpp`）封装了大疆 GM6020、M3508、M2006 三种电机的 CAN 通信驱动，负责反馈解析和电流指令发送。

## 支持的电机类型

| 类型 | 用途 | CAN帧类型 |
|------|------|----------|
| DJI_M3508 | 底盘四轮驱动 | 标准帧 |
| DJI_M2006 | 升降辅助轮 | 标准帧 |
| DJI_GM6020 | Yaw轴云台（预留） | 标准帧 |

## 全局实例

```cpp
extern DJI_Motor_Class chassis_motor;  // 底盘电机（Chassis_Motor[0~7]）
extern DJI_Motor_Class lift_motor;     // 升降电机（Lift_2006[0~1]）
extern DJI_Motor_Class yaw_motor;      // 云台Yaw电机（DJI_Yaw_Motor，预留）
```

## 电机数据结构

```cpp
DJI_Motor_Data_Typedef {
    bool    Initlized;      // 是否已初始化
    int16_t Current;        // 实际电流
    int16_t Velocity;       // 速度（原始值）
    float   Rpm;            // 转速（rpm）
    int16_t Encoder;        // 编码器值（0~8191）
    int16_t Last_Encoder;   // 上次编码器值
    float   Angle;          // 累计角度（度）
    uint8_t Temperature;    // 温度
}
```

## 核心接口

```cpp
// 解析CAN反馈帧
static void RxHandler(uint32_t *Identifier, uint8_t Data[8]);

// 发送4电机电流指令（一帧同时控制4个电机）
static void Send_CurrentCommand(
    FDCAN_TxFrame_TypeDef *DJI_Motor,
    uint32_t STDID,     // 0x200（M3508/M2006）/ 0x1FF（升降2006）
    int16_t M1, M2, M3, M4
);
```

## CAN 帧地址规则

| 发送ID | 控制对象 |
|--------|---------|
| 0x200 | 底盘电机 ID 1~4（M3508） |
| 0x1FF | 升降2006 电机 |
| 0x2FF | 备用（GM6020等） |

反馈帧 ID：电机1=0x201，电机2=0x202，依此类推。

## 编码器解算

- 原始编码器值：0~8191（13位）
- 每圈 8192 个计数
- 跨零点时通过 `Last_Encoder` 检测方向，累计 `Angle`
- M3508 减速比 1:19，M2006 减速比 1:36

## 使用的 FDCAN 通道

- **底盘电机**：FDCAN3（`FDCAN3_TxFrame`）
- **升降2006**：FDCAN3（`FDCAN3_TxFrame`, 0x1FF）

## 相关文件

- `MDK-ARM/Device/Motor/dji_motor.h`
- `MDK-ARM/Device/Motor/dji_motor.cpp`

## 参见

- [[chassis_task - 底盘控制任务]] — 底盘M3508电机控制
- [[lift_task - 升降控制任务]] — 升降M2006电机控制
- [[FDCAN总线]] — CAN总线底层驱动
