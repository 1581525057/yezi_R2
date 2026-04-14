# DM达妙电机

[[00-项目总览]] | [[FDCAN总线]]

---

## 模块概述

`DM_Motor_Class`（`dm_motor.h/.cpp`）封装了达妙（DM）系列电机的 FDCAN 通信驱动，支持 MIT 控制、位置-速度控制、纯速度控制三种模式。工程中预留了 `DM_8009_Motor[4]` 共四个实例（当前版本未在主任务中启用）。

## 控制模式

| 模式 | 枚举值 | 说明 |
|------|--------|------|
| MIT | 0 | 位置+速度+力矩混合控制（MIT模式） |
| POSITION_VELOCITY | 1 | 位置-速度控制 |
| VELOCITY | 2 | 纯速度控制 |

## 命令类型

| 命令 | 说明 |
|------|------|
| Motor_Enable | 使能电机 |
| Motor_Disable | 失能电机 |
| Motor_Save_Zero_Position | 存储当前位置为零点 |

## 核心接口

```cpp
// 发送使能/失能/存零点命令
static void Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                    DM_Motor_Info_Typedef *DM_Motor, uint8_t CMD);

// MIT模式控制帧
static void CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                          DM_Motor_Info_Typedef *DM_Motor,
                          float Postion, float Velocity,
                          float KP, float KD, float Torque);

// 解析反馈帧
static void RxHandler(uint32_t *Identifier, uint8_t Data[8]);
```

## 数据结构

```cpp
DM_Motor_Data_Typedef {
    bool    Initlized;
    uint8_t State;              // 错误状态
    float   Position;           // 当前位置（rad）
    float   Velocity;           // 当前速度（rad/s）
    float   Torque;             // 当前力矩（N·m）
    float   Temperature_MOS;    // MOS管温度
    float   Temperature_Rotor;  // 转子温度
    float   Angle;              // 累计角度（度）
}

DM_Motor_Param_Range_Typedef {
    float P_MAX;   // 位置范围（±rad）
    float V_MAX;   // 速度范围（rad/s）
    float T_MAX;   // 力矩范围（N·m）
}
```

## 相关文件

- `MDK-ARM/Device/Motor/dm_motor.h`
- `MDK-ARM/Device/Motor/dm_motor.cpp`

## 参见

- [[FDCAN总线]] — CAN通信底层
- [[00-项目总览]] — 工程整体结构
