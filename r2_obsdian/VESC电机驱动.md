# VESC电机驱动

[[00-项目总览]] | [[chassis_task - 底盘控制任务]] | [[FDCAN总线]]

---

## 模块概述

`VescMotor`（`VescMotor.h/.cpp`）是对 VESC 开源电调的 C++ 封装驱动，使用 FDCAN2 以 CAN 扩展帧（29位ID）通信。工程中配置 4 个实例，目前底盘任务使用 `VescMotors[0]`（节点ID=80）。

## VESC CAN 协议帧格式

```
扩展帧 29位 ID 布局：
    Bits[7:0]   = VESC节点ID（0~255）
    Bits[15:8]  = 命令类型（CanPacketID枚举）
    Bits[28:16] = 0（未使用）

发送ID = (node_id & 0xFF) | ((cmd) << 8)
```

## 支持的命令类型

| 命令 | 值 | 说明 |
|------|----|------|
| SET_DUTY | 0 | 设置占空比（-1.0~1.0） |
| SET_CURRENT | 1 | 设置目标电流（mA） |
| SET_RPM | 3 | 设置转速（eRPM） |
| SET_POS | 4 | 设置目标位置（度） |
| STATUS | 9 | 状态帧1：eRPM/电流/占空比 |
| STATUS_4 | 16 | 状态帧4：温度/PID位置 |

## eRPM 与机械转速换算

```
工程电机：24N28P → 极对数 = 28/2 = 14
eRPM = 机械RPM × 14
发送RPM指令时：内部自动 × 14 转为 eRPM
接收状态帧时：eRPM ÷ 14 = 机械RPM
```

## 使用方法

```cpp
// 初始化绑定外设和节点ID
VescMotors[0].init(&hfdcan2, 80);

// 控制接口
VescMotors[0].setRpm(300);         // 设置 300 RPM
VescMotors[0].setCurrent(5000);    // 设置 5000 mA
VescMotors[0].setPwm(0.5f);        // 设置 50% 占空比

// 读取反馈（在CAN中断回调中更新）
VescMotors[0].canRxHandler(&hfdcan2, rx_data);
float speed = VescMotors[0].getRxData().rpm;
float angle = VescMotors[0].getRxData().totalPosition;
```

## 接收数据结构

```cpp
VescRxData {
    float eRpm;             // 电气转速（eRPM）
    float rpm;              // 机械转速（RPM）
    float duty;             // 当前占空比（-1.0~+1.0）
    float totalCurrent;     // 总相电流（A）
    float pidPositionNow;   // 当前PID角度（度，0~360）
    float pidPositionLast;  // 上次PID角度
    int   turnCount;        // 累计圈数（正转+1，反转-1）
    float totalPosition;    // 累计角度（度）= turnCount×360 + pidPositionNow
}
```

## 在工程中的应用

```
chassis_task:
  VescMotors[0].setRpm(rpm)  — 控制辅助电机（底盘或其他机构）
  VescMotors[0].canRxHandler() — 在CAN中断中刷新
```

## 相关文件

- `MDK-ARM/Device/Motor/VescMotor.h`
- `MDK-ARM/Device/Motor/VescMotor.cpp`
- 参考文档：`说明文档/VescMotor 驱动使用文档.md`

## 参见

- [[chassis_task - 底盘控制任务]] — VESC控制应用
- [[FDCAN总线]] — FDCAN2通信层
