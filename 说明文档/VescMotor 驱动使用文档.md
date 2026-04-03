# VescMotor 驱动使用文档

> **适用平台**：STM32H7xx（FDCAN 外设）  
> **驱动文件**：`VescMotor.hpp` / `VescMotor.cpp`  
> **依赖层**：`bsp_can.h` / `bsp_can.cpp`（板级 CAN 统一接口）  
> **VESC 协议版本**：兼容 VESC 6.x CAN 通信协议  

---

## 目录

1. [文件结构说明](#1-文件结构说明)
2. [STM32H7 FDCAN 外设基础](#2-stm32h7-fdcan-外设基础)
3. [STM32CubeMX 配置指南](#3-stm32cubemx-配置指南)
4. [VESC CAN 协议详解](#4-vesc-can-协议详解)
5. [驱动架构说明](#5-驱动架构说明)
6. [API 接口文档](#6-api-接口文档)
7. [完整使用示例](#7-完整使用示例)
8. [接收数据解析说明](#8-接收数据解析说明)
9. [常见问题排查](#9-常见问题排查)
10. [与原 C 代码的对比](#10-与原-c-代码的对比)

---

## 1. 文件结构说明

```
vesc (2)/
├── VescMotor.hpp       # C++ 类头文件（接口声明）
├── VescMotor.cpp       # C++ 类实现文件
├── bsp_can.h           # 板级 CAN BSP 头文件（FDCAN_TxFrame_TypeDef / FDCAN_RxFrame_TypeDef）
└── bsp_can.cpp         # 板级 CAN BSP 实现（统一初始化、发送、接收分发）
```

### 层次关系

```
VescMotor（应用层）
    │  发送：调用 BSP_CAN::AddMessageToTxFifoQ()
    │  接收：由 bsp_can 分发层调用 motor.canRxHandler(&FDCAN_RxFIFO0Frame)
    ▼
BSP_CAN（板级驱动层）
    │  封装 HAL_FDCAN_AddMessageToTxFifoQ
    │  统一管理三路 FDCAN 的初始化、发送帧对象、接收分发
    ▼
STM32 HAL / FDCAN 硬件
```

### VescMotor.hpp 结构

| 内容 | 说明 |
|------|------|
| `enum class CanPacketID` | VESC CAN 命令类型枚举，对应帧 ID 的 Bits[15:8] |
| `struct VescRxData` | 电机接收数据结构体，存储转速、电流、位置等 |
| `class VescMotor` | 主类，封装单个 VESC 电调的全部操作 |
| `VescMotor VescMotors[4]` | 全局电机数组，支持最多 4 个电调 |

### VescMotor.cpp 结构

| 函数 | 功能 |
|------|------|
| `VescMotor()` | 构造函数，初始化清零 |
| `init()` | 绑定 FDCAN 句柄和节点 ID |
| `packInt32BigEndian()` | 私有辅助：大端序打包 int32 |
| `sendFrame()` | 私有辅助：填充 BSP_CAN TxFrame 并调用 `BSP_CAN::AddMessageToTxFifoQ` |
| `setCurrent()` | 设置目标电流（力矩控制） |
| `setRpm()` | 设置目标转速（速度控制） |
| `setPwm()` | 设置占空比（开环电压控制） |
| `setPos()` | 设置目标位置（位置控制） |
| `setBrakeCurrent()` | 设置制动电流 |
| `setHandbrakeCurrent()` | 设置手刹电流（位置保持） |
| `setHandbrakeCurrentRel()` | 设置相对手刹电流（比例制动） |
| `canRxHandler()` | CAN 接收帧解析，由 bsp_can 分发层调用 |

### bsp_can.cpp 关键结构

| 成员 | 说明 |
|------|------|
| `BSP_CAN::FDCAN1_TxFrame` | FDCAN1 发送帧对象（含 `hcan=&hfdcan1`） |
| `BSP_CAN::FDCAN2_TxFrame` | FDCAN2 发送帧对象 |
| `BSP_CAN::FDCAN3_TxFrame` | FDCAN3 发送帧对象 |
| `BSP_CAN::FDCAN_RxFIFO0Frame` | FIFO0 接收帧缓存（由 HAL 中断填充） |
| `BSP_CAN::FDCAN_RxFIFO1Frame` | FIFO1 接收帧缓存 |
| `BSP_CAN::AddMessageToTxFifoQ()` | 统一发送接口，内部调用 `HAL_FDCAN_AddMessageToTxFifoQ` |
| `BSP_CAN::FDCAN1_RxFifo0RxHandler()` | FDCAN1 接收分发，内含 VescMotors 遍历 |

---

## 2. STM32H7 FDCAN 外设基础

### 2.1 bxCAN 与 FDCAN 的核心区别

STM32F1/F4 使用的是 **bxCAN**（Basic Extended CAN），STM32H7 使用的是 **FDCAN**（Flexible Data-Rate CAN）。虽然本驱动只使用经典 CAN（500kbps），但必须使用 H7 专用的 FDCAN HAL API。

| 对比项 | bxCAN（F1/F4） | FDCAN（H7） |
|--------|---------------|------------|
| 句柄类型 | `CAN_HandleTypeDef` | `FDCAN_HandleTypeDef` |
| 发送函数 | `HAL_CAN_AddTxMessage` | `HAL_FDCAN_AddMessageToTxFifoQ` |
| 接收回调 | `HAL_CAN_RxFifo0MsgPendingCallback` | `HAL_FDCAN_RxFifo0Callback` |
| 帧头类型 | `CAN_TxHeaderTypeDef` | `FDCAN_TxHeaderTypeDef` |
| 接收头类型 | `CAN_RxHeaderTypeDef` | `FDCAN_RxHeaderTypeDef` |
| 最大数据长度 | 8 字节 | 64 字节（FD模式）/ 8字节（经典模式） |

### 2.2 BSP_CAN 层的 TxFrame / RxFrame 结构

```c
// bsp_can.h 中定义：

// 发送帧结构体：包含 hcan 指针 + 帧头 + 数据
typedef struct {
    FDCAN_HandleTypeDef  *hcan;    // 指向 hfdcan1/hfdcan2/hfdcan3
    FDCAN_TxHeaderTypeDef Header;  // 帧头（ID、类型、长度等）
    uint8_t               Data[8]; // 8字节数据负载
} FDCAN_TxFrame_TypeDef;

// 接收帧结构体：包含 hcan 指针 + 帧头 + 数据
typedef struct {
    FDCAN_HandleTypeDef  *hcan;    // 触发中断的 FDCAN 句柄（可选，用于多总线区分）
    FDCAN_RxHeaderTypeDef Header;  // 帧头（含 Identifier 等）
    uint8_t               Data[8]; // 8字节数据负载
} FDCAN_RxFrame_TypeDef;
```

**VescMotor 的发送流程**：
```
sendFrame(cmd, data)
  → 按 hfdcan_ 选择 BSP_CAN::FDCANx_TxFrame
  → 填充 TxFrame.Header（ID = nodeId_ | cmd<<8, IdType=EXTENDED, DLC=8）
  → memcpy(TxFrame.Data, data, 8)
  → BSP_CAN::AddMessageToTxFifoQ(&TxFrame)
      → HAL_FDCAN_AddMessageToTxFifoQ(TxFrame.hcan, &TxFrame.Header, TxFrame.Data)
```

**VescMotor 的接收流程**：
```
HAL_FDCAN_RxFifo0Callback()
  → BSP_CAN::RxFifo0Callback()
      → HAL_FDCAN_GetRxMessage() → 填入 FDCAN_RxFIFO0Frame
      → 若 hfdcan == &hfdcan1：调用 FDCAN1_RxFifo0RxHandler()
          → Yun_J60_Class::RxHandler(...)
          → for (auto& motor : VescMotors) motor.canRxHandler(&FDCAN_RxFIFO0Frame)
              → 按 nodeId_ 过滤 → 解析 rxData_
```

### 2.3 FDCAN_TxHeaderTypeDef 字段详解

```c
FDCAN_TxHeaderTypeDef header;

header.IdType      = FDCAN_EXTENDED_ID;   // 29位扩展ID（VESC协议要求）
                   // FDCAN_STANDARD_ID = 11位标准ID（VESC不用）

header.TxFrameType = FDCAN_DATA_FRAME;    // 数据帧
                   // FDCAN_REMOTE_FRAME = 远程帧（VESC不用）

header.DataLength  = FDCAN_DLC_BYTES_8;   // 8字节（经典CAN最大值）

header.Identifier  = nodeId | (cmd << 8); // VESC 协议 29位 ID 格式

// 以下字段仅在 FDCAN FD 模式下有意义，经典CAN填默认值（HAL 要求必须填写）：
header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 主动错误状态
header.BitRateSwitch       = FDCAN_BRS_OFF;       // 不切换比特率
header.FDFormat            = FDCAN_CLASSIC_CAN;   // 经典CAN格式
header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;  // 不记录TX事件
header.MessageMarker       = 0;                   // 用户标签，不用填0
```

---

## 3. STM32CubeMX 配置指南

### 3.1 FDCAN 基础配置步骤

#### Step 1：启用 FDCAN 外设

在 **Pinout & Configuration → Connectivity** 中找到 `FDCAN1`（或 FDCAN2/FDCAN3），点击启用。

选择对应的引脚，例如：
- FDCAN1：PD0（RX），PD1（TX）
- FDCAN2：PB5（RX），PB6（TX）

#### Step 2：配置波特率（500 kbps）

进入 **FDCAN1 → Parameter Settings**，配置以下参数：

> VESC 默认 CAN 波特率为 **500 kbps**，必须与之匹配。

以 **STM32H7 内核时钟 80 MHz** 为例：

```
Frame Format        : Classic CAN   （不使用 FD 模式）
Mode                : Normal        （正常工作模式）
Auto Retransmission : ENABLE        （自动重传，建议开启）
Transmit Pause      : DISABLE
Protocol Exception  : DISABLE

--- Nominal Bit Timing ---
Prescaler (Pre-scaler of fdcan_ker_ck)  : 8
Nominal Sync Jump Width                  : 1
Nominal Time Seg1 (Prop_Seg + Phase_Seg1): 12
Nominal Time Seg2 (Phase_Seg2)           : 3

实际波特率计算：
  波特率 = fdcan_ker_ck / Prescaler / (1 + Seg1 + Seg2)
         = 80MHz / 8 / (1 + 12 + 3)
         = 80MHz / 8 / 16
         = 625 kHz  ← 需根据实际时钟重新调整！
```

> **重要**：STM32H7 的 FDCAN 时钟源可能不是 80MHz，请在 **Clock Configuration** 页确认 `fdcan_ker_ck` 的实际频率后再计算。常见配置如下：

| fdcan_ker_ck | Prescaler | Seg1 | Seg2 | 实际波特率 |
|-------------|-----------|------|------|-----------|
| 40 MHz | 4 | 14 | 5 | 500 kbps |
| 80 MHz | 8 | 14 | 5 | 500 kbps |
| 120 MHz | 12 | 14 | 5 | 500 kbps |

**验证方式**：使用 **VESC Tool** 连接电调，若通信正常则波特率配置正确。

#### Step 3：启用接收中断

在 **FDCAN1 → NVIC Settings** 中，勾选：
- `FDCAN1 IT0 interrupt` ✅

在 **NVIC** 中确认中断优先级（建议设为 5 或更低，高于任务优先级）。

#### Step 4：生成代码

点击 **Generate Code**，CubeMX 会在 `fdcan.c` 中生成 `MX_FDCAN1_Init()` 函数。

---

### 3.2 滤波器配置

本工程的滤波器配置**已由 `BSP_CAN::Init()` 统一管理**，无需手动添加到 `fdcan.c`。

`BSP_CAN::Init()` 中对每路 FDCAN 的配置如下：

```c
// FDCAN1（FIFO0，用于 Yun_J60 + VESC）
FilterConfig.IdType       = FDCAN_STANDARD_ID;
FilterConfig.FilterIndex  = 0;
FilterConfig.FilterType   = FDCAN_FILTER_MASK;
FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
FilterConfig.FilterID1    = 0x00000000;  // ID基准（全0）
FilterConfig.FilterID2    = 0x00000000;  // 掩码（全0=全通，接受所有帧）
HAL_FDCAN_ConfigFilter(&hfdcan1, &FilterConfig);
HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                              FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
HAL_FDCAN_Start(&hfdcan1);
```

> **注意**：VESC 使用 29 位**扩展帧**，而上述滤波器配置的 `IdType = FDCAN_STANDARD_ID`。
> 当 `FilterID2（掩码）= 0` 时，所有帧（包括扩展帧）均通过，因此不影响 VESC 帧的接收。
> 若需要精确过滤，可将 `IdType` 改为 `FDCAN_EXTENDED_ID` 并配合相应 ID 掩码。

### 3.3 在 main.c 中调用 BSP_CAN::Init()

只需在 main.c 的外设初始化阶段调用一次：

```c
// main.c
#include "bsp_can.h"
#include "VescMotor.hpp"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_FDCAN1_Init();   // CubeMX 生成的基础初始化（不含滤波器）
    MX_FDCAN2_Init();
    MX_FDCAN3_Init();

    // BSP_CAN::Init() 完成：滤波器配置 + HAL_FDCAN_Start + 中断激活
    BSP_CAN::Init();

    // 绑定 VESC 电机对象
    VescMotors[0].init(&hfdcan1, 10);   // FDCAN1，节点ID=10
    VescMotors[1].init(&hfdcan1, 11);   // FDCAN1，节点ID=11

    while (1) { /* ... */ }
}
```

---

## 4. VESC CAN 协议详解

### 4.1 扩展帧 ID 格式（29位）

VESC 使用 CAN **扩展帧**（29位 ID），ID 字段布局如下：

```
Bits[28:16] = 0x0000  （高13位，VESC 未使用，固定为0）
Bits[15: 8] = 命令类型（CanPacketID 枚举值，0~27）
Bits[ 7: 0] = 节点 ID  （每个 VESC 电调的唯一地址，0~255）
```

**发送示例**（向节点ID=10的电调发送转速命令）：

```c
// 命令类型 SET_RPM = 3，节点ID = 10
uint32_t identifier = 10 | ((uint32_t)3 << 8);
// identifier = 0x00000310
```

**接收解析**：

```c
uint32_t rawId  = rxFrame->Header.Identifier;
uint8_t nodeId  = rawId & 0xFF;          // 低8位 = 发送方节点ID
uint8_t cmd     = (rawId >> 8) & 0xFF;   // 第9~16位 = 命令/状态类型
```

### 4.2 命令类型枚举（CanPacketID）

| 值 | 枚举名 | 方向 | 说明 |
|----|--------|------|------|
| 0 | SET_DUTY | TX→VESC | 设置占空比（-1.0~+1.0，×100000存储） |
| 1 | SET_CURRENT | TX→VESC | 设置目标电流（mA，×1000存储为μA） |
| 2 | SET_CURRENT_BRAKE | TX→VESC | 设置制动电流（mA，×1000） |
| 3 | SET_RPM | TX→VESC | 设置目标转速（eRPM） |
| 4 | SET_POS | TX→VESC | 设置目标位置 |
| 9 | STATUS | VESC→RX | 状态帧1：eRPM、电流、占空比 |
| 14 | STATUS_2 | VESC→RX | 状态帧2：安时消耗/回充 |
| 15 | STATUS_3 | VESC→RX | 状态帧3：瓦时消耗/回充 |
| 16 | STATUS_4 | VESC→RX | 状态帧4：温度、PID位置 |
| 27 | STATUS_5 | VESC→RX | 状态帧5：输入电压、里程计 |
| 12 | SET_CURRENT_HANDBRAKE | TX→VESC | 手刹电流（位置保持） |
| 13 | SET_CURRENT_HANDBRAKE_REL | TX→VESC | 相对手刹（0~1.0） |

### 4.3 数据大端序规则

VESC 协议所有多字节字段均采用**大端序（Big-Endian，MSB First）**：

```
int32_t val = 0x12345678

data[0] = 0x12  ← 最高字节（bits 31..24）
data[1] = 0x34
data[2] = 0x56
data[3] = 0x78  ← 最低字节（bits  7.. 0）
```

ARM Cortex-M 是小端序，因此必须手动拆字节（本驱动的 `packInt32BigEndian()` 完成此工作）。

### 4.4 极对数与 eRPM 换算

VESC 内部始终使用**电气转速（eRPM）**：

```
eRPM = 机械转速(RPM) × 极对数(pole pairs)
```

本项目电机参数：**24N28P**（定子槽数24，转子磁极数28）

```
极对数 = 磁极数 / 2 = 28 / 2 = 14
```

| 机械转速 | 发送给VESC的eRPM |
|---------|----------------|
| 100 RPM | 1400 eRPM |
| 300 RPM | 4200 eRPM |
| -500 RPM | -7000 eRPM |

> 代码中 `POLE_PAIRS = 14`，如更换电机需修改此常量。

---

## 5. 驱动架构说明

### 5.1 完整数据流

```
【发送方向：MCU → VESC】

主循环/任务
    │
    ├── VescMotors[0].setCurrent(5000)
    │       └── sendFrame(SET_CURRENT, data)
    │               │
    │               ├── 按 hfdcan_ 选择 BSP_CAN::FDCANx_TxFrame
    │               ├── 填充 TxFrame.Header.Identifier = nodeId_ | (cmd<<8)
    │               ├── memcpy(TxFrame.Data, data, 8)
    │               └── BSP_CAN::AddMessageToTxFifoQ(&TxFrame)
    │                       └── HAL_FDCAN_AddMessageToTxFifoQ(hcan, Header, Data)
    │                                       │
    │                                  FDCAN TX FIFO 硬件缓冲
    │                                       │
    │                                  CAN 总线 → VESC 电调
    │
    └── VescMotors[0].getRxData().rpm  ← 从 rxData_ 读取（只读）


【接收方向：VESC → MCU】

VESC 电调广播 STATUS / STATUS_4 帧
    │
    CAN 总线
    │
    FDCAN1 RX FIFO0 中断
    │
    HAL_FDCAN_RxFifo0Callback(hfdcan, ITs)  [HAL 自动调用]
    │
    BSP_CAN::RxFifo0Callback(hfdcan, ITs)
    │   ├── HAL_FDCAN_GetRxMessage() → 填入 BSP_CAN::FDCAN_RxFIFO0Frame
    │   └── if (hfdcan == &hfdcan1) FDCAN1_RxFifo0RxHandler(...)
    │           ├── Yun_J60_Class::RxHandler(...)     ← 其他设备
    │           └── for (auto& motor : VescMotors)
    │                   motor.canRxHandler(&FDCAN_RxFIFO0Frame)
    │                       ├── 读 FDCAN_RxFIFO0Frame.Header.Identifier
    │                       ├── 解析 nodeId（低8位）、cmd（高8位）
    │                       ├── if nodeId != nodeId_ → return（过滤非本机帧）
    │                       └── switch(cmd) → 解析 Data[8] → 更新 rxData_
    │
    主循环读取 VescMotors[0].getRxData().rpm / .totalPosition / ...
```

### 5.2 VescRxData 数据结构

```cpp
struct VescRxData {
    // 来自 STATUS（状态帧1，每帧都有）
    float eRpm;           // 电气转速 (eRPM)
    float rpm;            // 机械转速 (RPM) = eRpm / POLE_PAIRS
    float duty;           // 当前占空比 (-1.0 ~ +1.0)
    float totalCurrent;   // 相电流 (A)
    float dutyCycle;      // duty 的别名（兼容性保留）

    // 来自 STATUS_4（状态帧4）
    float pidPositionNow;   // 当前 PID 角度 (度, 0~360)
    float pidPositionLast;  // 上次 PID 角度（用于跨零检测）
    int   turnCount;        // 累计圈数（正转+1，反转-1）
    float totalPosition;    // 累计总角度 = turnCount×360 + pidPositionNow
};
```

### 5.3 全局电机数组与路由

```cpp
// VescMotor.cpp 中定义：
VescMotor VescMotors[4];   // 4个电机实例，下标0~3

// 使用前各自初始化（绑定哪路FDCAN取决于电机接在哪条总线上）：
VescMotors[0].init(&hfdcan1, 10);   // FDCAN1，节点ID=10
VescMotors[1].init(&hfdcan1, 11);   // FDCAN1，节点ID=11（同一总线）
VescMotors[2].init(&hfdcan2, 12);   // FDCAN2，节点ID=12
VescMotors[3].init(&hfdcan3, 13);   // FDCAN3，节点ID=13
```

**接收路由对应关系**：

| FDCAN总线 | 接收 FIFO | bsp_can 分发函数 | 已接入设备 |
|-----------|----------|-----------------|-----------|
| FDCAN1 | FIFO0 | `FDCAN1_RxFifo0RxHandler` | Yun_J60 + **VescMotors** |
| FDCAN2 | FIFO1 | `FDCAN2_RxFifo1RxHandler` | 空（可扩展） |
| FDCAN3 | FIFO0 | `FDCAN3_RxFifo0RxHandler` | DJI_Motor |

> 若 VESC 接在 FDCAN2 或 FDCAN3，需在对应的 `FDCAN2_RxFifo1RxHandler` 或 `FDCAN3_RxFifo0RxHandler` 中同样添加 VescMotors 遍历。

---

## 6. API 接口文档

### 6.1 初始化

```cpp
void VescMotor::init(FDCAN_HandleTypeDef* hfdcan, uint16_t nodeId);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `hfdcan` | `FDCAN_HandleTypeDef*` | HAL FDCAN 句柄（`&hfdcan1` / `&hfdcan2` / `&hfdcan3`） |
| `nodeId` | `uint16_t` | VESC 节点 ID（0~255），需与 VESC Tool 中配置一致 |

**前提条件**：调用 `init()` 前，`BSP_CAN::Init()` 必须已经调用完毕。

---

### 6.2 控制接口（TX 方向）

#### setCurrent — 电流控制（力矩控制）

```cpp
void VescMotor::setCurrent(int32_t current_mA);
```

| 参数 | 范围 | 说明 |
|------|------|------|
| `current_mA` | ±受VESC配置限制 | 目标电流，单位 **毫安（mA）**，正=正转，负=反转 |

协议编码：`int32 = current_mA × 1000`（发送μA精度值）

```cpp
motor.setCurrent(5000);    // 5A 驱动
motor.setCurrent(-3000);   // 3A 反转驱动
motor.setCurrent(0);       // 释放（自由转动，非刹车）
```

---

#### setRpm — 转速控制（速度闭环）

```cpp
void VescMotor::setRpm(int32_t rpm);
```

| 参数 | 范围 | 说明 |
|------|------|------|
| `rpm` | ±受VESC配置限制 | 目标机械转速，单位 **RPM**，内部自动×14转为eRPM |

```cpp
motor.setRpm(300);    // 正转 300 RPM
motor.setRpm(-200);   // 反转 200 RPM
motor.setRpm(0);      // 停止（速度闭环维持0转）
```

---

#### setPwm — 占空比控制（开环电压）

```cpp
void VescMotor::setPwm(double pwm);
```

| 参数 | 范围 | 说明 |
|------|------|------|
| `pwm` | -1.0 ~ +1.0 | 占空比，+1.0=满正转，-1.0=满反转，0.0=停止 |

协议编码：`int32 = pwm × 100000`

```cpp
motor.setPwm(0.5);    // 50% 占空比正转
motor.setPwm(-0.3);   // 30% 占空比反转
motor.setPwm(0.0);    // 停止（无制动）
```

---

#### setPos — 位置控制

```cpp
void VescMotor::setPos(int32_t pos);
```

直接发送，不做缩放。具体单位请参考 VESC Tool 的 Encoder 配置。

---

#### setBrakeCurrent — 制动电流（减速制动）

```cpp
void VescMotor::setBrakeCurrent(int32_t current_mA);
```

与 `setCurrent(负值)` 的区别：
- `setCurrent(负值)`：四象限控制，可反转驱动
- `setBrakeCurrent(正值)`：专用制动指令，只减速，不会反转

```cpp
motor.setBrakeCurrent(3000);   // 3A 制动电流
```

---

#### setHandbrakeCurrent — 手刹（位置保持）

```cpp
void VescMotor::setHandbrakeCurrent(int32_t current_mA);
```

锁定电机在当前位置，适合停车保持。

```cpp
motor.setHandbrakeCurrent(5000);   // 5A 手刹锁定
```

---

#### setHandbrakeCurrentRel — 相对手刹

```cpp
void VescMotor::setHandbrakeCurrentRel(float relative);
```

| 参数 | 范围 | 说明 |
|------|------|------|
| `relative` | 0.0 ~ 1.0 | 0=无制动，1.0=最大制动电流 |

```cpp
motor.setHandbrakeCurrentRel(0.8f);   // 80% 最大手刹电流
```

---

### 6.3 接收接口（RX 方向）

#### canRxHandler — CAN 接收帧处理

```cpp
void VescMotor::canRxHandler(const FDCAN_RxFrame_TypeDef* rxFrame);
```

**由 `bsp_can.cpp` 的分发函数自动调用，用户代码无需手动调用。**

| 参数 | 说明 |
|------|------|
| `rxFrame` | 指向 `BSP_CAN::FDCAN_RxFIFO0Frame`，由 HAL 中断填充完毕后传入 |

内部过滤逻辑：只处理 `Identifier[7:0] == nodeId_` 的帧，其余直接返回。

---

#### getRxData — 读取接收数据

```cpp
const VescRxData& VescMotor::getRxData() const;
```

返回最新解析的状态数据。

```cpp
float speed   = motor.getRxData().rpm;
float angle   = motor.getRxData().totalPosition;
float current = motor.getRxData().totalCurrent;
```

---

## 7. 完整使用示例

### 7.1 工程文件接入

在 CubeMX 生成的工程中，将以下文件加入编译：
- `VescMotor.hpp`
- `VescMotor.cpp`
- `bsp_can.h`
- `bsp_can.cpp`（通常工程中已有）

### 7.2 main.c 初始化代码

```c
// main.c
#include "bsp_can.h"
#include "VescMotor.hpp"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // CubeMX 生成的外设基础初始化（不含滤波器，由 BSP_CAN::Init 统一配置）
    MX_FDCAN1_Init();
    MX_FDCAN2_Init();
    MX_FDCAN3_Init();

    // BSP_CAN 统一初始化：滤波器 + Start + 中断激活（三路 FDCAN 一次完成）
    BSP_CAN::Init();

    // 初始化 VESC 电机对象，绑定 FDCAN 总线和节点 ID
    // 节点 ID 需与 VESC Tool → App Configuration → CAN → Controller ID 一致
    VescMotors[0].init(&hfdcan1, 10);   // FDCAN1 总线，VESC 节点ID=10
    VescMotors[1].init(&hfdcan1, 11);   // FDCAN1 总线，VESC 节点ID=11

    while (1)
    {
        // 电流控制（力矩模式）
        VescMotors[0].setCurrent(5000);   // 5A

        // 速度控制
        VescMotors[1].setRpm(300);        // 300 RPM

        // 读取反馈（数据由中断自动更新，主循环直接读）
        float speed = VescMotors[0].getRxData().rpm;
        float angle = VescMotors[0].getRxData().totalPosition;
        float curr  = VescMotors[0].getRxData().totalCurrent;

        HAL_Delay(10);  // 控制周期 10ms
    }
}
```

### 7.3 bsp_can.cpp 中的接收分发（已配置好，此处仅说明）

```cpp
// bsp_can.cpp 中 FDCAN1 的接收分发（已修改，含 VESC 分发）：

void BSP_CAN::FDCAN1_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
    // Yun_J60 设备接收处理（原有逻辑）
    Yun_J60_Class::RxHandler(&FDCAN_RxFIFO0Frame, Data);

    // VESC 电机接收分发：
    // canRxHandler 内部按 nodeId_ 过滤，不匹配的帧自动忽略（不影响性能）
    for (auto& motor : VescMotors)
    {
        motor.canRxHandler(&FDCAN_RxFIFO0Frame);
    }
}
```

> **无需手动实现 `HAL_FDCAN_RxFifo0Callback`**，bsp_can.cpp 已实现：
> ```cpp
> extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t ITs)
> {
>     BSP_CAN::RxFifo0Callback(hfdcan, ITs);  // HAL 自动调用，bsp_can 内部处理
> }
> ```

### 7.4 RTOS（FreeRTOS）环境下的使用

```c
// 控制任务
void MotorControlTask(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        VescMotors[0].setRpm(target_rpm);
        vTaskDelayUntil(&lastWake, period);
    }
}

// 数据读取任务
void DataReadTask(void *pvParameters)
{
    for (;;)
    {
        float rpm   = VescMotors[0].getRxData().rpm;
        float angle = VescMotors[0].getRxData().totalPosition;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

> **注意**：`canRxHandler` 在中断上下文中调用，`getRxData()` 在任务上下文中调用，对 `rxData_` 的读写存在竞态风险。如需严格线程安全，读取数据时可临时关闭中断或使用 FreeRTOS 互斥量。

---

## 8. 接收数据解析说明

### 8.1 STATUS（状态帧1）数据格式

CAN ID 高8位 = `9`（CanPacketID::STATUS）

```
Byte 偏移    内容                   解析方式
─────────────────────────────────────────────────────
Byte 0-1     符号/高位标志          特殊：若 Byte0==0xFF && Byte1==0xFF 则为负转速
Byte 2-3     eRPM 低16位（绝对值）  uint16，负值时按位取反后取负
Byte 4-5     相电流×10 (A)          int16 big-endian，÷10.0 → 实际电流(A)
Byte 6-7     占空比×1000            int16 big-endian，÷1000.0 → -1.0~+1.0
```

**eRPM 解析的特殊性**（沿用原始 C 代码逻辑）：

```cpp
if (data[0] == 0xFF && data[1] == 0xFF)
{
    // 负转速：对 Byte2/Byte3 按位取反，得绝对值后取负
    uint16_t abs_val = ((uint8_t)(~data[2]) << 8) | (uint8_t)(~data[3]);
    rxData_.eRpm = -(float)abs_val;
}
else
{
    // 正转速：直接拼接
    rxData_.eRpm = (float)((data[2] << 8) | data[3]);
}
```

> 此解析方式只使用了帧的低16位 eRPM，最大表示 65535 eRPM ≈ 4681 RPM（14极对数）。若需更高转速，需修改解析逻辑为标准 int32 big-endian 解析。

**int16 有符号解析（电流、占空比）的正确写法**：

```cpp
// 正确：先拼接为 uint16_t，再 cast 为 int16_t，再转 float
rxData_.totalCurrent = static_cast<float>(
    static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) |
                          static_cast<uint16_t>(data[5]))
) / 10.0f;

// 错误写法（会导致符号扩展时机不对，负值解析错误）：
// rxData_.totalCurrent = ((int16_t)(data[4]) << 8) | (int16_t)(data[5]);
```

### 8.2 STATUS_4（状态帧4）数据格式

CAN ID 高8位 = `16`（CanPacketID::STATUS_4）

```
Byte 偏移    内容                   解析方式
─────────────────────────────────────────────────────
Byte 0-1     MOSFET温度×10(℃)      int16 big-endian，÷10.0（当前未解析）
Byte 2-3     电机温度×10(℃)        int16 big-endian，÷10.0（当前未解析）
Byte 4-5     输入电流×10(A)         int16 big-endian，÷10.0（当前未解析）
Byte 6-7     PID角度×50(°)          int16 big-endian，÷50.0 → 0~360°
```

### 8.3 累计角度跟踪算法

VESC 发送的 PID 角度范围为 **0°~360°**，跨零时发生跳变。驱动内实现了简单的圈数跟踪：

```
delta = pidPositionNow - pidPositionLast

若 delta < -180°  → 正转经过 360°→0° 边界，turnCount++
若 delta > +180°  → 反转经过 0°→360° 边界，turnCount--
否则              → 正常变化，turnCount 不变

totalPosition = turnCount × 360 + pidPositionNow
```

**示例**：

| 时刻 | pidPositionNow | pidPositionLast | delta | turnCount | totalPosition |
|------|---------------|----------------|-------|-----------|---------------|
| T0 | 350° | 0° | - | 0 | 350° |
| T1 | 5° | 350° | -345° < -180 | 1 | 365° |
| T2 | 60° | 5° | 55° | 1 | 420° |
| T3 | 355° | 60° | 295° > 180 | 0 | 355° |

---

## 9. 常见问题排查

### Q1：发送命令后电机没有响应

**排查步骤**：
1. 确认 `BSP_CAN::Init()` 在 `VescMotors[i].init()` 之前调用
2. 用示波器或 CAN 分析仪确认 CAN 总线上有波形
3. 确认节点 ID 与 VESC Tool 中 `App Configuration → CAN → Controller ID` 一致
4. 确认波特率为 500 kbps（用 VESC Tool 的 `Realtime Data` 页验证连接）
5. 检查 CAN 终端电阻（总线两端各一个 120Ω）

### Q2：canRxHandler 从未被调用（rxData_ 始终为初始值）

**排查步骤**：
1. 确认 `bsp_can.cpp` 中 `FDCAN1_RxFifo0RxHandler` 已加入 VescMotors 遍历
2. 确认 VESC 接的是 FDCAN1（若接 FDCAN2/FDCAN3，需在对应分发函数里加）
3. 确认 VESC Tool 中已启用 STATUS 帧广播：`App Configuration → CAN → Send Status Messages`
4. 在 `canRxHandler` 开头加断点，确认是否进入

### Q3：接收到的转速始终为 0 或乱跳

**排查步骤**：
1. 检查 `rxNodeId` 比较逻辑：旧版本使用 `int8_t` 转换，节点ID>127时会符号溢出；新版本已改为 `uint8_t`，节点ID 0~255 均可正确匹配
2. 确认 `FDCAN_RxFIFO0Frame.Header.Identifier` 字段名与 `bsp_can.h` 中 `FDCAN_RxFrame_TypeDef` 定义一致

### Q4：`BSP_CAN::FDCAN1_TxFrame` 被多个设备（Yun_J60 + VESC）共用，会冲突吗？

BSP_CAN 的 TxFrame 静态对象在发送时**先填充再调用 AddMessageToTxFifoQ**，只要不在中断和主循环中同时调用不同设备的发送函数，就不会冲突。如果使用 RTOS，建议在调用 `sendFrame` 前后加互斥锁，或为 VESC 单独维护一个 TxFrame 对象。

### Q5：累计角度（totalPosition）在快速换向时跳变

**原因**：STATUS_4 帧的发送频率不够高，导致相邻两帧角度差超过 180° 被误判为跨零。

**解决**：在 VESC Tool 中提高 `Status Message 4` 的发送频率，或降低电机换向速度。

### Q6：`HAL_FDCAN_AddMessageToTxFifoQ` 返回 HAL_ERROR

**原因**：TX FIFO 队列已满（队列深度通常为 3），发送速率过高。

**解决**：降低发送频率，或在连续发送时加入短暂延时（`HAL_Delay(1)`）。

---

## 10. 与原 C 代码的对比

本驱动由 `vesc_can.h` / `vesc_can.c` 重构而来，并对接了本工程的 BSP_CAN 层：

| 对比项 | 原 C 代码 | C++ 重构版（对接 BSP_CAN） |
|--------|-----------|-----------------------------|
| 接口风格 | 函数指针模拟方法 | 真正的类成员函数 |
| 发送接口 | 直接调 `HAL_FDCAN_AddMessageToTxFifoQ` | 通过 `BSP_CAN::AddMessageToTxFifoQ` 统一发送 |
| 接收全局变量 | `extern FDCAN_RxFrame_TypeDef FIFO0_FDCAN_RxDATA`（不存在）| 直接传入 `&BSP_CAN::FDCAN_RxFIFO0Frame` 指针 |
| 接收分发 | 外部手动遍历 VescMotors | `bsp_can.cpp::FDCAN1_RxFifo0RxHandler` 内自动遍历 |
| nodeId 比较 | `int8_t` 强制转换（节点ID>127时符号溢出）| `uint8_t`（节点ID 0~255 全范围正确） |
| int16 有符号解析 | `(int16_t)(data[4]) << 8 | (int16_t)(data[5])` | 先拼 uint16_t 再 cast int16_t（符号扩展正确）|
| 全局变量 | tx_header 全局 | tx_header 通过 BSP_CAN TxFrame 管理 |
| 数据封装 | 独立结构体通过指针传递 | `VescRxData rxData_`（类私有成员）|
| 多电机支持 | `Vesc_Motor_U8[4]` 全局数组 | `VescMotors[4]` 全局数组（同样设计）|

---

*文档更新日期：2026-03-31 | 驱动版本：VescMotor C++ v1.1（BSP_CAN 接口版）*
