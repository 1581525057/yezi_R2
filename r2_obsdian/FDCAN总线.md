# FDCAN总线

[[00-项目总览]] | [[DJI电机驱动]] | [[云深J60电机]] | [[VESC电机驱动]] | [[DM达妙电机]]

---

## 模块概述

STM32H723 集成三路 FDCAN 外设（FDCAN1/2/3），本工程使用经典 CAN 模式（非FD模式），通过 HAL 驱动层封装为统一的 BSP_CAN 接口。

## 三路 FDCAN 分配

| 总线 | 挂载设备 | 说明 |
|------|---------|------|
| FDCAN1 | — | 预留 |
| FDCAN2 | VESC电机（节点ID=80） | 扩展帧（29位ID） |
| FDCAN3 | DJI M3508底盘电机 + M2006升降电机 + J60 | 标准帧（11位ID） |

## HAL 发送流程

```cpp
// 发送标准帧（以 DJI 电机为例）
FDCAN_TxHeaderTypeDef tx_header;
tx_header.IdType      = FDCAN_STANDARD_ID;
tx_header.TxFrameType = FDCAN_DATA_FRAME;
tx_header.Identifier  = 0x200;          // 底盘电机ID
tx_header.DataLength  = FDCAN_DLC_BYTES_8;

HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_header, data);
```

## HAL 接收流程（中断回调）

```cpp
// 在 HAL_FDCAN_RxFifo0Callback 中：
FDCAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8];
HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);

// 根据 rx_header.Identifier 分发给对应电机驱动的 RxHandler
```

## 过滤器配置

```cpp
// 全通（接受所有 ID）
filter.FilterID1 = 0x00000000;
filter.FilterID2 = 0x00000000;  // 掩码全0=全通
filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);

// 非匹配帧同样进 FIFO0
HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,
    FDCAN_ACCEPT_IN_RX_FIFO0,
    FDCAN_ACCEPT_IN_RX_FIFO0,
    FDCAN_FILTER_REMOTE,
    FDCAN_FILTER_REMOTE);
```

## BSP_CAN 层自定义类型

```cpp
// 发送帧缓冲（工程自定义）
typedef struct FDCAN_TxFrame_TypeDef {
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];
} FDCAN_TxFrame_TypeDef;

// 接收帧缓冲（工程自定义）
typedef struct FDCAN_RxFrame_TypeDef {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
} FDCAN_RxFrame_TypeDef;
```

## FDCAN 与 bxCAN 对比（H7 vs F4/F1）

| 项目 | bxCAN (F4/F1) | FDCAN (H7) |
|------|--------------|------------|
| 句柄 | CAN_HandleTypeDef | FDCAN_HandleTypeDef |
| 发送 | HAL_CAN_AddTxMessage | HAL_FDCAN_AddMessageToTxFifoQ |
| 接收 | HAL_CAN_GetRxMessage | HAL_FDCAN_GetRxMessage |
| 最大速率 | 1Mbps | 5Mbps（经典） |

## 相关文件

- `R2_Lift_auto/Core/Src/fdcan.c`
- `R2_Lift_auto/Core/Inc/fdcan.h`
- `MDK-ARM/Device/BSP/bsp_can.h/.cpp`

## 参见

- [[DJI电机驱动]] — FDCAN3，0x200/0x1FF
- [[云深J60电机]] — FDCAN3，扩展帧
- [[VESC电机驱动]] — FDCAN2，29位扩展帧
- [[DM达妙电机]] — FDCAN，MIT模式
