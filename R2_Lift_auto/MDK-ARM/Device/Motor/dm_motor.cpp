 /* USER CODE BEGIN Header */
// 负责：
// 1、定义 DM 电机对象
// 2、发送 DM 电机命令（使能 / 失能 / 存零点）
// 3、发送 MIT / 位置速度 / 速度 控制帧
// 4、收到 CAN 数据后，解析 DM 电机反馈
/* USER CODE END Header */

#include "dm_motor.h"
#include "bsp_can.h"



/**
 * @brief DM 8009 电机对象数组（共 4 个）
 *        每个对象记录：控制模式、CAN 收发 ID、参数范围上限、运行数据
 *        MIT 模式：TxID = 0x01~0x04，RxID = 0x11~0x14
 */
DM_Motor_Info_Typedef DM_Motor_Class::DM_8009_Motor[4] =
    {
        [0] = {
            .Control_Mode = MIT,
            .FDCANFrame   = {.TxIdentifier = 0x01, .RxIdentifier = 0x11},
            .Param_Range  = {.P_MAX = 3.141593f, .V_MAX = 45.0f, .T_MAX = 54.0f},
            .Data         = {0},
        },
        [1] = {
            .Control_Mode = MIT,
            .FDCANFrame   = {.TxIdentifier = 0x02, .RxIdentifier = 0x12},
            .Param_Range  = {.P_MAX = 3.141593f,   .V_MAX = 45.0f, .T_MAX = 54.0f},
            .Data         = {0},
        },
        [2] = {
            .Control_Mode = MIT,
            .FDCANFrame   = {.TxIdentifier = 0x03, .RxIdentifier = 0x13},
            .Param_Range  = {.P_MAX = 3.141593f, .V_MAX = 45.0f, .T_MAX = 54.0f},
            .Data         = {0},
        },
        [3] = {
            .Control_Mode = MIT,
            .FDCANFrame   = {.TxIdentifier = 0x04, .RxIdentifier = 0x14},
            .Param_Range  = {.P_MAX = 3.141593f, .V_MAX = 45.0f, .T_MAX = 54.0f},
            .Data         = {0},
        },
};

/**
 * @brief DM 电机控制目标数组
 */
DM_Motor_Contorl_Info_Typedef DM_Motor_Class::DM_Motor_Contorl_Info[4] = {0};

uint32_t id_dm = 0;

/**
 * @brief  解析单个 DM 电机的 CAN 反馈帧
 * @param  Identifier : 接收到的 CAN ID
 * @param  Rx_Buf     : 接收数据缓冲区（8 字节）
 * @param  DM_Motor   : 目标电机对象指针
 * @retval None
 *
 * DM 电机反馈帧格式（8 字节）：
 *   Byte[0]      高 4 位 = 电机状态
 *   Byte[1~2]    位置原始值（16 位）
 *   Byte[3~4]    速度原始值（12 位，跨字节）
 *   Byte[4~5]    力矩原始值（12 位，跨字节）
 *   Byte[6]      MOS 温度（°C，整数）
 *   Byte[7]      转子温度（°C，整数）
 */
void DM_Motor_Class::Info_Update(uint32_t *Identifier,
                                 uint8_t *Rx_Buf,
                                 DM_Motor_Info_Typedef *DM_Motor)
{
    if (*Identifier != DM_Motor->FDCANFrame.RxIdentifier)
        return;

    // 电机状态位，位于 Byte[0] 高 4 位
    DM_Motor->Data.State = Rx_Buf[0] >> 4;

    // 位置原始值：16 位，Byte[1] 为高字节，Byte[2] 为低字节
    DM_Motor->Data.P_int = ((uint16_t)(Rx_Buf[1]) << 8) | ((uint16_t)(Rx_Buf[2]));

    // 速度原始值：12 位，Byte[3] 为高 8 位，Byte[4] 高 4 位为低 4 位
    DM_Motor->Data.V_int = ((uint16_t)(Rx_Buf[3]) << 4) | ((uint16_t)(Rx_Buf[4]) >> 4);

    // 力矩原始值：12 位，Byte[4] 低 4 位为高 4 位，Byte[5] 为低 8 位
    DM_Motor->Data.T_int = ((uint16_t)(Rx_Buf[4] & 0x0F) << 8) | ((uint16_t)(Rx_Buf[5]));

    // 将 12 位原始力矩值映射回物理量（单位：N·m）
    DM_Motor->Data.Torque = uint_to_float(DM_Motor->Data.T_int,
                                          -DM_Motor->Param_Range.T_MAX,
                                          DM_Motor->Param_Range.T_MAX,
                                          12);

    // 将 16 位原始位置值映射回物理量（单位：rad）
    DM_Motor->Data.Position = uint_to_float(DM_Motor->Data.P_int,
                                            -DM_Motor->Param_Range.P_MAX,
                                            DM_Motor->Param_Range.P_MAX,
                                            16);

    // 将 12 位原始速度值映射回物理量（单位：rad/s）
    DM_Motor->Data.Velocity = uint_to_float(DM_Motor->Data.V_int,
                                            -DM_Motor->Param_Range.V_MAX,
                                            DM_Motor->Param_Range.V_MAX,
                                            12);

    // 温度直接取整数字节，单位：°C
    DM_Motor->Data.Temperature_MOS   = (float)Rx_Buf[6];
    DM_Motor->Data.Temperature_Rotor = (float)Rx_Buf[7];
}

/**
 * @brief  统一解析 FDCAN2 上的 DM 电机反馈
 * @param  Identifier : 接收标准 ID
 * @param  Data       : 接收数据
 * @retval None
 */
void DM_Motor_Class::RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
    for (int i = 0; i < 4; i++) {
        Info_Update(Identifier, Data, &DM_8009_Motor[i]);
    }
}

/**
 * @brief  发送控制命令帧（使能 / 失能 / 存零点）
 * @param  FDCAN_TxFrame : CAN 发送帧指针
 * @param  DM_Motor      : 目标电机对象指针
 * @param  CMD           : 命令类型（Motor_Enable / Motor_Disable / Motor_Save_Zero_Position）
 * @retval None
 *
 * DM 电机命令帧格式：
 *   Byte[0~6] = 0xFF（固定填充）
 *   Byte[7]   = 0xFC 使能 / 0xFD 失能 / 0xFE 存零点
 */
void DM_Motor_Class::Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                             DM_Motor_Info_Typedef *DM_Motor,
                             uint8_t CMD)
{
    FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;
    FDCAN_TxFrame->Header.DataLength = FDCAN_DLC_BYTES_8;

    id_dm = DM_Motor->FDCANFrame.TxIdentifier;

    // Byte[0~6] 固定为 0xFF，Byte[7] 由命令类型决定
    FDCAN_TxFrame->Data[0] = 0xFF;
    FDCAN_TxFrame->Data[1] = 0xFF;
    FDCAN_TxFrame->Data[2] = 0xFF;
    FDCAN_TxFrame->Data[3] = 0xFF;
    FDCAN_TxFrame->Data[4] = 0xFF;
    FDCAN_TxFrame->Data[5] = 0xFF;
    FDCAN_TxFrame->Data[6] = 0xFF;

    switch (CMD) {
        case Motor_Enable:
            FDCAN_TxFrame->Data[7] = 0xFC;  // 使能电机
            break;

        case Motor_Disable:
            FDCAN_TxFrame->Data[7] = 0xFD;  // 失能电机
            break;

        case Motor_Save_Zero_Position:
            FDCAN_TxFrame->Data[7] = 0xFE;  // 保存当前位置为零点
            break;

        default:
            return;
    }

    BSP_CAN::AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
 * @brief  打包并发送 DM 电机控制帧（支持三种控制模式）
 * @param  FDCAN_TxFrame : CAN 发送帧指针
 * @param  DM_Motor      : 目标电机对象指针
 * @param  Postion       : 目标位置（rad）
 * @param  Velocity      : 目标速度（rad/s）
 * @param  KP            : 位置环增益（MIT 模式，范围 0~500）
 * @param  KD            : 速度环增益（MIT 模式，范围 0~5）
 * @param  Torque        : 前馈力矩（N·m）
 * @retval None
 */
void DM_Motor_Class::CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                                   DM_Motor_Info_Typedef *DM_Motor,
                                   float Postion,
                                   float Velocity,
                                   float KP,
                                   float KD,
                                   float Torque)
{
    if (DM_Motor->Control_Mode == MIT) {
        /*
         * MIT 模式控制帧（8 字节）：
         *   Byte[0~1] : 位置（16 位）
         *   Byte[2]   : 速度高 8 位
         *   Byte[3]   : 速度低 4 位 + KP 高 4 位
         *   Byte[4]   : KP 低 8 位
         *   Byte[5]   : KD 高 8 位
         *   Byte[6]   : KD 低 4 位 + 力矩高 4 位
         *   Byte[7]   : 力矩低 8 位
         */
        uint16_t Postion_Tmp;
        uint16_t Velocity_Tmp;
        uint16_t Torque_Tmp;
        uint16_t KP_Tmp;
        uint16_t KD_Tmp;

        // 将浮点物理量映射为固定位宽整数
        Postion_Tmp  = float_to_uint(Postion, -DM_Motor->Param_Range.P_MAX, DM_Motor->Param_Range.P_MAX, 16);
        Velocity_Tmp = float_to_uint(Velocity, -DM_Motor->Param_Range.V_MAX, DM_Motor->Param_Range.V_MAX, 12);
        Torque_Tmp   = float_to_uint(Torque, -DM_Motor->Param_Range.T_MAX, DM_Motor->Param_Range.T_MAX, 12);
        KP_Tmp       = float_to_uint(KP, 0, 500, 12);
        KD_Tmp       = float_to_uint(KD, 0, 5, 12);

        FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;
        FDCAN_TxFrame->Header.DataLength = FDCAN_DLC_BYTES_8;

        // 按协议格式拼装各字节
        FDCAN_TxFrame->Data[0] = (uint8_t)(Postion_Tmp >> 8);
        FDCAN_TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
        FDCAN_TxFrame->Data[2] = (uint8_t)(Velocity_Tmp >> 4);
        FDCAN_TxFrame->Data[3] = (uint8_t)((Velocity_Tmp & 0x0F) << 4) | (uint8_t)(KP_Tmp >> 8);
        FDCAN_TxFrame->Data[4] = (uint8_t)(KP_Tmp);
        FDCAN_TxFrame->Data[5] = (uint8_t)(KD_Tmp >> 4);
        FDCAN_TxFrame->Data[6] = (uint8_t)((KD_Tmp & 0x0F) << 4) | (uint8_t)(Torque_Tmp >> 8);
        FDCAN_TxFrame->Data[7] = (uint8_t)(Torque_Tmp);

    } else if (DM_Motor->Control_Mode == POSITION_VELOCITY) {
        /*
         * 位置速度模式控制帧（8 字节）：
         *   Byte[0~3] : 目标位置（float，小端序）
         *   Byte[4~7] : 目标速度（float，小端序）
         *   CAN ID 偏移 +0x100
         */
        uint8_t *Postion_Tmp  = (uint8_t *)&Postion;
        uint8_t *Velocity_Tmp = (uint8_t *)&Velocity;

        FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x100;
        FDCAN_TxFrame->Header.DataLength = FDCAN_DLC_BYTES_8;

        FDCAN_TxFrame->Data[0] = *(Postion_Tmp);
        FDCAN_TxFrame->Data[1] = *(Postion_Tmp + 1);
        FDCAN_TxFrame->Data[2] = *(Postion_Tmp + 2);
        FDCAN_TxFrame->Data[3] = *(Postion_Tmp + 3);
        FDCAN_TxFrame->Data[4] = *(Velocity_Tmp);
        FDCAN_TxFrame->Data[5] = *(Velocity_Tmp + 1);
        FDCAN_TxFrame->Data[6] = *(Velocity_Tmp + 2);
        FDCAN_TxFrame->Data[7] = *(Velocity_Tmp + 3);

    } else if (DM_Motor->Control_Mode == VELOCITY) {
        /*
         * 速度模式控制帧（8 字节）：
         *   Byte[0~3] : 目标速度（float，小端序）
         *   Byte[4~7] : 固定为 0
         *   CAN ID 偏移 +0x200
         */
        uint8_t *Velocity_Tmp = (uint8_t *)&Velocity;

        FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x200;
        FDCAN_TxFrame->Header.DataLength = FDCAN_DLC_BYTES_8;

        FDCAN_TxFrame->Data[0] = *(Velocity_Tmp);
        FDCAN_TxFrame->Data[1] = *(Velocity_Tmp + 1);
        FDCAN_TxFrame->Data[2] = *(Velocity_Tmp + 2);
        FDCAN_TxFrame->Data[3] = *(Velocity_Tmp + 3);
        FDCAN_TxFrame->Data[4] = 0;
        FDCAN_TxFrame->Data[5] = 0;
        FDCAN_TxFrame->Data[6] = 0;
        FDCAN_TxFrame->Data[7] = 0;

    } else {
        return;
    }

    BSP_CAN::AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
 * @brief  将无符号整数映射回浮点物理量
 * @param  X_int : 原始整数值（来自 CAN 帧）
 * @param  X_min : 物理量下限
 * @param  X_max : 物理量上限
 * @param  Bits  : 编码位宽（12 或 16）
 * @retval 对应的浮点物理量
 */
float DM_Motor_Class::uint_to_float(int X_int, float X_min, float X_max, int Bits)
{
    float span   = X_max - X_min;
    float offset = X_min;
    return ((float)X_int) * span / ((float)((1 << Bits) - 1)) + offset;
}

/**
 * @brief  将浮点物理量编码为无符号整数（用于打包 CAN 控制帧）
 * @param  x     : 目标物理量
 * @param  x_min : 物理量下限
 * @param  x_max : 物理量上限
 * @param  bits  : 编码位宽（12 或 16）
 * @retval 编码后的整数值
 */
int DM_Motor_Class::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
