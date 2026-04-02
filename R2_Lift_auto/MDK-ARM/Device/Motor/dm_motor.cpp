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
 * @brief The structure that contains the Information of joint motor.
 *        Use DM 8009 motor.
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
 * @brief  Update the DM_Motor Information
 * @param  Identifier : pointer to standard identifier
 * @param  Rx_Buf     : pointer to Rx data buffer
 * @param  DM_Motor   : pointer to DM motor object
 * @retval None
 */
void DM_Motor_Class::Info_Update(uint32_t *Identifier,
                                 uint8_t *Rx_Buf,
                                 DM_Motor_Info_Typedef *DM_Motor)
{
    if (*Identifier != DM_Motor->FDCANFrame.RxIdentifier)
        return;

    DM_Motor->Data.State = Rx_Buf[0] >> 4;
    DM_Motor->Data.P_int = ((uint16_t)(Rx_Buf[1]) << 8) | ((uint16_t)(Rx_Buf[2]));
    DM_Motor->Data.V_int = ((uint16_t)(Rx_Buf[3]) << 4) | ((uint16_t)(Rx_Buf[4]) >> 4);
    DM_Motor->Data.T_int = ((uint16_t)(Rx_Buf[4] & 0x0F) << 8) | ((uint16_t)(Rx_Buf[5]));

    DM_Motor->Data.Torque = uint_to_float(DM_Motor->Data.T_int,
                                          -DM_Motor->Param_Range.T_MAX,
                                          DM_Motor->Param_Range.T_MAX,
                                          12);

    DM_Motor->Data.Position = uint_to_float(DM_Motor->Data.P_int,
                                            -DM_Motor->Param_Range.P_MAX,
                                            DM_Motor->Param_Range.P_MAX,
                                            16);

    DM_Motor->Data.Velocity = uint_to_float(DM_Motor->Data.V_int,
                                            -DM_Motor->Param_Range.V_MAX,
                                            DM_Motor->Param_Range.V_MAX,
                                            12);

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
 * @brief  Transmit enable / disable / save zero position command to DM motor
 * @param  FDCAN_TxFrame : pointer to FDCAN_TxFrame_TypeDef
 * @param  DM_Motor      : pointer to the DM_Motor
 * @param  CMD           : transmit command
 * @retval None
 */
void DM_Motor_Class::Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                             DM_Motor_Info_Typedef *DM_Motor,
                             uint8_t CMD)
{
    FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;
    FDCAN_TxFrame->Header.DataLength = FDCAN_DLC_BYTES_8;

    id_dm = DM_Motor->FDCANFrame.TxIdentifier;

    FDCAN_TxFrame->Data[0] = 0xFF;
    FDCAN_TxFrame->Data[1] = 0xFF;
    FDCAN_TxFrame->Data[2] = 0xFF;
    FDCAN_TxFrame->Data[3] = 0xFF;
    FDCAN_TxFrame->Data[4] = 0xFF;
    FDCAN_TxFrame->Data[5] = 0xFF;
    FDCAN_TxFrame->Data[6] = 0xFF;

    switch (CMD) {
        case Motor_Enable:
            FDCAN_TxFrame->Data[7] = 0xFC;
            break;

        case Motor_Disable:
            FDCAN_TxFrame->Data[7] = 0xFD;
            break;

        case Motor_Save_Zero_Position:
            FDCAN_TxFrame->Data[7] = 0xFE;
            break;

        default:
            return;
    }

    BSP_CAN::AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
 * @brief  CAN Transmit DM motor target information
 * @param  FDCAN_TxFrame : pointer to FDCAN Tx frame
 * @param  DM_Motor      : pointer to DM motor object
 * @param  Postion       : target position
 * @param  Velocity      : target velocity
 * @param  KP            : target KP
 * @param  KD            : target KD
 * @param  Torque        : target torque
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
        uint16_t Postion_Tmp;
        uint16_t Velocity_Tmp;
        uint16_t Torque_Tmp;
        uint16_t KP_Tmp;
        uint16_t KD_Tmp;

        Postion_Tmp  = float_to_uint(Postion, -DM_Motor->Param_Range.P_MAX, DM_Motor->Param_Range.P_MAX, 16);
        Velocity_Tmp = float_to_uint(Velocity, -DM_Motor->Param_Range.V_MAX, DM_Motor->Param_Range.V_MAX, 12);
        Torque_Tmp   = float_to_uint(Torque, -DM_Motor->Param_Range.T_MAX, DM_Motor->Param_Range.T_MAX, 12);
        KP_Tmp       = float_to_uint(KP, 0, 500, 12);
        KD_Tmp       = float_to_uint(KD, 0, 5, 12);

        FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;
        FDCAN_TxFrame->Header.DataLength = FDCAN_DLC_BYTES_8;

        FDCAN_TxFrame->Data[0] = (uint8_t)(Postion_Tmp >> 8);
        FDCAN_TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
        FDCAN_TxFrame->Data[2] = (uint8_t)(Velocity_Tmp >> 4);
        FDCAN_TxFrame->Data[3] = (uint8_t)((Velocity_Tmp & 0x0F) << 4) | (uint8_t)(KP_Tmp >> 8);
        FDCAN_TxFrame->Data[4] = (uint8_t)(KP_Tmp);
        FDCAN_TxFrame->Data[5] = (uint8_t)(KD_Tmp >> 4);
        FDCAN_TxFrame->Data[6] = (uint8_t)((KD_Tmp & 0x0F) << 4) | (uint8_t)(Torque_Tmp >> 8);
        FDCAN_TxFrame->Data[7] = (uint8_t)(Torque_Tmp);
    } else if (DM_Motor->Control_Mode == POSITION_VELOCITY) {
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
 * @brief  Convert uint to float
 */
float DM_Motor_Class::uint_to_float(int X_int, float X_min, float X_max, int Bits)
{
    float span   = X_max - X_min;
    float offset = X_min;
    return ((float)X_int) * span / ((float)((1 << Bits) - 1)) + offset;
}

/**
 * @brief  Convert float to uint
 */
int DM_Motor_Class::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
