/* USER CODE BEGIN Header */
// 负责：
// 1、定义 DJI 电机对象（M3508 / GM6020 / M2006）
// 2、收到 CAN 数据后，解析电机反馈
// 3、把编码器值转换成角度
// 4、发送 DJI 电机 4 合 1 电流控制帧
/* USER CODE END Header */

#include "dji_motor.h"
#include "bsp_can.h"
#include "dm_motor.h"

DJI_Motor_Class chassis_motor;
DJI_Motor_Class yaw_motor;
DJI_Motor_Class lift_motor;

DJI_Motor_Info_Typedef DJI_Motor_Class::DJI_Yaw_Motor =
    {
        .Type = DJI_GM6020,
        .FDCANFrame =
            {
                .TxIdentifier = 0x1FF,
                .RxIdentifier = 0x205,
            },
        .Data = {0},
};

DJI_Motor_Info_Typedef DJI_Motor_Class::Chassis_Motor[8] =
    {
        {DJI_M3508, {0x200, 0x201}, {0}},
        {DJI_M3508, {0x200, 0x202}, {0}},
        {DJI_M3508, {0x200, 0x203}, {0}},
        {DJI_M3508, {0x200, 0x204}, {0}},
        {DJI_M3508, {0x1FF, 0x205}, {0}},
        {DJI_M3508, {0x1FF, 0x206}, {0}},
        {DJI_M3508, {0x1FF, 0x207}, {0}},
        {DJI_M3508, {0x1FF, 0x208}, {0}}};

DJI_Motor_Info_Typedef DJI_Motor_Class::Lift_2006[2]{
    {DJI_M2006, {0x1FF, 0x205}, {0}},
    {DJI_M2006, {0x1FF, 0x206}, {0}}};
/**
 * @brief  Update the DJI motor Information
 * @param  Identifier : pointer to standard identifier
 * @param  Rx_Buf     : pointer to Rx data buffer
 * @param  DJI_Motor  : pointer to DJI motor object
 * @retval None
 */

void
    DJI_Motor_Class::Info_Update(uint32_t *Identifier,
                                 uint8_t *Rx_Buf,
                                 DJI_Motor_Info_Typedef *DJI_Motor)
{
    if (*Identifier != DJI_Motor->FDCANFrame.RxIdentifier)
        return;

    DJI_Motor->Data.Temperature = Rx_Buf[6];
    DJI_Motor->Data.Encoder = ((int16_t)Rx_Buf[0] << 8) | ((int16_t)Rx_Buf[1]);
    DJI_Motor->Data.Velocity = ((int16_t)Rx_Buf[2] << 8) | ((int16_t)Rx_Buf[3]);
    DJI_Motor->Data.Current = ((int16_t)Rx_Buf[4] << 8) | ((int16_t)Rx_Buf[5]);

    switch (DJI_Motor->Type)
    {
    case DJI_GM6020:
        DJI_Motor->Data.Angle = Encoder_To_Angle(&DJI_Motor->Data, 1.0f, 8192);
        break;

    case DJI_M3508:
        DJI_Motor->Data.Angle = Encoder_To_Anglesum(&DJI_Motor->Data, 3591.0f / 187.0f, 8192);
        DJI_Motor->Data.Rpm   = DJI_Motor->Data.Velocity / 19.0f;
        break;

    case DJI_M2006:
        DJI_Motor->Data.Angle = Encoder_To_Angle(&DJI_Motor->Data, 36.0f, 8192);
        DJI_Motor->Data.Rpm   = DJI_Motor->Data.Velocity / 36.0f;
        break;

    default:
        break;
    }
}

/**
 * @brief  统一解析 FDCAN1 上的 DJI 电机反馈
 * @param  Identifier : 接收标准 ID
 * @param  Data       : 接收数据
 * @retval None
 */

void DJI_Motor_Class::RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
    for (int i = 0; i < 8; i++)
    {
        Info_Update(Identifier, Data, &Chassis_Motor[i]);
    }
	
	 for (int i = 0; i < 4; i++)
    {
        Info_Update(Identifier, Data, &Lift_2006[i]);
    }

    Info_Update(Identifier, Data, &DJI_Yaw_Motor);
}

/**
 * @brief  Pack DJI motor current command and transmit.
 * @param  *DJI_Motor : FDCAN Tx frame object
 * @param  STDID      : standard identifier
 * @param  M1~M4      : motor current command
 * @retval None
 */

void DJI_Motor_Class::Send_CurrentCommand(FDCAN_TxFrame_TypeDef *DJI_Motor,
                                          uint32_t STDID,
                                          int16_t M1,
                                          int16_t M2,
                                          int16_t M3,
                                          int16_t M4)
{
    DJI_Motor->Header.Identifier = STDID;
    DJI_Motor->Header.DataLength = FDCAN_DLC_BYTES_8;

    DJI_Motor->Data[0] = (uint8_t)(M1 >> 8);
    DJI_Motor->Data[1] = (uint8_t)(M1 >> 8);
    DJI_Motor->Data[2] = (uint8_t)(M2 >> 8);
    DJI_Motor->Data[3] = (uint8_t)(M2 >> 8);
    DJI_Motor->Data[4] = (uint8_t)(M3 >> 8);
    DJI_Motor->Data[5] = (uint8_t)(M3 >> 8);
    DJI_Motor->Data[6] = (uint8_t)(M4 >> 8);
    DJI_Motor->Data[7] = (uint8_t)(M4 >> 8);

    BSP_CAN::AddMessageToTxFifoQ(DJI_Motor);
}

/**
 * @brief  float loop constrain
 * 把一个浮点数 Input 限制到 [Min_Value, Max_Value] 这个区间里，但不是硬夹紧，而是“循环回绕”进去。
它常用于：
角度处理，比如把角度限制到 -180~180 或 0~360
编码器圈数取模
周期量处理
 */
float DJI_Motor_Class::F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
    if (Max_Value < Min_Value)
        return Input;

    float len = Max_Value - Min_Value;

    if (Input > Max_Value)
    {
        do
        {
            Input -= len;
        } while (Input > Max_Value);
    }
    else if (Input < Min_Value)
    {
        do
        {
            Input += len;
        } while (Input < Min_Value);
    }

    return Input;
}

/**
 * @brief  transform the Encoder(0-8192) to anglesum
 * 把电机编码器当前值 Encoder(0 ~ MAXEncoder) 转换成“累计角度 Angle”
 */
float DJI_Motor_Class::Encoder_To_Anglesum(DJI_Motor_Data_Typedef *Data,
                                           float Torque_Ratio,
                                           uint16_t MAXEncoder)
{
    float res1 = 0.0f;
    float res2 = 0.0f;

    if (Data == nullptr)
        return 0.0f;

    if (Data->Initlized != true)
    {
        Data->Last_Encoder = Data->Encoder;
        Data->Angle = 0.0f;
        Data->Initlized = true;
    }

    if (Data->Encoder < Data->Last_Encoder)
    {
        res1 = Data->Encoder - Data->Last_Encoder + MAXEncoder;
    }
    else if (Data->Encoder > Data->Last_Encoder)
    {
        res1 = Data->Encoder - Data->Last_Encoder - MAXEncoder;
    }

    res2 = Data->Encoder - Data->Last_Encoder;
    Data->Last_Encoder = Data->Encoder;

    if (fabsf(res1) > fabsf(res2))
    {
        Data->Angle += (float)res2 / (MAXEncoder * Torque_Ratio) * 360.0f;
    }
    else
    {
        Data->Angle += (float)res1 / (MAXEncoder * Torque_Ratio) * 360.0f;
    }

    return Data->Angle;
}

/**
 * @brief  transform the Encoder(0-8192) to angle(-180~180)
 */
float DJI_Motor_Class::Encoder_To_Angle(DJI_Motor_Data_Typedef *Data,
                                        float Torque_Ratio,
                                        uint16_t MAXEncoder)
{
    float Encoder_Err = 0.0f;

    if (Data->Initlized != true)
    {
        Data->Last_Encoder = Data->Encoder;
        Data->Angle = Data->Encoder / (MAXEncoder * Torque_Ratio) * 360.0f;
        Data->Initlized = true;
    }

    Encoder_Err = Data->Encoder - Data->Last_Encoder;

    if (Encoder_Err > MAXEncoder * 0.5f)
    {
        Data->Angle += (float)(Encoder_Err - MAXEncoder) / (MAXEncoder * Torque_Ratio) * 360.0f;
    }
    else if (Encoder_Err < -MAXEncoder * 0.5f)
    {
        Data->Angle += (float)(Encoder_Err + MAXEncoder) / (MAXEncoder * Torque_Ratio) * 360.0f;
    }
    else
    {
        Data->Angle += (float)(Encoder_Err) / (MAXEncoder * Torque_Ratio) * 360.0f;
    }

    Data->Last_Encoder = Data->Encoder;
    Data->Angle = F_Loop_Constrain(Data->Angle, -180.0f, 180.0f);

    return Data->Angle;
}
