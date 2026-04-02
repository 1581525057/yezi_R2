#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include <stdbool.h>
#include <math.h>

/**
 * @brief typedef enum that contains the type of DJI Motor Device.
 */
typedef enum
{
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
    DJI_MOTOR_TYPE_NUM,
} DJI_Motor_Type_e;

/**
 * @brief typedef structure that contains CAN Tx / Rx identifier.
 */
typedef struct
{
    uint32_t TxIdentifier;
    uint32_t RxIdentifier;
} DJI_Motor_CANFrameInfo_Typedef;

/**
 * @brief typedef structure that contains the data of DJI motor.
 */
typedef struct
{
    bool Initlized;
    int16_t Current;
    int16_t Velocity;
    float Rpm;
    int16_t Encoder;
    int16_t Last_Encoder;
    float Angle;
    uint8_t Temperature;
} DJI_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the information of DJI motor.
 */
typedef struct
{
    DJI_Motor_Type_e Type;
    DJI_Motor_CANFrameInfo_Typedef FDCANFrame;
    DJI_Motor_Data_Typedef Data;
} DJI_Motor_Info_Typedef;

/**
 * @brief
 *        DJI 电机类
 *        负责：
 *        1、定义 DJI 电机对象
 *        2、解析 DJI 电机反馈
 *        3、编码器角度解算
 *        4、发送 4 电机电流控制帧
 */
class   DJI_Motor_Class
{
public:
    /* 电机对象 --------------------------------------------------------------*/
    static DJI_Motor_Info_Typedef DJI_Yaw_Motor;
    static DJI_Motor_Info_Typedef Chassis_Motor[8];
    static DJI_Motor_Info_Typedef Lift_2006[2];

public:
    /* 对外接口 --------------------------------------------------------------*/
    static void Info_Update(uint32_t *Identifier,
                            uint8_t *Rx_Buf,
                            DJI_Motor_Info_Typedef *DJI_Motor);

    static void Send_CurrentCommand(FDCAN_TxFrame_TypeDef *DJI_Motor,
                                    uint32_t STDID,
                                    int16_t M1,
                                    int16_t M2,
                                    int16_t M3,
                                    int16_t M4);

    static void RxHandler(uint32_t *Identifier, uint8_t Data[8]);

    static float F_Loop_Constrain(float Input, float Min_Value, float Max_Value);

private:
    static float Encoder_To_Anglesum(DJI_Motor_Data_Typedef *Data,
                                     float Torque_Ratio,
                                     uint16_t MAXEncoder);

    static float Encoder_To_Angle(DJI_Motor_Data_Typedef *Data,
                                  float Torque_Ratio,
                                  uint16_t MAXEncoder);
};

extern DJI_Motor_Class chassis_motor;
extern DJI_Motor_Class yaw_motor;
extern DJI_Motor_Class lift_motor;
#endif
