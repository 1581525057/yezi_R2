#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "bsp_can.h"
#include <stdbool.h>

/**
 * @brief typedef enum that control mode the type of DM_Motor.
 */
typedef enum {
    MIT,
    POSITION_VELOCITY,
    VELOCITY,
} DM_Motor_Control_Mode_Type_e;

/**
 * @brief typedef enum that CMD of DM_Motor.
 */
typedef enum {
    Motor_Enable,
    Motor_Disable,
    Motor_Save_Zero_Position,
    DM_Motor_CMD_Type_Num,
} DM_Motor_CMD_Type_e;

/**
 * @brief typedef structure that contains CAN Tx / Rx identifier.
 */
typedef struct
{
    uint32_t TxIdentifier;
    uint32_t RxIdentifier;
} DM_Motor_CANFrameInfo_Typedef;

/**
 * @brief typedef structure that contains the param range for the DM_Motor.
 */
typedef struct
{
    float P_MAX;
    float V_MAX;
    float T_MAX;
} DM_Motor_Param_Range_Typedef;

/**
 * @brief typedef structure that contains the data for the DM_Motor.
 */
typedef struct
{
    bool Initlized;
    uint8_t State;
    uint16_t P_int;
    uint16_t V_int;
    uint16_t T_int;
    float Position;
    float Velocity;
    float Torque;
    float Temperature_MOS;
    float Temperature_Rotor;
    float Angle;
} DM_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the information for the DM Motor Device.
 */
typedef struct
{
    DM_Motor_Control_Mode_Type_e Control_Mode;
    DM_Motor_CANFrameInfo_Typedef FDCANFrame;
    DM_Motor_Param_Range_Typedef Param_Range;
    DM_Motor_Data_Typedef Data;
} DM_Motor_Info_Typedef;

/**
 * @brief typedef structure that contains the control information for the DM Motor Device.
 */
typedef struct
{
    float Position;
    float Velocity;
    float KP;
    float KD;
    float Torque;
    float Angle;
} DM_Motor_Contorl_Info_Typedef;

/**
 * @brief
 *        DM 电机类
 *        负责：
 *        1、定义 DM 电机对象
 *        2、发送使能 / 失能 / 存零点命令
 *        3、打包 MIT / 位置速度 / 速度控制帧
 *        4、解析 DM 电机反馈数据
 */
class DM_Motor_Class
{
public:
    /* 电机对象 --------------------------------------------------------------*/
    static DM_Motor_Info_Typedef DM_8009_Motor[4];
    static DM_Motor_Contorl_Info_Typedef DM_Motor_Contorl_Info[4];

public:
    /* 对外接口 --------------------------------------------------------------*/
    static void Info_Update(uint32_t *Identifier,
                            uint8_t *Rx_Buf,
                            DM_Motor_Info_Typedef *DM_Motor);

    static void RxHandler(uint32_t *Identifier, uint8_t Data[8]);

    static void Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                        DM_Motor_Info_Typedef *DM_Motor,
                        uint8_t CMD);

    static void CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                              DM_Motor_Info_Typedef *DM_Motor,
                              float Postion,
                              float Velocity,
                              float KP,
                              float KD,
                              float Torque);

private:
    static float uint_to_float(int X_int, float X_min, float X_max, int Bits);
    static int float_to_uint(float x, float x_min, float x_max, int bits);
};

#endif