#ifndef __ROBSTRIDE_H__
#define __ROBSTRIDE_H__

#include "fdcan.h"
#include "main.h"

#define Set_mode 'j'
#define Set_parameter 'p'

/* RobStride 电机控制模式 */
#define move_control_mode 0
#define Pos_control_mode 1
#define Speed_control_mode 2
#define Elect_control_mode 3
#define Set_Zero_mode 4
#define CSP_control_mode 5

/* RobStride 扩展 CAN 通信类型 */
#define Communication_Type_Get_ID 0x00
#define Communication_Type_MotionControl 0x01
#define Communication_Type_MotorRequest 0x02
#define Communication_Type_MotorEnable 0x03
#define Communication_Type_MotorStop 0x04
#define Communication_Type_SetPosZero 0x06
#define Communication_Type_Can_ID 0x07
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11
#define Communication_Type_SetSingleParameter 0x12
#define Communication_Type_ErrorFeedback 0x15
#define Communication_Type_MotorDataSave 0x16
#define Communication_Type_BaudRateChange 0x17
#define Communication_Type_ProactiveEscalationSet 0x18
#define Communication_Type_MotorModeSet 0x19

#define CAN_ID_STD FDCAN_STANDARD_ID
#define CAN_ID_EXT FDCAN_EXTENDED_ID
#define CAN_RTR_DATA FDCAN_DATA_FRAME

typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
} FTM_CanTxHeaderTypeDef;

/* 单个参数读写项 */
class data_read_write_one
{
public:
    uint16_t index; // 参数索引
    float data;     // 参数值
};

/* 驱动参数索引表 */
static const uint16_t Index_List[] = {
    0X7005, 0X7006, 0X700A, 0X700B, 0X7010,
    0X7011, 0X7014, 0X7016, 0X7017, 0X7018,
    0x7019, 0x701A, 0x701B, 0x701C, 0x701D};

/* 电机参数读写缓存 */
class data_read_write
{
public:
    data_read_write_one run_mode;      // 运行模式
    data_read_write_one iq_ref;        // 电流参考
    data_read_write_one spd_ref;       // 速度参考
    data_read_write_one imit_torque;   // 转矩参考
    data_read_write_one cur_kp;        // 电流环比例系数
    data_read_write_one cur_ki;        // 电流环积分系数
    data_read_write_one cur_filt_gain; // 电流滤波增益
    data_read_write_one loc_ref;       // 位置参考
    data_read_write_one limit_spd;     // 速度限制
    data_read_write_one limit_cur;     // 电流限制
    data_read_write_one mechPos;       // 机械位置
    data_read_write_one iqf;           // 滤波后电流
    data_read_write_one mechVel;       // 机械速度
    data_read_write_one VBUS;          // 母线电压
    data_read_write_one rotation;      // 旋转角度

    data_read_write(const uint16_t *index_list = Index_List);
};

/* 电机反馈状态 */
typedef struct
{
    float Angle;   // 当前角度，单位：rad
    float Speed;   // 当前速度，单位：rad/s
    float Torque;  // 当前转矩，单位：Nm
    float Temp;    // 当前温度，单位：℃
    int pattern;   // 运行模式
} Motor_Pos_RobStride_Info;

/* 电机设定值缓存 */
typedef struct
{
    int set_motor_mode;      // 设定电机模式
    float set_current;       // 设定电流，单位：A
    float set_speed;         // 设定速度，单位：rad/s
    float set_acceleration;  // 设定加速度，单位：rad/s²
    float set_Torque;        // 设定转矩，单位：Nm
    float set_angle;         // 设定角度，单位：rad
    float set_limit_cur;     // 设定电流限制，单位：A
    float set_limit_speed;   // 设定速度限制，单位：rad/s
    float set_Kp;            // 设定比例系数
    float set_Ki;            // 设定积分系数
    float set_Kd;            // 设定微分系数
} Motor_Set;

/* MIT 模式控制子模式 */
enum MIT_TYPE
{
    operationControl = 0, // 操作控制模式
    positionControl = 1,  // 位置控制模式
    speedControl = 2      // 速度控制模式
};

/* RobStride 电机对象 */
class RobStride_Motor
{
private:
    FDCAN_HandleTypeDef *can;                        // FDCAN 句柄
    uint8_t CAN_ID;                                // 设备 ID
    uint64_t Unique_ID;                            // 唯一标识
    uint16_t Master_CAN_ID;                        // 主控 CAN ID
    float (*Motor_Offset_MotoFunc)(float Motor_Tar); // 角度偏移函数
    Motor_Set Motor_Set_All;                       // 设定值缓存
    uint8_t error_code;                            // 错误码
    bool MIT_Mode;                                 // MIT 模式标志
    MIT_TYPE MIT_Type;                             // MIT 控制子模式

    void Set_MIT_Mode(bool MIT_Mode);
    void Set_MIT_Type(MIT_TYPE MIT_Type);

public:
    float output;                     // 输出值
    int Can_Motor;                    // CAN 电机标志
    Motor_Pos_RobStride_Info Pos_Info; // 电机反馈信息
    data_read_write drw;              // 参数读写对象

    RobStride_Motor(FDCAN_HandleTypeDef *can_handle, uint8_t CAN_Id, bool MIT_Mode);
    RobStride_Motor(FDCAN_HandleTypeDef *can_handle,
                    float (*Offset_MotoFunc)(float Motor_Tar),
                    uint8_t CAN_Id,
                    bool MIT_mode);

    void RobStride_Get_CAN_ID();
    void Set_RobStride_Motor_parameter(uint16_t Index, float Value, char Value_mode);
    void Get_RobStride_Motor_parameter(uint16_t Index);
    void RobStride_Motor_Analysis(uint8_t *DataFrame, uint32_t ID_ExtId);

    void RobStride_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd);
    void RobStride_Motor_Pos_control(float Speed, float Angle);
    void RobStride_Motor_CSP_control(float Angle, float limit_spd);
    void RobStride_Motor_Speed_control(float Speed, float limit_cur);
    void RobStride_Motor_current_control(float current);
    void RobStride_Motor_Set_Zero_control();
    void RobStride_Motor_MotorModeSet(uint8_t F_CMD);
    void Enable_Motor();
    void Disenable_Motor(uint8_t clear_error);
    void Set_CAN_ID(uint8_t Set_CAN_ID);
    void Set_ZeroPos();

    bool Get_MIT_Mode();
    MIT_TYPE get_MIT_Type();
    void RobStride_Motor_MIT_Control(float Angle, float Speed, float Kp, float Kd, float Torque);
    void RobStride_Motor_MIT_PositionControl(float position_rad, float speed_rad_per_s);
    void RobStride_Motor_MIT_SpeedControl(float speed_rad_per_s, float current_limit);
    void RobStride_Motor_MIT_Enable();
    void RobStride_Motor_MIT_Disable();
    void RobStride_Motor_MIT_SetZeroPos();
    void RobStride_Motor_MIT_ClearOrCheckError(uint8_t F_CMD);
    void RobStride_Motor_MIT_SetMotorType(uint8_t F_CMD);
    void RobStride_Motor_MIT_SetMotorId(uint8_t F_CMD);
    void RobStride_Motor_MotorDataSave();
    void RobStride_Motor_BaudRateChange(uint8_t F_CMD);
    void RobStride_Motor_ProactiveEscalationSet(uint8_t F_CMD);
    void RobStride_Motor_MIT_MotorModeSet(uint8_t F_CMD);
};

#endif
