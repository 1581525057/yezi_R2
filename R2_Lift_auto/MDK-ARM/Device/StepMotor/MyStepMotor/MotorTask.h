#ifndef __MOTORTASK_H__
#define __MOTORTASK_H__

#include "Emm_V5.h"
#define pi 3.14159265358979323846
#define R1  21.0 // X轴滚轮直径，单位mm
#define R2  18.0 // Z轴滚轮直径，单位mm
//步进电机参数
#define X_MOTOR_ADDR       0x02    //X轴电机地址
#define X_MOTOR_DIR        0x00    //X轴电机正方向 0为CW 其余值为CWW（设为1）
#define X_MOTOR_KP         1.0f    //X轴电机Kp
#define X_MOTOR_KI         0    //X轴电机Ki
#define Z_MOTOR_ADDR       0x01    //Z轴电机地址
#define Z_MOTOR_DIR        0x01    //Z轴电机正方向 0为CW 其余值为CWW（设为1）
#define Z_MOTOR_KP         1.0f    //Z轴电机Kp
#define Z_MOTOR_KI         0    //Z轴电机Ki
#define MAX_SPEED          50   //最大速度(RPM)
#define Motor_ACC          0      //电机加速度 0为直接启动
//步进电机结构体
typedef struct
{
    uint8_t addr; //电机地址
    uint8_t dir; //电机方向
    float speed; //电机速度
    uint8_t acc; //电机加速度
    float kp;
    float ki;
    int16_t error; //误差
    int16_t i_sum; //积分和
}StepMotor_HanldeTypedef;
extern StepMotor_HanldeTypedef StepMotor_X;
extern StepMotor_HanldeTypedef StepMotor_Z;

void Motor_SendCmd();
int X_MotorParam_Init(StepMotor_HanldeTypedef *motor);
int Z_MotorParam_Init(StepMotor_HanldeTypedef *motor); 
void StepMotor_Init();
void X_Motor_Speed_Cal(StepMotor_HanldeTypedef *motor);
void Z_Motor_Speed_Cal(StepMotor_HanldeTypedef *motor);
void step_motor_position_control_x(uint8_t dir, uint16_t vel, uint8_t acc, float distance_mm);
void step_motor_position_control_z(uint8_t dir, uint16_t vel, uint8_t acc, float distance_mm);
void Motor_Ctrl(StepMotor_HanldeTypedef *xmotor, StepMotor_HanldeTypedef *zmotor);

extern DMA_HandleTypeDef hdma_uart8_rx;

#endif