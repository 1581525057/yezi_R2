#include "MotorTask.h"
#include "VisData.h"
#include "chassis_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "PID.h"
#include "fdcan.h"
#include "bsp_usart.h"
#include "bsp_dwt.h"
#include "qt_remove.h"
#include "lift_class.h"
#include "Motor.h"
#include "dm_imu.h"
#include "bsp_remove.h"
#include "omni_chassis.h"
#include "yun_j60.h"
#include "usart_task.h"
#define INIT 0
#define MOVE 1
int MotorState = 0;

StepMotor_HanldeTypedef StepMotor_X; // X轴电机
StepMotor_HanldeTypedef StepMotor_Z; // Z轴电机
extern float Yaw_value;
extern float speed;
extern float angle;
#define yaw (Yaw_value * pi) / 180.0f
/**
 * @brief 发送多电机命令函数
 * @note 该函数用于发送多电机命令，广播方式发送
 */
void Motor_SendCmd()
{
    Emm_V5_Multi_Motor_Cmd(0); // 广播发送多电机命令
}

/**
 * @brief X轴电机参数初始化函数
 * @note 该函数用于初始化X轴电机参数，包括地址、方向、速度、加速度、Kp、Ki、Kd和误差
 * @param motor 指向X轴电机结构体的指针
 * @retval 0 初始化成功
 */
int X_MotorParam_Init(StepMotor_HanldeTypedef *motor)
{
    motor->addr = X_MOTOR_ADDR;
    motor->dir = X_MOTOR_DIR;
    motor->speed = 0;
    motor->acc = Motor_ACC;
    motor->kp = X_MOTOR_KP;
    motor->ki = X_MOTOR_KI;
    motor->error = 0;
    motor->i_sum = 0;
    return 0;
}

/**
 * @brief Z轴电机参数初始化函数
 * @note 该函数用于初始化Z轴电机参数，包括地址、方向、速度、加速度、Kp、Ki、Kd和误差
 * @param motor 指向Z轴电机结构体的指针
 * @retval 0 初始化成功
 */
int Z_MotorParam_Init(StepMotor_HanldeTypedef *motor)
{
    motor->addr = Z_MOTOR_ADDR;
    motor->dir = Z_MOTOR_DIR;
    motor->speed = 0;
    motor->acc = Motor_ACC;
    motor->kp = Z_MOTOR_KP;
    motor->ki = Z_MOTOR_KI;
    motor->error = 0;
    motor->i_sum = 0;
    return 0;
}

/**
 * @brief 步进电机初始化函数
 * @note 该函数用于初始化步进电机，包括使能电机和清零当前位置
 *       每个操作后等待驱动器响应完成
 */
void StepMotor_Init()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart_cmd, RxCmd, sizeof(RxCmd)); // 接收电机反馈数据
    __HAL_DMA_DISABLE_IT(&hdma_usart_cmd_rx, DMA_IT_HT);            // 关闭传输过半中断
    osDelay(100);                                                   // 上电延时100毫秒等待Y42闭环初始化完毕
    X_MotorParam_Init(&StepMotor_X);                                // 初始化X轴电机参数
    Z_MotorParam_Init(&StepMotor_Z);                                // 初始化Z轴电机参数
    Emm_V5_MMCL_En_Control(StepMotor_X.addr, true, false);          // 使能X轴电机 不启用多机同步
    Emm_V5_MMCL_En_Control(StepMotor_Z.addr, true, false);          // 使能Z轴电机 不启用多机同步
    Motor_SendCmd();
    osDelay(10);
    Emm_V5_Reset_CurPos_To_Zero(StepMotor_X.addr); // 将X轴当前位置清零
    Emm_V5_Reset_CurPos_To_Zero(StepMotor_Z.addr); // 将Z轴当前位置清零
    Motor_SendCmd();
    osDelay(10);
    //		Emm_V5_Reset_Clog_Pro(StepMotor_Z.addr);
    //		osDelay(10);
}

/**
 * @brief X轴电机速度计算函数
 * @note 该函数用于根据误差计算X轴电机的速度，并根据速度判断方向
 * @param motor 指向X轴电机结构体的指针
 */
void X_Motor_Speed_Cal(StepMotor_HanldeTypedef *motor)
{
    // PI控制器计算速度
    motor->i_sum += motor->error;
    motor->speed = motor->kp * motor->error + motor->ki * motor->i_sum;
    // 判断速度是否超过最大速度，正负都要判断
    if (motor->speed > MAX_SPEED)
    {
        motor->speed = MAX_SPEED;
    }
    else if (motor->speed < -MAX_SPEED)
    {
        motor->speed = -MAX_SPEED;
    }
    // 判断速度，如果为负 则取绝对值并改变方向
    if (motor->speed < 0)
    {
        motor->speed = -motor->speed;
        motor->dir = !X_MOTOR_DIR;
    }
    else
    {
        motor->dir = X_MOTOR_DIR;
    }
}

/**
 * @brief Z轴电机速度计算函数
 * @note 该函数用于根据误差计算Z轴电机的速度，并根据速度判断方向
 * @param motor 指向Z轴电机结构体的指针
 */
void Z_Motor_Speed_Cal(StepMotor_HanldeTypedef *motor)
{
    // PI控制器计算速度
    motor->i_sum += motor->error;
    motor->speed = motor->kp * (float)motor->error + motor->ki * (float)motor->i_sum;
    // 判断速度是否超过最大速度，正负都要判断
    if (motor->speed > MAX_SPEED)
    {
        motor->speed = MAX_SPEED;
    }
    else if (motor->speed < -MAX_SPEED)
    {
        motor->speed = -MAX_SPEED;
    }
    // 判断速度，如果为负 则取绝对值并改变方向
    if (motor->speed < 0)
    {
        motor->speed = -motor->speed;
        motor->dir = !Z_MOTOR_DIR; // 反方向
    }
    else
    {
        motor->dir = Z_MOTOR_DIR; // 正方向
    }
}

/**
 * @brief X轴步进电机位置控制函数
 * @param dir 运动方向：CW（顺时针）或 CCW（逆时针） 1为向右
 * @param vel 运动速度（RPM - 转/分钟）
 * @param acc 加速度参数
 * @param distance_mm 目标移动距离（毫米）
 *
 * @note 该函数将距离转换为脉冲数，并调用EMM_V5驱动器进行位置控制
 *       等待驱动器响应完成后函数返回
 */
void step_motor_position_control_x(uint8_t dir, uint16_t vel, uint8_t acc, float distance_mm)
{
    uint32_t clk = (uint32_t)(distance_mm / (pi * R1) * 3200.0); // 计算所需脉冲数
    Emm_V5_Pos_Control(X_MOTOR_ADDR, dir, vel, acc, clk, false, false);
}

/**
 * @brief Z轴步进电机位置控制函数
 * @param dir 运动方向：CW（顺时针）或 CCW（逆时针）1的时候为向上
 * @param vel 运动速度（RPM - 转/分钟）
 * @param acc 加速度参数
 * @param distance_mm 目标移动距离（毫米）
 *
 * @note 该函数将距离转换为脉冲数，并调用EMM_V5驱动器进行位置控制
 *       等待驱动器响应完成后函数返回
 */
void step_motor_position_control_z(uint8_t dir, uint16_t vel, uint8_t acc, float distance_mm)
{
    uint32_t clk = (uint32_t)(distance_mm / (pi * R2) * 3200.0 * 2); // 计算所需脉冲数
    Emm_V5_Pos_Control(Z_MOTOR_ADDR, dir, vel, acc, clk, false, false);
}

/**
 * @brief 步进电机控制函数
 * @note 该函数用于根据X轴和Z轴电机的误差计算速度并加载到多电机命令中，最后发送多电机命令
 * @param xmotor 指向X轴电机结构体的指针
 * @param zmotor 指向Z轴电机结构体的指针
 */
void Motor_Ctrl(StepMotor_HanldeTypedef *xmotor, StepMotor_HanldeTypedef *zmotor)
{
    // 计算两轴电机的速度
    X_Motor_Speed_Cal(xmotor);
    Z_Motor_Speed_Cal(zmotor);
    // 加载两个电机速度到多电机命令中
    Emm_V5_MMCL_Vel_Control(xmotor->addr, xmotor->dir, xmotor->speed, xmotor->acc, false);
    Emm_V5_MMCL_Vel_Control(zmotor->addr, zmotor->dir, zmotor->speed, zmotor->acc, false);
    // 发送多电机命令
    Motor_SendCmd();
}

void Fine_tuning_task(void *argument)
{
    for (;;)
    {
        if (MotorState == INIT)
        {

            StepMotor_Init();
            step_motor_position_control_z(1, 50, 0, 30);
            osDelay(10);
            MotorState = MOVE; // 切换状态，防止下一次循环重复初始化
        }
        else if (MotorState == MOVE)
        {

           
            Motor_Ctrl(&StepMotor_X, &StepMotor_Z);

        }
        osDelay(5); // 延时10ms
    }
}

