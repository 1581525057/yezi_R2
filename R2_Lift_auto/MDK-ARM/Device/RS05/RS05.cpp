#include "RS05.h"
#include "cmsis_os.h"

constexpr float kRs05DegreesToRadians = 0.01745329251994329577f;  // 角度转弧度比例系数

uint8_t g_rs05_initialized = 0U;  // 电机初始化标志，0 表示未初始化

RobStride_Motor g_rs05_motor(&hfdcan3, RS05_CANID, false);
float Angle = 1.575f;
float Speed = 10.0f;

/**
 * @brief 初始化 RS05 电机并切换到位置模式
 *
 * @return 无
 */
void RS05_Init(void)
{
    if (g_rs05_initialized != 0U)
    {
        return;
    }

    g_rs05_motor.Set_RobStride_Motor_parameter(0x7005, Pos_control_mode, Set_mode);
    osDelay(10);
    g_rs05_motor.Enable_Motor();
    osDelay(20);

    g_rs05_initialized = 1U;
}

/**
 * @brief 转发 RS05 的 CAN 数据帧给底层驱动解析
 *
 * @param can_id 接收到的 CAN ID
 * @param data_frame 接收到的数据帧
 * @return 无
 */
void RS05_HandleCanMessage(uint32_t can_id, uint8_t *data_frame)
{
    if (data_frame == nullptr)
    {
        return;
    }

    g_rs05_motor.RobStride_Motor_Analysis(data_frame, can_id);
}

/**
 * @brief 使用弧度制位置命令控制 RS05
 *
 * @param speed 目标速度，单位：rad/s
 * @param angle 目标角度，单位：rad
 * @return 无
 */
void RS05_PositionControl(float speed, float angle)
{
    g_rs05_motor.RobStride_Motor_Pos_control(speed, angle);
}

/**
 * @brief 使用角度制位置命令控制 RS05
 *
 * @param speed 目标速度，单位：rad/s
 * @param angle_degree 目标角度，单位：度
 * @return 无
 */
void RS05_PositionControlDegree(float speed, float angle_degree)
{
    RS05_PositionControl(speed, angle_degree * kRs05DegreesToRadians);
}

/**
 * @brief 获取全局 RS05 电机实例
 *
 * @return RobStride_Motor& RS05 电机对象引用
 */
RobStride_Motor &RS05_GetMotor(void)
{
    return g_rs05_motor;
}
