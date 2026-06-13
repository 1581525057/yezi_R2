#include "RS05.h"
#include "cmsis_os.h"

constexpr float kRs05DegreesToRadians = 0.01745329251994329577f;  // 角度转弧度比例系数
constexpr float kRs05PositionLimitCurrent = 12.0f;                 // 速度位置模式电流限幅，单位：A
constexpr float kRs05ZeroLockLimitCurrent = 16.0f;                 // 0 度紧保持电流限幅，单位：A
constexpr float kRs05CspLimitSpeed = 10.0f;                        // CSP 位置模式速度限制，单位：rad/s
constexpr float kRs05TorqueLimitNm = 5.5f;                          // RS05 峰值力矩限制，单位：Nm
constexpr float kRs05ZeroLockTorqueLimitNm = 10.0f;                 // 0 度紧保持转矩限制，单位：Nm
constexpr float kRs05PositionKp = 75.0f;                            // 位置环 Kp，默认 40，增大保持刚度
constexpr float kRs05SpeedKp = 7.0f;                                // 速度环 Kp，默认 6
constexpr float kRs05SpeedKi = 0.03f;                               // 速度环 Ki，默认 0.02
constexpr float kRs05ZeroLockPositionKp = 120.0f;                   // 0 度紧保持位置环 Kp
constexpr float kRs05ZeroLockSpeedKp = 8.0f;                         // 0 度紧保持速度环 Kp
constexpr float kRs05ZeroLockSpeedKi = 0.04f;                        // 0 度紧保持速度环 Ki
constexpr uint32_t kRs05CanStartCheckIntervalMs = 1U;              // 等待 CAN3 启动的轮询间隔
constexpr uint32_t kRs05PowerOnDelayMs = 200U;                     // CAN 启动后等待电机上电完成
constexpr uint32_t kRs05StopDelayMs = 20U;                         // 停止/失能后等待电机进入可切模式状态
constexpr uint32_t kRs05ModeSwitchDelayMs = 30U;                   // 模式切换后等待电机处理 CAN 帧
constexpr uint32_t kRs05ParamWriteDelayMs = 10U;                   // 参数写入后等待电机处理 CAN 帧

uint8_t g_rs05_initialized = 0U;  // 电机初始化标志，0 表示未初始化
uint8_t g_rs05_zero_lock_enabled = 0U;

RobStride_Motor g_rs05_motor(&hfdcan3, RS05_CANID, false);
float Angle = -94.0f;  // Keil Watch 可调目标角度，单位：degree
float Speed = 10.0f;

namespace
{
void RS05_WaitCan3Started(void)
{
    while (HAL_FDCAN_GetState(&hfdcan3) != HAL_FDCAN_STATE_BUSY)
    {
        osDelay(kRs05CanStartCheckIntervalMs);
    }
}

void RS05_WriteParameter(uint16_t index, float value)
{
    g_rs05_motor.Set_RobStride_Motor_parameter(index, value, Set_parameter);
    osDelay(kRs05ParamWriteDelayMs);
}

void RS05_ApplyCspHoldParameters(void)
{
    RS05_WriteParameter(0x700B, kRs05TorqueLimitNm);
    RS05_WriteParameter(0x7018, kRs05PositionLimitCurrent);
    RS05_WriteParameter(0x701E, kRs05PositionKp);
    RS05_WriteParameter(0x701F, kRs05SpeedKp);
    RS05_WriteParameter(0x7020, kRs05SpeedKi);
    RS05_WriteParameter(0x7017, kRs05CspLimitSpeed);
}

void RS05_ApplyZeroLockParameters(void)
{
    RS05_WriteParameter(0x700B, kRs05ZeroLockTorqueLimitNm);
    RS05_WriteParameter(0x7018, kRs05ZeroLockLimitCurrent);
    RS05_WriteParameter(0x701E, kRs05ZeroLockPositionKp);
    RS05_WriteParameter(0x701F, kRs05ZeroLockSpeedKp);
    RS05_WriteParameter(0x7020, kRs05ZeroLockSpeedKi);
    RS05_WriteParameter(0x7017, kRs05CspLimitSpeed);
}
}

/**
 * @brief 初始化 RS05 电机并切换到 CSP 位置模式
 *
 * @return 无
 */
void RS05_Init(void)
{
    if (g_rs05_initialized != 0U)
    {
        return;
    }

    RS05_WaitCan3Started();
    osDelay(kRs05PowerOnDelayMs);

    g_rs05_motor.Disenable_Motor(0);
    osDelay(kRs05StopDelayMs);

    g_rs05_motor.Set_RobStride_Motor_parameter(0x7005, CSP_control_mode, Set_mode);
    osDelay(kRs05ModeSwitchDelayMs);
    g_rs05_motor.Enable_Motor();
    osDelay(kRs05ModeSwitchDelayMs);
    RS05_ApplyCspHoldParameters();
    RS05_WriteParameter(0x7016, 0.0f);
    g_rs05_zero_lock_enabled = 0U;

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

void RS05_SetZeroLock(uint8_t enable)
{
    const uint8_t new_state = (enable == 0U) ? 0U : 1U;

    if (g_rs05_zero_lock_enabled == new_state)
    {
        return;
    }

    if (new_state != 0U)
    {
        RS05_ApplyZeroLockParameters();
    }
    else
    {
        RS05_ApplyCspHoldParameters();
    }

    g_rs05_zero_lock_enabled = new_state;
}

/**
 * @brief 使用 CSP 位置命令控制 RS05
 *
 * @param speed 目标速度，单位：rad/s
 * @param angle 目标角度，单位：rad
 * @return 无
 */
void RS05_PositionControl(float speed, float angle)
{
    g_rs05_motor.RobStride_Motor_CSP_control(angle, speed);
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
