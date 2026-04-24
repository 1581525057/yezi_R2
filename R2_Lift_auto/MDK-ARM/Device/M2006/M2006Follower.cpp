#include "M2006Follower.h"

namespace
{
constexpr uint16_t kMotorAllControlStdId = 0x1FFU;   // 电机控制标准 CAN ID
constexpr uint8_t kMotorCountPerFrame = 4U;          // 每帧可控制的电机数量
constexpr uint8_t kCanPayloadSize = 8U;              // CAN 负载长度
constexpr uint8_t kDefaultStepDescendDir = 0U;       // 默认步进下降方向
constexpr int8_t kDefaultCommandSign = -1;           // 默认命令方向符号
constexpr int8_t kDefaultFeedbackSign = 1;           // 默认反馈方向符号
constexpr float kDefaultStepToRotorRatio = 11.5714f; // 步进到转子转速比
constexpr float kDefaultKff = 1.5f;                  // 默认前馈系数
constexpr float kDefaultKp = 0.02f;                  // 默认比例系数
constexpr float kDefaultKi = 0.0f;                   // 默认积分系数
constexpr int16_t kDefaultStartCurrent = 900;        // 默认启动电流，单位：mA
constexpr int16_t kDefaultMaxAssistCurrent = 3500;   // 默认最大辅助电流，单位：mA
constexpr uint32_t kDefaultCanTimeoutMs = 50U;       // 默认 CAN 超时时间，单位：ms

/**
 * @brief 对浮点数进行限幅
 *
 * @param value 输入值
 * @param min_value 下限
 * @param max_value 上限
 * @return float 限幅后的结果
 */
float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}
}

M2006FollowerMotor g_m2006(&hfdcan3, 0x207U, 2U);

/**
 * @brief 构造 M2006 随动电机对象
 *
 * @param hcan CAN 句柄
 * @param rx_id 接收 CAN ID
 * @param tx_slot 发送槽位
 * @return 无
 */
M2006FollowerMotor::M2006FollowerMotor(FDCAN_HandleTypeDef *hfdcan, uint16_t rx_id, uint8_t tx_slot)
    : hfdcan_(hfdcan),
      rx_id_(rx_id),
      tx_std_id_(kMotorAllControlStdId),
      tx_slot_(tx_slot),
      step_descend_dir_(kDefaultStepDescendDir),
      cmd_sign_(kDefaultCommandSign),
      feedback_sign_(kDefaultFeedbackSign),
      step_to_rotor_ratio_(kDefaultStepToRotorRatio),
      kff_(kDefaultKff),
      kp_(kDefaultKp),
      ki_(kDefaultKi),
      i_start_min_(kDefaultStartCurrent),
      i_max_assist_(kDefaultMaxAssistCurrent),
      can_timeout_ms_(kDefaultCanTimeoutMs),
      ecd_(0U),
      speed_rpm_raw_(0),
      given_current_(0),
      last_rx_tick_(0U),
      feedback_received_(0U),
      step_dir_(0U),
      step_speed_rpm_(0.0f),
      target_speed_rpm_(0.0f),
      integral_(0.0f),
      current_cmd_(0),
      descend_active_(0U)
{
}

/**
 * @brief 初始化随动电机运行状态
 *
 * @return 无
 */
void M2006FollowerMotor::Init()
{
    feedback_received_ = 0U;
    speed_rpm_raw_ = 0;
    given_current_ = 0;
    ecd_ = 0U;
    last_rx_tick_ = 0U;
    ResetControl();
}

/**
 * @brief 清空控制器内部状态
 *
 * @return 无
 */
void M2006FollowerMotor::ResetControl()
{
    target_speed_rpm_ = 0.0f;
    integral_ = 0.0f;
    current_cmd_ = 0;
    descend_active_ = 0U;
}

/**
 * @brief 停止电机输出
 *
 * @return 无
 */
void M2006FollowerMotor::Stop()
{
    ResetControl();
    SendCurrent(0);
}

/**
 * @brief 更新电机反馈数据
 *
 * @param data CAN 数据帧
 * @param now_ms 当前时刻，单位：ms
 * @return 无
 */
void M2006FollowerMotor::UpdateFeedback(const uint8_t data[8], uint32_t now_ms)
{
    if (data == nullptr)
    {
        return;
    }

    ecd_ = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    speed_rpm_raw_ = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
    given_current_ = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
    last_rx_tick_ = now_ms;
    feedback_received_ = 1U;
}

/**
 * @brief 设置步进轴参考方向和速度
 *
 * @param step_dir 步进方向
 * @param step_speed_rpm 步进速度，单位：RPM
 * @return 无
 */
void M2006FollowerMotor::SetStepReference(uint8_t step_dir, float step_speed_rpm)
{
    step_dir_ = step_dir;
    step_speed_rpm_ = step_speed_rpm;
}

/**
 * @brief 执行一次随动控制计算
 *
 * @param now_ms 当前时刻，单位：ms
 * @return 无
 */
void M2006FollowerMotor::ControlTick(uint32_t now_ms)
{
    float measured_rpm = 0.0f;
    float error = 0.0f;
    float feedforward = 0.0f;
    float output = 0.0f;
    float integral_limit = 0.0f;
    int16_t current_abs = 0;

    UpdateTargetSpeed();
    if (descend_active_ == 0U)
    {
        Stop();
        return;
    }

    if (IsFeedbackAlive(now_ms) == false)
    {
        Stop();
        return;
    }

    measured_rpm = GetMeasuredSpeedRpmInternal();
    error = target_speed_rpm_ - measured_rpm;
    integral_ += error;

    if (ki_ > 0.0f)
    {
        integral_limit = static_cast<float>(i_max_assist_) / ki_;
        integral_ = ClampFloat(integral_, -integral_limit, integral_limit);
    }

    feedforward = kff_ * target_speed_rpm_;
    if (feedforward > 0.0f && feedforward < static_cast<float>(i_start_min_))
    {
        feedforward = static_cast<float>(i_start_min_);
    }

    output = feedforward + kp_ * error + ki_ * integral_;
    output = ClampFloat(output, 0.0f, static_cast<float>(i_max_assist_));

    current_abs = static_cast<int16_t>(output + 0.5f);
    if (current_abs > 0 && current_abs < i_start_min_)
    {
        current_abs = i_start_min_;
    }

    current_cmd_ = static_cast<int16_t>(cmd_sign_ * current_abs);
    SendCurrent(current_cmd_);
}

/**
 * @brief 判断反馈是否仍然有效
 *
 * @param now_ms 当前时刻，单位：ms
 * @return bool 有效返回 true
 */
bool M2006FollowerMotor::IsFeedbackAlive(uint32_t now_ms) const
{
    if (feedback_received_ == 0U)
    {
        return false;
    }
    return static_cast<uint32_t>(now_ms - last_rx_tick_) <= can_timeout_ms_;
}

/**
 * @brief 获取当前电流命令
 *
 * @return int16_t 电流命令值，单位：mA
 */
int16_t M2006FollowerMotor::GetCurrentCommand() const
{
    return current_cmd_;
}

/**
 * @brief 获取原始转速反馈
 *
 * @return int16_t 原始转速，单位：RPM
 */
int16_t M2006FollowerMotor::GetSpeedRpmRaw() const
{
    return speed_rpm_raw_;
}

/**
 * @brief 获取按方向换算后的转速反馈
 *
 * @return float 转速，单位：RPM
 */
float M2006FollowerMotor::GetMeasuredSpeedRpm() const
{
    return GetMeasuredSpeedRpmInternal();
}

/**
 * @brief 获取当前目标转速
 *
 * @return float 目标转速，单位：RPM
 */
float M2006FollowerMotor::GetTargetSpeedRpm() const
{
    return target_speed_rpm_;
}

/**
 * @brief 判断下降随动是否激活
 *
 * @return bool 激活返回 true
 */
bool M2006FollowerMotor::IsDescendingActive() const
{
    return descend_active_ != 0U;
}

/**
 * @brief 设置电流命令方向符号
 *
 * @param sign 方向符号，只允许 1 或 -1
 * @return 无
 */
void M2006FollowerMotor::SetCommandSign(int8_t sign)
{
    if (sign == 1 || sign == -1)
    {
        cmd_sign_ = sign;
    }
}

/**
 * @brief 设置反馈方向符号
 *
 * @param sign 方向符号，只允许 1 或 -1
 * @return 无
 */
void M2006FollowerMotor::SetFeedbackSign(int8_t sign)
{
    if (sign == 1 || sign == -1)
    {
        feedback_sign_ = sign;
    }
}

/**
 * @brief 设置步进下降方向
 *
 * @param dir 下降方向
 * @return 无
 */
void M2006FollowerMotor::SetDescendDirection(uint8_t dir)
{
    step_descend_dir_ = dir;
}

/**
 * @brief 根据步进参考更新目标转速
 *
 * @return 无
 */
void M2006FollowerMotor::UpdateTargetSpeed(void)
{
    if (step_dir_ == step_descend_dir_ && step_speed_rpm_ > 0.0f)
    {
        descend_active_ = 1U;
        target_speed_rpm_ = step_to_rotor_ratio_ * step_speed_rpm_;
    }
    else
    {
        descend_active_ = 0U;
        target_speed_rpm_ = 0.0f;
    }
}

/**
 * @brief 发送电流控制帧
 *
 * @param current 目标电流，单位：mA
 * @return 无
 */
void M2006FollowerMotor::SendCurrent(int16_t current)
{
    (void)current;

    if (hfdcan_ == nullptr || tx_slot_ >= kMotorCountPerFrame)
    {
        return;
    }
}

/**
 * @brief 获取内部换算后的转速反馈
 *
 * @return float 转速，单位：RPM
 */
float M2006FollowerMotor::GetMeasuredSpeedRpmInternal(void) const
{
    return static_cast<float>(feedback_sign_) * static_cast<float>(speed_rpm_raw_);
}
