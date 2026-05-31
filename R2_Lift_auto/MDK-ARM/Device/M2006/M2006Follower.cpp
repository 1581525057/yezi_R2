#include "M2006Follower.h"

namespace
{
// M2006 的电流命令共用 DJI 电机 0x1FF 标准帧，每帧最多携带 4 路电机电流。
constexpr uint16_t kMotorAllControlStdId = 0x1FFU;
constexpr uint8_t kMotorCountPerFrame = 4U;

// 默认参数按当前升降机构安装方向和实测随动效果配置，可通过设置函数修正方向符号。
constexpr uint8_t kDefaultStepDescendDir = 0U;
constexpr int8_t kDefaultCommandSign = -1;
constexpr int8_t kDefaultFeedbackSign = 1;
constexpr float kDefaultStepToRotorRatio = 11.5714f;
constexpr float kDefaultKp = 1.5f;
constexpr float kDefaultKi = 0.6f;
constexpr float kDefaultKd = 0.0f;
constexpr float kDefaultSpeedDeadbandRpm = 1.0f;
constexpr float kMaxControlDtSec = 0.05f;
constexpr int16_t kDefaultStartCurrent = 100;
constexpr int16_t kDefaultMaxAssistCurrent = 3500;

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

M2006FollowerMotor::M2006FollowerMotor(FDCAN_HandleTypeDef *hfdcan, uint16_t rx_id, uint8_t tx_slot)
    : kp(kDefaultKp),
      ki(kDefaultKi),
      kd(kDefaultKd),
      hfdcan_(hfdcan),
      rx_id_(rx_id),
      tx_std_id_(kMotorAllControlStdId),
      tx_slot_(tx_slot),
      step_descend_dir_(kDefaultStepDescendDir),
      cmd_sign_(kDefaultCommandSign),
      feedback_sign_(kDefaultFeedbackSign),
      step_to_rotor_ratio_(kDefaultStepToRotorRatio),
      speed_deadband_rpm_(kDefaultSpeedDeadbandRpm),
      max_control_dt_sec_(kMaxControlDtSec),
      i_start_min_(kDefaultStartCurrent),
      i_max_assist_(kDefaultMaxAssistCurrent),
      ecd_(0U),
      speed_rpm_raw_(0),
      given_current_(0),
      step_dir_(0U),
      step_speed_rpm_(0.0f),
      target_speed_rpm_(0.0f),
      integral_current_(0.0f),
      last_error_(0.0f),
      last_control_tick_ms_(0U),
      current_cmd_(0),
      descend_active_(0U),
      has_last_error_(0U)
{
}

void M2006FollowerMotor::Init()
{
    speed_rpm_raw_ = 0;
    given_current_ = 0;
    ecd_ = 0U;
    ResetControl();
}

void M2006FollowerMotor::ResetControl()
{
    target_speed_rpm_ = 0.0f;
    integral_current_ = 0.0f;
    last_error_ = 0.0f;
    last_control_tick_ms_ = 0U;
    current_cmd_ = 0;
    descend_active_ = 0U;
    has_last_error_ = 0U;
}

void M2006FollowerMotor::Stop()
{
    ResetControl();
    SendCurrent(0);
}

void M2006FollowerMotor::UpdateFeedback(const uint8_t data[8], uint32_t now_ms)
{
    (void)now_ms;

    if (data == nullptr)
    {
        return;
    }

    // DJI 电机反馈帧格式：编码器值、转速、电流均为高字节在前的 16 位数据。
    ecd_ = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    speed_rpm_raw_ = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
    given_current_ = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
}

void M2006FollowerMotor::SetStepReference(uint8_t step_dir, float step_speed_rpm)
{
    step_dir_ = step_dir;
    step_speed_rpm_ = step_speed_rpm;
}

void M2006FollowerMotor::ControlTick(uint32_t now_ms)
{
    float measured_rpm = 0.0f;
    float error = 0.0f;
    float dt_s = 0.0f;
    float derivative = 0.0f;
    float output = 0.0f;
    float kp_value = 0.0f;
    float ki_value = 0.0f;
    float kd_value = 0.0f;
    int16_t current_abs = 0;

    UpdateTargetSpeed();
    if (descend_active_ == 0U)
    {
        // 只在步进电机执行下降动作时随动助力，其他方向立即撤掉电流。
        Stop();
        return;
    }

    measured_rpm = GetMeasuredSpeedRpmInternal();
    error = target_speed_rpm_ - measured_rpm;
    if (error > -speed_deadband_rpm_ && error < speed_deadband_rpm_)
    {
        error = 0.0f;
    }

    if (has_last_error_ != 0U)
    {
        dt_s = static_cast<float>(now_ms - last_control_tick_ms_) * 0.001f;
        dt_s = ClampFloat(dt_s, 0.0f, max_control_dt_sec_);
        if (dt_s > 0.0f)
        {
            derivative = (error - last_error_) / dt_s;
        }
    }

    kp_value = (kp > 0.0f) ? kp : 0.0f;
    ki_value = (ki > 0.0f) ? ki : 0.0f;
    kd_value = (kd > 0.0f) ? kd : 0.0f;

    if (ki_value > 0.0f)
    {
        integral_current_ += ki_value * error * dt_s;
    }
    else
    {
        integral_current_ = 0.0f;
    }
    integral_current_ = ClampFloat(integral_current_, 0.0f, static_cast<float>(i_max_assist_));

    // 单向速度 PID：正误差增加下降助力，负误差只撤掉助力。
    output = kp_value * error + integral_current_ + kd_value * derivative;
    output = ClampFloat(output, 0.0f, static_cast<float>(i_max_assist_));

    current_abs = static_cast<int16_t>(output + 0.5f);
    if (current_abs > 0 && current_abs < i_start_min_)
    {
        // 小电流不足以克服静摩擦时，抬到起动电流，保证下降随动能跟上。
        current_abs = i_start_min_;
    }

    last_error_ = error;
    last_control_tick_ms_ = now_ms;
    has_last_error_ = 1U;

    current_cmd_ = static_cast<int16_t>(cmd_sign_ * current_abs);
    SendCurrent(current_cmd_);
}

int16_t M2006FollowerMotor::GetCurrentCommand() const
{
    return current_cmd_;
}

int16_t M2006FollowerMotor::GetSpeedRpmRaw() const
{
    return speed_rpm_raw_;
}

float M2006FollowerMotor::GetMeasuredSpeedRpm() const
{
    return GetMeasuredSpeedRpmInternal();
}

float M2006FollowerMotor::GetTargetSpeedRpm() const
{
    return target_speed_rpm_;
}

bool M2006FollowerMotor::IsDescendingActive() const
{
    return descend_active_ != 0U;
}

void M2006FollowerMotor::SetCommandSign(int8_t sign)
{
    if (sign == 1 || sign == -1)
    {
        cmd_sign_ = sign;
    }
}

void M2006FollowerMotor::SetFeedbackSign(int8_t sign)
{
    if (sign == 1 || sign == -1)
    {
        feedback_sign_ = sign;
    }
}

void M2006FollowerMotor::SetDescendDirection(uint8_t dir)
{
    step_descend_dir_ = dir;
}

void M2006FollowerMotor::SetPidGains(float kp, float ki, float kd)
{
    if (kp >= 0.0f && ki >= 0.0f && kd >= 0.0f)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }
}

void M2006FollowerMotor::UpdateTargetSpeed(void)
{
    if (step_dir_ == step_descend_dir_ && step_speed_rpm_ > 0.0f)
    {
        // 步进电机下降速度换算成 M2006 转子侧目标速度，作为随动助力参考。
        descend_active_ = 1U;
        target_speed_rpm_ = step_to_rotor_ratio_ * step_speed_rpm_;
    }
    else
    {
        descend_active_ = 0U;
        target_speed_rpm_ = 0.0f;
    }
}

void M2006FollowerMotor::SendCurrent(int16_t current)
{
    /* 电流由 FTM_PatchDjiCurrentCommand 注入到升降任务的 0x1FF 帧中发送，
       此函数本身不需要操作 CAN 硬件。 */
    (void)current;

    if (hfdcan_ == nullptr || tx_slot_ >= kMotorCountPerFrame)
    {
        return;
    }
}

float M2006FollowerMotor::GetMeasuredSpeedRpmInternal(void) const
{
    return static_cast<float>(feedback_sign_) * static_cast<float>(speed_rpm_raw_);
}
