#include "M2006AngleMotor.h"
#include "main.h"
#include <math.h>

namespace
{
constexpr uint16_t kEncoderMax = 8192U;
constexpr float kReductionRatio = 36.0f;
constexpr float kEncoderToOutputDegree = 360.0f / (static_cast<float>(kEncoderMax) * kReductionRatio);
constexpr float kPositionKp = 35.0f;
constexpr float kPositionKi = 0.0f;
constexpr float kPositionKd = 2.0f;
constexpr float kSpeedKp = 25.0f;
constexpr float kSpeedKi = 0.16f;
constexpr float kSpeedKd = 0.0f;
constexpr float kMaxSpeedDps = 720.0f;
constexpr float kMaxCurrent = 8000.0f;
constexpr float kIntegralLimit = 1000.0f;

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

float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

float PidStep(float error,
              float &integral,
              float &last_error,
              float kp,
              float ki,
              float kd,
              float dt_s,
              float integral_limit,
              float output_limit)
{
    float derivative = 0.0f;

    integral += error * ki * dt_s;
    integral = ClampFloat(integral, -integral_limit, integral_limit);

    if (dt_s > 0.0f)
    {
        derivative = (error - last_error) / dt_s;
    }

    last_error = error;
    return ClampFloat(kp * error + integral + kd * derivative,
                      -output_limit,
                      output_limit);
}
}

M2006AngleContext g_m2006_angle = {
    0U,
    0U,
    0,
    0.0f,
    0.0f,
    kPositionKp,
    kPositionKi,
    kPositionKd,
    kIntegralLimit,
    kMaxSpeedDps,
    kSpeedKp,
    kSpeedKi,
    kSpeedKd,
    kIntegralLimit,
    kMaxCurrent,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0U,
    0,
    0U
};

void M2006Angle_Init(void)
{
    g_m2006_angle.angle_degree = 0.0f;
    g_m2006_angle.target_angle_degree = 0.0f;
    g_m2006_angle.position_integral = 0.0f;
    g_m2006_angle.speed_integral = 0.0f;
    g_m2006_angle.last_position_error = 0.0f;
    g_m2006_angle.last_speed_error = 0.0f;
    g_m2006_angle.last_tick = HAL_GetTick();
    g_m2006_angle.current_cmd = 0;
}

void M2006Angle_UpdateFeedback(const uint8_t data[8])
{
    if (data == 0)
    {
        return;
    }

    const uint16_t ecd = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    const int16_t speed_rpm = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);

    if (g_m2006_angle.initialized == 0U)
    {
        g_m2006_angle.last_ecd = ecd;
        g_m2006_angle.ecd = ecd;
        g_m2006_angle.angle_degree = 0.0f;
        g_m2006_angle.initialized = 1U;
        g_m2006_angle.speed_rpm = speed_rpm;
        return;
    }

    int32_t diff = static_cast<int32_t>(ecd) - static_cast<int32_t>(g_m2006_angle.last_ecd);
    if (diff > static_cast<int32_t>(kEncoderMax / 2U))
    {
        diff -= kEncoderMax;
    }
    else if (diff < -static_cast<int32_t>(kEncoderMax / 2U))
    {
        diff += kEncoderMax;
    }

    g_m2006_angle.angle_degree += static_cast<float>(diff) * kEncoderToOutputDegree;
    g_m2006_angle.last_ecd = ecd;
    g_m2006_angle.ecd = ecd;
    g_m2006_angle.speed_rpm = speed_rpm;
}

void M2006Angle_SetTarget(float target_angle_degree)
{
    g_m2006_angle.target_angle_degree = target_angle_degree;
}

void M2006Angle_ControlTick(void)
{
    const uint32_t now = HAL_GetTick();
    float dt_s = static_cast<float>(now - g_m2006_angle.last_tick) * 0.001f;
    dt_s = ClampFloat(dt_s, 0.001f, 0.05f);
    g_m2006_angle.last_tick = now;

    const float position_error = g_m2006_angle.target_angle_degree - g_m2006_angle.angle_degree;
    const float target_speed_dps = PidStep(position_error,
                                           g_m2006_angle.position_integral,
                                           g_m2006_angle.last_position_error,
                                           g_m2006_angle.angle_kp,
                                           g_m2006_angle.angle_ki,
                                           g_m2006_angle.angle_kd,
                                           dt_s,
                                           g_m2006_angle.angle_integral_limit,
                                           g_m2006_angle.angle_speed_limit_dps);

    const float current_speed_dps = static_cast<float>(g_m2006_angle.speed_rpm) * 360.0f / 60.0f / kReductionRatio;
    const float speed_error = target_speed_dps - current_speed_dps;
    const float current = PidStep(speed_error,
                                  g_m2006_angle.speed_integral,
                                  g_m2006_angle.last_speed_error,
                                  g_m2006_angle.speed_kp,
                                  g_m2006_angle.speed_ki,
                                  g_m2006_angle.speed_kd,
                                  dt_s,
                                  g_m2006_angle.speed_integral_limit,
                                  g_m2006_angle.current_limit);

    g_m2006_angle.current_cmd = static_cast<int16_t>(current);
}

void M2006Angle_Stop(void)
{
    g_m2006_angle.target_angle_degree = g_m2006_angle.angle_degree;
    g_m2006_angle.position_integral = 0.0f;
    g_m2006_angle.speed_integral = 0.0f;
    g_m2006_angle.last_position_error = 0.0f;
    g_m2006_angle.last_speed_error = 0.0f;
    g_m2006_angle.current_cmd = 0;
}

uint8_t M2006Angle_IsAtTarget(float tolerance_degree)
{
    return (AbsFloat(g_m2006_angle.target_angle_degree - g_m2006_angle.angle_degree) <= tolerance_degree) ? 1U : 0U;
}

int16_t M2006Angle_GetCurrentCommand(void)
{
    return g_m2006_angle.current_cmd;
}

float M2006Angle_GetAngleDegree(void)
{
    return g_m2006_angle.angle_degree;
}
