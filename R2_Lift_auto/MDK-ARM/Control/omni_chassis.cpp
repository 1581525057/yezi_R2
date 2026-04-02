#include "omni_chassis.h"
#include "arm_math.h" 
#include <string.h> // memset, memcpy

/*
右上角 135° 0
左上角 -135° 1
左下角 -45° 2
右下角 45° 3
*/

OmniChassis omni_chassis;


// 设置遥控输入
void OmniChassis::setRemote(float Vx, float Vy, float Vz)
{
    target.Vx = Vx;
    target.Vy = Vy;
    target.Vz = Vz;
}

// 全向轮逆解算
void OmniChassis::inverseKinematics()
{
    if (flag == 0)
    {
        target.rpm[0] = -(-a * target.Vx + a * target.Vy - target.Vz * b) * 60.0f / wheel_circumference;
        target.rpm[1] = -(-a * target.Vx - a * target.Vy - target.Vz * b) * 60.0f / wheel_circumference;
        target.rpm[2] = -(a * target.Vx - a * target.Vy - target.Vz * b) * 60.0f / wheel_circumference;
        target.rpm[3] = -(a * target.Vx + a * target.Vy - target.Vz * b) * 60.0f / wheel_circumference;
    }
    else if (flag == 1)
    {

    }

    for (int i = 0; i < 4; i++)
    {
        if (target.rpm[i] >= max_rpm)
        {
            target.rpm[i] = max_rpm;
        }
        else if (target.rpm[i] <= -max_rpm)
        {
            target.rpm[i] = -max_rpm;
        }
    }
}

// 全向轮正解算
void OmniChassis::forwardKinematics()
{
    float C = wheel_circumference;

    float u0 = now.rpm[0] * C / 60.0f;
    float u1 = now.rpm[1] * C / 60.0f;
    float u2 = now.rpm[2] * C / 60.0f;
    float u3 = now.rpm[3] * C / 60.0f;

    // 先算车体系速度
    float Vx_body = (u0 + u1 - u2 - u3) / (4.0f * a);
    float Vy_body = (u1 + u2 - u0 - u3) / (4.0f * a);
    float Vz_body = (u0 + u1 + u2 + u3) / (4.0f * b);

    now.Vz = Vz_body;

    if (flag == 1)
    {
    }
    else
    {
        now.Vx = Vx_body;
        now.Vy = Vy_body;
    }
}

// 动力学逆解算
void OmniChassis::dynamicsInverse(float Fx, float Fy, float T)
{
    float motor_out[4];
    float k = 1.41421356f / 4.0f;

    motor_out[0] = -(-k * Fx + k * Fy - T / (4.0f * b)) * wheel_r;
    motor_out[1] = -(-k * Fx - k * Fy - T / (4.0f * b)) * wheel_r;
    motor_out[2] = -(k * Fx - k * Fy - T / (4.0f * b)) * wheel_r;
    motor_out[3] = -(k * Fx + k * Fy - T / (4.0f * b)) * wheel_r;

    for (uint8_t i = 0; i < 4; i++)
    {
        Current_rpm[i] = torqueToCurrent(motor_out[i]);
    }
}

// 扭矩 -> 电流原始值
float OmniChassis::torqueToCurrent(float T)
{
    if (T > 6.0f)
        T = 6.0f;
    if (T < -6.0f)
        T = -6.0f;

    return T * 2730.67f;
}

// 电流原始值 -> 扭矩
float OmniChassis::currentToTorque(int current)
{
    if (current > 16384)
        current = 16384;
    if (current < -16384)
        current = -16384;

    float T_A = (float)current / 16384.0f * 20.0f;
    float T = 0.3f * T_A;
    return T;
}