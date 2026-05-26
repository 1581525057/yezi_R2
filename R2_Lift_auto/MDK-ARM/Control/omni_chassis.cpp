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
    float legacy_vx = 0.0f;
    float legacy_vy = 0.0f;
    OmniChassis::currentToLegacy(target.Vx, target.Vy, &legacy_vx, &legacy_vy);

    target.rpm[0] = -(-a * legacy_vx + a * legacy_vy - target.Vz * b) * 60.0f / wheel_circumference;
    target.rpm[1] = -(-a * legacy_vx - a * legacy_vy - target.Vz * b) * 60.0f / wheel_circumference;
    target.rpm[2] = -(a * legacy_vx - a * legacy_vy - target.Vz * b) * 60.0f / wheel_circumference;
    target.rpm[3] = -(a * legacy_vx + a * legacy_vy - target.Vz * b) * 60.0f / wheel_circumference;

    for (int i = 0; i < 4; i++) {
        if (target.rpm[i] >= max_rpm) {
            target.rpm[i] = max_rpm;
        } else if (target.rpm[i] <= -max_rpm) {
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

    OmniChassis::legacyToCurrent(Vx_body, Vy_body, &now.Vx, &now.Vy);
}

// 动力学逆解算
void OmniChassis::dynamicsInverse(float Fx, float Fy, float T)
{
    float motor_out[4];
    float k         = 1.41421356f / 4.0f;
    float legacy_fx = 0.0f;
    float legacy_fy = 0.0f;
    OmniChassis::currentToLegacy(Fx, Fy, &legacy_fx, &legacy_fy);

    motor_out[0] = -(-k * legacy_fx + k * legacy_fy - T / (4.0f * b)) * wheel_r;
    motor_out[1] = -(-k * legacy_fx - k * legacy_fy - T / (4.0f * b)) * wheel_r;
    motor_out[2] = -(k * legacy_fx - k * legacy_fy - T / (4.0f * b)) * wheel_r;
    motor_out[3] = -(k * legacy_fx + k * legacy_fy - T / (4.0f * b)) * wheel_r;

    for (uint8_t i = 0; i < 4; i++) {
        feedforward_current[i] = torqueToCurrent(motor_out[i]);
    }
}

// 轮上扭矩 -> VESC电流指令，单位 mA
int32_t OmniChassis::torqueToCurrent(float wheel_T)
{
    // 20A 对应的最大轮上扭矩
    float max_wheel_torque = WHEEL_KT_NM_PER_A * VESC_MAX_CURRENT_A;

    if (wheel_T > max_wheel_torque)
        wheel_T = max_wheel_torque;
    if (wheel_T < -max_wheel_torque)
        wheel_T = -max_wheel_torque;

    // wheel_T = Kt_motor * I * 减速比 * 效率
    float current_A = wheel_T / WHEEL_KT_NM_PER_A;

    int32_t current_mA = (int32_t)(current_A * 1000.0f);

    if (current_mA > VESC_MAX_CURRENT_MA)
        current_mA = VESC_MAX_CURRENT_MA;
    if (current_mA < -VESC_MAX_CURRENT_MA)
        current_mA = -VESC_MAX_CURRENT_MA;

    return current_mA;
}

// VESC电流指令 mA -> 轮上扭矩
float OmniChassis::currentToTorque(int32_t current_mA)
{
    if (current_mA > VESC_MAX_CURRENT_MA)
        current_mA = VESC_MAX_CURRENT_MA;
    if (current_mA < -VESC_MAX_CURRENT_MA)
        current_mA = -VESC_MAX_CURRENT_MA;

    float current_A = (float)current_mA / 1000.0f;

    // 轮上扭矩
    float wheel_T = WHEEL_KT_NM_PER_A * current_A;

    return wheel_T;
}