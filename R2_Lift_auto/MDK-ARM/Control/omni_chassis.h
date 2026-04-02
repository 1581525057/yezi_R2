#ifndef __OMNI_CHASSIS_H__
#define __OMNI_CHASSIS_H__

#include <stdint.h>
#include <math.h>
#include "arm_math.h" // CMSIS-DSP

#define wheel_r             (0.15252f / 2.0f)
#define wheel_circumference (wheel_r * 2 * PI)

typedef struct
{
    /* data */
    float Vx;
    float Vy;
    float Vz;
    float rpm[4];
} Speed_t;

class OmniChassis
{

public:
    OmniChassis(/* args */)
    {
        memset(&target, 0, sizeof(target));
        memset(&now, 0, sizeof(now));
        memset(Current_rpm, 0, sizeof(Current_rpm));
        flag = 0;
    }

    // 设置控制输入
    void setRemote(float Vx, float Vy, float Vz);

    // 全向轮逆解算
    void inverseKinematics();

    // 全向轮正解算
    void forwardKinematics();

    // 动力学逆解算（Fx, Fy, T -> 各轮电流）
    void dynamicsInverse(float Fx, float Fy, float T);

    // 工具函数
    float torqueToCurrent(float T);
    float currentToTorque(int current);

public:
    Speed_t target; // 目标速度
    Speed_t now;    // 当前速度

    float Current_rpm[4];
    uint8_t flag; // 0: 普通模式  1: 无头模式

private:
    static constexpr float a       = 0.7071f;
    static constexpr float b       = 0.3684f;
    static constexpr float max_rpm = 450.0f;
};

extern OmniChassis omni_chassis;

#endif
