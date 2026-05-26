#ifndef __OMNI_CHASSIS_H__
#define __OMNI_CHASSIS_H__

#include <stdint.h>
#include <math.h>
#include "arm_math.h" // CMSIS-DSP

#define wheel_r                 (0.149f / 2.0f)
#define wheel_circumference     (wheel_r * 2 * PI)
#define MN705_KV225_KT_NM_PER_A 0.04244f
#define VESC_MAX_CURRENT_A      20.0f
#define VESC_MAX_CURRENT_MA     20000

// 36:82 减速比
#define GEAR_RATIO (82.0f / 36.0f)

// 减速箱效率，先估算 85%
#define GEAR_EFFICIENCY 0.85f

// 轮上等效力矩系数
#define WHEEL_KT_NM_PER_A (MN705_KV225_KT_NM_PER_A * GEAR_RATIO * GEAR_EFFICIENCY)

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
        memset(feedforward_current, 0, sizeof(feedforward_current));
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
    int32_t torqueToCurrent(float T);
    float currentToTorque(int current);

    // 旧坐标系 ↔ 新坐标系转换
    // 旧系: +X=右, +Y=前  →  新系: +X=前, +Y=左
    static inline void legacyToCurrent(float legacy_x, float legacy_y, float *current_x, float *current_y)
    {
        *current_x = legacy_y;
        *current_y = -legacy_x;
    }

    static inline void currentToLegacy(float current_x, float current_y, float *legacy_x, float *legacy_y)
    {
        *legacy_x = -current_y;
        *legacy_y = current_x;
    }

public:
    Speed_t target; // 目标速度
    Speed_t now;    // 当前速度

    float feedforward_current[4];
    uint8_t flag; // 0: 普通模式  1: 无头模式

private:
    static constexpr float a       = 0.7071f;
    static constexpr float b       = 0.29212f;
    static constexpr float max_rpm = 450.0f;
};

extern OmniChassis omni_chassis;

#endif
