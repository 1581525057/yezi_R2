#ifndef REOLCATION_H
#define REOLCATION_H

#include <stdint.h>
#include "main.h"

#define SENSOR_FRONT 0x01U
#define SENSOR_LEFT 0x02U
#define SENSOR_RIGHT 0x04U
#define SENSOR_ALL 0x07U

class AreaOneRelocation
{
public:
    static const uint8_t WAITING = 0U; // 还未满足连续稳定上传条件，或已上传但还在等待小电脑确认。
    static const uint8_t SENT = 1U;    // 已上传本次重定位坐标，且小电脑确认可以继续。

    // 清空一区重定位状态，重新累计稳定周期。
    void reset(void);

    // 执行一区重定位计算，按 sensor_mask 选择左/右/双侧激光，且底盘速度为零后才累计稳定周期。
    uint8_t update(uint8_t sensor_mask, uint8_t chassis_speed_zero);

    // 嵌入式调试时直接查看这些状态量，不再额外包 getter。
    uint16_t stable_count_ = 0U;
    uint8_t position_sent_ = 0U;
    float last_x_m_ = 0.0f;
    float last_y_m_ = 0.0f;
};

class AreaThreeRelocation
{
public:
    static const uint8_t WAITING = 0U; // 还未满足连续稳定上传条件。
    static const uint8_t SENT = 1U;    // 已上传本次重定位坐标，可以继续后续流程。

    // 清空三区重定位状态，重新累计稳定周期。
    void reset(void);

    // 执行三区重定位计算，使用右侧两个平行激光计算 Y 和 yaw，X 保持当前视觉值不校准。
    uint8_t update(uint8_t chassis_speed_zero);

    // 嵌入式调试时直接查看这些状态量，不再额外包 getter。
    uint16_t stable_count_ = 0U;
    uint8_t position_sent_ = 0U;
    float last_x_m_ = 0.0f;
    float last_y_m_ = 0.0f;
    float last_yaw_rad_ = 0.0f;
    float last_yaw_deg_ = 0.0f;
};

extern AreaOneRelocation area_one_relocation;
extern AreaThreeRelocation area_three_relocation;
extern float abs_limit(float x, float min, float max);

#endif
