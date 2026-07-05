/*
 * 重定位模块实现：
 * 一区根据前方 DT35、左右激光测距计算雷达坐标，稳定后上传给上位机。
 */

#include "reolcation.h"
#include "DT35.h"
#include "laser_distance.h"
#include "usart_task.h"

namespace
{
    const float AREA_ONE_X_FIELD_MM = 3200.0f;       // 一区 X 方向场地基准长度，单位 mm。
    const float AREA_ONE_X_FIELD_DELATE_MM = 385.5f; // 一区X方向场地原点减去
    const float AREA_ONE_Y_FIELD_MM = 6000.0f;       // 左右激光反推时使用的场地宽度，单位 mm。
    const float AREA_ONE_ORIGIN_L_MM = 1357.0f;      // 雷达 Y 坐标原点偏移量 L，单位 mm。
    const float FRONT_DT35_TO_CENTER_MM = 308.18f;   // 前方 DT35 到车中心的安装距离，单位 mm。
    const float SIDE_LASER_TO_CENTER_MM = 309.23f;   // 左右激光到车中心的安装距离，单位 mm。
    const float SIDE_LASER_TO_WALL = 125.0f;         // 墙的距离 单位mm。
    const float LEFT_LASER_Y_COMPENSATION_MM = 50.0f; // 左侧单激光重定位后 Y 偏 -5cm，补回 50mm。
    const float SIDE_Y_ERROR_LIMIT_MM = 10.0f;       // 左右两侧推算出的 Y 误差阈值，单位 mm。
    const uint16_t RELOCATION_STABLE_COUNT = 100U;   // 误差连续满足要求的周期数。

    // 本文件只需要简单绝对值，避免额外依赖通用数学库。
    float calcAbs(float value)
    {
        return (value < 0.0f) ? -value : value;
    }
}

AreaOneRelocation area_one_relocation;
AreaThreeRelocation area_three_relocation;

// 重新开始一区重定位前，清空稳定计数和上一次计算结果。
void AreaOneRelocation::reset(void)
{
    stable_count_ = 0U;
    position_sent_ = 0U;
    last_x_m_ = 0.0f;
    last_y_m_ = 0.0f;
}

/*
 * 一区重定位主流程：
 * 1. 读取前方 DT35 和 sensor_mask 选中的侧向激光数据；
 * 2. 前方距离换算雷达 X；
 * 3. 单侧模式直接换算雷达 Y，双侧模式先校验两份 Y 的误差；
 * 4. 底盘速度为 0 且条件连续满足 100 个周期后，上传雷达坐标。
 */
uint8_t AreaOneRelocation::update(uint8_t sensor_mask, uint8_t chassis_speed_zero)
{
    const uint8_t side_mask = sensor_mask & (SENSOR_LEFT | SENSOR_RIGHT);
    const uint8_t use_left = ((side_mask & SENSOR_LEFT) != 0U) ? 1U : 0U;
    const uint8_t use_right = ((side_mask & SENSOR_RIGHT) != 0U) ? 1U : 0U;

    if (chassis_speed_zero == 0U ||
        dt35.ch2.valid == 0U ||
        (use_left == 0U && use_right == 0U) ||
        (use_left != 0U && laser_left.data.valid == 0U) ||
        (use_right != 0U && laser_right.data.valid == 0U))
    {
        stable_count_ = 0U;
        position_sent_ = 0U;
        return WAITING;
    }

    const float front_mm = dt35.ch2.distance_mm;

    // X 使用前方 DT35 加安装偏移后，再由场地基准反算雷达坐标。
    const float x_mm = AREA_ONE_X_FIELD_MM - AREA_ONE_X_FIELD_DELATE_MM - (front_mm + FRONT_DT35_TO_CENTER_MM);

    float y_mm = 0.0f;
    uint8_t y_ready = 1U;
    if (use_left != 0U && use_right != 0U)
    {
        const float left_mm = static_cast<float>(laser_left.data.distance_mm);
        const float right_mm = static_cast<float>(laser_right.data.distance_mm);
        const float y_left_mm = AREA_ONE_ORIGIN_L_MM - (left_mm + SIDE_LASER_TO_CENTER_MM);
        const float left_from_right_mm = AREA_ONE_Y_FIELD_MM - (right_mm + SIDE_LASER_TO_CENTER_MM);
        const float y_right_mm = AREA_ONE_ORIGIN_L_MM - left_from_right_mm;

        y_mm = y_left_mm;
        y_ready = (calcAbs(y_left_mm - y_right_mm) < SIDE_Y_ERROR_LIMIT_MM) ? 1U : 0U;
    }
    else if (use_left != 0U)
    {
        const float left_mm = static_cast<float>(laser_left.data.distance_mm);
        y_mm = AREA_ONE_ORIGIN_L_MM - (left_mm + SIDE_LASER_TO_CENTER_MM + SIDE_LASER_TO_WALL) + LEFT_LASER_Y_COMPENSATION_MM;
    }
    else
    {
        const float right_mm = static_cast<float>(laser_right.data.distance_mm);
        const float left_from_right_mm = AREA_ONE_Y_FIELD_MM - (right_mm + SIDE_LASER_TO_CENTER_MM);
        y_mm = AREA_ONE_ORIGIN_L_MM - left_from_right_mm;
    }

    // 上位机接口使用米，这里保存最近一次算出的雷达坐标。
    last_x_m_ = x_mm * 0.001f;
    last_y_m_ = y_mm * 0.001f;

    if (y_ready != 0U)
    {
        if (stable_count_ < RELOCATION_STABLE_COUNT)
        {
            stable_count_++;
        }

        if (stable_count_ >= RELOCATION_STABLE_COUNT)
        {
            if (position_sent_ == 0U)
            {
                // 只在首次稳定达标时上传一次，避免每个周期重复校准雷达。
                send_position_to_pc(1, 1, last_x_m_, last_y_m_, 0.0f);
                position_sent_ = 1U;
            }
            return SENT;
        }
    }
    else
    {
        stable_count_ = 0U;
        position_sent_ = 0U;
    }

    return WAITING;
}

// 绝对值限幅工具：小于最小值时补到最小值，大于最大值时钳到最大值。
float abs_limit(float x, float min, float max)
{
    float abs_min = calcAbs(min);
    float abs_max = calcAbs(max);

    if (abs_max < abs_min)
    {
        const float temp = abs_max;
        abs_max = abs_min;
        abs_min = temp;
    }

    if (x > abs_max)
    {
        return abs_max;
    }
    if (x < -abs_max)
    {
        return -abs_max;
    }
    if (x > 0.0f && x < abs_min)
    {
        return abs_min;
    }
    if (x < 0.0f && x > -abs_min)
    {
        return -abs_min;
    }

    return x;
}
