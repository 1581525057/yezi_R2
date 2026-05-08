#ifndef WUQIQU_H
#define WUQIQU_H

#include <stdint.h>

class WuqiquPathPlanner
{
public:
    enum PlannerState
    {
        STATE_IDLE = 0,
        STATE_MOVING,          // 移动中
        STATE_YAW_CORRECTING,  // 角度修正中
        STATE_FINISHED,
        STATE_DEVIATED
    };

    // 目标点结构（规划X/Y对齐底盘Vx/Vy）
    class TargetPoint
    {
    public:
        float x_m;       // 规划X / 底盘+Vx方向坐标，单位m
        float y_m;       // 规划Y / 底盘+Vy方向坐标，单位m
        float yaw_deg;   // 目标航向角，单位度
    };

    // 当前位姿结构
    class Pose
    {
    public:
        float x;          // 规划X / 底盘+Vx方向坐标，单位m
        float y;          // 规划Y / 底盘+Vy方向坐标，单位m
        float yaw;        // 航向角，单位rad
        float yaw_360;    // 航向角，单位度
        float car_speed_x;  // 车体系速度X，单位m/s
        float car_speed_y;  // 车体系速度Y，单位m/s
        float world_speed_x; // 规划X / 底盘Vx方向速度，单位m/s
        float world_speed_y; // 规划Y / 底盘Vy方向速度，单位m/s
        float omega;      // 角速度，单位rad/s
    };

    // 输出结构（规划X/Y速度）
    class Output
    {
    public:
        float world_vx_mps; // 规划X / 底盘Vx方向速度，单位m/s
        float world_vy_mps; // 规划Y / 底盘Vy方向速度，单位m/s
        float wz_radps;     // 角速度，单位rad/s
    };

    WuqiquPathPlanner();

    // 设置目标点（规划X/Y对齐底盘Vx/Vy，单位m, m, 度）
    void setTarget(float x_m, float y_m, float yaw_deg);

    // 执行导航（每周期调用）
    int follow(const Pose &current_pose);

    // 复位
    void reset(void);

    // 获取输出
    const Output &getOutput(void) const;

    // 获取状态
    PlannerState getState(void) const;
    bool isFinished(void) const;

private:
    TargetPoint target_;       // 目标点
    Output output_;            // 输出速度
    PlannerState state_;       // 当前状态

    // PD控制历史误差
    float last_err_x_;
    float last_err_y_;
    float last_err_yaw_rad_;

    // 到达判定
    uint8_t on_target_flag_;
    uint16_t xy_stable_count_;
    uint16_t theta_stable_count_;

    // 工具函数
    float normalizeAngleDeg(float angle) const;
    float safeSqrt(float value) const;
};

extern WuqiquPathPlanner wuqiqu;

#endif
