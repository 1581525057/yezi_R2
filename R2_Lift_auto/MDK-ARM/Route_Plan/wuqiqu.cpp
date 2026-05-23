#include "wuqiqu.h"
#include <math.h>

namespace
{
    static const float WUQIQU_PI = 3.14159265358979323846f;
    static const float kDegToRad = WUQIQU_PI / 180.0f;

    constexpr float kVisionYToPlannerX = -1.0f;
    constexpr float kVisionXToPlannerY =  1.0f;

    // 目标点列表（视觉坐标系：x_diff期望, y_diff期望, angle_x期望）
    static const WuqiquPathPlanner::TargetPoint kWaypoints[] = {
        { 0.77f, 0.62f, 0.0f },
    };
    static const uint8_t kWaypointCount = sizeof(kWaypoints) / sizeof(kWaypoints[0]);
}

WuqiquPathPlanner wuqiqu;

WuqiquPathPlanner::WuqiquPathPlanner()
{
    reset();

    pos_kp_ = 5.0f;
    pos_kd_ = 0.0f;
    yaw_kp_ = 0.5f;
    yaw_kd_ = 0.0f;
    pos_tolerance_m_ = 0.010f;
    yaw_tolerance_deg_ = 1.0f;
    stable_count_ = 80U;
    max_angular_speed_radps_ = 0.8f;
    moving_yaw_max_radps_ = 0.3f;

    waypoint_count_ = kWaypointCount;
    if (waypoint_count_ > MAX_WAYPOINTS)
        waypoint_count_ = MAX_WAYPOINTS;

    for (uint8_t i = 0U; i < waypoint_count_; ++i)
        waypoints_[i] = kWaypoints[i];

    current_index_ = 0U;
    loadCurrentWaypoint();
}

void WuqiquPathPlanner::loadCurrentWaypoint(void)
{
    if (current_index_ < waypoint_count_)
    {
        const TargetPoint &wp = waypoints_[current_index_];
        target_.x_m = kVisionYToPlannerX * wp.y_m;
        target_.y_m = kVisionXToPlannerY * wp.x_m;
        target_.yaw_deg = wp.yaw_deg;
    }
}

void WuqiquPathPlanner::setParams(float pos_kp, float pos_kd,
                                   float yaw_kp, float yaw_kd,
                                   float pos_tolerance_m, float yaw_tolerance_deg,
                                   uint16_t stable_count,
                                   float max_angular_speed_radps,
                                   float moving_yaw_max_radps)
{
    pos_kp_ = pos_kp;
    pos_kd_ = pos_kd;
    yaw_kp_ = yaw_kp;
    yaw_kd_ = yaw_kd;
    pos_tolerance_m_ = pos_tolerance_m;
    yaw_tolerance_deg_ = yaw_tolerance_deg;
    stable_count_ = stable_count;
    max_angular_speed_radps_ = max_angular_speed_radps;
    moving_yaw_max_radps_ = moving_yaw_max_radps;
}

void WuqiquPathPlanner::reset(void)
{
    state_ = STATE_IDLE;
    on_target_flag_ = 0U;
    xy_stable_count_ = 0U;
    theta_stable_count_ = 0U;

    last_err_x_ = 0.0f;
    last_err_y_ = 0.0f;
    last_err_yaw_rad_ = 0.0f;

    output_.world_vx_mps = 0.0f;
    output_.world_vy_mps = 0.0f;
    output_.wz_radps = 0.0f;
}

void WuqiquPathPlanner::resetRoute(void)
{
    current_index_ = 0U;
    loadCurrentWaypoint();
    reset();
}

void WuqiquPathPlanner::advanceToNext(void)
{
    ++current_index_;
    if (current_index_ < waypoint_count_)
    {
        loadCurrentWaypoint();
        reset();
    }
}

bool WuqiquPathPlanner::isAllFinished(void) const
{
    return (current_index_ >= waypoint_count_);
}

uint8_t WuqiquPathPlanner::getCurrentIndex(void) const
{
    return current_index_;
}

uint8_t WuqiquPathPlanner::getWaypointCount(void) const
{
    return waypoint_count_;
}

int WuqiquPathPlanner::follow(const Pose &current_pose)
{
    const float err_x_m = target_.x_m - current_pose.x;
    const float err_y_m = target_.y_m - current_pose.y;
    const float distance_m = safeSqrt(err_x_m * err_x_m + err_y_m * err_y_m);
    const uint8_t xy_reached = (distance_m < pos_tolerance_m_) ? 1U : 0U;

    const float err_yaw_deg = normalizeAngleDeg(target_.yaw_deg - current_pose.yaw_360);
    const float err_yaw_rad = err_yaw_deg * kDegToRad;

    if (state_ == STATE_IDLE)
    {
        state_ = STATE_MOVING;
        last_err_x_ = err_x_m;
        last_err_y_ = err_y_m;
        last_err_yaw_rad_ = err_yaw_rad;
    }

    switch (state_)
    {
    case STATE_IDLE:
        output_.world_vx_mps = 0.0f;
        output_.world_vy_mps = 0.0f;
        output_.wz_radps = 0.0f;
        break;

    case STATE_MOVING:
        if (xy_reached != 0U)
        {
            ++xy_stable_count_;
            output_.world_vx_mps = 0.0f;
            output_.world_vy_mps = 0.0f;

            if (xy_stable_count_ >= stable_count_)
            {
                on_target_flag_ = 1U;
                xy_stable_count_ = 0U;
                last_err_yaw_rad_ = err_yaw_rad;
                state_ = STATE_YAW_CORRECTING;
            }
        }
        else
        {
            xy_stable_count_ = 0U;

            float world_vx_mps = pos_kp_ * err_x_m + pos_kd_ * (err_x_m - last_err_x_);
            float world_vy_mps = pos_kp_ * err_y_m + pos_kd_ * (err_y_m - last_err_y_);

            last_err_x_ = err_x_m;
            last_err_y_ = err_y_m;

            output_.world_vx_mps = world_vx_mps;
            output_.world_vy_mps = world_vy_mps;
        }

        {
            float wz_cmd = -(yaw_kp_ * err_yaw_rad + yaw_kd_ * (err_yaw_rad - last_err_yaw_rad_));
            last_err_yaw_rad_ = err_yaw_rad;

            if (wz_cmd > moving_yaw_max_radps_)
                wz_cmd = moving_yaw_max_radps_;
            else if (wz_cmd < -moving_yaw_max_radps_)
                wz_cmd = -moving_yaw_max_radps_;

            output_.wz_radps = wz_cmd;
        }
        break;

    case STATE_YAW_CORRECTING:
        if (fabsf(err_yaw_deg) < yaw_tolerance_deg_)
        {
            output_.world_vx_mps = 0.0f;
            output_.world_vy_mps = 0.0f;
            output_.wz_radps = 0.0f;

            ++theta_stable_count_;
            if (theta_stable_count_ >= stable_count_)
            {
                reset();
                state_ = STATE_FINISHED;
                return 1;
            }
        }
        else
        {
            theta_stable_count_ = 0U;

            output_.wz_radps = -(yaw_kp_ * err_yaw_rad + yaw_kd_ * (err_yaw_rad - last_err_yaw_rad_));
            last_err_yaw_rad_ = err_yaw_rad;

            if (fabsf(output_.wz_radps) > max_angular_speed_radps_)
            {
                output_.wz_radps = (output_.wz_radps > 0.0f) ? max_angular_speed_radps_ : -max_angular_speed_radps_;
            }
        }

        output_.world_vx_mps = 0.0f;
        output_.world_vy_mps = 0.0f;
        break;

    case STATE_FINISHED:
    case STATE_DEVIATED:
        output_.world_vx_mps = 0.0f;
        output_.world_vy_mps = 0.0f;
        output_.wz_radps = 0.0f;
        break;
    }

    return 0;
}

const WuqiquPathPlanner::Output &WuqiquPathPlanner::getOutput(void) const
{
    return output_;
}

WuqiquPathPlanner::PlannerState WuqiquPathPlanner::getState(void) const
{
    return state_;
}

bool WuqiquPathPlanner::isFinished(void) const
{
    return (state_ == STATE_FINISHED);
}

float WuqiquPathPlanner::normalizeAngleDeg(float angle) const
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

float WuqiquPathPlanner::safeSqrt(float value) const
{
    if (value <= 0.0f)
        return 0.0f;
    return sqrtf(value);
}
