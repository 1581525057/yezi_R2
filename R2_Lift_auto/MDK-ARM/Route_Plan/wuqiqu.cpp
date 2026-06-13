#include "wuqiqu.h"
#include "main.h"
#include <math.h>

static const float WUQIQU_PI = 3.14159265358979323846f;
static const float kDegToRad = WUQIQU_PI / 180.0f;
static const float kMinMoveMotorRpm = 20.0f;
static const float kDriveWheelDiameterM = 0.149f;
static const float kDriveGearRatio = 82.0f / 36.0f;
static const float kOmniAxisScale = 0.7071f;
static const float kMinMoveSpeedMps =
    (kMinMoveMotorRpm * kDriveWheelDiameterM * WUQIQU_PI) /
    (60.0f * kDriveGearRatio * kOmniAxisScale);

// 目标点为视觉置零后的绝对坐标，当前约定雷达 X/Y 与车体 X/Y 对齐。
static const WuqiquPathPlanner::TargetPoint kWaypoints[] = {
    {-0.92f, -0.49f, 0.0f, 0.90f, 1.0f, 0.0f},
    {-0.52f, -0.53f, 0.0f, 1.0f, 1.0f, 0.0f},
    {-0.52f, -0.53f, -179.5f, 1.0f, 1.2f, 1.2f},
    {-0.92f, -0.49f, -179.5f, 1.0f, 1.0f, 0.0f},
};
static const uint8_t kWaypointCount = sizeof(kWaypoints) / sizeof(kWaypoints[0]);

WuqiquPathPlanner wuqiqu;

WuqiquPathPlanner::WuqiquPathPlanner()
{
    reset();

    approach_v_max_ = 0.90f;
    slow_v_max_ = 0.55f;
    contact_v_max_ = 0.25f;
    finish_v_max_ = 0.12f;
    min_move_v_ = 0.18f;

    slow_dist_ = 0.16f;
    contact_dist_ = 0.02f;
    finish_dist_ = 0.010f;
    decel_ = 1.50f;

    kp_approach_ = 2.6f;
    kd_approach_ = 0.08f;
    kp_slow_ = 3.0f;
    kd_slow_ = 0.10f;
    kp_contact_ = 2.2f;
    kd_contact_ = 0.06f;

    yaw_sign_ = 1.0f;
    yaw_kp_ = 2.8f;
    min_yaw_wz_ = 0.18f;
    strong_yaw_wz_ = 0.35f;
    strong_yaw_error_deg_ = 6.0f;
    moving_wz_max_ = 0.80f;
    settle_wz_max_ = 0.55f;
    yaw_tolerance_deg_ = 1.0f;

    stable_cycles_ = 120U;
    contact_hold_ms_ = 500U;
    contact_timeout_ms_ = 1500U;

    waypoint_count_ = kWaypointCount;
    if (waypoint_count_ > MAX_WAYPOINTS)
    {
        waypoint_count_ = MAX_WAYPOINTS;
    }

    for (uint8_t i = 0U; i < waypoint_count_; ++i)
    {
        waypoints_[i] = kWaypoints[i];
    }

    current_index_ = 0U;
    loadCurrentWaypoint();
}

void WuqiquPathPlanner::loadCurrentWaypoint(void)
{
    if (current_index_ < waypoint_count_)
    {
        target_ = waypoints_[current_index_];
    }
}

void WuqiquPathPlanner::reset(void)
{
    state_ = STATE_IDLE;
    soft_contact_start_tick_ = 0U;
    soft_contact_stable_count_ = 0U;
    setZeroOutput();
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
    const uint32_t now_tick = HAL_GetTick();
    const float err_x_m = target_.x_m - current_pose.x;
    const float err_y_m = target_.y_m - current_pose.y;
    const float distance_m = safeSqrt(err_x_m * err_x_m + err_y_m * err_y_m);
    const float err_yaw_deg = normalizeAngleDeg(target_.yaw_deg - current_pose.yaw_360);
    const float yaw_control_deg = normalizeAngleDeg(current_pose.yaw_360 - target_.yaw_deg);
    const float yaw_abs_deg = fabsf(err_yaw_deg);

    updateState(distance_m, now_tick);

    if (state_ == STATE_FINISHED)
    {
        setZeroOutput();
        return 1;
    }

    float kp = kp_approach_;
    float kd = kd_approach_;
    float xy_limit = approach_v_max_;
    float wz_limit = moving_wz_max_;

    if (state_ == STATE_SLOW_APPROACH)
    {
        kp = kp_slow_;
        kd = kd_slow_;
        xy_limit = slow_v_max_;
        wz_limit = settle_wz_max_;
    }
    else if (state_ == STATE_SOFT_CONTACT)
    {
        kp = kp_contact_;
        kd = kd_contact_;
        xy_limit = contact_v_max_;
        wz_limit = settle_wz_max_;
    }

    const float yaw_kp_scale = limitFloat(target_.yaw_kp_scale, 0.0f, 3.0f);
    const float yaw_wz_max = (target_.yaw_wz_max > 0.0f) ? target_.yaw_wz_max : wz_limit;
    wz_limit = limitFloat(yaw_wz_max, 0.0f, 2.0f);

    float vx_cmd = kp * err_x_m - kd * current_pose.world_speed_x;
    float vy_cmd = kp * err_y_m - kd * current_pose.world_speed_y;

    const float move_speed_scale = limitFloat(target_.move_speed_scale, 0.0f, 1.0f);
    vx_cmd *= move_speed_scale;
    vy_cmd *= move_speed_scale;

    const float scaled_xy_limit = xy_limit * move_speed_scale;
    const float scaled_min_move_v = min_move_v_ * move_speed_scale;
    const float scaled_finish_v_max = finish_v_max_ * move_speed_scale;
    const float scaled_contact_v_max = contact_v_max_ * move_speed_scale;

    limitVector(vx_cmd, vy_cmd, scaled_xy_limit);

    const float brake_distance_m = (distance_m > finish_dist_) ? (distance_m - finish_dist_) : 0.0f;
    const float brake_v_max = safeSqrt(2.0f * decel_ * brake_distance_m);
    limitVector(vx_cmd, vy_cmd, brake_v_max * move_speed_scale);

    float yaw_xy_scale = 1.0f;
    if (yaw_abs_deg > 30.0f)
    {
        yaw_xy_scale = 0.6f;
    }
    else if (yaw_abs_deg > 15.0f)
    {
        yaw_xy_scale = 0.8f;
    }

    vx_cmd *= yaw_xy_scale;
    vy_cmd *= yaw_xy_scale;

    if (distance_m > finish_dist_ && yaw_abs_deg <= 35.0f)
    {
        raiseVectorToMin(vx_cmd, vy_cmd, scaled_min_move_v);
    }

    if (distance_m <= finish_dist_)
    {
        limitVector(vx_cmd, vy_cmd, scaled_finish_v_max);
    }
    else if (state_ == STATE_SOFT_CONTACT)
    {
        limitVector(vx_cmd, vy_cmd, scaled_contact_v_max);
    }

    float wz_cmd = yaw_sign_ * yaw_kp_ * yaw_kp_scale * yaw_control_deg * kDegToRad;
    wz_cmd = limitFloat(wz_cmd, -wz_limit, wz_limit);
    if (yaw_abs_deg > yaw_tolerance_deg_)
    {
        const float min_wz = (yaw_abs_deg >= strong_yaw_error_deg_) ? strong_yaw_wz_ : min_yaw_wz_;
        const float scaled_min_wz = limitFloat(min_wz * yaw_kp_scale, 0.0f, wz_limit);
        if (fabsf(wz_cmd) < scaled_min_wz)
        {
            wz_cmd = (wz_cmd >= 0.0f) ? scaled_min_wz : -scaled_min_wz;
            wz_cmd = limitFloat(wz_cmd, -wz_limit, wz_limit);
        }
    }

    output_.world_vx_mps = vx_cmd;
    output_.world_vy_mps = vy_cmd;
    output_.wz_radps = wz_cmd;

    if (state_ == STATE_SOFT_CONTACT)
    {
        const uint32_t contact_time_ms = now_tick - soft_contact_start_tick_;
        const uint8_t pose_stable = (distance_m <= finish_dist_ && yaw_abs_deg <= yaw_tolerance_deg_) ? 1U : 0U;

        if (pose_stable != 0U)
        {
            if (soft_contact_stable_count_ < stable_cycles_)
            {
                ++soft_contact_stable_count_;
            }
        }
        else
        {
            soft_contact_stable_count_ = 0U;
        }

        if ((soft_contact_stable_count_ >= stable_cycles_ && contact_time_ms >= contact_hold_ms_) ||
            ((pose_stable != 0U) && contact_time_ms >= contact_timeout_ms_))
        {
            state_ = STATE_FINISHED;
            setZeroOutput();
            return 1;
        }
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

void WuqiquPathPlanner::setZeroOutput(void)
{
    output_.world_vx_mps = 0.0f;
    output_.world_vy_mps = 0.0f;
    output_.wz_radps = 0.0f;
}

void WuqiquPathPlanner::updateState(float distance_m, uint32_t now_tick)
{
    if (state_ == STATE_IDLE)
    {
        state_ = STATE_APPROACH;
    }

    if (state_ == STATE_APPROACH && distance_m <= slow_dist_)
    {
        state_ = STATE_SLOW_APPROACH;
    }

    if ((state_ == STATE_APPROACH || state_ == STATE_SLOW_APPROACH) &&
        distance_m <= contact_dist_)
    {
        state_ = STATE_SOFT_CONTACT;
        soft_contact_start_tick_ = now_tick;
        soft_contact_stable_count_ = 0U;
    }
}

void WuqiquPathPlanner::limitVector(float &vx, float &vy, float max_speed) const
{
    if (max_speed < 0.0f)
    {
        max_speed = 0.0f;
    }

    const float speed = safeSqrt(vx * vx + vy * vy);
    if (speed > max_speed && speed > 0.000001f)
    {
        const float scale = max_speed / speed;
        vx *= scale;
        vy *= scale;
    }
}

void WuqiquPathPlanner::raiseVectorToMin(float &vx, float &vy, float min_speed) const
{
    if (min_speed <= 0.0f)
    {
        return;
    }

    const float speed = safeSqrt(vx * vx + vy * vy);
    if (speed > 0.000001f && speed < min_speed)
    {
        const float scale = min_speed / speed;
        vx *= scale;
        vy *= scale;
    }
}

float WuqiquPathPlanner::limitFloat(float value, float min_value, float max_value) const
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

float WuqiquPathPlanner::normalizeAngleDeg(float angle) const
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

float WuqiquPathPlanner::safeSqrt(float value) const
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }
    return sqrtf(value);
}
