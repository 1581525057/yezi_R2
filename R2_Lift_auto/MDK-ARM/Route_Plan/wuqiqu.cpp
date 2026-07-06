#include "wuqiqu.h"
#include "main.h"
#include <math.h>

static const float WUQIQU_PI = 3.14159265358979323846f;  // 圆周率，用于角度转弧度
static const float kDegToRad = WUQIQU_PI / 180.0f;       // 角度转弧度系数
static const float kApproachVMaxMps = 4.80f;             // 接近阶段最大平移速度，单位 m/s
static const float kSlowVMaxMps = 3.60f;                 // 减速阶段最大平移速度，单位 m/s
static const float kContactVMaxMps = 1.20f;              // 接触/贴近阶段最大平移速度，单位 m/s
static const float kPlannerMaxAngularSpeedRadps = 2.80f; // 规划器输出角速度上限，单位 rad/s
static const float kMinYawCommandFloorRadps = 0.18f;     // yaw 修正硬下限，避免比例缩小后转速过低

// 加快第二个点到第三个点之间的切换，减少到点后等待时间
static const uint8_t kFastLinkWaypointIndex = 1U;      // 当前对应第 2 个目标点
static const uint16_t kFastLinkStableCycles = 8U;      // 快速衔接点的姿态稳定计数阈值
static const uint32_t kFastLinkContactHoldMs = 20U;    // 快速衔接点接触后的最短保持时间
static const uint32_t kFastLinkContactTimeoutMs = 80U; // 快速衔接点接触阶段超时时间

// 目标点为视觉置零后的绝对坐标，当前约定雷达 X/Y 与车体 X/Y 对齐。
// 参数顺序：x y yaw xy_tolerance yaw_tolerance
static const WuqiquPathPlanner::TargetPoint kWaypoints[] = {
    {0.04f, 0.91f, -90.0f, 0.015f, 1.5f},
    {0.35f, 0.60f, -90.0f, 0.035f, 3.0f},
    {0.35f, 0.60f, 90.0f, 0.035f, 2.0f},
    {0.96f, -1.64f, 0.0f, 0.030f, 1.5f},
    {0.35f, 0.87f, 90.0f, 0.030f, 2.0f},
};
static const uint8_t kWaypointCount = sizeof(kWaypoints) / sizeof(kWaypoints[0]);

WuqiquPathPlanner wuqiqu;

WuqiquPathPlanner::WuqiquPathPlanner()
{
    reset();

    min_move_v_ = 0.45f; // 平移输出下限，避免小误差时底盘不动，单位 m/s

    slow_dist_ = 0.12f;     // 距目标小于该值后进入减速区，单位 m
    contact_dist_ = 0.020f; // 距目标小于该值后进入接触/贴近段，单位 m
    finish_dist_ = 0.010f;  // XY 到点判定距离，单位 m

    kp_approach_ = 9.0f; // 接近阶段位置比例增益
    kd_approach_ = 1.0f; // 接近阶段位置微分增益
    kp_slow_ = 6.5f;     // 减速阶段位置比例增益
    kd_slow_ = 0.8f;    // 减速阶段位置微分增益
    kp_contact_ = 5.0f;  // 接触/贴近阶段位置比例增益
    kd_contact_ = 1.2f;  // 接触/贴近阶段位置微分增益

    yaw_sign_ = 1.0f;             // yaw 输出方向修正，1 表示保持当前方向
    yaw_kp_ = 2.8f;               // yaw 角度误差比例增益
    min_yaw_wz_ = 0.55f;          // yaw 最小角速度输出，单位 rad/s
    strong_yaw_wz_ = 0.90f;       // 大角度误差时的最小角速度输出，单位 rad/s
    strong_yaw_error_deg_ = 6.0f; // 判定为大角度误差的阈值，单位 deg
    yaw_tolerance_deg_ = 2.0f;    // yaw 到位判定角度误差，单位 deg

    stable_cycles_ = 35U;       // 软接触稳定计数阈值，按 runOnce 调用周期计数
    contact_hold_ms_ = 80U;    // 接触后最短保持时间，单位 ms
    contact_timeout_ms_ = 500U; // 接触阶段超时时间，单位 ms

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

float WuqiquPathPlanner::getWaypointYawDeg(uint8_t waypoint_index) const
{
    if (waypoint_index < waypoint_count_)
    {
        return waypoints_[waypoint_index].yaw_deg;
    }

    return 0.0f;
}

int WuqiquPathPlanner::follow(const Pose &current_pose)
{
    loadCurrentWaypoint();

    const uint32_t now_tick = HAL_GetTick();
    const float err_x_m = target_.x_m - current_pose.x;
    const float err_y_m = target_.y_m - current_pose.y;
    const float distance_m = safeSqrt(err_x_m * err_x_m + err_y_m * err_y_m);
    const float xy_tolerance_m = (target_.xy_tolerance_m > 0.0f) ? target_.xy_tolerance_m : finish_dist_;
    const float target_yaw_tolerance_deg =
        (target_.yaw_tolerance_deg > 0.0f) ? target_.yaw_tolerance_deg : yaw_tolerance_deg_;
    const uint8_t xy_in_tolerance =
        (fabsf(err_x_m) <= xy_tolerance_m && fabsf(err_y_m) <= xy_tolerance_m) ? 1U : 0U;
    const float err_yaw_deg = normalizeAngleDeg(target_.yaw_deg - current_pose.yaw_360);
    const float yaw_control_deg = normalizeAngleDeg(current_pose.yaw_360 - target_.yaw_deg);
    const float yaw_abs_deg = fabsf(err_yaw_deg);

    updateState(distance_m, xy_in_tolerance, now_tick);

    if (state_ == STATE_FINISHED)
    {
        setZeroOutput();
        return 1;
    }

    float kp = kp_approach_;
    float kd = kd_approach_;
    const float wz_limit = kPlannerMaxAngularSpeedRadps;
    float v_limit = kApproachVMaxMps;

    if (state_ == STATE_SLOW_APPROACH)
    {
        kp = kp_slow_;
        kd = kd_slow_;
        v_limit = kSlowVMaxMps;
    }
    else if (state_ == STATE_SOFT_CONTACT)
    {
        kp = kp_contact_;
        kd = kd_contact_;
        v_limit = kContactVMaxMps;
    }

    float vx_cmd = kp * err_x_m - kd * current_pose.world_speed_x;
    float vy_cmd = kp * err_y_m - kd * current_pose.world_speed_y;

    if (xy_in_tolerance == 0U && yaw_abs_deg <= 35.0f)
    {
        raiseVectorToMin(vx_cmd, vy_cmd, min_move_v_);
    }
    limitVectorToMax(vx_cmd, vy_cmd, v_limit);

    float wz_cmd = yaw_sign_ * yaw_kp_ * yaw_control_deg * kDegToRad;
    wz_cmd = limitFloat(wz_cmd, -wz_limit, wz_limit);
    if (yaw_abs_deg > target_yaw_tolerance_deg)
    {
        const float min_wz = (yaw_abs_deg >= strong_yaw_error_deg_) ? strong_yaw_wz_ : min_yaw_wz_;
        const float scaled_min_wz = limitFloat((min_wz >= kMinYawCommandFloorRadps) ? min_wz : kMinYawCommandFloorRadps,
                                               0.0f,
                                               wz_limit);
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
        const uint8_t pose_stable = (xy_in_tolerance != 0U && yaw_abs_deg <= target_yaw_tolerance_deg) ? 1U : 0U;
        const uint16_t stable_cycles = (current_index_ == kFastLinkWaypointIndex) ? kFastLinkStableCycles : stable_cycles_;
        const uint32_t contact_hold_ms = (current_index_ == kFastLinkWaypointIndex) ? kFastLinkContactHoldMs : contact_hold_ms_;
        const uint32_t contact_timeout_ms =
            (current_index_ == kFastLinkWaypointIndex) ? kFastLinkContactTimeoutMs : contact_timeout_ms_;

        if (pose_stable != 0U)
        {
            if (soft_contact_stable_count_ < stable_cycles)
            {
                ++soft_contact_stable_count_;
            }
        }
        else
        {
            soft_contact_stable_count_ = 0U;
        }

        if ((soft_contact_stable_count_ >= stable_cycles && contact_time_ms >= contact_hold_ms) ||
            ((pose_stable != 0U) && contact_time_ms >= contact_timeout_ms))
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

void WuqiquPathPlanner::updateState(float distance_m, uint8_t xy_in_tolerance, uint32_t now_tick)
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
        (distance_m <= contact_dist_ || xy_in_tolerance != 0U))
    {
        state_ = STATE_SOFT_CONTACT;
        soft_contact_start_tick_ = now_tick;
        soft_contact_stable_count_ = 0U;
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

void WuqiquPathPlanner::limitVectorToMax(float &vx, float &vy, float max_speed) const
{
    if (max_speed <= 0.0f)
    {
        return;
    }

    const float speed = safeSqrt(vx * vx + vy * vy);
    if (speed > max_speed)
    {
        const float scale = max_speed / speed;
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
