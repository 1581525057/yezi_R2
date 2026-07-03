#include "wuqiqu.h"
#include "main.h"
#include <math.h>

static const float WUQIQU_PI = 3.14159265358979323846f; // 圆周率，用于角度换算
static const float kDegToRad = WUQIQU_PI / 180.0f;      // 角度转弧度系数
static const float kApproachVMaxMps = 5.20f;            // 接近阶段最大平移速度，单位 m/s
static const float kSlowVMaxMps = 2.80f;                // 减速阶段最大平移速度，单位 m/s
static const float kContactVMaxMps = 0.95f;             // 接触/贴近阶段最大平移速度，单位 m/s
static const float kPlannerMaxAngularSpeedRadps = 2.80f; // 规划器输出角速度上限，单位 rad/s
static const float kMinYawCommandFloorRadps = 0.18f;     // yaw 修正硬下限，避免比例缩小后转速过低
static const float kBrakeSpeedMinMps = 0.16f;            // 距离制动保底速度，避免近目标速度被压到无法动作
static const float kBrakeSpeedGain = 12.0f;              // 距离制动斜率，距离越近允许速度越低
static const float kSlowMinMoveGain = 2.0f;              // 减速区最小平移速度随距离递减的斜率
static const float kSlowMinMoveFloorMps = 0.10f;         // 减速区最小平移输出下限，单位 m/s
static const float kContactMinMoveMaxMps = 0.12f;        // 贴近段最小平移输出上限，避免近目标硬推
//加快第二个点到第三个点之间的流程连贯性设置的参数
static const uint8_t kFastLinkWaypointIndex = 1U;       // 快速衔接航点索引，当前对应第 2 个目标点
static const uint16_t kFastLinkStableCycles = 8U;       // 快速衔接航点姿态稳定计数阈值，按 runOnce 调用周期计数
static const uint32_t kFastLinkContactHoldMs = 20U;     // 快速衔接航点接触后最短保持时间，单位 ms
static const uint32_t kFastLinkContactTimeoutMs = 80U;  // 快速衔接航点接触阶段超时时间，单位 ms
static const float kFastLinkTimeoutXyScale = 1.20f;      // 快速衔接点超时判定时的 XY 放宽倍率
static const float kFastLinkTimeoutYawExtraDeg = 3.0f;   // 快速衔接点超时判定时额外放宽的 yaw 角度

// 目标点为视觉置零后的绝对坐标，当前约定雷达 X/Y 与车体 X/Y 对齐。
WuqiquPathPlanner::TargetPoint kWaypoints[] = {
    {0.04f, 0.91f, -90.0f, 1.0f, 0.015f, 2.0f},
    {0.30f, 0.55f, -90.0f, 1.0f, 0.035f, 3.0f},
    {0.30f, 0.55f, 90.0f, 1.0f, 0.035f, 2.0f},
    {0.96f, -1.64f, 0.0f, 1.0f, 0.020f, 1.5f},
};
static const uint8_t kWaypointCount = sizeof(kWaypoints) / sizeof(kWaypoints[0]);

WuqiquPathPlanner wuqiqu;

void Wuqiqu_SetFirstWaypointX(float x_m)
{
    // 同步默认点表和规划器当前副本，避免运行中只改一处导致目标不一致。
    kWaypoints[0].x_m = x_m;
    if (wuqiqu.waypoint_count_ > 0U)
    {
        wuqiqu.waypoints_[0].x_m = x_m;
        if (wuqiqu.current_index_ == 0U)
        {
            wuqiqu.target_.x_m = x_m;
        }
    }
}

static bool IsFastLinkWaypoint(uint8_t waypoint_index)
{
    return ((waypoint_index == kFastLinkWaypointIndex) || (waypoint_index == 2U));
}

WuqiquPathPlanner::WuqiquPathPlanner()
{
    reset();

    min_move_v_ = 0.45f;          // 平移输出下限，避免小误差时底盘不动，单位 m/s

    slow_dist_ = 0.12f;           // 距目标小于该值后进入减速区，单位 m
    contact_dist_ = 0.020f;       // 距目标小于该值后进入接触/贴近段，单位 m
    finish_dist_ = 0.010f;        // XY 到点判定距离，单位 m

    kp_approach_ = 7.0f;          // 接近阶段位置比例增益
    kd_approach_ = 0.8f;          // 接近阶段位置微分增益
    kp_slow_ = 6.0f;              // 减速阶段位置比例增益
    kd_slow_ = 0.75f;             // 减速阶段位置微分增益
    kp_contact_ = 3.2f;           // 接触/贴近阶段位置比例增益
    kd_contact_ = 0.75f;          // 接触/贴近阶段位置微分增益

    yaw_sign_ = 1.0f;             // yaw 输出方向修正，1 表示保持当前方向
    yaw_kp_ = 2.8f;               // yaw 角度误差比例增益
    min_yaw_wz_ = 0.55f;          // yaw 最小角速度输出，单位 rad/s
    strong_yaw_wz_ = 0.90f;       // 大角度误差时的最小角速度输出，单位 rad/s
    strong_yaw_error_deg_ = 6.0f; // 判定为大角度误差的阈值，单位 deg
    yaw_tolerance_deg_ = 2.0f;    // yaw 到位判定角度误差，单位 deg

    stable_cycles_ = 35U;         // 软接触稳定计数阈值，按 runOnce 调用周期计数
    contact_hold_ms_ = 120U;      // 接触后最短保持时间，单位 ms
    contact_timeout_ms_ = 300U;   // 接触阶段超时时间，单位 ms

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

    const float brake_v_limit = limitFloat(kBrakeSpeedMinMps + kBrakeSpeedGain * distance_m,
                                           kBrakeSpeedMinMps,
                                           kApproachVMaxMps);
    if (v_limit > brake_v_limit)
    {
        v_limit = brake_v_limit;
    }

    const float yaw_kp_scale = limitFloat(target_.yaw_kp_scale, 0.0f, 3.0f);

    float vx_cmd = kp * err_x_m - kd * current_pose.world_speed_x;
    float vy_cmd = kp * err_y_m - kd * current_pose.world_speed_y;

    if (xy_in_tolerance == 0U && yaw_abs_deg <= 35.0f)
    {
        float min_move_speed = min_move_v_;
        if (state_ == STATE_SLOW_APPROACH)
        {
            min_move_speed = limitFloat(kSlowMinMoveGain * distance_m,
                                        kSlowMinMoveFloorMps,
                                        min_move_v_);
        }
        else if (state_ == STATE_SOFT_CONTACT)
        {
            min_move_speed = limitFloat(kSlowMinMoveGain * distance_m,
                                        0.0f,
                                        kContactMinMoveMaxMps);
        }
        raiseVectorToMin(vx_cmd, vy_cmd, min_move_speed);
    }
    limitVectorToMax(vx_cmd, vy_cmd, v_limit);

    float wz_cmd = yaw_sign_ * yaw_kp_ * yaw_kp_scale * yaw_control_deg * kDegToRad;
    wz_cmd = limitFloat(wz_cmd, -wz_limit, wz_limit);
    if (yaw_abs_deg > target_yaw_tolerance_deg)
    {
        const float min_wz = (yaw_abs_deg >= strong_yaw_error_deg_) ? strong_yaw_wz_ : min_yaw_wz_;
        const float min_wz_scaled = min_wz * yaw_kp_scale;
        const float scaled_min_wz = limitFloat((min_wz_scaled >= kMinYawCommandFloorRadps) ? min_wz_scaled : kMinYawCommandFloorRadps,
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
        const uint8_t is_fast_link_waypoint = IsFastLinkWaypoint(current_index_) ? 1U : 0U;
        const uint8_t pose_stable = (xy_in_tolerance != 0U && yaw_abs_deg <= target_yaw_tolerance_deg) ? 1U : 0U;
        const uint16_t stable_cycles = (is_fast_link_waypoint != 0U) ? kFastLinkStableCycles : stable_cycles_;
        const uint32_t contact_hold_ms = (is_fast_link_waypoint != 0U) ? kFastLinkContactHoldMs : contact_hold_ms_;
        const uint32_t contact_timeout_ms = (is_fast_link_waypoint != 0U) ? kFastLinkContactTimeoutMs : contact_timeout_ms_;
        const float timeout_xy_tolerance_m =
            (is_fast_link_waypoint != 0U) ? (xy_tolerance_m * kFastLinkTimeoutXyScale) : xy_tolerance_m;
        const float timeout_yaw_tolerance_deg =
            (is_fast_link_waypoint != 0U) ? (target_yaw_tolerance_deg + kFastLinkTimeoutYawExtraDeg) : target_yaw_tolerance_deg;
        const uint8_t timeout_pose_ready =
            (fabsf(err_x_m) <= timeout_xy_tolerance_m &&
             fabsf(err_y_m) <= timeout_xy_tolerance_m &&
             yaw_abs_deg <= timeout_yaw_tolerance_deg)
                ? 1U
                : 0U;

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
            ((timeout_pose_ready != 0U) && contact_time_ms >= contact_timeout_ms))
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
