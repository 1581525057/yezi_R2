#include "wuqiqu.h"
#include "main.h"
#include <math.h>

static const float WUQIQU_PI = 3.14159265358979323846f;  // 圆周率，用于角度转弧度
static const float kDegToRad = WUQIQU_PI / 180.0f;       // 角度转弧度系数
static const float kFastVMaxMps = 1.80f;                 // 快速阶段最大平移速度，单位 m/s
static const float kSlowVMaxMps = 0.45f;                 // 减速阶段最大平移速度，单位 m/s
static const float kPlannerMaxAngularSpeedRadps = 2.00f; // 规划器输出角速度上限，单位 rad/s
static const float kMinYawCommandFloorRadps = 0.05f;     // 规划器内部 yaw 修正硬下限，底盘有效下限在任务层处理

// 加快第二个点到第三个点之间的切换，减少到点后等待时间
static const uint8_t kFastLinkWaypointIndex = 1U;      // 当前对应第 2 个目标点
static const uint16_t kFastLinkStableCycles = 8U;      // 快速衔接点的姿态稳定计数阈值
static const uint32_t kFastLinkContactHoldMs = 20U;    // 快速衔接点进入 SETTLE 后的最短保持时间
static const uint32_t kFastLinkContactTimeoutMs = 80U; // 快速衔接点 SETTLE 阶段超时时间

// 目标点为视觉置零后的世界/雷达绝对坐标，任务层会把 world 速度转成底盘车体系速度。
// 参数顺序：x y yaw xy_tolerance yaw_tolerance
static const WuqiquPathPlanner::TargetPoint kWaypoints[] = {
    {0.04f, 0.91f, -90.0f, 0.015f, 1.5f},
    {0.35f, 0.60f, -90.0f, 0.035f, 3.0f},
    {0.35f, 0.60f, 90.0f, 0.035f, 2.0f},
    {0.96f, -1.64f, 0.0f, 0.030f, 1.5f},
    {0.35f, 0.87f, 90.0f, 0.030f, 2.0f},
};
static const uint8_t kWaypointCount = sizeof(kWaypoints) / sizeof(kWaypoints[0]);

static const WuqiquPathPlanner::TargetPoint kPrelimWeaponHeadPoints[] = {
    {0.04f, 0.90f, -90.0f, 0.015f, 1.5f},
    {0.23f, 0.90f, -90.0f, 0.015f, 1.5f},
    {0.44f, 0.90f, -90.0f, 0.015f, 1.5f},
};
static const uint8_t kPrelimWeaponHeadCount =
    sizeof(kPrelimWeaponHeadPoints) / sizeof(kPrelimWeaponHeadPoints[0]);

WuqiquPathPlanner wuqiqu;

WuqiquPathPlanner::WuqiquPathPlanner()
{
    reset();

    finish_dist_ = 0.010f;                  // XY 到点判定距离，单位 m
    brake_margin_m_ = 0.15f;                // 固定刹车提前距离，单位 m
    finish_speed_tolerance_mps_ = 0.08f;    // 到点稳定确认的平移速度阈值，单位 m/s

    kp_fast_ = 5.5f; // 快速阶段位置比例增益
    kd_fast_ = 0.75f; // 快速阶段位置微分增益
    kp_slow_ = 5.0f; // 减速/稳定确认阶段位置比例增益
    kd_slow_ = 0.2f; // 减速/稳定确认阶段位置微分增益

    yaw_sign_ = 1.0f;             // yaw 输出方向修正，1 表示保持当前方向
    yaw_kp_ = 2.4f;               // yaw 角度误差比例增益
    yaw_kd_ = 0.1f;              // yaw 角速度阻尼增益
    min_yaw_wz_ = 0.10f;          // yaw 最小角速度输出，单位 rad/s
    strong_yaw_wz_ = 0.35f;       // 大角度误差时的最小角速度输出，单位 rad/s
    strong_yaw_error_deg_ = 6.0f; // 判定为大角度误差的阈值，单位 deg
    yaw_tolerance_deg_ = 2.0f;    // yaw 到位判定角度误差，单位 deg

    stable_cycles_ = 80U;       // SETTLE 稳定计数阈值，按 runOnce 调用周期计数
    contact_hold_ms_ = 80U;     // 进入 SETTLE 后最短保持时间，单位 ms
    contact_timeout_ms_ = 500U; // SETTLE 阶段超时时间，单位 ms

    reloadDefaultWaypoints();
    current_index_ = 0U;
    loadCurrentWaypoint();
}

void WuqiquPathPlanner::reloadDefaultWaypoints(void)
{
    waypoint_count_ = kWaypointCount;
    if (waypoint_count_ > MAX_WAYPOINTS)
    {
        waypoint_count_ = MAX_WAYPOINTS;
    }

    for (uint8_t i = 0U; i < waypoint_count_; ++i)
    {
        waypoints_[i] = kWaypoints[i];
    }
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
    settle_start_tick_ = 0U;
    settle_stable_count_ = 0U;
    setZeroOutput();
}

void WuqiquPathPlanner::resetRoute(void)
{
    reloadDefaultWaypoints();
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

uint8_t WuqiquPathPlanner::overrideFirstWaypointWithPrelimWeaponHead(uint8_t weapon_index)
{
    if ((weapon_index >= kPrelimWeaponHeadCount) || (waypoint_count_ == 0U))
    {
        return 0U;
    }

    waypoints_[0] = kPrelimWeaponHeadPoints[weapon_index];
    if (current_index_ == 0U)
    {
        target_ = waypoints_[0];
    }

    return 1U;
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
    const uint8_t xy_in_tolerance = (distance_m <= xy_tolerance_m) ? 1U : 0U;
    const float speed_mps = safeSqrt(current_pose.world_speed_x * current_pose.world_speed_x +
                                     current_pose.world_speed_y * current_pose.world_speed_y);
    const float err_yaw_deg = normalizeAngleDeg(target_.yaw_deg - current_pose.yaw_360);
    const float yaw_control_rad = yaw_sign_ * normalizeAngleDeg(current_pose.yaw_360 - target_.yaw_deg) * kDegToRad;
    const float yaw_abs_deg = fabsf(err_yaw_deg);

    updateState(distance_m, speed_mps, xy_in_tolerance, now_tick);

    if (state_ == STATE_FINISHED)
    {
        setZeroOutput();
        return 1;
    }

    float kp = kp_fast_;
    float kd = kd_fast_;
    const float wz_limit = kPlannerMaxAngularSpeedRadps;
    float v_limit = kFastVMaxMps;

    if (state_ == STATE_SLOW || state_ == STATE_SETTLE)
    {
        kp = kp_slow_;
        kd = kd_slow_;
        v_limit = kSlowVMaxMps;
    }

    float vx_cmd = kp * err_x_m - kd * current_pose.world_speed_x;
    float vy_cmd = kp * err_y_m - kd * current_pose.world_speed_y;

    limitVectorToMax(vx_cmd, vy_cmd, v_limit);

    float wz_cmd = yaw_kp_ * yaw_control_rad - yaw_kd_ * current_pose.omega;
    wz_cmd = limitFloat(wz_cmd, -wz_limit, wz_limit);
    if (yaw_abs_deg > target_yaw_tolerance_deg)
    {
        const float min_wz = (yaw_abs_deg >= strong_yaw_error_deg_) ? strong_yaw_wz_ : min_yaw_wz_;
        const float scaled_min_wz = limitFloat((min_wz >= kMinYawCommandFloorRadps) ? min_wz : kMinYawCommandFloorRadps,
                                               0.0f,
                                               wz_limit);
        if (fabsf(wz_cmd) < scaled_min_wz)
        {
            wz_cmd = (yaw_control_rad >= 0.0f) ? scaled_min_wz : -scaled_min_wz;
            wz_cmd = limitFloat(wz_cmd, -wz_limit, wz_limit);
        }
    }

    output_.world_vx_mps = vx_cmd;
    output_.world_vy_mps = vy_cmd;
    output_.wz_radps = wz_cmd;

    if (state_ == STATE_SETTLE)
    {
        const uint32_t contact_time_ms = now_tick - settle_start_tick_;
        const uint8_t pose_stable =
            (xy_in_tolerance != 0U &&
             yaw_abs_deg <= target_yaw_tolerance_deg &&
             speed_mps <= finish_speed_tolerance_mps_)
                ? 1U
                : 0U;
        const uint16_t stable_cycles = (current_index_ == kFastLinkWaypointIndex) ? kFastLinkStableCycles : stable_cycles_;
        const uint32_t contact_hold_ms = (current_index_ == kFastLinkWaypointIndex) ? kFastLinkContactHoldMs : contact_hold_ms_;
        const uint32_t contact_timeout_ms =
            (current_index_ == kFastLinkWaypointIndex) ? kFastLinkContactTimeoutMs : contact_timeout_ms_;

        if (pose_stable != 0U)
        {
            if (settle_stable_count_ < stable_cycles)
            {
                ++settle_stable_count_;
            }
        }
        else
        {
            settle_stable_count_ = 0U;
        }

        if ((settle_stable_count_ >= stable_cycles && contact_time_ms >= contact_hold_ms) ||
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

void WuqiquPathPlanner::updateState(float distance_m, float speed_mps, uint8_t xy_in_tolerance, uint32_t now_tick)
{
    if (state_ == STATE_IDLE)
    {
        state_ = STATE_FAST;
    }

    (void)speed_mps;
    const float brake_dist_m = brake_margin_m_;
    if (state_ == STATE_FAST && distance_m <= brake_dist_m)
    {
        state_ = STATE_SLOW;
    }

    if ((state_ == STATE_FAST || state_ == STATE_SLOW) && xy_in_tolerance != 0U)
    {
        state_ = STATE_SETTLE;
        settle_start_tick_ = now_tick;
        settle_stable_count_ = 0U;
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
