#include "wuqiqu.h"
#include <math.h>
//debug变量
WuqiquDebug_t wuqiqu_debug = {};
namespace
{
    static const float WUQIQU_PI = 3.14159265358979323846f;
    static const float kDegToRad = WUQIQU_PI / 180.0f;

    /* PD控制参数 */
    float kPosKp = 5.0f;   // 位置P增益
    float kPosKd = 0.0f;   // 位置D增益
    float kYawKp = 0.5f;   // 角度P增益
    float kYawKd = 0.0f;   // 角度D增益

    /* 速度限幅 */
    static const float kMaxAngularSpeedRadps = 0.8f;
    /* 平移阶段 yaw 并行修正角速度限幅（较小，避免旋转干扰平移精度） */
    static const float kMovingYawMaxRadps = 0.3f;

    /* 到达判定阈值 */
    static const float kPosToleranceM = 0.010f;      // 位置容差 10mm
    static const float kYawToleranceDeg = 1.0f;      // 角度容差 1度
    static const uint16_t kStableCount = 80U;        // 稳定计数

    /* debug 目标点（视觉坐标系，单位：m, m, 度）
     * kVisionTargetX  对应 vision.x_diff 的期望值
     * kVisionTargetY  对应 vision.y_diff 的期望值
     * kVisionTargetYawDeg 对应 vision.angle_x 的期望值
     * 内部会自动转换为规划坐标，中间映射链不变。 */
    float kVisionTargetX = 1.0f;         // 期望 vision.x_diff
    float kVisionTargetY = 0.0f;         // 期望 vision.y_diff
    float kVisionTargetYawDeg = 0.0f;    // 期望 vision.angle_x，度

    /* wuqiqu_task.cpp 中的映射常量（保持一致）：
     * plannerX = -1.0 * vision.y_diff    (kVisionYToPlannerX = -1)
     * plannerY = +1.0 * vision.x_diff    (kVisionXToPlannerY = +1)
     * 因此反推：
     * targetPlannerX = -1.0 * kVisionTargetY
     * targetPlannerY = +1.0 * kVisionTargetX */
    constexpr float kVisionYToPlannerX = -1.0f;
    constexpr float kVisionXToPlannerY =  1.0f;
}

WuqiquPathPlanner wuqiqu;

WuqiquPathPlanner::WuqiquPathPlanner()
{
    reset();
    // 设置默认目标点（视觉坐标 → 规划坐标）
    target_.x_m = kVisionYToPlannerX * kVisionTargetY;
    target_.y_m = kVisionXToPlannerY * kVisionTargetX;
    target_.yaw_deg = kVisionTargetYawDeg;
}

void WuqiquPathPlanner::setTarget(float x_m, float y_m, float yaw_deg)
{
    target_.x_m = x_m;
    target_.y_m = y_m;
    target_.yaw_deg = yaw_deg;
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

    wuqiqu_debug = WuqiquDebug_t();
    wuqiqu_debug.state = (uint8_t)state_;
}

int WuqiquPathPlanner::follow(const Pose &current_pose)
{
    /* 每次都同步目标点（视觉坐标 → 规划坐标），方便 debug 时修改实时生效 */
    target_.x_m = kVisionYToPlannerX * kVisionTargetY;
    target_.y_m = kVisionXToPlannerY * kVisionTargetX;
    target_.yaw_deg = kVisionTargetYawDeg;

    /* 计算位置误差 */
    const float err_x_m = target_.x_m - current_pose.x;
    const float err_y_m = target_.y_m - current_pose.y;
    const float distance_m = safeSqrt(err_x_m * err_x_m + err_y_m * err_y_m);
    const uint8_t xy_reached = (distance_m < kPosToleranceM) ? 1U : 0U;

    /* 计算角度误差 */
    const float err_yaw_deg = normalizeAngleDeg(target_.yaw_deg - current_pose.yaw_360);
    const float err_yaw_rad = err_yaw_deg * kDegToRad;

    if (state_ == STATE_IDLE)
    {
        state_ = STATE_MOVING;
        last_err_x_ = err_x_m;
        last_err_y_ = err_y_m;
        last_err_yaw_rad_ = err_yaw_rad;
    }

    /* 状态机 */
    switch (state_)
    {
    case STATE_IDLE:
        output_.world_vx_mps = 0.0f;
        output_.world_vy_mps = 0.0f;
        output_.wz_radps = 0.0f;
        break;

    case STATE_MOVING:
        /* 位置到达判定 */
        if (xy_reached != 0U)
        {
            ++xy_stable_count_;
            output_.world_vx_mps = 0.0f;
            output_.world_vy_mps = 0.0f;

            if (xy_stable_count_ >= kStableCount)
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

            /* 位置环控制（规划X/Y对齐底盘Vx/Vy） */
            float world_vx_mps = kPosKp * err_x_m + kPosKd * (err_x_m - last_err_x_);
            float world_vy_mps = kPosKp * err_y_m + kPosKd * (err_y_m - last_err_y_);

            /* 更新历史误差 */
            last_err_x_ = err_x_m;
            last_err_y_ = err_y_m;

            output_.world_vx_mps = world_vx_mps;
            output_.world_vy_mps = world_vy_mps;
        }

        /* 平移阶段并行修正 yaw：PD 控制，角速度限幅较小以减少对平移的干扰 */
        {
            float wz_cmd = -(kYawKp * err_yaw_rad + kYawKd * (err_yaw_rad - last_err_yaw_rad_));
            last_err_yaw_rad_ = err_yaw_rad;

            if (wz_cmd > kMovingYawMaxRadps)
                wz_cmd = kMovingYawMaxRadps;
            else if (wz_cmd < -kMovingYawMaxRadps)
                wz_cmd = -kMovingYawMaxRadps;

            output_.wz_radps = wz_cmd;
        }
        break;

    case STATE_YAW_CORRECTING:
        /* 角度到达判定 */
        if (fabsf(err_yaw_deg) < kYawToleranceDeg)
        {
            output_.world_vx_mps = 0.0f;
            output_.world_vy_mps = 0.0f;
            output_.wz_radps = 0.0f;

            ++theta_stable_count_;
            if (theta_stable_count_ >= kStableCount)
            {
                reset();
                state_ = STATE_FINISHED;
                wuqiqu_debug.state = (uint8_t)state_;
                return 1;  // 完成
            }
        }
        else
        {
            theta_stable_count_ = 0U;

            /* 角度环PD控制 */
            output_.wz_radps = -(kYawKp * err_yaw_rad + kYawKd * (err_yaw_rad - last_err_yaw_rad_));
            last_err_yaw_rad_ = err_yaw_rad;

            /* 角速度限幅 */
            if (fabsf(output_.wz_radps) > kMaxAngularSpeedRadps)
            {
                output_.wz_radps = (output_.wz_radps > 0.0f) ? kMaxAngularSpeedRadps : -kMaxAngularSpeedRadps;
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

    /* 更新调试信息 */
    wuqiqu_debug.state = (uint8_t)state_;
    wuqiqu_debug.on_target_flag = on_target_flag_;
    wuqiqu_debug.xy_stable_count = xy_stable_count_;
    wuqiqu_debug.theta_stable_count = theta_stable_count_;
    wuqiqu_debug.pose_x = current_pose.x;
    wuqiqu_debug.pose_y = current_pose.y;
    wuqiqu_debug.pose_yaw = current_pose.yaw;
    wuqiqu_debug.target_x = target_.x_m;
    wuqiqu_debug.target_y = target_.y_m;
    wuqiqu_debug.target_yaw = target_.yaw_deg;
    wuqiqu_debug.err_x = err_x_m;
    wuqiqu_debug.err_y = err_y_m;
    wuqiqu_debug.err_distance = distance_m;
    wuqiqu_debug.err_theta = err_yaw_deg;
    wuqiqu_debug.out_vx = output_.world_vx_mps;
    wuqiqu_debug.out_vy = output_.world_vy_mps;
    wuqiqu_debug.out_wz = output_.wz_radps;

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
