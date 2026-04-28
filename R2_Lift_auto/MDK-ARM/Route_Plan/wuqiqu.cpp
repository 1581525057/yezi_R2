#include "wuqiqu.h"
#include <math.h>

WuqiquDebug_t wuqiqu_debug = {};

namespace
{
    static const float WUQIQU_PI = 3.14159265358979323846f;
    static const float kDegToRad = WUQIQU_PI / 180.0f;

    /* P控制参数（D暂时关闭） */
    static const float kPosKp = 0.5f;   // 位置P增益
    static const float kPosKd = 0.0f;   // 位置D增益
    static const float kYawKp = 0.5f;   // 角度P增益
    static const float kYawKd = 0.0f;   // 角度D增益

    /* 速度限幅 */
    static const float kMaxLinearSpeedMps = 2.0f;
    static const float kMaxAngularSpeedRadps = 0.8f;

    /* 到达判定阈值 */
    static const float kPosToleranceM = 0.010f;      // 位置容差 10mm
    static const float kYawToleranceDeg = 1.0f;      // 角度容差 1度
    static const uint16_t kStableCount = 80U;        // 稳定计数

    /* 预定义目标点（规划X/Y对齐底盘Vx/Vy，单位：m, m, 度）*/
    static const float kTargetX = 0.0f;        // 底盘 +Vx 方向 0m
    static const float kTargetY = 1.0f;        // 底盘 +Vy 方向 1m
    static const float kTargetYawDeg = 0.0f;   // 度

    uint8_t hasPassedTargetPlane(float pose_x, float pose_y, float target_x, float target_y)
    {
        const float target_len_square = target_x * target_x + target_y * target_y;

        if (target_len_square <= 0.000001f)
        {
            return 0U;
        }

        const float progress = pose_x * target_x + pose_y * target_y;
        return (progress >= target_len_square) ? 1U : 0U;
    }

    void limitPlanarVelocity(float &vx, float &vy, float max_speed)
    {
        const float speed = sqrtf(vx * vx + vy * vy);
        if (speed > max_speed && speed > 0.000001f)
        {
            const float scale = max_speed / speed;
            vx *= scale;
            vy *= scale;
        }
    }
}

WuqiquPathPlanner wuqiqu;

WuqiquPathPlanner::WuqiquPathPlanner()
{
    reset();
    // 设置默认目标点
    target_.x_m = kTargetX;
    target_.y_m = kTargetY;
    target_.yaw_deg = kTargetYawDeg;
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

            /* 位置环P控制（规划X/Y对齐底盘Vx/Vy） */
            float world_vx_mps = kPosKp * err_x_m + kPosKd * (err_x_m - last_err_x_);
            float world_vy_mps = kPosKp * err_y_m + kPosKd * (err_y_m - last_err_y_);

            /* 更新历史误差 */
            last_err_x_ = err_x_m;
            last_err_y_ = err_y_m;

            /* 速度限幅 */
            limitPlanarVelocity(world_vx_mps, world_vy_mps, kMaxLinearSpeedMps);

            output_.world_vx_mps = world_vx_mps;
            output_.world_vy_mps = world_vy_mps;
        }

        /* 平移阶段不修正yaw，避免平移和旋转互相耦合。 */
        output_.wz_radps = 0.0f;
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

            /* 角度环P控制 */
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

float WuqiquPathPlanner::normalizeAngleRad(float angle) const
{
    while (angle > WUQIQU_PI)
        angle -= 2.0f * WUQIQU_PI;
    while (angle < -WUQIQU_PI)
        angle += 2.0f * WUQIQU_PI;
    return angle;
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
