#include "wuqiqu.h"
#include "cmsis_os.h"
#include "omni_chassis.h"
#include "usart_task.h"
#include "laser_distance.h"
#include <math.h>
#include <stdint.h>

/*
 * 武器区路径任务接入层。
 * 负责把全局 vision 当前位姿转换为规划器 Pose，
 * 再把规划器输出的规划坐标速度下发成底盘车体系速度目标。
 */

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

/* 下发给底盘的角速度上限。 */
constexpr float kMaxAngularSpeedRadps = 2.5f;

/* 每个控制周期允许的速度变化量，用于让目标速度平滑变化。 */
constexpr float kLinearAccStepMps = 0.045f;
constexpr float kLinearDecStepMps = 0.065f;
constexpr float kAngularAccStepRadps = 0.014f;
constexpr float kAngularDecStepRadps = 0.022f;

/* 第三点到位后的对接前微调参数。 */
constexpr float kDockAdjustLaserMinM = 0.060f;
constexpr float kDockAdjustLaserMaxM = 0.070f;
constexpr float kDockAdjustLaserTargetM = 0.0625f;
constexpr float kDockAdjustLaserKp = 8.0f;
constexpr float kDockAdjustLaserMinVyMps = 0.15f;
constexpr float kDockAdjustLaserMaxVyMps = 0.35f;
constexpr float kDockAdjustYawTargetDeg = 90.0f;
constexpr float kDockAdjustYawToleranceDeg = 1.0f;
constexpr float kDockAdjustYawKp = 1.8f;
constexpr float kDockAdjustYawMinWzRadps = 0.12f;
constexpr float kDockAdjustYawMaxWzRadps = 0.80f;
constexpr uint16_t kDockAdjustStableCycles = 80U;

/*
 * 当前约定雷达 X/Y 与车体 X/Y 对齐：
 * vision.x_diff -> planner X -> 底盘 Vx，
 * vision.y_diff -> planner Y -> 底盘 Vy。
 */
constexpr float kVisionXToPlannerX = 1.0f;
constexpr float kVisionYToPlannerY = 1.0f;

/* 普通数值限幅。 */
float limitFloat(float value, float min_value, float max_value)
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

/* 单轴斜率限制：根据速度绝对值增减判断加速/减速。 */
float slewRateLimit(float target, float current, float acc_step, float dec_step)
{
    const float err = target - current;

    /* 判断是加速还是减速：
     * 绝对值增大 → 加速（远离零点），用较小的 acc_step
     * 绝对值减小 → 减速（靠近零点），用较大的 dec_step */
    const float step = (fabsf(target) > fabsf(current)) ? acc_step : dec_step;

    if (err > step)
    {
        return current + step;
    }
    if (err < -step)
    {
        return current - step;
    }
    return target;
}

float normalizeAngleDeg(float angle)
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

/* 武器区路径任务状态机，封装启动、停止、单周期更新和底盘目标输出。 */
class WuqiquTask
{
public:
    WuqiquTask()
        : active_(0U),
          finished_(0U),
          dock_adjust_mode_(0U),
          dock_adjust_stable_count_(0U),
          vx_target_(0.0f),
          vy_target_(0.0f),
          wz_target_(0.0f)
    {
    }

    void start()
    {
        startAt(0U);
    }

    void startAt(uint8_t waypoint_index)
    {
        active_ = 0U;
        finished_ = 0U;
        dock_adjust_mode_ = 0U;
        dock_adjust_stable_count_ = 0U;
        wuqiqu.resetRoute();
        clearOutput();

        for (uint8_t i = 0U; i < waypoint_index; ++i)
        {
            wuqiqu.advanceToNext();
        }

        if (wuqiqu.isAllFinished())
        {
            finished_ = 1U;
            return;
        }

        active_ = 1U;
    }

    void startDockAdjust()
    {
        active_ = 1U;
        finished_ = 0U;
        dock_adjust_mode_ = 1U;
        dock_adjust_stable_count_ = 0U;
        clearOutput();
        wuqiqu.reset();
    }

    void stop()
    {
        active_ = 0U;
        finished_ = 0U;
        dock_adjust_mode_ = 0U;
        dock_adjust_stable_count_ = 0U;
        clearOutput();
        wuqiqu.reset();
    }

    uint8_t runOnce()
    {
        /* 未激活时不参与底盘控制。 */
        if (active_ == 0U)
        {
            return 0U;
        }

        /* 每周期读取当前位姿，运行规划器，并更新底盘速度目标。 */
        if (dock_adjust_mode_ != 0U)
        {
            return runDockAdjustOnce();
        }

        const WuqiquPathPlanner::Pose pose = buildPose();
        const int finished = wuqiqu.follow(pose);
        updateOutput(wuqiqu.getOutput());

        if (finished != 0)
        {
            active_ = 0U;
            finished_ = 1U;
            clearOutput();
            return 1U;
        }

        return 0U;
    }

    uint8_t isActive() const
    {
        return active_;
    }

    uint8_t isFinished() const
    {
        return finished_;
    }

    void advanceToNext()
    {
        active_ = 0U;
        finished_ = 0U;
        clearOutput();
        wuqiqu.advanceToNext();
        if (wuqiqu.isAllFinished())
        {
            finished_ = 1U;
            return;
        }
        active_ = 1U;
    }

    uint8_t isAllFinished() const
    {
        return wuqiqu.isAllFinished() ? 1U : 0U;
    }

    float getWaypointYawDeg(uint8_t waypoint_index) const
    {
        return wuqiqu.getWaypointYawDeg(waypoint_index);
    }

    float getChassisVxTarget(float manual) const
    {
        /* 任务激活时覆盖遥控/上层给定速度，否则透传原速度。 */
        return (active_ != 0U) ? vx_target_ : manual;
    }

    float getChassisVyTarget(float manual) const
    {
        return (active_ != 0U) ? vy_target_ : manual;
    }

    float getChassisVzTarget(float manual) const
    {
        return (active_ != 0U) ? wz_target_ : manual;
    }

private:
    /* active_ 为 1 表示当前任务接管底盘速度目标。 */
    volatile uint8_t active_;
    volatile uint8_t finished_;
    volatile uint8_t dock_adjust_mode_;
    volatile uint16_t dock_adjust_stable_count_;

    /* 下发到底盘的车体系目标速度，单位 m/s、rad/s。 */
    volatile float vx_target_;
    volatile float vy_target_;
    volatile float wz_target_;

    void clearOutput()
    {
        vx_target_ = 0.0f;
        vy_target_ = 0.0f;
        wz_target_ = 0.0f;
    }

    uint8_t runDockAdjustOnce()
    {
        const uint8_t laser_valid = laser_left.data.valid;
        const float laser_distance_m = laser_left.data.distance_m;
        const uint8_t laser_ok =
            (laser_valid != 0U &&
             laser_distance_m >= kDockAdjustLaserMinM &&
             laser_distance_m <= kDockAdjustLaserMaxM)
                ? 1U
                : 0U;

        const float yaw_error_deg = normalizeAngleDeg(kDockAdjustYawTargetDeg - vision.angle_x);
        const uint8_t yaw_ok = (fabsf(yaw_error_deg) <= kDockAdjustYawToleranceDeg) ? 1U : 0U;

        float vy_cmd = 0.0f;
        if (laser_valid != 0U && laser_ok == 0U)
        {
            const float laser_error_m = laser_distance_m - kDockAdjustLaserTargetM;
            vy_cmd = limitFloat(kDockAdjustLaserKp * laser_error_m,
                                -kDockAdjustLaserMaxVyMps,
                                kDockAdjustLaserMaxVyMps);
            if (fabsf(vy_cmd) < kDockAdjustLaserMinVyMps)
            {
                vy_cmd = (vy_cmd >= 0.0f) ? kDockAdjustLaserMinVyMps : -kDockAdjustLaserMinVyMps;
            }
        }

        float wz_cmd = 0.0f;
        if (yaw_ok == 0U)
        {
            const float yaw_control_deg = normalizeAngleDeg(vision.angle_x - kDockAdjustYawTargetDeg);
            wz_cmd = limitFloat(kDockAdjustYawKp * yaw_control_deg * kDegToRad,
                                -kDockAdjustYawMaxWzRadps,
                                kDockAdjustYawMaxWzRadps);
            if (fabsf(wz_cmd) < kDockAdjustYawMinWzRadps)
            {
                wz_cmd = (wz_cmd >= 0.0f) ? kDockAdjustYawMinWzRadps : -kDockAdjustYawMinWzRadps;
            }
        }

        vx_target_ = 0.0f;
        vy_target_ = slewRateLimit(vy_cmd, vy_target_, kLinearAccStepMps, kLinearDecStepMps);
        wz_target_ = slewRateLimit(wz_cmd, wz_target_, kAngularAccStepRadps, kAngularDecStepRadps);

        if (laser_ok != 0U && yaw_ok != 0U)
        {
            if (dock_adjust_stable_count_ < kDockAdjustStableCycles)
            {
                ++dock_adjust_stable_count_;
            }
        }
        else
        {
            dock_adjust_stable_count_ = 0U;
        }

        if (dock_adjust_stable_count_ >= kDockAdjustStableCycles)
        {
            active_ = 0U;
            finished_ = 1U;
            dock_adjust_mode_ = 0U;
            clearOutput();
            return 1U;
        }

        return 0U;
    }

    WuqiquPathPlanner::Pose buildPose() const
    {
        WuqiquPathPlanner::Pose pose = {};

        pose.x = kVisionXToPlannerX * vision.x_diff; // m, planner X / chassis Vx
        pose.y = kVisionYToPlannerY * vision.y_diff; // m, planner Y / chassis Vy
        pose.yaw = vision.angle_x * kDegToRad;       // rad
        pose.yaw_360 = vision.angle_x;               // deg

        /* 底盘当前速度来自运动学解算，单位 m/s */
        pose.car_speed_x = omni_chassis.now.Vx;
        pose.car_speed_y = omni_chassis.now.Vy;
        pose.omega = omni_chassis.now.Vz;

        /* 当前规划坐标已对齐底盘速度轴，用于调试时直接记录底盘速度。 */
        chassisToWorldVelocity(pose.car_speed_x,
                               pose.car_speed_y,
                               pose.yaw,
                               &pose.world_speed_x,
                               &pose.world_speed_y);

        return pose;
    }

    void updateOutput(const WuqiquPathPlanner::Output &output)
    {
        /* 规划器 X/Y 已对齐底盘 Vx/Vy，输出直接作为底盘车体系速度。 */
        float vx_limited = 0.0f;
        float vy_limited = 0.0f;
        worldToChassisVelocity(output.world_vx_mps,
                               output.world_vy_mps,
                               vision.angle_x * kDegToRad,
                               &vx_limited,
                               &vy_limited);

        const float wz_limited = limitFloat(output.wz_radps, -kMaxAngularSpeedRadps, kMaxAngularSpeedRadps);

        /* 目标速度再经过斜率限制，降低底盘指令突变。 */
        vx_target_ = slewRateLimit(vx_limited, vx_target_, kLinearAccStepMps, kLinearDecStepMps);
        vy_target_ = slewRateLimit(vy_limited, vy_target_, kLinearAccStepMps, kLinearDecStepMps);
        wz_target_ = slewRateLimit(wz_limited, wz_target_, kAngularAccStepRadps, kAngularDecStepRadps);
    }

    static void worldToChassisVelocity(float world_vx, float world_vy, float yaw_rad, float *chassis_vx, float *chassis_vy)
    {
        const float cos_yaw = cosf(yaw_rad);
        const float sin_yaw = sinf(yaw_rad);

        *chassis_vx = cos_yaw * world_vx + sin_yaw * world_vy;
        *chassis_vy = -sin_yaw * world_vx + cos_yaw * world_vy;
    }

    static void chassisToWorldVelocity(float chassis_vx, float chassis_vy, float yaw_rad, float *world_vx, float *world_vy)
    {
        const float cos_yaw = cosf(yaw_rad);
        const float sin_yaw = sinf(yaw_rad);

        *world_vx = cos_yaw * chassis_vx - sin_yaw * chassis_vy;
        *world_vy = sin_yaw * chassis_vx + cos_yaw * chassis_vy;
    }
};

WuqiquTask g_wuqiqu_task;

extern "C" void wuqiqu_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        (void)g_wuqiqu_task.runOnce();
        osDelay(1);
    }
}

extern "C" void WuqiquTask_Start(void)
{
    g_wuqiqu_task.start();
}

extern "C" void WuqiquTask_StartAt(uint8_t waypoint_index)
{
    g_wuqiqu_task.startAt(waypoint_index);
}

extern "C" void WuqiquTask_StartDockAdjust(void)
{
    g_wuqiqu_task.startDockAdjust();
}

extern "C" void WuqiquTask_Stop(void)
{
    g_wuqiqu_task.stop();
}

extern "C" uint8_t WuqiquTask_RunOnce(void)
{
    return g_wuqiqu_task.runOnce();
}

extern "C" uint8_t WuqiquTask_IsActive(void)
{
    return g_wuqiqu_task.isActive();
}

extern "C" uint8_t WuqiquTask_IsFinished(void)
{
    return g_wuqiqu_task.isFinished();
}

extern "C" float WuqiquTask_GetChassisVxTarget(float manual)
{
    return g_wuqiqu_task.getChassisVxTarget(manual);
}

extern "C" float WuqiquTask_GetChassisVyTarget(float manual)
{
    return g_wuqiqu_task.getChassisVyTarget(manual);
}

extern "C" float WuqiquTask_GetChassisVzTarget(float manual)
{
    return g_wuqiqu_task.getChassisVzTarget(manual);
}

extern "C" void WuqiquTask_AdvanceToNext(void)
{
    g_wuqiqu_task.advanceToNext();
}

extern "C" uint8_t WuqiquTask_IsAllFinished(void)
{
    return g_wuqiqu_task.isAllFinished();
}

extern "C" float WuqiquTask_GetWaypointYawDeg(uint8_t waypoint_index)
{
    return g_wuqiqu_task.getWaypointYawDeg(waypoint_index);
}
