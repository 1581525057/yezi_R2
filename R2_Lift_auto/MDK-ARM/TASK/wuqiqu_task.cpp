#include "wuqiqu.h"
#include "wuqiqu_task.h"
#include "cmsis_os.h"
#include "omni_chassis.h"
#include "usart_task.h"
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
constexpr float kMaxAngularSpeedRadps = 2.0f;

/* FAST 阶段平移速度上限，负责快速接近目标*/
constexpr float kFastLinearMaxMps = 1.80f;
/* SLOW 阶段平移速度上限，负责近目标稳定收敛*/
constexpr float kSlowLinearMaxMps = 0.45f;

/* FAST 阶段最大加速度 单位 m/s^2。 */
constexpr float kFastLinearAccMps2 = 3.2f;
/* FAST 阶段最大减速度  */
constexpr float kFastLinearDecMps2 = 4.8f;
/* SLOW 阶段最大加速度*/
constexpr float kSlowLinearAccMps2 = 1.2f;
/* SLOW 阶段最大减速度*/
constexpr float kSlowLinearDecMps2 = 2.0f;

/* FAST 阶段任务层最低平移速度补偿，用于克服底盘静摩擦*/
constexpr float kFastMinLinearMps = 0.12f;
/* SLOW 阶段最低平移速度补偿*/
constexpr float kSlowMinLinearMps = 0.07f;

/* yaw 指令最大加速度 单位 rad/s^2。 */
constexpr float kAngularAccRadps2 = 14.0f;
/* yaw 指令最大减速度*/
constexpr float kAngularDecRadps2 = 22.0f;

/* 控制周期 dt 下限，防止同一 tick 内重复调用导致斜率限制步长变成 0。 */
constexpr float kMinControlDtS = 0.001f;
/* 控制周期 dt 上限，防止任务偶发阻塞后一次性放大速度变化量。 */
constexpr float kMaxControlDtS = 0.030f;

/* wuqiqu 下发到底盘的最小有效速度，避免给了速度但底盘克服不了摩擦力。*/
constexpr float kMinLinearCommandMps = 0.07f;
constexpr float kMinAngularCommandRadps = 0.18f;

/*
 * vision.x_diff / y_diff 是视觉置零后的世界/雷达坐标。
 * 规划器输出 world_vx / world_vy，任务层再转换成底盘车体系 Vx / Vy。
 */
constexpr float kVisionXToPlannerX = 1.0f;
constexpr float kVisionYToPlannerY = 1.0f;

struct LinearShapeParams
{
    float max_speed_mps;
    float max_acc_mps2;
    float max_dec_mps2;
    float min_speed_mps;
};

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

void limitVectorToMax(float &vx, float &vy, float max_speed)
{
    if (max_speed <= 0.0f)
    {
        return;
    }

    const float speed = sqrtf(vx * vx + vy * vy);
    if (speed > max_speed)
    {
        const float scale = max_speed / speed;
        vx *= scale;
        vy *= scale;
    }
}

/* 向量整体限斜率，避免分轴限幅改变斜向运动方向。 */
void limitVectorDelta(float target_vx,
                      float target_vy,
                      float current_vx,
                      float current_vy,
                      float max_delta,
                      float *out_vx,
                      float *out_vy)
{
    const float dvx = target_vx - current_vx;
    const float dvy = target_vy - current_vy;
    const float dv = sqrtf(dvx * dvx + dvy * dvy);

    if (dv > max_delta && max_delta > 0.0f)
    {
        const float scale = max_delta / dv;
        *out_vx = current_vx + dvx * scale;
        *out_vy = current_vy + dvy * scale;
    }
    else
    {
        *out_vx = target_vx;
        *out_vy = target_vy;
    }
}

float slewScalarByDt(float target, float current, float acc_rate, float dec_rate, float dt_s)
{
    const float step = ((fabsf(target) > fabsf(current)) ? acc_rate : dec_rate) * dt_s;
    const float err = target - current;

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

float applyCommandFloor(float value, float target, float min_abs)
{
    if (min_abs <= 0.0f)
    {
        return value;
    }

    const float value_abs = fabsf(value);

    if (target == 0.0f)
    {
        return (value_abs < min_abs) ? 0.0f : value;
    }

    if (value_abs < min_abs)
    {
        return (target > 0.0f) ? min_abs : -min_abs;
    }

    return value;
}

void applyVectorCommandFloor(float &vx, float &vy, float target_vx, float target_vy, float min_abs)
{
    if (min_abs <= 0.0f)
    {
        return;
    }

    const float target_speed = sqrtf(target_vx * target_vx + target_vy * target_vy);
    const float current_speed = sqrtf(vx * vx + vy * vy);

    if (target_speed <= 0.000001f)
    {
        if (current_speed < min_abs)
        {
            vx = 0.0f;
            vy = 0.0f;
        }
        return;
    }

    if (current_speed >= min_abs)
    {
        return;
    }

    // 用向量整体抬到最小有效速度，避免 X/Y 分轴补偿改变移动方向。
    if (current_speed > 0.000001f)
    {
        const float scale = min_abs / current_speed;
        vx *= scale;
        vy *= scale;
    }
    else
    {
        const float scale = min_abs / target_speed;
        vx = target_vx * scale;
        vy = target_vy * scale;
    }
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
          vx_target_(0.0f),
          vy_target_(0.0f),
          wz_target_(0.0f),
          last_update_tick_(0U)
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
        wuqiqu.resetRoute();
        clearOutput();
        last_update_tick_ = HAL_GetTick();

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

    void startAtPrelimWeaponHead(uint8_t weapon_index)
    {
        startAt(0U);
        if (active_ != 0U)
        {
            if (wuqiqu.overrideFirstWaypointWithPrelimWeaponHead(weapon_index) == 0U)
            {
                active_ = 0U;
                finished_ = 1U;
                clearOutput();
            }
        }
    }

    void stop()
    {
        active_ = 0U;
        finished_ = 0U;
        clearOutput();
        last_update_tick_ = 0U;
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
        last_update_tick_ = HAL_GetTick();
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

    /* 下发到底盘的车体系目标速度，单位 m/s、rad/s。 */
    volatile float vx_target_;
    volatile float vy_target_;
    volatile float wz_target_;
    uint32_t last_update_tick_;

    void clearOutput()
    {
        vx_target_ = 0.0f;
        vy_target_ = 0.0f;
        wz_target_ = 0.0f;
    }

    LinearShapeParams getLinearShape() const
    {
        if (wuqiqu.getState() == WuqiquPathPlanner::STATE_FAST)
        {
            return {kFastLinearMaxMps, kFastLinearAccMps2, kFastLinearDecMps2, kFastMinLinearMps};
        }

        return {kSlowLinearMaxMps, kSlowLinearAccMps2, kSlowLinearDecMps2, kSlowMinLinearMps};
    }

    WuqiquPathPlanner::Pose buildPose() const
    {
        WuqiquPathPlanner::Pose pose = {};

        pose.x = kVisionXToPlannerX * vision.x_diff; // m，世界/雷达坐标 X
        pose.y = kVisionYToPlannerY * vision.y_diff; // m，世界/雷达坐标 Y
        pose.yaw = vision.angle_x * kDegToRad;       // rad
        pose.yaw_360 = vision.angle_x;               // deg

        /* 底盘当前速度来自运动学解算，单位 m/s */
        pose.car_speed_x = omni_chassis.now.Vx;
        pose.car_speed_y = omni_chassis.now.Vy;
        pose.omega = omni_chassis.now.Vz;

        /* 规划器 PD 使用世界系速度，因此把底盘当前车体系速度转回世界系。 */
        chassisToWorldVelocity(pose.car_speed_x,
                               pose.car_speed_y,
                               pose.yaw,
                               &pose.world_speed_x,
                               &pose.world_speed_y);

        return pose;
    }

    void updateOutput(const WuqiquPathPlanner::Output &output)
    {
        const LinearShapeParams shape = getLinearShape();

        /* 规划器输出 world 速度，底盘接管接口使用车体系速度。 */
        float vx_limited = 0.0f;
        float vy_limited = 0.0f;
        worldToChassisVelocity(output.world_vx_mps,
                               output.world_vy_mps,
                               vision.angle_x * kDegToRad,
                               &vx_limited,
                               &vy_limited);
        limitVectorToMax(vx_limited, vy_limited, shape.max_speed_mps);

        const float wz_limited = limitFloat(output.wz_radps, -kMaxAngularSpeedRadps, kMaxAngularSpeedRadps);

        const uint32_t now_tick = HAL_GetTick();
        float dt_s = kMinControlDtS;
        if (last_update_tick_ != 0U)
        {
            dt_s = static_cast<float>(now_tick - last_update_tick_) * 0.001f;
            dt_s = limitFloat(dt_s, kMinControlDtS, kMaxControlDtS);
        }
        last_update_tick_ = now_tick;

        const float target_speed = sqrtf(vx_limited * vx_limited + vy_limited * vy_limited);
        const float current_speed = sqrtf(vx_target_ * vx_target_ + vy_target_ * vy_target_);
        const float max_delta =
            ((target_speed > current_speed) ? shape.max_acc_mps2 : shape.max_dec_mps2) * dt_s;

        float vx_slewed = 0.0f;
        float vy_slewed = 0.0f;
        limitVectorDelta(vx_limited, vy_limited, vx_target_, vy_target_, max_delta, &vx_slewed, &vy_slewed);
        const float wz_slewed = slewScalarByDt(wz_limited, wz_target_, kAngularAccRadps2, kAngularDecRadps2, dt_s);

        float vx_cmd = vx_slewed;
        float vy_cmd = vy_slewed;
        applyVectorCommandFloor(vx_cmd, vy_cmd, vx_limited, vy_limited, shape.min_speed_mps);
        applyVectorCommandFloor(vx_cmd, vy_cmd, vx_limited, vy_limited, kMinLinearCommandMps);

        vx_target_ = vx_cmd;
        vy_target_ = vy_cmd;
        wz_target_ = applyCommandFloor(wz_slewed, wz_limited, kMinAngularCommandRadps);
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

extern "C" {

void wuqiqu_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        (void)g_wuqiqu_task.runOnce();
        osDelay(1);
    }
}

void WuqiquTask_Start(void)
{
    g_wuqiqu_task.start();
}

void WuqiquTask_StartAt(uint8_t waypoint_index)
{
    g_wuqiqu_task.startAt(waypoint_index);
}

void WuqiquTask_StartAtPrelimWeaponHead(uint8_t weapon_index)
{
    g_wuqiqu_task.startAtPrelimWeaponHead(weapon_index);
}

void WuqiquTask_Stop(void)
{
    g_wuqiqu_task.stop();
}

uint8_t WuqiquTask_RunOnce(void)
{
    return g_wuqiqu_task.runOnce();
}

uint8_t WuqiquTask_IsActive(void)
{
    return g_wuqiqu_task.isActive();
}

uint8_t WuqiquTask_IsFinished(void)
{
    return g_wuqiqu_task.isFinished();
}

float WuqiquTask_GetChassisVxTarget(float manual)
{
    return g_wuqiqu_task.getChassisVxTarget(manual);
}

float WuqiquTask_GetChassisVyTarget(float manual)
{
    return g_wuqiqu_task.getChassisVyTarget(manual);
}

float WuqiquTask_GetChassisVzTarget(float manual)
{
    return g_wuqiqu_task.getChassisVzTarget(manual);
}

void WuqiquTask_AdvanceToNext(void)
{
    g_wuqiqu_task.advanceToNext();
}

uint8_t WuqiquTask_IsAllFinished(void)
{
    return g_wuqiqu_task.isAllFinished();
}

float WuqiquTask_GetWaypointYawDeg(uint8_t waypoint_index)
{
    return g_wuqiqu_task.getWaypointYawDeg(waypoint_index);
}

}
