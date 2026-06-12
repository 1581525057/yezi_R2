#include "wuqiqu.h"
#include "omni_chassis.h"
#include "usart_task.h"
#include <math.h>
#include <stdint.h>

/*
 * 武器区路径任务接入层。
 * 负责把全局 vision 当前位姿转换为规划器 Pose，
 * 再把规划器输出的世界系速度转换成底盘车体系速度目标。
 */
namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kDegToRad = kPi / 180.0f;
    constexpr float kMmToM = 0.001f;
    constexpr float kMToMm = 1000.0f;

    /* 下发给底盘的线速度和角速度上限。 */
    constexpr float kMaxLinearSpeedMps = 2.0f;
    constexpr float kMaxAngularSpeedRadps = 0.8f;

    /* 每个控制周期允许的速度变化量，用于让目标速度平滑变化。 */
    constexpr float kLinearAccStepMps = 0.008f;
    constexpr float kLinearDecStepMps = 0.012f;
    constexpr float kAngularAccStepRadps = 0.006f;
    constexpr float kAngularDecStepRadps = 0.010f;

    /*
     * 视觉坐标到规划坐标的符号映射。
     * vision.x_diff / vision.y_diff 单位为 m，vision.angle_x 单位为 degree。
     */
    constexpr float kVisionXToPlanner = 1.0f;
    constexpr float kVisionYToPlanner = 1.0f;
    constexpr float kVisionYawToPlanner = 1.0f;

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

    /* 将角度归一化到 [-pi, pi]。 */
    float normalizeAngle(float angle)
    {
        while (angle > kPi)
        {
            angle -= 2.0f * kPi;
        }
        while (angle < -kPi)
        {
            angle += 2.0f * kPi;
        }
        return angle;
    }

    /* 单轴斜率限制：加速和减速可以设置不同步长。 */
    float slewRateLimit(float target, float current, float acc_step, float dec_step)
    {
        const float err = target - current;

        if (err > acc_step)
        {
            return current + acc_step;
        }
        if (err < -dec_step)
        {
            return current - dec_step;
        }
        return target;
    }

    /* 武器区路径任务状态机，封装启动、停止、单周期更新和底盘目标输出。 */
    class WuqiquTask
    {
    public:
        WuqiquTask()
            : active_(0U),
              vx_target_(0.0f),
              vy_target_(0.0f),
              wz_target_(0.0f),
              origin_x_mm_(0.0f),
              origin_y_mm_(0.0f),
              origin_yaw_deg_(0.0f)
        {
        }

        void start()
        {
            /* 捕获启动瞬间的视觉位姿，当前实现使用相对启动点的坐标跟踪路径。 */
            captureVisionOrigin();
            wuqiqu.reset();
            clearOutput();
            active_ = 1U;
        }

        void stop()
        {
            /* 停止任务时立即清空速度输出，避免底盘继续执行旧指令。 */
            active_ = 0U;
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
            const WuqiquPathPlanner::Pose pose = buildPose();
            const int finished = wuqiqu.follow(pose);
            updateOutput(wuqiqu.getOutput(), pose.yaw);

            if (finished != 0)
            {
                stop();
                return 1U;
            }

            return 0U;
        }

        uint8_t isActive() const
        {
            return active_;
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
        uint8_t active_;

        /* 下发到底盘的车体系目标速度，单位 m/s、rad/s。 */
        float vx_target_;
        float vy_target_;
        float wz_target_;

        /* 任务启动瞬间的视觉位姿，用于把视觉世界坐标转换为相对坐标。 */
        float origin_x_mm_;
        float origin_y_mm_;
        float origin_yaw_deg_;

        void clearOutput()
        {
            vx_target_ = 0.0f;
            vy_target_ = 0.0f;
            wz_target_ = 0.0f;
        }

        float visionPlannerXmm() const
        {
            /* vision.x_diff 是世界系当前位置 X，单位 m；规划器使用 mm。 */
            return kVisionXToPlanner * vision.x_diff * kMToMm;
        }

        float visionPlannerYmm() const
        {
            /* vision.y_diff 是世界系当前位置 Y，单位 m；符号由 kVisionYToPlanner 统一管理。 */
            return kVisionYToPlanner * vision.y_diff * kMToMm;
        }

        float visionPlannerYawDeg() const
        {
            /* vision.angle_x 是当前 yaw，单位 degree；符号由 kVisionYawToPlanner 统一管理。 */
            return kVisionYawToPlanner * vision.angle_x;
        }

        void captureVisionOrigin()
        {
            /*
             * 注意：这里会把后续 pose 转换为“相对启动点”的坐标。
             * 如果路径点本身就是绝对世界坐标，应去掉 buildPose() 中对 origin 的相减。
             */
            origin_x_mm_ = visionPlannerXmm();
            origin_y_mm_ = visionPlannerYmm();
            origin_yaw_deg_ = visionPlannerYawDeg();
        }

        WuqiquPathPlanner::Pose buildPose() const
        {
            WuqiquPathPlanner::Pose pose = {};

            /* yaw 使用相对启动时刻的角度，并转换为规划器使用的 rad。 */
            const float yaw_deg = visionPlannerYawDeg() - origin_yaw_deg_;
            const float yaw_rad = normalizeAngle(yaw_deg * kDegToRad);
            const float cos_yaw = cosf(yaw_rad);
            const float sin_yaw = sinf(yaw_rad);

            /* 底盘当前速度来自运动学解算，单位 m/s；规划器内部统一用 mm/s。 */
            const float car_speed_x_mmps = omni_chassis.now.Vx * kMToMm;
            const float car_speed_y_mmps = omni_chassis.now.Vy * kMToMm;

            /* 当前实现给规划器的是相对启动点的位置。 */
            pose.x = visionPlannerXmm() - origin_x_mm_;
            pose.y = visionPlannerYmm() - origin_y_mm_;
            pose.yaw = yaw_rad;
            pose.yaw_360 = yaw_deg;
            pose.car_speed_x = car_speed_x_mmps;
            pose.car_speed_y = car_speed_y_mmps;

            /* 根据当前 yaw 将车体系速度估算为世界系速度，便于后续扩展使用。 */
            pose.world_speed_x = cos_yaw * car_speed_x_mmps - sin_yaw * car_speed_y_mmps;
            pose.world_speed_y = sin_yaw * car_speed_x_mmps + cos_yaw * car_speed_y_mmps;
            pose.omega = omni_chassis.now.Vz;

            return pose;
        }

        void updateOutput(const WuqiquPathPlanner::Output &output, float yaw_rad)
        {
            const float cos_yaw = cosf(yaw_rad);
            const float sin_yaw = sinf(yaw_rad);

            /* 规划器输出世界系速度，底盘接口需要车体系速度。 */
            const float body_vx_mps = (cos_yaw * output.world_vx_set + sin_yaw * output.world_vy_set) * kMmToM;
            const float body_vy_mps = (-sin_yaw * output.world_vx_set + cos_yaw * output.world_vy_set) * kMmToM;

            float vx_limited = body_vx_mps;
            float vy_limited = body_vy_mps;
            const float linear_speed = sqrtf(vx_limited * vx_limited + vy_limited * vy_limited);

            if (linear_speed > kMaxLinearSpeedMps && linear_speed > 0.000001f)
            {
                /* 对平面速度整体缩放限幅，保留速度方向。 */
                const float scale = kMaxLinearSpeedMps / linear_speed;
                vx_limited *= scale;
                vy_limited *= scale;
            }

            const float wz_limited = limitFloat(output.wz_set, -kMaxAngularSpeedRadps, kMaxAngularSpeedRadps);

            /* 目标速度再经过斜率限制，降低底盘指令突变。 */
            vx_target_ = slewRateLimit(vx_limited, vx_target_, kLinearAccStepMps, kLinearDecStepMps);
            vy_target_ = slewRateLimit(vy_limited, vy_target_, kLinearAccStepMps, kLinearDecStepMps);
            wz_target_ = slewRateLimit(wz_limited, wz_target_, kAngularAccStepRadps, kAngularDecStepRadps);
        }
    };

    WuqiquTask wuqiqu_task;
}

extern "C" void WuqiquTask_Start(void)
{
    wuqiqu_task.start();
}

extern "C" void WuqiquTask_Stop(void)
{
    wuqiqu_task.stop();
}

extern "C" uint8_t WuqiquTask_RunOnce(void)
{
    return wuqiqu_task.runOnce();
}

extern "C" uint8_t WuqiquTask_IsActive(void)
{
    return wuqiqu_task.isActive();
}

extern "C" float WuqiquTask_GetChassisVxTarget(float manual)
{
    return wuqiqu_task.getChassisVxTarget(manual);
}

extern "C" float WuqiquTask_GetChassisVyTarget(float manual)
{
    return wuqiqu_task.getChassisVyTarget(manual);
}

extern "C" float WuqiquTask_GetChassisVzTarget(float manual)
{
    return wuqiqu_task.getChassisVzTarget(manual);
}
