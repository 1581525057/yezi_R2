#include "wuqiqu.h"
#include <math.h>

WuqiquDebug_t wuqiqu_debug = {};

/*
 * 武器区路径规划器。
 * 路径点格式为：vx, vy, x, y, yaw。
 * vx/vy 是世界坐标系速度，单位 mm/s；x/y 是世界坐标系位置，单位 mm；yaw 单位 rad。
 */
namespace
{
    static const float WUQIQU_PI = 3.14159265358979323846f;

    /* 跟踪保护和到点阈值。 */
    static const float FOLLOWING_SAFE_DISTANCE = 1200.0f;
    static const float POINT_ALLOW_DISTANCE = 25.0f;
    static const float ALLOW_DISTANCE = 10.0f;
    static const float ALLOW_ANGLE = 0.017f;

    /* 终点需要连续稳定若干控制周期，避免视觉噪声导致提前结束。 */
    static const uint16_t FINAL_XY_STABLE_COUNT = 80U;
    static const uint16_t FINAL_THETA_STABLE_COUNT = 80U;

    /* 中间段法向修正参数：在路径给定速度的基础上修正横向偏差。 */
    static const float NORMAL_P = 0.001f;
    static const float NORMAL_D = 0.001f;

    /* 终点位置精修参数。 */
    static const float FINAL_POINT_P = 2.0f;
    static const float FINAL_POINT_MIN_VEL = 50.0f;
    static const float FINAL_POINT_MAX_VEL = 500.0f;

    /* 终点角度精修参数。 */
    static const float FINAL_DIFF_P = 2.5f;
    static const float FINAL_DIFF_MAX_VEL = 0.1f;

    /* 路径跟随过程中的 yaw 保持参数。 */
    static const float THETA_P = 3.8f;
    static const float THETA_D = 1.8f;

    /* 按二维速度模长限幅，同时保留原速度方向。 */
    float safeDistanceForPoint(const WuqiquPathPlanner::Point &point)
    {
        if (point.vx == 0.0f && point.vy == 0.0f)
        {
            return FOLLOWING_SAFE_DISTANCE * 4.0f;
        }

        return FOLLOWING_SAFE_DISTANCE;
    }

    void limitPlanarVelocity(float &vx, float &vy, float min_vel, float max_vel)
    {
        const float speed = sqrtf(vx * vx + vy * vy);

        if (speed <= 0.000001f)
        {
            return;
        }

        if (speed > max_vel)
        {
            const float scale = max_vel / speed;
            vx *= scale;
            vy *= scale;
        }
        else if (speed < min_vel)
        {
            const float scale = min_vel / speed;
            vx *= scale;
            vy *= scale;
        }
    }
}

/* 固定路径表：首点和末点速度为 0，中间点提供世界系速度前馈。 */
const WuqiquPathPlanner::Point WuqiquPathPlanner::action_path_[WuqiquPathPlanner::PATH_POINT_COUNT] = {
    {0.000f, 0.000f, 3.389f, 8.873f, 0.000f},
    {402.549f, 801.483f, 48.480f, 98.749f, 0.000f},
    {571.542f, 1136.839f, 94.184f, 189.746f, 0.000f},
    {701.050f, 1393.218f, 139.857f, 280.595f, 0.000f},
    {808.988f, 1606.422f, 184.948f, 370.205f, 0.000f},
    {824.129f, 1635.224f, 230.537f, 460.731f, 0.000f},
    {773.326f, 1533.281f, 276.012f, 550.962f, 0.000f},
    {718.829f, 1424.190f, 321.490f, 641.132f, 0.000f},
    {659.754f, 1306.197f, 366.956f, 731.211f, 0.000f},
    {594.977f, 1177.081f, 412.256f, 820.898f, 0.000f},
    {521.977f, 1031.872f, 457.627f, 910.658f, 0.000f},
    {436.832f, 862.859f, 502.977f, 1000.309f, 0.000f},
    {330.247f, 651.662f, 548.328f, 1089.889f, 0.000f},
    {0.000f, 0.000f, 608.644f, 1208.909f, 0.000f},
};

WuqiquPathPlanner wuqiqu;

WuqiquPathPlanner::WuqiquPathPlanner()
{
    reset();
}

int WuqiquPathPlanner::follow(const Pose &current_pose)
{
    if (state_ == STATE_DEVIATED)
    {
        output_.world_vx_set = 0.0f;
        output_.world_vy_set = 0.0f;
        output_.wz_set = 0.0f;
        wuqiqu_debug.state = (uint8_t)state_;
        wuqiqu_debug.deviation_flag = deviation_flag_;
        wuqiqu_debug.deviation_index = deviation_index_;
        wuqiqu_debug.pose_x = current_pose.x;
        wuqiqu_debug.pose_y = current_pose.y;
        wuqiqu_debug.pose_yaw = current_pose.yaw;
        wuqiqu_debug.out_vx = output_.world_vx_set;
        wuqiqu_debug.out_vy = output_.world_vy_set;
        wuqiqu_debug.out_wz = output_.wz_set;
        return 0;
    }

    /* 索引异常时复位，防止数组越界。 */
    if (current_path_index_ < 0 || current_path_index_ >= PATH_POINT_COUNT)
    {
        reset();
        return 0;
    }

    if (safeCheck(action_path_[current_path_index_], current_pose) != 0U)
    {
        const Point &deviation_target = action_path_[current_path_index_];
        const int error_index = current_path_index_;
        const float error_x = deviation_target.x - current_pose.x;
        const float error_y = deviation_target.y - current_pose.y;
        const float error_distance = safeSqrt(error_x * error_x + error_y * error_y);
        const float safe_distance = safeDistanceForPoint(deviation_target);

        reset();
        state_ = STATE_DEVIATED;
        deviation_flag_ = 1U;
        deviation_index_ = error_index;

        wuqiqu_debug.state = (uint8_t)state_;
        wuqiqu_debug.deviation_flag = deviation_flag_;
        wuqiqu_debug.current_index = error_index;
        wuqiqu_debug.deviation_index = deviation_index_;
        wuqiqu_debug.pose_x = current_pose.x;
        wuqiqu_debug.pose_y = current_pose.y;
        wuqiqu_debug.pose_yaw = current_pose.yaw;
        wuqiqu_debug.target_x = deviation_target.x;
        wuqiqu_debug.target_y = deviation_target.y;
        wuqiqu_debug.target_yaw = deviation_target.yaw;
        wuqiqu_debug.err_x = error_x;
        wuqiqu_debug.err_y = error_y;
        wuqiqu_debug.err_distance = error_distance;
        wuqiqu_debug.safe_distance = safe_distance;
        wuqiqu_debug.out_vx = output_.world_vx_set;
        wuqiqu_debug.out_vy = output_.world_vy_set;
        wuqiqu_debug.out_wz = output_.wz_set;
        return 0;
    }

    /* 首点只是起始零速点，真正跟踪从第 1 个运动点开始。 */
    if (current_path_index_ == 0)
    {
        current_path_index_++;
    }

    if (current_path_index_ < ((int)PATH_POINT_COUNT - 1))
    {
        state_ = STATE_RUNNING;
        pointCalculate(current_pose);
        return 0;
    }

    return finPointCalculate(current_pose);
}

void WuqiquPathPlanner::reset(void)
{
    /* 清空路径状态、历史误差和输出，准备下一次完整跟踪。 */
    current_path_index_ = 0;
    vel_calculate_enter_flag_ = 0U;
    final_point_xy_correct_flag_ = 0U;
    final_point_theta_correct_flag_ = 0U;
    on_final_point_flag_ = 0U;
    point_stable_count_ = 0U;
    final_xy_stable_count_ = 0U;
    final_theta_stable_count_ = 0U;
    state_ = STATE_IDLE;
    deviation_flag_ = 0U;
    deviation_index_ = -1;

    last_err_theta_normal_ = 0.0f;
    last_vector4_[0] = 0.0f;
    last_vector4_[1] = 0.0f;
    last_err_x_ = 0.0f;
    last_err_y_ = 0.0f;
    last_err_theta_final_ = 0.0f;

    output_.world_vx_set = 0.0f;
    output_.world_vy_set = 0.0f;
    output_.wz_set = 0.0f;

    wuqiqu_debug = WuqiquDebug_t();
    wuqiqu_debug.state = (uint8_t)state_;
    wuqiqu_debug.current_index = current_path_index_;
    wuqiqu_debug.deviation_index = deviation_index_;
}

const WuqiquPathPlanner::Output &WuqiquPathPlanner::getOutput(void) const
{
    return output_;
}

int WuqiquPathPlanner::getCurrentIndex(void) const
  {
    return current_path_index_;
}

WuqiquPathPlanner::PlannerState WuqiquPathPlanner::getState(void) const
{
    return state_;
}

uint8_t WuqiquPathPlanner::getDeviationFlag(void) const
{
    return deviation_flag_;
}

int WuqiquPathPlanner::getDeviationIndex(void) const
{
    return deviation_index_;
}

const WuqiquPathPlanner::Point *WuqiquPathPlanner::getPath(void) const
{
    return action_path_;
}

uint16_t WuqiquPathPlanner::getPathPointCount(void) const
{
    return PATH_POINT_COUNT;
}

float WuqiquPathPlanner::slewRateLimit(float target_vel, float current_vel, float max_acc_step, float max_dec_step) const
{
    /* 单轴斜率限制工具，目前主要保留给外部或后续调速使用。 */
    const float speed_err = target_vel - current_vel;

    if (speed_err > max_acc_step)
    {
        return current_vel + max_acc_step;
    }
    if (speed_err < -max_dec_step)
    {
        return current_vel - max_dec_step;
    }

    return target_vel;
}

uint8_t WuqiquPathPlanner::safeCheck(const Point &cur_tar_point, const Pose &current_pose) const
{
    const float safe_distance = safeDistanceForPoint(cur_tar_point);

    /* 终点速度为 0，允许更大的距离窗口，避免最后一段误判掉线。 */
    if (cur_tar_point.vx == 0.0f && cur_tar_point.vy == 0.0f)
    {
        (void)safe_distance;
    }

    const float distance_x = cur_tar_point.x - current_pose.x;
    const float distance_y = cur_tar_point.y - current_pose.y;
    const float distance = safeSqrt(distance_x * distance_x + distance_y * distance_y);

    return (distance > safe_distance) ? 1U : 0U;
}

void WuqiquPathPlanner::pointCalculate(const Pose &current_pose)
{
    /*
     * 中间路径点切换逻辑：
     * 1. 进入当前点允许半径，切到下一点；
     * 2. 或者已经越过当前点，也切到下一点，避免掉头追旧点。
     */
    while (current_path_index_ < ((int)PATH_POINT_COUNT - 1))
    {
        const Point &target = action_path_[current_path_index_];
        const Point &next_target = action_path_[current_path_index_ + 1];
        const float err_x = target.x - current_pose.x;
        const float err_y = target.y - current_pose.y;
        const float distance_square = err_x * err_x + err_y * err_y;
        const float next_segment_x = next_target.x - target.x;
        const float next_segment_y = next_target.y - target.y;
        const float dot_product = err_x * next_segment_x + err_y * next_segment_y;

        /* dot_product < 0 表示当前位置已经越过 target，继续追它会导致回拉。 */
        if (distance_square >= POINT_ALLOW_DISTANCE * POINT_ALLOW_DISTANCE &&
            dot_product >= 0.0f)
        {
            break;
        }

        ++current_path_index_;
        point_stable_count_ = 0U;
        vel_calculate_enter_flag_ = 0U;
    }

    /* 如果切点后已经来到末点，交给终点精定位逻辑。 */
    if (current_path_index_ >= ((int)PATH_POINT_COUNT - 1))
    {
        (void)finPointCalculate(current_pose);
        return;
    }

    const Point &target = action_path_[current_path_index_];
    const Point &previous_target = action_path_[current_path_index_ - 1];
    const Point &next_target = action_path_[current_path_index_ + 1];
    const float err_x = target.x - current_pose.x;
    const float err_y = target.y - current_pose.y;
    const float dot_product = err_x * (next_target.x - target.x) + err_y * (next_target.y - target.y);
    const float path_back_x = previous_target.x - target.x;
    const float path_back_y = previous_target.y - target.y;
    const float path_inv_len = safeInvSqrt(path_back_x * path_back_x + path_back_y * path_back_y);
    const float feedforward_inv_speed = safeInvSqrt(target.vx * target.vx + target.vy * target.vy);
    /* normal_vector 表示当前位置相对路径线段的法向偏差方向。 */
    float normal_vector_x = 0.0f;
    float normal_vector_y = 0.0f;
    float normal_vel_x = 0.0f;
    float normal_vel_y = 0.0f;

    if (path_inv_len > 0.0f)
    {
        /* 将位置误差投影到路径方向，再消掉切向分量，得到主要用于纠偏的法向量。 */
        const float shadow = fabsf((err_x * path_back_x + err_y * path_back_y) * path_inv_len);
        const float projection_x = path_back_x * path_inv_len * shadow;
        const float projection_y = path_back_y * path_inv_len * shadow;
        normal_vector_x = err_x + projection_x;
        normal_vector_y = err_y + projection_y;
    }

    const uint8_t first_enter = (vel_calculate_enter_flag_ == 0U) ? 1U : 0U;

    if (feedforward_inv_speed > 0.0f)
    {
        /* 速度越大，法向修正增益越大，减少高速段横向偏离。 */
        const float normal_p = NORMAL_P / feedforward_inv_speed;
        const float normal_d = NORMAL_D / feedforward_inv_speed;

        if (first_enter == 0U)
        {
            normal_vel_x = normal_p * normal_vector_x + normal_d * (normal_vector_x - last_vector4_[0]);
            normal_vel_y = normal_p * normal_vector_y + normal_d * (normal_vector_y - last_vector4_[1]);
        }
        else
        {
            normal_vel_x = normal_p * normal_vector_x;
            normal_vel_y = normal_p * normal_vector_y;
        }
    }

    last_vector4_[0] = normal_vector_x;
    last_vector4_[1] = normal_vector_y;

    const float err_theta = normalizeAngle(target.yaw - current_pose.yaw);

    /* 中间段持续按路径点 yaw 保持姿态。 */
    if (first_enter == 0U)
    {
        output_.wz_set = THETA_P * err_theta + THETA_D * (err_theta - last_err_theta_normal_);
    }
    else
    {
        output_.wz_set = THETA_P * err_theta;
        vel_calculate_enter_flag_ = 1U;
    }

    last_err_theta_normal_ = err_theta;
    /* 最终输出 = 路径表世界系速度前馈 + 法向纠偏速度。 */
    output_.world_vx_set = target.vx + normal_vel_x;
    output_.world_vy_set = target.vy + normal_vel_y;

    wuqiqu_debug.state = (uint8_t)state_;
    wuqiqu_debug.deviation_flag = deviation_flag_;
    wuqiqu_debug.on_final_point_flag = on_final_point_flag_;
    wuqiqu_debug.final_xy_stable_count = final_xy_stable_count_;
    wuqiqu_debug.final_theta_stable_count = final_theta_stable_count_;
    wuqiqu_debug.current_index = current_path_index_;
    wuqiqu_debug.deviation_index = deviation_index_;
    wuqiqu_debug.pose_x = current_pose.x;
    wuqiqu_debug.pose_y = current_pose.y;
    wuqiqu_debug.pose_yaw = current_pose.yaw;
    wuqiqu_debug.target_x = target.x;
    wuqiqu_debug.target_y = target.y;
    wuqiqu_debug.target_yaw = target.yaw;
    wuqiqu_debug.err_x = err_x;
    wuqiqu_debug.err_y = err_y;
    wuqiqu_debug.err_distance = safeSqrt(err_x * err_x + err_y * err_y);
    wuqiqu_debug.safe_distance = safeDistanceForPoint(target);
    wuqiqu_debug.dot_product = dot_product;
    wuqiqu_debug.normal_vector_x = normal_vector_x;
    wuqiqu_debug.normal_vector_y = normal_vector_y;
    wuqiqu_debug.normal_vel_x = normal_vel_x;
    wuqiqu_debug.normal_vel_y = normal_vel_y;
    wuqiqu_debug.err_theta = err_theta;
    wuqiqu_debug.out_vx = output_.world_vx_set;
    wuqiqu_debug.out_vy = output_.world_vy_set;
    wuqiqu_debug.out_wz = output_.wz_set;
}

int WuqiquPathPlanner::finPointCalculate(const Pose &current_pose)
{
    const Point &final_target = action_path_[current_path_index_];
    const float debug_err_x = final_target.x - current_pose.x;
    const float debug_err_y = final_target.y - current_pose.y;
    const float debug_err_theta = normalizeAngle(final_target.yaw - current_pose.yaw);

    if (on_final_point_flag_ == 0U)
    {
        state_ = STATE_FINAL_POSITION;
        /* 终点阶段先只修正 XY，姿态暂时不转，避免平移和旋转互相干扰。 */
        const float err_x = final_target.x - current_pose.x;
        const float err_y = final_target.y - current_pose.y;

        if ((err_x * err_x + err_y * err_y) < (ALLOW_DISTANCE * ALLOW_DISTANCE))
        {
            ++final_xy_stable_count_;
            output_.world_vx_set = 0.0f;
            output_.world_vy_set = 0.0f;

            if (final_xy_stable_count_ >= FINAL_XY_STABLE_COUNT)
            {
                on_final_point_flag_ = 1U;
                final_xy_stable_count_ = 0U;
            }

            wuqiqu_debug.state = (uint8_t)state_;
            wuqiqu_debug.deviation_flag = deviation_flag_;
            wuqiqu_debug.on_final_point_flag = on_final_point_flag_;
            wuqiqu_debug.final_xy_stable_count = final_xy_stable_count_;
            wuqiqu_debug.final_theta_stable_count = final_theta_stable_count_;
            wuqiqu_debug.current_index = current_path_index_;
            wuqiqu_debug.deviation_index = deviation_index_;
            wuqiqu_debug.pose_x = current_pose.x;
            wuqiqu_debug.pose_y = current_pose.y;
            wuqiqu_debug.pose_yaw = current_pose.yaw;
            wuqiqu_debug.target_x = final_target.x;
            wuqiqu_debug.target_y = final_target.y;
            wuqiqu_debug.target_yaw = final_target.yaw;
            wuqiqu_debug.err_x = debug_err_x;
            wuqiqu_debug.err_y = debug_err_y;
            wuqiqu_debug.err_distance = safeSqrt(debug_err_x * debug_err_x + debug_err_y * debug_err_y);
            wuqiqu_debug.safe_distance = safeDistanceForPoint(final_target);
            wuqiqu_debug.err_theta = debug_err_theta;
            wuqiqu_debug.out_vx = output_.world_vx_set;
            wuqiqu_debug.out_vy = output_.world_vy_set;
            wuqiqu_debug.out_wz = output_.wz_set;
            return 0;
        }

        final_xy_stable_count_ = 0U;

        float vel_x = FINAL_POINT_P * err_x;
        float vel_y = FINAL_POINT_P * err_y;
        limitPlanarVelocity(vel_x, vel_y, FINAL_POINT_MIN_VEL, FINAL_POINT_MAX_VEL);

        output_.wz_set = 0.0f;
        output_.world_vx_set = vel_x;
        output_.world_vy_set = vel_y;
    }
    else
    {
        /* XY 稳定后再单独修正 yaw。 */
        const float err_theta = normalizeAngle(final_target.yaw - current_pose.yaw);
        state_ = STATE_FINAL_YAW;

        if (fabsf(err_theta) < ALLOW_ANGLE)
        {
            output_.world_vx_set = 0.0f;
            output_.world_vy_set = 0.0f;
            output_.wz_set = 0.0f;

            ++final_theta_stable_count_;
            if (final_theta_stable_count_ >= FINAL_THETA_STABLE_COUNT)
            {
                reset();
                state_ = STATE_FINISHED;
                wuqiqu_debug.state = (uint8_t)state_;
                return 1;
            }

            wuqiqu_debug.state = (uint8_t)state_;
            wuqiqu_debug.deviation_flag = deviation_flag_;
            wuqiqu_debug.on_final_point_flag = on_final_point_flag_;
            wuqiqu_debug.final_xy_stable_count = final_xy_stable_count_;
            wuqiqu_debug.final_theta_stable_count = final_theta_stable_count_;
            wuqiqu_debug.current_index = current_path_index_;
            wuqiqu_debug.deviation_index = deviation_index_;
            wuqiqu_debug.pose_x = current_pose.x;
            wuqiqu_debug.pose_y = current_pose.y;
            wuqiqu_debug.pose_yaw = current_pose.yaw;
            wuqiqu_debug.target_x = final_target.x;
            wuqiqu_debug.target_y = final_target.y;
            wuqiqu_debug.target_yaw = final_target.yaw;
            wuqiqu_debug.err_x = debug_err_x;
            wuqiqu_debug.err_y = debug_err_y;
            wuqiqu_debug.err_distance = safeSqrt(debug_err_x * debug_err_x + debug_err_y * debug_err_y);
            wuqiqu_debug.safe_distance = safeDistanceForPoint(final_target);
            wuqiqu_debug.err_theta = debug_err_theta;
            wuqiqu_debug.out_vx = output_.world_vx_set;
            wuqiqu_debug.out_vy = output_.world_vy_set;
            wuqiqu_debug.out_wz = output_.wz_set;
            return 0;
        }

        final_theta_stable_count_ = 0U;

        float angle_vel = FINAL_DIFF_P * err_theta;
        if (fabsf(angle_vel) > FINAL_DIFF_MAX_VEL)
        {
            angle_vel = (angle_vel > 0.0f) ? FINAL_DIFF_MAX_VEL : -FINAL_DIFF_MAX_VEL;
        }

        output_.world_vx_set = 0.0f;
        output_.world_vy_set = 0.0f;
        output_.wz_set = angle_vel;
    }

    wuqiqu_debug.state = (uint8_t)state_;
    wuqiqu_debug.deviation_flag = deviation_flag_;
    wuqiqu_debug.on_final_point_flag = on_final_point_flag_;
    wuqiqu_debug.final_xy_stable_count = final_xy_stable_count_;
    wuqiqu_debug.final_theta_stable_count = final_theta_stable_count_;
    wuqiqu_debug.current_index = current_path_index_;
    wuqiqu_debug.deviation_index = deviation_index_;
    wuqiqu_debug.pose_x = current_pose.x;
    wuqiqu_debug.pose_y = current_pose.y;
    wuqiqu_debug.pose_yaw = current_pose.yaw;
    wuqiqu_debug.target_x = final_target.x;
    wuqiqu_debug.target_y = final_target.y;
    wuqiqu_debug.target_yaw = final_target.yaw;
    wuqiqu_debug.err_x = debug_err_x;
    wuqiqu_debug.err_y = debug_err_y;
    wuqiqu_debug.err_distance = safeSqrt(debug_err_x * debug_err_x + debug_err_y * debug_err_y);
    wuqiqu_debug.safe_distance = safeDistanceForPoint(final_target);
    wuqiqu_debug.dot_product = 0.0f;
    wuqiqu_debug.normal_vector_x = 0.0f;
    wuqiqu_debug.normal_vector_y = 0.0f;
    wuqiqu_debug.normal_vel_x = output_.world_vx_set;
    wuqiqu_debug.normal_vel_y = output_.world_vy_set;
    wuqiqu_debug.err_theta = debug_err_theta;
    wuqiqu_debug.out_vx = output_.world_vx_set;
    wuqiqu_debug.out_vy = output_.world_vy_set;
    wuqiqu_debug.out_wz = output_.wz_set;

    return 0;
}

float WuqiquPathPlanner::normalizeAngle(float angle) const
{
    /* 将角度误差归一化到 [-pi, pi]，保证走最短旋转方向。 */
    while (angle > WUQIQU_PI)
    {
        angle -= 2.0f * WUQIQU_PI;
    }
    while (angle < -WUQIQU_PI)
    {
        angle += 2.0f * WUQIQU_PI;
    }

    return angle;
}

float WuqiquPathPlanner::safeSqrt(float value) const
{
    /* 避免传入负数或极小异常值造成 sqrtf 问题。 */
    if (value <= 0.0f)
    {
        return 0.0f;
    }

    return sqrtf(value);
}

float WuqiquPathPlanner::safeInvSqrt(float value) const
{
    /* 求 1 / sqrt(value)，过小时返回 0 避免除零。 */
    if (value <= 0.000001f)
    {
        return 0.0f;
    }

    return 1.0f / sqrtf(value);
}
