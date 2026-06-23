/*
 * path_follow.cpp — 通用路径跟随器
 *
 * 消费预制路径表 {vx, vy, x, y, theta}，输出全向底盘世界系速度 (vx, vy, wz)。
 *
 * 算法流程（来自 BR_PathFollowing.c）：
 *   1. 安全检查：当前位置离目标点过远 → STATE_DEVIATED 急停
 *   2. 跳过首点（起始零速点），从第 1 个运动点开始跟踪
 *   3. 中间段：点积判断是否越过当前目标 → 切换到下一点
 *      速度输出 = 路径前馈 + 法向PD纠偏，yaw 用 PD 保持
 *   4. 终点段：先 XY 位置收敛 → 再 yaw 精定位（两阶段分离避免互扰）
 *
 * 与 BR 原版区别：
 *   - BR 输出 (Vel, Dir, Omega) 给舵轮底盘
 *   - 本版输出 (vx, vy, wz) 给全向底盘，坐标系统一为世界系
 */

#include "path_follow.h"
#include <math.h>

PathFollower path_follow;

/* 运行时可调参数默认值，调试时可通过串口或调试器直接修改。 */
float PathFollower::SAFE_DISTANCE = 1200.0f;  /* mm */
float PathFollower::POINT_ALLOW_DIST = 25.0f; /* mm */
float PathFollower::FINAL_ALLOW_DIST = 35.0f; /* mm */
float PathFollower::NORMAL_P = 0.001f;
float PathFollower::NORMAL_D = 0.001f;
float PathFollower::THETA_P = 3.8f;
float PathFollower::THETA_D = 1.8f;
float PathFollower::FINAL_XY_P = 2.0f;
float PathFollower::FINAL_XY_MIN_VEL = 50.0f;  /* mm/s */
float PathFollower::FINAL_XY_MAX_VEL = 500.0f; /* mm/s */
const float PathFollower::FOLLOW_PI = 3.14159265358979323846f;

/* ============================== 构造与重置 ============================== */

PathFollower::PathFollower()
{
    path_count_ = 0U;
    reset();
}

void PathFollower::reset(void)
{
    current_index_ = 0;
    state_ = STATE_IDLE;
    vel_enter_flag_ = 0U;
    final_xy_stable_cnt_ = 0U;

    last_normal_x_ = 0.0f;
    last_normal_y_ = 0.0f;
    last_err_x_ = 0.0f;
    last_err_y_ = 0.0f;
    last_err_theta_ = 0.0f;

    output_.world_vx = 0.0f;
    output_.world_vy = 0.0f;
    output_.wz = 0.0f;
}

/* ============================== 路径加载 ============================== */

void PathFollower::loadPath(const PathPoint *path, uint16_t count)
{
    reset();

    if (path == (void *)0 || count < 3U || count > MAX_PATH_POINTS)
    {
        return;
    }

    /* 拷贝路径到内部缓存，避免外部数据被修改后影响跟踪。 */
    for (uint16_t i = 0; i < count; i++)
    {
        path_[i] = path[i];
    }
    path_count_ = count;
}

/* ============================== 跟踪主循环 ============================== */

PathFollower::State PathFollower::follow(const Pose &current_pose)
{
    /* 偏离状态：保持急停，等待外部 reset()。 */
    if (state_ == STATE_DEVIATED)
    {
        output_.world_vx = 0.0f;
        output_.world_vy = 0.0f;
        output_.wz = 0.0f;
        return state_;
    }

    /* 索引越界保护。 */
    if (current_index_ < 0 || current_index_ >= (int)path_count_)
    {
        reset();
        return state_;
    }

    /* 安全检查：偏离过远则急停。 */
    if (safeCheck(path_[current_index_], current_pose) != 0U)
    {
        reset();
        state_ = STATE_DEVIATED;
        return state_;
    }

    /* 首点只是起始零速点，跳过。 */
    if (current_index_ == 0)
    {
        current_index_++;
    }

    /* 中间段：还没到末点。 */
    if (current_index_ < ((int)path_count_ - 1))
    {
        state_ = STATE_RUNNING;
        pointCalculate(current_pose);
        return state_;
    }

    /* 终点段。 */
    finPointCalculate(current_pose);
    return state_;
}

/* ============================== 安全检查 ============================== */

/*
 * 判断当前位置是否偏离目标点过远。
 * 终点速度为 0 时允许 4 倍安全距离（最后一段容忍更大偏差）。
 * 返回 0 正常，1 偏离。
 */
uint8_t PathFollower::safeCheck(const PathPoint &target, const Pose &pose) const
{
    float safe_dist = SAFE_DISTANCE;

    /* 终点零速点放宽安全距离。 */
    if (target.vx == 0.0f && target.vy == 0.0f)
    {
        safe_dist *= 4.0f;
    }

    float dx = target.x - pose.x;
    float dy = target.y - pose.y;
    float dist = safeSqrt(dx * dx + dy * dy);

    return (dist > safe_dist) ? 1U : 0U;
}

/* ============================== 中间段跟随 ============================== */

/*
 * 中间路径点跟踪逻辑。
 *
 * 点切换：当满足以下任一条件时，切换到下一点：
 *   1. 进入当前目标点允许半径（POINT_ALLOW_DIST）
 *   2. 当前位置已经越过目标点（点积 < 0，避免掉头追旧点）
 *
 * 速度输出 = 路径前馈速度 + 法向PD纠偏速度
 *   - 前馈：直接取路径表的 vx/vy
 *   - 法向纠偏：将位置误差投影到路径切线方向，消掉切向分量，只保留法向偏差
 *     用 PD 控制器生成法向修正速度，增益随主速度自适应缩放
 *   - 航向角：PD 保持目标 yaw
 */
void PathFollower::pointCalculate(const Pose &pose)
{
    /* 点切换循环：一次可能跳过多个已到达的点。 */
    while (current_index_ < ((int)path_count_ - 1))
    {
        const PathPoint &target = path_[current_index_];
        const PathPoint &next = path_[current_index_ + 1];

        float err_x = target.x - pose.x;
        float err_y = target.y - pose.y;
        float dist_sq = err_x * err_x + err_y * err_y;

        /* 下一段方向向量。 */
        float seg_x = next.x - target.x;
        float seg_y = next.y - target.y;

        /* 点积 < 0 表示已越过当前目标点。 */
        float dot = err_x * seg_x + err_y * seg_y;

        if (dist_sq >= POINT_ALLOW_DIST * POINT_ALLOW_DIST && dot >= 0.0f)
        {
            break;
        }

        ++current_index_;
        vel_enter_flag_ = 0U;
    }

    /* 切点后如果已来到末点，交给终点逻辑。 */
    if (current_index_ >= ((int)path_count_ - 1))
    {
        finPointCalculate(pose);
        return;
    }

    const PathPoint &target = path_[current_index_];
    const PathPoint &prev = path_[current_index_ - 1];

    /* 位置误差。 */
    float err_x = target.x - pose.x;
    float err_y = target.y - pose.y;

    /* 路径切线方向（当前目标点指向前一目标点，即路径反方向）。 */
    float path_back_x = prev.x - target.x;
    float path_back_y = prev.y - target.y;
    float path_inv_len = safeInvSqrt(path_back_x * path_back_x + path_back_y * path_back_y);

    /* 前馈速度模长，用于法向增益自适应。 */
    float feedforward_inv_speed = safeInvSqrt(target.vx * target.vx + target.vy * target.vy);

    /* 法向纠偏向量计算。 */
    float normal_x = 0.0f;
    float normal_y = 0.0f;
    float normal_vel_x = 0.0f;
    float normal_vel_y = 0.0f;

    if (path_inv_len > 0.0f)
    {
        /*
         * 将位置误差投影到路径切线方向，得到切向分量的长度。
         * 用误差减去切向分量，剩余的就是法向偏差。
         */
        float shadow = fabsf((err_x * path_back_x + err_y * path_back_y) * path_inv_len);
        float proj_x = path_back_x * path_inv_len * shadow;
        float proj_y = path_back_y * path_inv_len * shadow;
        normal_x = err_x + proj_x;
        normal_y = err_y + proj_y;
    }

    uint8_t first_enter = (vel_enter_flag_ == 0U) ? 1U : 0U;

    if (feedforward_inv_speed > 0.0f)
    {
        /*
         * 法向PD纠偏，增益与前馈速度成反比。
         * 速度越快，同样的横向偏差需要更大的修正力。
         */
        float np = NORMAL_P / feedforward_inv_speed;
        float nd = NORMAL_D / feedforward_inv_speed;

        if (first_enter == 0U)
        {
            normal_vel_x = np * normal_x + nd * (normal_x - last_normal_x_);
            normal_vel_y = np * normal_y + nd * (normal_y - last_normal_y_);
        }
        else
        {
            normal_vel_x = np * normal_x;
            normal_vel_y = np * normal_y;
        }
    }

    last_normal_x_ = normal_x;
    last_normal_y_ = normal_y;

    /* 航向角 PD 控制。 */
    float err_theta = normalizeAngle(target.theta - pose.yaw);

    if (first_enter == 0U)
    {
        output_.wz = THETA_P * err_theta + THETA_D * (err_theta - last_err_theta_);
    }
    else
    {
        output_.wz = THETA_P * err_theta;
        vel_enter_flag_ = 1U;
    }

    last_err_theta_ = err_theta;

    /* 最终输出 = 路径前馈 + 法向纠偏。 */
    output_.world_vx = target.vx + normal_vel_x;
    output_.world_vy = target.vy + normal_vel_y;
}

/* ============================== 终点精定位 ============================== */

/*
 * 终点逻辑：直接完成，不做精定位。
 */
void PathFollower::finPointCalculate(const Pose &pose)
{
    (void)pose; // 参数保留以兼容调用接口
    reset();
    state_ = STATE_FINISHED;
}

/* ============================== 公开读取接口 ============================== */

const PathFollower::Output &PathFollower::getOutput(void) const
{
    return output_;
}

PathFollower::State PathFollower::getState(void) const
{
    return state_;
}

int PathFollower::getCurrentIndex(void) const
{
    return current_index_;
}

void PathFollower::worldToBody(float world_x, float world_y, float yaw_rad, float *body_x, float *body_y)
{
    if (body_x == (void *)0 || body_y == (void *)0)
    {
        return;
    }

    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);

    // 世界系转车体系，相当于按 -yaw 旋转二维向量。
    *body_x = cos_yaw * world_x + sin_yaw * world_y;
    *body_y = -sin_yaw * world_x + cos_yaw * world_y;
}

/* ============================== 工具函数 ============================== */

float PathFollower::normalizeAngle(float angle) const
{
    /* 将角度归一化到 [-pi, pi]，保证走最短旋转方向。 */
    while (angle > FOLLOW_PI)
    {
        angle -= 2.0f * FOLLOW_PI;
    }
    while (angle < -FOLLOW_PI)
    {
        angle += 2.0f * FOLLOW_PI;
    }
    return angle;
}

float PathFollower::safeSqrt(float value)
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }
    return sqrtf(value);
}

float PathFollower::safeInvSqrt(float value)
{
    if (value <= 0.000001f)
    {
        return 0.0f;
    }
    return 1.0f / sqrtf(value);
}

void PathFollower::limitVelocity(float &vx, float &vy, float max_vel)
{
    /* 二维速度限幅：超速时等比缩放，保持方向不变。 */
    float speed = sqrtf(vx * vx + vy * vy);

    if (speed > max_vel && speed > 0.001f)
    {
        float scale = max_vel / speed;
        vx *= scale;
        vy *= scale;
    }
}
