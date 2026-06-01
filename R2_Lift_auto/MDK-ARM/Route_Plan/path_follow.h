#ifndef PATH_FOLLOW_H
#define PATH_FOLLOW_H

#include <stdint.h>
#include "main.h"
/*
 * 通用路径跟随器。
 *
 * 消费预制路径表 {vx, vy, x, y, theta}，输出全向底盘世界系速度指令 (vx, vy, wz)。
 * 算法来自 BR_PathFollowing，核心思路：
 *   1. 安全检查：偏离过远则急停
 *   2. 点切换：点积判断是否越过当前目标点
 *   3. 中间段：路径速度前馈 + 法向PD纠偏 + 航向角PD保持
 *   4. 终点段：先 XY 收敛到位，再 yaw 精定位（两阶段分离，避免互扰）
 */
class PathFollower
{
public:
    /* 路径点格式：{vx, vy, x, y, theta}，单位 mm/s、mm、rad。 */
    struct PathPoint
    {
        float vx;
        float vy;
        float x;
        float y;
        float theta;
    };

    /* 机器人位姿输入。 */
    struct Pose
    {
        float x;
        float y;
        float yaw;
    };

    /* 底盘速度输出。 */
    struct Output
    {
        float world_vx;
        float world_vy;
        float wz;
    };

    /* 状态机。 */
    enum State
    {
        STATE_IDLE = 0,
        STATE_RUNNING,
        STATE_FINAL_POSITION,
        STATE_FINAL_YAW,
        STATE_FINISHED,
        STATE_DEVIATED
    };

    PathFollower();

    /*
     * 加载路径表，重置所有状态，准备开始跟踪。
     *   path  — 路径点数组，首末点速度必须为 0（起止零速点）。
     *   count — 路径点数量，必须 >= 3（首点 + 至少一个中间点 + 末点）。
     */
    void loadPath(const PathPoint *path, uint16_t count);

    /*
     * 跟踪主循环，每帧调用一次。
     * 返回当前状态：RUNNING / FINISHED / DEVIATED。
     * 停止跟踪后需调用 reset() 才能重新开始。
     */
    State follow(const Pose &current_pose);

    void reset(void);

    const Output &getOutput(void) const;
    State getState(void) const;
    int getCurrentIndex(void) const;

    /*
     * 运行时可调参数，方便串口或调试器实时修改。
     * 声明为 static 成员，定义和默认值在 cpp 文件中。
     */
    static float SAFE_DISTANCE;        /* 安全距离阈值，超出则判定偏离。单位 mm。 */
    static float POINT_ALLOW_DIST;     /* 中间段点切换允许距离。单位 mm。 */
    static float FINAL_ALLOW_DIST;     /* 终点 XY 到位判定距离。单位 mm。 */
    static float FINAL_ALLOW_ANGLE;    /* 终点 yaw 到位判定角度。单位 rad。 */
    static float NORMAL_P;             /* 中间段法向纠偏 P 增益。 */
    static float NORMAL_D;             /* 中间段法向纠偏 D 增益。 */
    static float THETA_P;              /* 航向角 P 增益。 */
    static float THETA_D;              /* 航向角 D 增益。 */
    static float FINAL_XY_P;           /* 终点 XY 精定位 P 增益。 */
    static float FINAL_XY_MIN_VEL;     /* 终点 XY 最小速度。单位 mm/s。 */
    static float FINAL_XY_MAX_VEL;     /* 终点 XY 最大速度。单位 mm/s。 */
    static float FINAL_YAW_P;          /* 终点 yaw 精定位 P 增益。 */
    static float FINAL_YAW_MAX_VEL;    /* 终点 yaw 最大角速度。单位 rad/s。 */

private:
    static const uint16_t MAX_PATH_POINTS = 256U;
    static const uint16_t FINAL_XY_STABLE_COUNT = 80U;
    static const uint16_t FINAL_THETA_STABLE_COUNT = 80U;
    static const float FOLLOW_PI;

    /* 路径表拷贝到内部，不依赖外部指针生命周期。 */
    PathPoint path_[MAX_PATH_POINTS];
    uint16_t path_count_;

    int current_index_;
    State state_;

    /* 法向PD历史量。 */
    float last_normal_x_;
    float last_normal_y_;
    uint8_t vel_enter_flag_;

    /* 终点精定位状态。 */
    uint8_t on_final_flag_;
    uint16_t final_xy_stable_cnt_;
    uint16_t final_theta_stable_cnt_;
    float last_err_x_;
    float last_err_y_;
    float last_err_theta_;

    Output output_;

    /* 内部逻辑。 */
    uint8_t safeCheck(const PathPoint &target, const Pose &pose) const;
    void pointCalculate(const Pose &pose);
    void finPointCalculate(const Pose &pose);
    float normalizeAngle(float angle) const;
    static float safeSqrt(float value);
    static float safeInvSqrt(float value);
    static void limitVelocity(float &vx, float &vy, float max_vel);
};

extern PathFollower path_follow;

#endif
