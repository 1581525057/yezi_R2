#ifndef _ROUTE_TASK_H
#define _ROUTE_TASK_H

#include "main.h"

#ifdef __cplusplus
#include "path_follow.h"
#include "BR_McuBsplinePathGenerator.h"
#endif

enum Route_state
{
    PHASE_IDLE = 0,
    PHASE_VISION,

    FIRST_RELOCATION,         // 1区重定位 第一次重定位
    SECOND_RELOCATION,        // 第二次重定位
    THIRD_RELOCATION,         // 第三次重定位
    PHASE_FIND_KFS,           // 寻找KFS的位置
    PHASE_STEP_UP,            // 上台阶
    PHASE_STEP_DOWN,          // 下台阶
    PHASE_PICK_KFS,           // 取 KFS
    PHASE_TURN_LEFT90,        // 转左 90 度
    PHASE_TURN_RIGHT90,       // 转右 90 度
    PHASE_TURN180,            // 转 180 度
    PHASE_GET_KFS_HEIGHT_200, // 取高 200mm 的 KFS
    PHASE_GET_KFS_HEIGHT_400, // 取高 400mm 的 KFS
    PHASE_GET_KFS_SHORT_200,  // 取低 200mm 的 KFS4
    PHASE_GO_3
};

class ROUTE_TASK
{
private:
    enum
    {
        ROUTE_GENERATE_PATH_MAX_POINTS = 256U
    };

    uint8_t flag_relocation;
    uint8_t relocation_number;
    uint8_t relocation_position_sent_; // 是否已经同步过第一次重定位坐标。
    uint16_t relocation_stop_stable_count_; // 底盘速度连续达标计数。
    uint16_t yaw_stable_count;
    uint8_t yaw_target_valid_;       // 当前转向阶段是否已经锁存相对 yaw 目标。
    int8_t last_turn_90_direction_;  // 根据雷达 yaw 分类的当前朝向：0 为 0 度，1 为 +90 度，-1 为 -90 度。
    uint8_t last_turn_180_;          // 根据雷达 yaw 分类的当前朝向是否为 180 度。
    float last_step_center_x_;       // 接线层记录的最近一次台阶中心 X 坐标。
    float last_step_center_y_;       // 接线层记录的最近一次台阶中心 Y 坐标。
    uint8_t last_step_center_valid_; // 最近一次台阶中心是否有效。
    uint8_t already_step_up_;        // 是否已经完成过一次上台阶，用于取 KFS 前是否预走 Xcm。
    float pick_kfs_center_x_;        // 取 KFS 所在方块中心 X 坐标。
    float pick_kfs_center_y_;        // 取 KFS 所在方块中心 Y 坐标。
    uint8_t pick_kfs_center_valid_;  // 取 KFS 方块中心是否有效。
    float turn_final_yaw_;           // 当前转弯最终目标 yaw。
    uint8_t path_active_;            // 1 区跑点是否正在接管底盘速度。
    uint8_t path_loaded_;            // 当前阶段路径是否已经加载到跟随器。
    float path_vx_target_;           // 1 区跑点底盘 X 速度目标，单位 m/s。
    float path_vy_target_;           // 1 区跑点底盘 Y 速度目标，单位 m/s。
    float path_wz_target_;           // 1 区跑点底盘旋转速度目标，单位 rad/s。
    uint8_t find_kfs_positioning_;   // B 样条结束后是否正在做 KFS 终点精定位。
    uint16_t find_kfs_position_stable_count_; // KFS 终点精定位连续到位计数。
    uint16_t entrence_KFS;           // KFS的入口对应的KFS位置
    BRPathPoint generated_path_[ROUTE_GENERATE_PATH_MAX_POINTS];
    PathFollower::PathPoint generated_follow_path_[ROUTE_GENERATE_PATH_MAX_POINTS];
    void start_turn_target(float yaw_delta_deg);
    void clear_path_output(void);
    uint8_t load_follow_plan(void);
    uint8_t find_KFS1(void);
    uint8_t find_KFS2(void);
    uint8_t find_KFS3(void);
    uint8_t loadGeneratedPathToGoal(const BRPathPose &goal,
                                    const BRPathControlPoint *middle_points,
                                    std::size_t middle_point_count);
    uint8_t runFindKfsToGoal(uint8_t index); // 寻找 KFS：先跑 B 样条，结束后接终点精定位。
    uint8_t runFindKfsPositionCloseLoop(const BRPathPose &goal); // 对 KFS 终点做二维位置 P 闭环。
    static void route_position_p_speed(float x_err,
                                       float y_err,
                                       float kp,
                                       float max_vel,
                                       float max_acc,
                                       float dt_s,
                                       float last_vx,
                                       float last_vy,
                                       float *vx,
                                       float *vy);

public:
    Route_state state;
    ROUTE_TASK()
    {
        route_reset();
    }

    void route_reset();
    void meiling_route();
    void vision_choice();
    void update_number_KFS_by_cmd(); // 根据机械臂回传状态同步 KFS 数量。
    uint8_t getPathChassisTarget(float manual_vx,
                                 float manual_vy,
                                 float manual_wz,
                                 float *target_vx,
                                 float *target_vy,
                                 float *target_wz) const;

    uint8_t flag_start;
    uint8_t flag_vision; // 等待视觉数据。
    uint8_t number_KFS;  // 机械臂手上和车内的 KFS 总数量。
};

extern ROUTE_TASK route_t;

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t RouteTask_IsMeilingAreaActive(void);

#ifdef __cplusplus
}
#endif

#endif
