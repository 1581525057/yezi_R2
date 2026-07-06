#ifndef _CONBAT_TASK_H
#define _CONBAT_TASK_H

#include <stdint.h>

#ifdef __cplusplus
#include <cstddef>
#include "path_follow.h"
#include "BR_McuBsplinePathGenerator.h"
#endif

typedef enum
{
    CONBAT_IDLE = 0,         // 空闲状态
    CONBAT_RAMP_UP,          // 上斜坡状态
    CONBAT_PLACE_KFS = 3,        // 放置 KFS 状态
    CONBAT_COMBINE = 5,          // 合体状态
    CONBAT_AVOID = 6             // 避让状态
} ConbatState;

#ifdef __cplusplus
class CONBAT_TASK
{
public:
    CONBAT_TASK();

    void reset(void);
    void runOnce(void);
    uint8_t isActive(void) const;
    uint8_t getChassisTarget(float manual_vx,
                             float manual_vy,
                             float manual_wz,
                             float *target_vx,
                             float *target_vy,
                             float *target_wz) const;
    void setKfsPlaceIndex(uint8_t index);
    uint8_t getKfsPlaceIndex(void) const;
    void setYawTarget(float yaw_degree);
    uint8_t getLiftSwitch(uint8_t manual_switch) const;
    float getLiftLinearSpeedTarget(float manual_target) const;
    // 清空放 KFS 激光检测状态，重新等待“挡住再松开”的完整过程。
    void resetKfsPlaceLaserFlag(void);
    // 左/右任意激光先被挡住、随后松开时，置位放 KFS 标志位。
    uint8_t updateKfsPlaceLaserFlag(void);

    ConbatState state;
    uint8_t conbat_start;
    uint8_t yaw_target_enabled;
    float yaw_target_degree;
    // 放 KFS 标志位：1 表示检测到激光经历过挡住后又松开。
    uint8_t kfs_place_laser_release_flag;
private:
    enum
    {
        CONBAT_GENERATE_PATH_MAX_POINTS = 256U
    };

    enum CombineStep
    {
        COMBINE_PRE_LIFT = 0,      // 阶段1：升降机构先到 1 档，完成后打开气缸。
        COMBINE_WAIT_CLIMB_HEIGHT, // 阶段2：切到 2 档并等待抬升高度真正到位。
        COMBINE_CLIMB_FORWARD,           // 阶段3：底盘不动，用前激光 ch2 驱动 2006 爬升。
        COMBINE_ZONE3_READY,             // 阶段4：发送九宫格预备机械臂命令。
        COMBINE_WAIT_FINAL_LIFT_HEIGHT,  // 阶段5-1：收气缸后等待升降实际回到 1 档高度。
        COMBINE_FINAL_FORWARD,           // 阶段5-2：1 档到位后，底盘按 ch2 车体前向靠近。
        COMBINE_WAIT_PLACE_HAND_DT35,    // 阶段5-3：等待前方 DT35 大于 10cm 后进入放手持 KFS。
        COMBINE_PLACE_HAND,              // 阶段5-4：等待手持 KFS 激光松开后发送放置命令。
        COMBINE_PLACE_LOWER_KFS          // 阶段5-5：等待车内 KFS 激光松开后发送放置命令。
    };
    enum PlaceKfsStep
    {
        PLACE_KFS_MOVE_TO_SELECTED = 0, // 跑到选定 KFS 放置点并精定位。
        PLACE_KFS_SEND_ACTION           // 发送机械臂放 KFS 命令。
    };

    PathFollower path_follower_;
    BRPathPoint generated_path_[CONBAT_GENERATE_PATH_MAX_POINTS];
    PathFollower::PathPoint generated_follow_path_[CONBAT_GENERATE_PATH_MAX_POINTS];
    ConbatState last_state_;
    uint8_t path_loaded_;
    uint8_t path_active_;
    uint8_t ramp_up_waiting_;
    PlaceKfsStep place_kfs_step_;
    CombineStep combine_step_;
    uint16_t kfs_place_stop_stable_count_;
    uint8_t kfs_place_index_;
    uint8_t kfs_place_precision_active_;
    uint8_t kfs_place_laser_blocked_;
    uint8_t combine_pre_lift_ready_;
    uint8_t combine_crossed_finish_height_;
    uint8_t combine_stable_count_;
    uint32_t combine_pre_lift_command_seq_;
    uint32_t combine_climb_lift_command_seq_;
    uint32_t combine_final_lift_command_seq_;
    uint8_t lift_switch_target_;
    float lift_linear_speed_target_;
    float path_vx_target_;
    float path_vy_target_;
    float path_wz_target_;

    void handleStateChanged(void);
    void clearPathOutput(void);
    uint8_t runRampUp(void);
    uint8_t runPlaceKfs(void);
    uint8_t runCombine(void);
    uint8_t runSelectKfsPlace(void);
    uint8_t loadGeneratedPathToGoal(const BRPathPose &goal,
                                    const BRPathControlPoint *middle_points,
                                    std::size_t middle_point_count,
                                    float max_vel_m_s = 0.0f,
                                    float max_acc_m_s2 = 0.0f);
    uint8_t load_follow_plan(void);
    static float normalizeYawDeg(float yaw_degree);
    static void conbat_position_p_speed(float x_err,
                                        float y_err,
                                        float kp,
                                        float max_vel,
                                        float max_acc,
                                        float dt_s,
                                        float last_vx,
                                        float last_vy,
                                        float *vx,
                                        float *vy);
};

extern CONBAT_TASK conbat_t;

extern "C"
{
#endif

    void conbat_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
