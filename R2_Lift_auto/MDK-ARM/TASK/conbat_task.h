#ifndef _CONBAT_TASK_H
#define _CONBAT_TASK_H

#include <stdint.h>

#ifdef __cplusplus
#include "path_follow.h"
#include "BR_McuBsplinePathGenerator.h"
#endif

typedef enum
{
    CONBAT_IDLE = 0,         // 空闲状态
    CONBAT_RAMP_UP,          // 上斜坡状态
    CONBAT_PICK_KFS,         // 吸取 KFS 状态
    CONBAT_PLACE_KFS,        // 放置 KFS 状态
    CONBAT_SELECT_KFS_PLACE, // 选择 KFS 放置点位状态
    CONBAT_COMBINE,          // 合体占位状态
    CONBAT_AVOID             // 避让状态
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

    ConbatState state;
    uint8_t conbat_start;
    uint8_t yaw_target_enabled;
    float yaw_target_degree;

private:
    enum
    {
        CONBAT_GENERATE_PATH_MAX_POINTS = 256U
    };

    PathFollower path_follower_;
    BRPathPoint generated_path_[CONBAT_GENERATE_PATH_MAX_POINTS];
    PathFollower::PathPoint generated_follow_path_[CONBAT_GENERATE_PATH_MAX_POINTS];
    ConbatState last_state_;
    uint8_t path_loaded_;
    uint8_t path_active_;
    uint8_t kfs_place_index_;
    float path_vx_target_;
    float path_vy_target_;
    float path_wz_target_;

    void handleStateChanged(void);
    void clearPathOutput(void);
    uint8_t runRampUp(void);
    uint8_t runSelectKfsPlace(void);
    uint8_t loadGeneratedPathToGoal(const BRPathPose& goal,
                                    const BRPathControlPoint* middle_points,
                                    std::size_t middle_point_count);
    uint8_t load_follow_plan(void);
    static float normalizeYawDeg(float yaw_degree);
};

extern CONBAT_TASK conbat_t;

extern "C" {
#endif

void conbat_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
