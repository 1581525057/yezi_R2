#ifndef _ROUTE_TASK_H
#define _ROUTE_TASK_H

#include "main.h"

enum Route_state
{
    PHASE_IDLE = 0,
    PHASE_VISION,
    FIRST_RELOCATION,         // 1 号方块重定位
    SECOND_RELOCATION,        // 2 号方块重定位
    THIRD_RELOCATION,         // 3 号方块重定位
    PHASE_STEP_UP,            // 上台阶
    PHASE_STEP_DOWN,          // 下台阶
    PHASE_PICK_KFS,           // 取 KFS
    PHASE_TURN_LEFT90,        // 转左 90 度
    PHASE_TURN_RIGHT90,       // 转右 90 度
    PHASE_TURN180,            // 转 180 度
    PHASE_GET_KFS_HEIGHT_200, // 取高200mm的KFS
    PHASE_GET_KFS_HEIGHT_400, // 取高400mm的KFS
    PHASE_GET_KFS_SHORT_200   // 取低200mm的KFS
};

class ROUTE_TASK
{
private:
    uint8_t flag_relocation;
    uint8_t relocation_number;
    uint16_t yaw_stable_count;
    uint8_t yaw_target_valid_;       // 当前转向阶段是否已经锁存相对 yaw 目标。
    int8_t last_turn_90_direction_;  // 根据雷达 yaw 分类的当前朝向：0 为 0 度，1 为 +90 度，-1 为 -90 度。
    uint8_t last_turn_180_;          // 根据雷达 yaw 分类的当前朝向是否为 180 度。
    float last_step_center_x_;       // 接线层记录的最近一次台阶中心 X 坐标。
    float last_step_center_y_;       // 接线层记录的最近一次台阶中心 Y 坐标。
    uint8_t last_step_center_valid_; // 最近一次台阶中心是否有效。
    uint8_t already_step_up_;        // 是否已经完成过一次上台阶，用于取 KFS 前是否预走 Xcm。

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
