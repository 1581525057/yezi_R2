#ifndef _ROUTE_TASK_H
#define _ROUTE_TASK_H

#include "main.h"

enum Route_state {
    PHASE_IDLE = 0,
    PHASE_VISION,
    FIRST_RELOCATION,   // 第一次重定位
    SECOND_RELOCATION,  // 第二次重定位
    PHASE_STEP_UP,      // 上台阶
    PHASE_STEP_DOWN,    // 下台阶
    PHASE_PICK_KFS,     // 取KFS
    PHASE_TURN_LEFT90,  // 转左90°
    PHASE_TURN_RIGHT90, // 转右90°
    PHASE_TURN180
};

class ROUTE_TASK
{
private:
    /* data */

    uint8_t flag_relocation;
    uint8_t relocation_number;
    uint16_t yaw_stable_count;

public:
    Route_state state;
    ROUTE_TASK()
    {
        route_reset();
    }
    void route_reset();
    void meiling_route();
    void vision_choice();

    uint8_t flag_start;
    uint8_t flag_vision; // 等待视觉数据s
};

extern ROUTE_TASK route_t;

#ifdef __cplusplus
extern "C" {
#endif

uint8_t RouteTask_IsMeilingAreaActive(void);

#ifdef __cplusplus
}
#endif

#endif
