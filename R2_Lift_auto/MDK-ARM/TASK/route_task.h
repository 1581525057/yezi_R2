#ifndef _ROUTE_TASK_H
#define _ROUTE_TASK_H
#include "main.h"
enum Route_state
{
    PHASE_IDLE = 0,
    FIRST_RELOCATION,  // 第一次重定位
    SECOND_RELOCATION, // 第二次重定位
    PHASE_STEP_UP,     // 上台阶
    PHASE_STEP_DOWN,   // 下台阶
    PHASE_PICK_KFS     // 取KFS
};

class ROUTE_TASK
{
private:
    /* data */
public:
    Route_state state;
    ROUTE_TASK()
    {
        route_reset();
    }
    void route_reset();
    void meiling_route();

    uint8_t flag;
    uint8_t flag_relocation;
    uint8_t relocation_number;
};

#endif
