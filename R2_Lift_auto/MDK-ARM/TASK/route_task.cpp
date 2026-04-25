#include "route_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "omni_chassis.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "lift_auto.h"
#include "mieling.h"
#include "plan_route.h"
#include "usart_task.h"

ROUTE_TASK route_t;

// x:0.80 y:-0.32 yaw:0.00
// f:291.8 l:357
namespace
{
    MeilingTarget_t first_relocation = {
        .preset_id   = 0,
        .L_ref       = 357.0f,
        .R_ref       = 662.0f,
        .F_ref       = 291.8f,
        .tol_lat     = 6.0f,
        .tol_lon     = 6.0f,
        .timeout_ms  = 500000U,
        .sensor_mask = SENSOR_FRONT | SENSOR_LEFT,
    };
}

void ROUTE_TASK::route_reset()
{
    state             = PHASE_IDLE;
    flag              = 0;
    flag_relocation   = 0;
    relocation_number = 0;
}

void ROUTE_TASK::meiling_route()
{
    if (flag != 1)
        return;

    if (state == PHASE_IDLE)
        state = FIRST_RELOCATION;

    switch (state) {
        case FIRST_RELOCATION:
            if (relocation_number == 0) {
                meiling.start(first_relocation);
                relocation_number = 1;
            } else if (relocation_number == 1) {
                uint8_t relocation_result = meiling.update();

                if (relocation_result == MeilingLocator::SUCCESS) {
                    send_position_to_pc(1, 1, 0.80, -0.32, 0.0);
                    relocation_number = 2;
                }
            } else {

                switch (vision.B) {
                    case 9: // 上中间那个台阶
                        state = PHASE_STEP_UP;
                        break;

                    default:
                        break;
                }
            }
            break;

        case PHASE_STEP_UP:
            lift_auto.start();

            if (lift_auto.isFinished()) {
                lift_auto.stop();
                route_reset();
            }
            break;

        default:
            break;
    }
}

extern "C" void plan_route(void *argument)
{
    for (;;) {
        route_t.meiling_route();
        lift_auto.update();
        osDelay(1);
    }
}
