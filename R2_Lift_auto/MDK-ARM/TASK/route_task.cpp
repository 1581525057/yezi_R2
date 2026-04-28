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
#include "PID.h"
#include <math.h>

ROUTE_TASK route_t;
extern float yaw_target;
extern PID pid_yaw;
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

    MeilingTarget_t second_relocation = {
        .preset_id   = 0,
        .L_ref       = 222.0f,
        .R_ref       = 293.0f,
        .F_ref       = 221.8f,
        .tol_lat     = 6.0f,
        .tol_lon     = 6.0f,
        .timeout_ms  = 500000U,
        .sensor_mask = SENSOR_FRONT | SENSOR_LEFT,
    };
}

void ROUTE_TASK::route_reset()
{
    state             = PHASE_IDLE;
    flag_start        = 0;
    flag_relocation   = 0;
    flag_vision       = 0;
    relocation_number = 0;
    yaw_stable_count  = 0;
}

void ROUTE_TASK::vision_choice()
{
    // 等待视觉指令
    if (state != PHASE_VISION || flag_vision != 1)
        return;

    switch (vision.B) {
        case 9:
            // Vision command 9: run the lift-up sequence.
            state = PHASE_STEP_UP;
            break;

        case 1:
            // Vision command 1: turn left 90 degrees.
            yaw_stable_count = 0;
            state            = PHASE_TURN_LEFT90;
            break;

        case 2:
            // Vision command 2: turn right 90 degrees.
            yaw_stable_count = 0;
            state            = PHASE_TURN_RIGHT90;
            break;

        case 7:
            // Vision command 7: start the second relocation at the moment it is requested.
            if (relocation_number == 2) {
                meiling.start(second_relocation);
                state = SECOND_RELOCATION;
            }
            break;

        default:
            break;
    }

    // Clear the command after consuming it so a stale B value cannot retrigger.
    vision.B    = 0;
    flag_vision = 0;
}

void ROUTE_TASK::meiling_route()
{
    if (flag_start != 1)
        return;

    if (state == PHASE_IDLE)
        state = FIRST_RELOCATION;

    switch (state) {
        case FIRST_RELOCATION:
            if (relocation_number == 0) {
                // 第一次重定位
                meiling.start(first_relocation);
                relocation_number = 1;
            } else if (relocation_number == 1) {
                uint8_t relocation_result = meiling.update();

                if (relocation_result == MeilingLocator::SUCCESS) {
                    send_position_to_pc(1, 1, 0.80, -0.32, 0.0);
                    relocation_number = 2;
                    // First relocation is done; wait for a vision command.
                    state = PHASE_VISION;
                } else if (relocation_result == MeilingLocator::TIMEOUT) {
                    meiling.start(first_relocation);
                }
            }
            break;

        case SECOND_RELOCATION: {
            uint8_t relocation_result = 0;
            if (relocation_number == 2) {
                relocation_result = meiling.update();
            }

            if (relocation_result == MeilingLocator::SUCCESS) {
                send_position_to_pc(1, 1, 0.80, -0.32, 0.0);
                relocation_number = 3;
                // Second relocation is done; ask the vision PC for the next action.
                send_position_to_pc(1, 0, 0, 0, 0);
                state = PHASE_VISION;
            } else if (relocation_result == MeilingLocator::TIMEOUT) {
                meiling.start(second_relocation);
            }
            break;
        }

        case PHASE_STEP_UP:
            // lift_auto.start() is idempotent; update() advances the lift state machine.
            lift_auto.start();

            if (lift_auto.isFinished()) {
                lift_auto.stop();
                // Lift action is done; ask the vision PC for the next action.
                send_position_to_pc(1, 0, 0, 0, 0);
                state = PHASE_VISION;
            }
            break;

        case PHASE_TURN_LEFT90:
            yaw_target = 90;

            if (fabsf(pid_yaw.pid.Err) < 3.0f) {
                yaw_stable_count++;
            } else {
                yaw_stable_count = 0;
            }

            // Hold yaw error inside tolerance for 200 cycles before finishing.
            if (yaw_stable_count >= 200) {
                yaw_stable_count = 0;
                send_position_to_pc(1, 0, 0, 0, 0);
                state = PHASE_VISION;
            }
            break;

        case PHASE_TURN_RIGHT90:
            yaw_target = -90;

            if (fabsf(pid_yaw.pid.Err) < 3.0f) {
                yaw_stable_count++;
            } else {
                yaw_stable_count = 0;
            }

            // Hold yaw error inside tolerance for 200 cycles before finishing.
            if (yaw_stable_count >= 200) {
                yaw_stable_count = 0;
                send_position_to_pc(1, 0, 0, 0, 0);
                state = PHASE_VISION;
            }
            break;

        default:
            break;
    }
}

extern "C" uint8_t RouteTask_IsMeilingAreaActive(void)
{
    if (route_t.flag_start != 1U) {
        return 0U;
    }

    switch (route_t.state) {
        case FIRST_RELOCATION:
        case SECOND_RELOCATION:
        case PHASE_STEP_UP:
        case PHASE_TURN_LEFT90:
        case PHASE_TURN_RIGHT90:
            return 1U;
        default:
            return 0U;
    }
}

uint16_t flag_meiling = 0;
extern "C" void plan_route(void *argument)
{
    for (;;) {

        if (flag_meiling == 1) {
            route_t.route_reset();
        }
        route_t.vision_choice();
        route_t.meiling_route();
        lift_auto.update();
        osDelay(1);
    }
}
