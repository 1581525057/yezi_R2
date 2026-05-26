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
extern Block_Vision block_vision_middle[10];
extern Block_Vision block_vision_climb[10];
// x:0.80 y:-0.32 yaw:0.00
// f:291.8 l:357
namespace
{
    MeilingTarget_t first_relocation = {
        .preset_id   = 0,
        .L_ref       = 2655.0f,
        .R_ref       = 0.0f,
        .F_ref       = 266.0f,
        .tol_lat     = 12.0f,
        .tol_lon     = 12.0f,
        .timeout_ms  = 500000U,
        .sensor_mask = SENSOR_FRONT | SENSOR_LEFT,
    };

    MeilingTarget_t second_relocation = {
        .preset_id   = 0,
        .L_ref       = 343.0f,
        .R_ref       = 0.0f,
        .F_ref       = 356.0f,
        .tol_lat     = 6.0f,
        .tol_lon     = 6.0f,
        .timeout_ms  = 500000U,
        .sensor_mask = SENSOR_FRONT | SENSOR_LEFT,
    };

    MeilingTarget_t third_relocation = {
        .preset_id   = 0,
        .L_ref       = 222.0f,
        .R_ref       = 292.0f,
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
    vision_command_clear();
}

void ROUTE_TASK::vision_choice()
{
    // 等待视觉指令
    if (state != PHASE_VISION)
        return;

    int cmd;
    if (vision_command_pop(&cmd) != 1U) {
        flag_vision = 0;
        return;
    }

    vision.B    = cmd;
    flag_vision = vision_command_has_pending();

    switch (cmd) {
        case 3: {
            // 视觉指令 9：执行上台阶动作。
            // 从方块队列取编号，查表设置雷达目标坐标
            int block_num = 0;
            vision_block_pop(&block_num);
            lift_auto.setStepUpBlockNum(block_num);
            lift_auto.setStepUpRadarTarget(
                block_vision_middle[block_num].x,
                block_vision_climb[block_num].x,
                block_vision_middle[block_num].y);
            state = PHASE_STEP_UP;
            break;
        }

        case 1:
            // 视觉指令 1：左转 90 度。
            yaw_stable_count = 0;
            state            = PHASE_TURN_LEFT90;
            break;

        case 2:
            // 视觉指令 2：右转 90 度。
            yaw_stable_count = 0;
            state            = PHASE_TURN_RIGHT90;
            break;

        case 7:
            // 视觉指令 7：按当前重定位次数启动下一段重定位。
            if (relocation_number == 2) {
                meiling.start(second_relocation);
                state = SECOND_RELOCATION;
            }

            if (relocation_number == 3) {
                meiling.start(third_relocation);
                state = THIRD_RELOCATION;
            }
            break;

        default:
            break;
    }
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
                    send_position_to_pc(1, 1,2.26, 1.56, 0.0);
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

        case THIRD_RELOCATION: {
            uint8_t relocation_result = 0;
            if (relocation_number == 3) {
                relocation_result = meiling.update();
            }

            if (relocation_result == MeilingLocator::SUCCESS) {
                // send_position_to_pc(1, 1, 0.80, -0.32, 0.0);
                relocation_number = 4;
                // Second relocation is done; ask the vision PC for the next action.

                state = PHASE_VISION;
            } else if (relocation_result == MeilingLocator::TIMEOUT) {
                meiling.start(second_relocation);
            }
            break;
        }

        case PHASE_STEP_UP:
            // 上台阶目标参数已在 vision_choice() 中配置。
            lift_auto.startStepUp();

            if (lift_auto.isStepUpFinished()) {
                lift_auto.stopStepUp();
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
        case THIRD_RELOCATION:
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
            flag_meiling = 0;
        }
        route_t.vision_choice();
        route_t.meiling_route();
        lift_auto.update();
        osDelay(1);
    }
}
