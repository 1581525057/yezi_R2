#include "route_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "omni_chassis.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "lift_step_up.h"
#include "mieling.h"
#include "usart_task.h"
#include "PID.h"
#include <math.h>
#include "lift_step_down.h"

ROUTE_TASK route_t;
extern float yaw_target;
extern PID pid_yaw;
extern Block_Vision block_vision_middle[10];
extern Block_Vision block_vision_climb[10];

extern Block_Vision block_vision_down_pre[13];
extern Block_Vision block_vision_down[10];

namespace
{
    MeilingTarget_t first_relocation = {
        .preset_id   = 0,
        .L_ref       = 343.0f,
        .R_ref       = 0.0f,
        .F_ref       = 356.0f,
        .tol_lat     = 6.0f,
        .tol_lon     = 6.0f,
        .timeout_ms  = 500000U,
        .sensor_mask = SENSOR_FRONT | SENSOR_LEFT,
    };

    MeilingTarget_t second_relocation = {
        .preset_id   = 0,
        .L_ref       = 2692.0f,
        .R_ref       = 0.0f,
        .F_ref       = 217.0f,
        .tol_lat     = 10.0f,
        .tol_lon     = 10.0f,
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

        case 0:

            state = FIRST_RELOCATION;
            break;
        case 1:
            state = SECOND_RELOCATION;
            break;
        case 2:
            state = THIRD_RELOCATION;
            break;

        case 3: {
            // 视觉指令 3：执行上台阶动作。
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

        case 5: {
            // 从方块队列取编号，查表设置雷达目标坐标
            int block_num = 0;
            vision_block_pop(&block_num);
            lift_step_down.setStepDownBlockNum(block_num);
            lift_step_down.setStepDownRadarTarget(
                block_vision_down_pre[block_num].x,
                block_vision_down[block_num].x,
                block_vision_middle[block_num].x,
                block_vision_middle[block_num].y);
        }

        case 7:
            // 视觉指令 7：左转 90 度。
            yaw_stable_count = 0;
            state            = PHASE_TURN_LEFT90;
            break;

        case 8:
            // 视觉指令 8：右转 90 度。
            yaw_stable_count = 0;
            state            = PHASE_TURN_RIGHT90;
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
        state = PHASE_VISION;

    switch (state) {
        case FIRST_RELOCATION:
            if (relocation_number == 0) {
                // 第一次重定位
                meiling.start(first_relocation);
                relocation_number = 1;
            } else if (relocation_number == 1) {
                uint8_t relocation_result = meiling.update();

                if (relocation_result == MeilingLocator::SUCCESS) {
                    send_position_to_pc(1, 1, -0.12, -1.47, 0.0);
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

uint16_t flag_meiling   = 0;
uint16_t flag_step_down = 0;
extern "C" void plan_route(void *argument)
{
    lift_step_down.setStepDownRadarTarget(0.64, 0.05, -0.19, -1.43);
    for (;;) {

        if (flag_meiling == 1) {
            route_t.route_reset();
            flag_meiling = 0;
        }
        if (flag_step_down == 1) {
            lift_step_down.update();
        }
        if (flag_step_down == 2) {
            lift_step_down.stopStepDown();
        }

        route_t.vision_choice();
        route_t.meiling_route();
        lift_auto.update();
        osDelay(1);
    }
}
