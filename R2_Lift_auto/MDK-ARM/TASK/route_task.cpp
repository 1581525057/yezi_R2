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
extern Block_Vision block_vision_middle[13];

namespace
{
    MeilingTarget_t first_relocation = {
        .preset_id   = 0,
        .L_ref       = 335.0f,
        .R_ref       = 0.0f,
        .F_ref       = 356.0f,
        .tol_lat     = 6.0f,
        .tol_lon     = 6.0f,
        .timeout_ms  = 500000U,
        .sensor_mask = SENSOR_FRONT | SENSOR_LEFT,
    };

    MeilingTarget_t second_relocation = {
        .preset_id   = 0,
        .L_ref       = 2630.0f,
        .R_ref       = 0.0f,
        .F_ref       = 300.0f,
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

    static float normalize_yaw_deg(float yaw_deg)
    {
        while (yaw_deg > 180.0f) {
            yaw_deg -= 360.0f;
        }
        while (yaw_deg < -180.0f) {
            yaw_deg += 360.0f;
        }
        return yaw_deg;
    }

    static void set_last_turn_flags_by_radar_yaw(float yaw_deg, int8_t *turn_90_direction, uint8_t *turn_180)
    {
        const float yaw = normalize_yaw_deg(yaw_deg);

        *turn_90_direction = 0;
        *turn_180          = 0U;

        if (yaw >= 45.0f && yaw < 135.0f) {
            *turn_90_direction = 1;
        } else if (yaw <= -45.0f && yaw > -135.0f) {
            *turn_90_direction = -1;
        } else if (yaw >= 135.0f || yaw <= -135.0f) {
            *turn_180 = 1U;
        }
    }

}

void ROUTE_TASK::route_reset()
{
    state                   = PHASE_IDLE;
    flag_start              = 0;
    flag_relocation         = 0;
    flag_vision             = 0;
    relocation_number       = 0;
    yaw_stable_count        = 0;
    yaw_target_valid_       = 0U;
    last_turn_90_direction_ = 0;
    last_turn_180_          = 0U;
    last_step_center_x_     = 0.0f;
    last_step_center_y_     = 0.0f;
    last_step_center_valid_ = 0U;
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
            float middle_x = block_vision_middle[block_num].x;
            float middle_y = block_vision_middle[block_num].y;
            lift_auto.setStepUpBlockNum(block_num);
            lift_auto.setStepUpRadarClimbDirection(last_turn_90_direction_);
            lift_auto.setStepUpRadarTarget(middle_x, middle_y);
            last_step_center_x_     = middle_x;
            last_step_center_y_     = middle_y;
            last_step_center_valid_ = 1U;
            state                   = PHASE_STEP_UP;
            break;
        }

        case 5: {
            // 从方块队列取编号，查表设置雷达目标坐标
            int block_num = 0;
            vision_block_pop(&block_num);
            float finish_x = block_vision_middle[block_num].x;
            float finish_y = block_vision_middle[block_num].y;
            float prepare_base_x = finish_x;
            float prepare_base_y = finish_y;
            if (last_step_center_valid_ != 0U) {
                // 下台阶准备点基准来自接线层记录的当前台阶中心，不能由 LiftStepDown 自己猜。
                prepare_base_x = last_step_center_x_;
                prepare_base_y = last_step_center_y_;
            }
            lift_step_down.setStepDownBlockNum(block_num);
            lift_step_down.setStepDownRadarTarget(
                prepare_base_x,
                prepare_base_y,
                finish_x,
                finish_y,
                (last_turn_90_direction_ > 0) ? 1U : 0U,
                (last_turn_90_direction_ < 0) ? 1U : 0U,
                last_turn_180_);
            last_step_center_x_     = finish_x;
            last_step_center_y_     = finish_y;
            last_step_center_valid_ = 1U;
            state                   = PHASE_STEP_DOWN;
            break;
        }

        case 7:
            // 视觉指令 7：左转 90 度。
            yaw_stable_count  = 0;
            yaw_target_valid_ = 0U;
            state             = PHASE_TURN_LEFT90;
            break;

        case 8:
            // 视觉指令 8：右转 90 度。
            yaw_stable_count  = 0;
            yaw_target_valid_ = 0U;
            state             = PHASE_TURN_RIGHT90;
            break;

        case 9:
            // 视觉指令 9：转 180 度。
            yaw_stable_count  = 0;
            yaw_target_valid_ = 0U;
            state             = PHASE_TURN180;
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

            break;

        case SECOND_RELOCATION: {
            if (relocation_number == 0) {
                // 第一次重定位
                meiling.start(second_relocation);
                relocation_number = 1;
            } else if (relocation_number == 1) {
                uint8_t relocation_result = meiling.update();

                if (relocation_result == MeilingLocator::SUCCESS) {
                    relocation_number = 2;
                    // First relocation is done; wait for a vision command.
                    state = PHASE_VISION;
                } else if (relocation_result == MeilingLocator::TIMEOUT) {
                    meiling.start(second_relocation);
                }
            }
        }

        case THIRD_RELOCATION: {
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

        case PHASE_STEP_DOWN:
            // 下台阶目标参数已在 vision_choice() 中配置。
            lift_step_down.startStepDown();

            if (lift_step_down.isStepDownFinished()) {
                lift_step_down.stopStepDown();
                state = PHASE_VISION;
            }
            break;

        case PHASE_TURN_LEFT90:
            if (yaw_target_valid_ == 0U) {
                yaw_target        = normalize_yaw_deg(vision.angle_x + 90.0f);
                yaw_stable_count  = 0;
                yaw_target_valid_ = 1U;
            }

            if (fabsf(pid_yaw.pid.Err) < 3.0f) {
                yaw_stable_count++;
            } else {
                yaw_stable_count = 0;
            }

            // yaw 误差连续稳定 200 个周期后，认为本次左转 90 度完成。
            if (yaw_stable_count >= 200) {
                yaw_stable_count        = 0;
                yaw_target_valid_       = 0U;
                // 上/下台阶使用雷达实际 yaw 分类，避免连续转向后把动作方向误当成当前朝向。
                set_last_turn_flags_by_radar_yaw(vision.angle_x, &last_turn_90_direction_, &last_turn_180_);

                state = PHASE_VISION;
            }
            break;

        case PHASE_TURN_RIGHT90:
            if (yaw_target_valid_ == 0U) {
                yaw_target        = normalize_yaw_deg(vision.angle_x - 90.0f);
                yaw_stable_count  = 0;
                yaw_target_valid_ = 1U;
            }

            if (fabsf(pid_yaw.pid.Err) < 3.0f) {
                yaw_stable_count++;
            } else {
                yaw_stable_count = 0;
            }

            // yaw 误差连续稳定 200 个周期后，认为本次右转 90 度完成。
            if (yaw_stable_count >= 200) {
                yaw_stable_count        = 0;
                yaw_target_valid_       = 0U;
                // 上/下台阶使用雷达实际 yaw 分类，避免连续转向后把动作方向误当成当前朝向。
                set_last_turn_flags_by_radar_yaw(vision.angle_x, &last_turn_90_direction_, &last_turn_180_);

                state = PHASE_VISION;
            }
            break;

        case PHASE_TURN180:
            if (yaw_target_valid_ == 0U) {
                yaw_target        = normalize_yaw_deg(vision.angle_x + 180.0f);
                yaw_stable_count  = 0;
                yaw_target_valid_ = 1U;
            }

            if (fabsf(pid_yaw.pid.Err) < 3.0f) {
                yaw_stable_count++;
            } else {
                yaw_stable_count = 0;
            }

            if (yaw_stable_count >= 200) {
                yaw_stable_count        = 0;
                yaw_target_valid_       = 0U;
                // 上/下台阶使用雷达实际 yaw 分类，避免连续转向后把动作方向误当成当前朝向。
                set_last_turn_flags_by_radar_yaw(vision.angle_x, &last_turn_90_direction_, &last_turn_180_);

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
        case PHASE_STEP_DOWN:
        case PHASE_TURN_LEFT90:
        case PHASE_TURN_RIGHT90:
        case PHASE_TURN180:
            return 1U;
        default:
            return 0U;
    }
}

uint16_t flag_meiling = 0;

extern "C" void plan_route(void *argument)
{
    lift_auto.setStepUpRadarClimbDistance(0.87);
    for (;;) {

        if (flag_meiling == 1) {
            route_t.route_reset();
            flag_meiling = 0;
        }

        route_t.vision_choice();
        route_t.meiling_route();
        lift_auto.update();
        lift_step_down.update();
        osDelay(1);
    }
}
