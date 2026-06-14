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
#include <math.h>
#include "lift_step_down.h"
#include "arm_comm.h"

ROUTE_TASK route_t;
extern float yaw_target;
extern Block_Vision block_vision_middle[13];

namespace
{
    MeilingTarget_t first_relocation = {
        .preset_id = 0,
        .L_ref = 335.0f,
        .R_ref = 0.0f,
        .F_ref = 356.0f,
        .tol_lat = 6.0f,
        .tol_lon = 6.0f,
        .timeout_ms = 500000U,
        .sensor_mask = SENSOR_FRONT | SENSOR_LEFT,
    };

    MeilingTarget_t second_relocation = {
        .preset_id = 0,
        .L_ref = 2630.0f,
        .R_ref = 0.0f,
        .F_ref = 500.0f,
        .tol_lat = 20.0f,
        .tol_lon = 20.0f,
        .timeout_ms = 500000U,
        .sensor_mask = SENSOR_FRONT | SENSOR_LEFT,

    };

    MeilingTarget_t third_relocation = {
        .preset_id = 0,
        .L_ref = 222.0f,
        .R_ref = 292.0f,
        .F_ref = 221.8f,
        .tol_lat = 6.0f,
        .tol_lon = 6.0f,
        .timeout_ms = 500000U,
        .sensor_mask = SENSOR_FRONT | SENSOR_LEFT,
    };

    // 取 KFS 后接转弯时，目标 yaw 按设定角速度线性推进。
    static const float PICK_KFS_SLOW_TURN_RATE_DPS = 80.0f;
    // 启动时先给一点目标角提前量，避免 yaw 误差为 0 导致底盘起步发愣。
    static const float PICK_KFS_SLOW_TURN_LEAD_DEG = 30.0f;

    // 将 yaw 角限制到 [-180, 180]，后面做角度差时统一走最短方向。
    static float normalize_yaw_deg(float yaw_deg)
    {
        while (yaw_deg > 180.0f)
        {
            yaw_deg -= 360.0f;
        }
        while (yaw_deg < -180.0f)
        {
            yaw_deg += 360.0f;
        }
        return yaw_deg;
    }

    // 根据雷达实际 yaw 记录当前朝向，供后续上下台阶判断方向使用。
    static void set_last_turn_flags_by_radar_yaw(float yaw_deg, int8_t *turn_90_direction, uint8_t *turn_180)
    {
        const float yaw = normalize_yaw_deg(yaw_deg);

        *turn_90_direction = 0;
        *turn_180 = 0U;

        // yaw 落在 +90 度附近，认为上一次结果是左转 90 度。
        if (yaw >= 45.0f && yaw < 135.0f)
        {
            *turn_90_direction = 1;
        }
        // yaw 落在 -90 度附近，认为上一次结果是右转 90 度。
        else if (yaw <= -45.0f && yaw > -135.0f)
        {
            *turn_90_direction = -1;
        }
        // yaw 接近正负 180 度，认为上一次结果是掉头。
        else if (yaw >= 135.0f || yaw <= -135.0f)
        {
            *turn_180 = 1U;
        }
    }

    // 把视觉命令 7/8/9 转成对应的路线转弯状态。
    static uint8_t command_to_turn_state(int cmd, Route_state *turn_state)
    {
        if (turn_state == 0)
        {
            return 0U;
        }

        if (cmd == 7)
        {
            *turn_state = PHASE_TURN_LEFT90;
            return 1U;
        }
        if (cmd == 8)
        {
            *turn_state = PHASE_TURN_RIGHT90;
            return 1U;
        }
        if (cmd == 9)
        {
            *turn_state = PHASE_TURN180;
            return 1U;
        }

        return 0U;
    }

    // 只查看队首命令是否为转弯，不弹出命令。
    static uint8_t next_command_is_turn(Route_state *turn_state)
    {
        int cmd = 0;
        if (vision_command_peek(&cmd) != 1U)
        {
            return 0U;
        }

        return command_to_turn_state(cmd, turn_state);
    }

    // 确认队首是转弯命令后再弹出，避免误吃掉非转弯命令。
    static uint8_t pop_next_turn_state(Route_state *turn_state)
    {
        int cmd = 0;
        if (vision_command_peek(&cmd) != 1U ||
            command_to_turn_state(cmd, turn_state) == 0U)
        {
            return 0U;
        }

        (void)vision_command_pop(&cmd);
        vision.B = cmd;
        return 1U;
    }

}

void ROUTE_TASK::start_turn_target(float yaw_delta_deg)
{
    const float current_yaw = vision.angle_x;

    // 先锁定最终目标角；完成判定始终看这个最终目标，避免慢速斜坡中途误判完成。
    turn_final_yaw_ = normalize_yaw_deg(current_yaw + yaw_delta_deg);
    yaw_target = turn_final_yaw_;
    yaw_stable_count = 0;
    yaw_target_valid_ = 1U;

    if (kfs_slow_turn_pending_ != 0U)
    {
        // 只有 KFS 后紧接的这一次转弯走慢速斜坡，普通转弯仍直接给最终角。
        kfs_slow_turn_pending_ = 0U;
        kfs_slow_turn_active_ = 1U;
        kfs_slow_turn_start_yaw_ = current_yaw;
        kfs_slow_turn_start_tick_ = HAL_GetTick();

        // 慢速转弯第一帧就给提前量，让底盘 PID 立刻有 yaw 误差可跟。
        yaw_target = normalize_yaw_deg(current_yaw +
                                       ((yaw_delta_deg < 0.0f) ? -PICK_KFS_SLOW_TURN_LEAD_DEG
                                                               : PICK_KFS_SLOW_TURN_LEAD_DEG));
    }
    else
    {
        kfs_slow_turn_active_ = 0U;
    }
}

void ROUTE_TASK::update_slow_turn_target(void)
{
    // 没有慢速转弯任务时不改 yaw_target。
    if (kfs_slow_turn_active_ == 0U)
    {
        return;
    }

    const float yaw_delta = normalize_yaw_deg(turn_final_yaw_ - kfs_slow_turn_start_yaw_);
    const float abs_yaw_delta = fabsf(yaw_delta);
    const float elapsed_s = (float)(HAL_GetTick() - kfs_slow_turn_start_tick_) * 0.001f;

    // 一元一次函数：目标角增量 = 启动提前量 + 设定角速度 * 时间。
    float target_delta = PICK_KFS_SLOW_TURN_LEAD_DEG + PICK_KFS_SLOW_TURN_RATE_DPS * elapsed_s;

    // 斜坡已经走完时直接钳到最终目标角，防止超过目标。
    if (target_delta >= abs_yaw_delta)
    {
        yaw_target = turn_final_yaw_;
        kfs_slow_turn_active_ = 0U;
        return;
    }

    // 目标在负方向时，把角度增量取反。
    if (yaw_delta < 0.0f)
    {
        target_delta = -target_delta;
    }

    // 每个周期刷新 yaw_target，让底盘 yaw PID 跟着这个线性目标慢慢转。
    yaw_target = normalize_yaw_deg(kfs_slow_turn_start_yaw_ + target_delta);
}

void ROUTE_TASK::route_reset()
{
    // 主流程回到空闲，等待外部重新启动路线。
    state = PHASE_IDLE;

    // 清空流程控制标志和重定位计数。
    flag_start = 0;
    flag_relocation = 0;
    flag_vision = 0;
    relocation_number = 0;

    // 清空转弯目标锁存和到位稳定计数。
    yaw_stable_count = 0;
    yaw_target_valid_ = 0U;

    // 清空最近一次转弯方向记录，避免影响后续上下台阶方向判断。
    last_turn_90_direction_ = 0;
    last_turn_180_ = 0U;

    // 清空上台阶记录的方块中心和是否已上台阶标志。
    last_step_center_x_ = 0.0f;
    last_step_center_y_ = 0.0f;
    last_step_center_valid_ = 0U;
    already_step_up_ = 0U;

    // 清空取 KFS 所在方块中心缓存。
    pick_kfs_center_x_ = 0.0f;
    pick_kfs_center_y_ = 0.0f;
    pick_kfs_center_valid_ = 0U;

    // 清空 KFS 后慢速转弯的最终目标角、触发标志和计时起点。
    turn_final_yaw_ = 0.0f;
    kfs_slow_turn_pending_ = 0U;
    kfs_slow_turn_active_ = 0U;
    kfs_slow_turn_start_yaw_ = 0.0f;
    kfs_slow_turn_start_tick_ = 0U;

    // 清空 KFS 数量和机械臂取 KFS 子状态。
    number_KFS = 0U;
    arm_comm.resetPickKFS();

    // 清空视觉命令队列，防止复位后继续执行旧命令。
    vision_command_clear();
}

void ROUTE_TASK::update_number_KFS_by_cmd()
{
    const ArmComm::RxData &rx_data = arm_comm.getRxData();
    number_KFS = (uint8_t)(rx_data.car_kfs + rx_data.arm_kfs + 1);
}

void ROUTE_TASK::vision_choice()
{
    // 等待视觉指令
    if (state != PHASE_VISION)
        return;

    int cmd;
    if (vision_command_pop(&cmd) != 1U)
    {
        flag_vision = 0;
        return;
    }

    vision.B = cmd;
    flag_vision = vision_command_has_pending();

    switch (cmd)
    {

    case 0:
        state = FIRST_RELOCATION;
        break;
    case 1:
        state = SECOND_RELOCATION;
        break;
    case 2:
        state = THIRD_RELOCATION;
        break;

    case 3:
    {
        // 视觉指令 3：执行上台阶动作。
        // 从方块队列取编号，查表设置雷达目标坐标
        int block_num = 0;
        vision_block_pop(&block_num);
        float middle_x = block_vision_middle[block_num].x;
        float middle_y = block_vision_middle[block_num].y;
        lift_auto.setStepUpBlockNum(block_num);
        lift_auto.setStepUpRadarClimbDirection(last_turn_90_direction_);
        lift_auto.setStepUpRadarTarget(middle_x, middle_y);
        last_step_center_x_ = middle_x;
        last_step_center_y_ = middle_y;
        last_step_center_valid_ = 1U;
        state = PHASE_STEP_UP;
        break;
    }

    case 5:
    {
        // 从方块队列取编号，查表设置雷达目标坐标
        int block_num = 0;
        vision_block_pop(&block_num);
        float finish_x = block_vision_middle[block_num].x;
        float finish_y = block_vision_middle[block_num].y;
        float prepare_base_x = finish_x;
        float prepare_base_y = finish_y;
        if (last_step_center_valid_ != 0U)
        {
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
        last_step_center_x_ = finish_x;
        last_step_center_y_ = finish_y;
        last_step_center_valid_ = 1U;
        state = PHASE_STEP_DOWN;
        break;
    }

    case 7:
        // 视觉指令 7：左转 90 度。
        yaw_stable_count = 0;
        yaw_target_valid_ = 0U;
        kfs_slow_turn_pending_ = 0U;
        kfs_slow_turn_active_ = 0U;
        state = PHASE_TURN_LEFT90;
        break;

    case 8:
        // 视觉指令 8：右转 90 度。
        yaw_stable_count = 0;
        yaw_target_valid_ = 0U;
        kfs_slow_turn_pending_ = 0U;
        kfs_slow_turn_active_ = 0U;
        state = PHASE_TURN_RIGHT90;
        break;

    case 9:
        // 视觉指令 9：转 180 度。
        yaw_stable_count = 0;
        yaw_target_valid_ = 0U;
        kfs_slow_turn_pending_ = 0U;
        kfs_slow_turn_active_ = 0U;
        state = PHASE_TURN180;
        break;
    case 10: // 取低200mm

        pick_kfs_center_x_ = last_step_center_x_;
        pick_kfs_center_y_ = last_step_center_y_;
        pick_kfs_center_valid_ = last_step_center_valid_;
        state = PHASE_GET_KFS_SHORT_200;
        break;

    case 11: // 取高200mm
        pick_kfs_center_x_ = last_step_center_x_;
        pick_kfs_center_y_ = last_step_center_y_;
        pick_kfs_center_valid_ = last_step_center_valid_;
        state = PHASE_GET_KFS_HEIGHT_200;

        break;

    case 12: // 取高400mm
        pick_kfs_center_x_ = last_step_center_x_;
        pick_kfs_center_y_ = last_step_center_y_;
        pick_kfs_center_valid_ = last_step_center_valid_;
        state = PHASE_GET_KFS_HEIGHT_400;

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

    switch (state)
    {
    case FIRST_RELOCATION:

        break;

    case SECOND_RELOCATION:
    {
        if (relocation_number == 0)
        {
            // 第一次重定位
            meiling.start(second_relocation);
            relocation_number = 1;
        }
        else if (relocation_number == 1)
        {
            uint8_t relocation_result = meiling.update();

            if (relocation_result == MeilingLocator::SUCCESS)
            {
                relocation_number = 2;
                // First relocation is done; wait for a vision command.
                state = PHASE_VISION;
            }
            else if (relocation_result == MeilingLocator::TIMEOUT)
            {
                meiling.start(second_relocation);
            }
        }
    }

    case THIRD_RELOCATION:
    {
        break;
    }

    case PHASE_STEP_UP:
        // 上台阶目标参数已在 vision_choice() 中配置。
        lift_auto.startStepUp();

        if (lift_auto.isStepUpFinished())
        {
            lift_auto.stopStepUp();
            already_step_up_ = 1U;
            state = PHASE_VISION;
        }
        break;

    case PHASE_STEP_DOWN:
        // 下台阶目标参数已在 vision_choice() 中配置。
        lift_step_down.startStepDown();

        if (lift_step_down.isStepDownFinished())
        {
            lift_step_down.stopStepDown();
            state = PHASE_VISION;
        }
        break;

    case PHASE_TURN_LEFT90:
        if (yaw_target_valid_ == 0U)
        {
            start_turn_target(90.0f);
        }
        update_slow_turn_target();

        if (fabsf(normalize_yaw_deg(turn_final_yaw_ - vision.angle_x)) < 1.5f)
        {
            yaw_stable_count++;
        }
        else
        {
            yaw_stable_count = 0;
        }

        // yaw 误差连续稳定 200 个周期后，认为本次左转 90 度完成。
        if (yaw_stable_count >= 200)
        {
            yaw_stable_count = 0;
            yaw_target_valid_ = 0U;
            kfs_slow_turn_active_ = 0U;
            // 上/下台阶使用雷达实际 yaw 分类，避免连续转向后把动作方向误当成当前朝向。
            set_last_turn_flags_by_radar_yaw(vision.angle_x, &last_turn_90_direction_, &last_turn_180_);

            state = PHASE_VISION;
        }
        break;

    case PHASE_TURN_RIGHT90:
        if (yaw_target_valid_ == 0U)
        {
            start_turn_target(-90.0f);
        }
        update_slow_turn_target();

        if (fabsf(normalize_yaw_deg(turn_final_yaw_ - vision.angle_x)) < 1.5f)
        {
            yaw_stable_count++;
        }
        else
        {
            yaw_stable_count = 0;
        }

        // yaw 误差连续稳定 200 个周期后，认为本次右转 90 度完成。
        if (yaw_stable_count >= 200)
        {
            yaw_stable_count = 0;
            yaw_target_valid_ = 0U;
            kfs_slow_turn_active_ = 0U;
            // 上/下台阶使用雷达实际 yaw 分类，避免连续转向后把动作方向误当成当前朝向。
            set_last_turn_flags_by_radar_yaw(vision.angle_x, &last_turn_90_direction_, &last_turn_180_);

            state = PHASE_VISION;
        }
        break;

    case PHASE_TURN180:
        if (yaw_target_valid_ == 0U)
        {
            start_turn_target(180.0f);
        }
        update_slow_turn_target();

        if (fabsf(normalize_yaw_deg(turn_final_yaw_ - vision.angle_x)) < 1.5f)
        {
            yaw_stable_count++;
        }
        else
        {
            yaw_stable_count = 0;
        }

        if (yaw_stable_count >= 200)
        {
            yaw_stable_count = 0;
            yaw_target_valid_ = 0U;
            kfs_slow_turn_active_ = 0U;
            // 上/下台阶使用雷达实际 yaw 分类，避免连续转向后把动作方向误当成当前朝向。
            set_last_turn_flags_by_radar_yaw(vision.angle_x, &last_turn_90_direction_, &last_turn_180_);

            state = PHASE_VISION;
        }
        break;

    case PHASE_GET_KFS_HEIGHT_200:
    {
        Route_state next_turn_state = PHASE_VISION;
        const uint8_t next_is_turn = next_command_is_turn(&next_turn_state);
        const uint8_t return_center = (pick_kfs_center_valid_ != 0U && next_is_turn != 0U) ? 1U : 0U;

        if (arm_comm.pickKFS(ArmComm::ACTION_PICK_HIGH_200,
                             number_KFS,
                             already_step_up_,
                             vision.x_diff,
                             vision.y_diff,
                             vision.angle_x,
                             return_center,
                             pick_kfs_center_x_,
                             pick_kfs_center_y_) != 0U)
        {
            if (next_is_turn != 0U && pop_next_turn_state(&next_turn_state) != 0U)
            {
                yaw_stable_count = 0U;
                yaw_target_valid_ = 0U;
                // KFS 完成后发现下一个命令是转弯，通知转弯状态启用慢速目标角。
                kfs_slow_turn_pending_ = 1U;
                state = next_turn_state;
            }
            else
            {
                state = PHASE_VISION;
            }
        }

        break;
    }

    case PHASE_GET_KFS_HEIGHT_400:
    {
        Route_state next_turn_state = PHASE_VISION;
        const uint8_t next_is_turn = next_command_is_turn(&next_turn_state);
        const uint8_t return_center = (pick_kfs_center_valid_ != 0U && next_is_turn != 0U) ? 1U : 0U;

        if (arm_comm.pickKFS(ArmComm::ACTION_PICK_HIGH_400,
                             number_KFS,
                             already_step_up_,
                             vision.x_diff,
                             vision.y_diff,
                             vision.angle_x,
                             return_center,
                             pick_kfs_center_x_,
                             pick_kfs_center_y_) != 0U)
        {
            if (next_is_turn != 0U && pop_next_turn_state(&next_turn_state) != 0U)
            {
                yaw_stable_count = 0U;
                yaw_target_valid_ = 0U;
                // KFS 完成后发现下一个命令是转弯，通知转弯状态启用慢速目标角。
                kfs_slow_turn_pending_ = 1U;
                state = next_turn_state;
            }
            else
            {
                state = PHASE_VISION;
            }
        }

        break;
    }

    case PHASE_GET_KFS_SHORT_200:
    {
        Route_state next_turn_state = PHASE_VISION;
        const uint8_t next_is_turn = next_command_is_turn(&next_turn_state);
        const uint8_t return_center = (pick_kfs_center_valid_ != 0U && next_is_turn != 0U) ? 1U : 0U;

        if (arm_comm.pickKFS(ArmComm::ACTION_PICK_LOW_200,
                             number_KFS,
                             already_step_up_,
                             vision.x_diff,
                             vision.y_diff,
                             vision.angle_x,
                             return_center,
                             pick_kfs_center_x_,
                             pick_kfs_center_y_) != 0U)
        {
            if (next_is_turn != 0U && pop_next_turn_state(&next_turn_state) != 0U)
            {
                yaw_stable_count = 0U;
                yaw_target_valid_ = 0U;
                // KFS 完成后发现下一个命令是转弯，通知转弯状态启用慢速目标角。
                kfs_slow_turn_pending_ = 1U;
                state = next_turn_state;
            }
            else
            {
                state = PHASE_VISION;
            }
        }

        break;
    }

    default:
        break;
    }
}

extern "C" uint8_t RouteTask_IsMeilingAreaActive(void)
{
    if (route_t.flag_start != 1U)
    {
        return 0U;
    }

    switch (route_t.state)
    {
    case FIRST_RELOCATION:
    case SECOND_RELOCATION:
    case THIRD_RELOCATION:
    case PHASE_STEP_UP:
    case PHASE_STEP_DOWN:
    case PHASE_TURN_LEFT90:
    case PHASE_TURN_RIGHT90:
    case PHASE_TURN180:
    case PHASE_GET_KFS_HEIGHT_200:
    case PHASE_GET_KFS_HEIGHT_400:
    case PHASE_GET_KFS_SHORT_200:
        return 1U;
    default:
        return 0U;
    }
}

uint16_t flag_meiling = 1;

extern "C" void plan_route(void *argument)
{
    for (;;)
    {

        if (flag_meiling == 1)
        {
            route_t.route_reset();
            flag_meiling = 0;
            arm_comm.executeAction(ArmComm::ACTION_POWER_ON_INIT, 0);
            arm_comm.send();
        }

        // 更新现在几个KFS
        route_t.update_number_KFS_by_cmd();

        route_t.vision_choice();
        route_t.meiling_route();
        lift_auto.update();
        lift_step_down.update();
        osDelay(1);
    }
}
