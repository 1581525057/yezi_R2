#include "route_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "omni_chassis.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "lift_step_up.h"
#include "reolcation.h"
#include "usart_task.h"
#include <math.h>
#include "lift_step_down.h"
#include "arm_comm.h"
#include "path_follow.h"
#include "path.h"
#include "FTMTask.h"
#include "conbat_task.h"

ROUTE_TASK route_t;
extern float yaw_target;
extern Block_Vision block_vision_middle[16];

float ROUTE_FIND_KFS_GENERATE_PATH_MAX_VEL_M_S = 4.0f;  // 寻找 KFS 自动生成路径的最大线速度，单位 m/s。
float ROUTE_FIND_KFS_GENERATE_PATH_MAX_ACC_M_S2 = 1.2f; // 寻找 KFS 自动生成路径的最大加速度，单位 m/s2。
float ROUTE_FIND_KFS_GENERATE_PATH_GAP_M = 0.04f;       // 寻找 KFS 自动生成路径点的间距，单位 m。

float ROUTE_FIND_KFS_POSITION_KP = 1.6f;                          // B 样条结束后 KFS 终点精定位的二维位置 P 闭环系数。
float ROUTE_FIND_KFS_POSITION_MAX_VEL_M_S = 0.6f;                 // KFS 终点精定位的最大线速度，单位 m/s。
float ROUTE_FIND_KFS_POSITION_MAX_ACC_M_S2 = 1.2f;                // KFS 终点精定位的最大加速度，单位 m/s2。
float ROUTE_FIND_KFS_POSITION_X_TOL_M = 0.10f;                    // KFS 终点精定位 X 允许误差，单位 m。
float ROUTE_FIND_KFS_POSITION_Y_TOL_M = 0.05f;                    // KFS 终点精定位 Y 允许误差，单位 m。
static const uint16_t ROUTE_FIND_KFS_POSITION_STABLE_COUNT = 10U; // KFS 终点精定位连续到位次数。

float ROUTE_RELOCATION_STOP_SPEED_LIMIT = 0.01f;                  // 一区重定位累计前，底盘解算速度需接近 0。
static const uint16_t ROUTE_RELOCATION_STOP_STABLE_COUNT = 1500U; // 底盘速度连续达标次数。

// 寻找 KFS 的终点表，单位：x/y 为 m，yaw 为 rad；按 entrence_KFS 0/1/2 选择。
static BRPathPose route_find_kfs_goals[] = {
    {2.36f, -2.78f, 0.0f},
    {2.36f, -1.56f, 0.0f},
    {2.33f, -0.39f, 0.0f},
};
// 寻找 KFS1 的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint route_find_kfs_0_middle_points[] = {
    {1.69f, -2.135f}};

// 寻找 KFS2 的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint route_find_kfs_1_middle_points[] = {
    {1.615f, -1.545f}};

// 寻找 KFS3 的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint route_find_kfs_2_middle_points[] = {
    {1.60f, -0.935f}};

static const BRPathControlPoint *route_find_kfs_middle_points[] = {
    route_find_kfs_0_middle_points,
    route_find_kfs_1_middle_points,
    route_find_kfs_2_middle_points,
};

static const std::size_t route_find_kfs_middle_point_counts[] = {
    sizeof(route_find_kfs_0_middle_points) / sizeof(route_find_kfs_0_middle_points[0]),
    sizeof(route_find_kfs_1_middle_points) / sizeof(route_find_kfs_1_middle_points[0]),
    sizeof(route_find_kfs_2_middle_points) / sizeof(route_find_kfs_2_middle_points[0]),
};

#ifndef ROUTE_DEBUG_MANUAL_STEP_CMD
#define ROUTE_DEBUG_MANUAL_STEP_CMD 0
#endif

#if ROUTE_DEBUG_MANUAL_STEP_CMD
uint8_t route_debug_step_next_cmd = 0U; // 调试用：置 1 后只执行一条视觉动作命令。
#endif

namespace
{
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

    // 判断命令是否为上台阶命令：3、4 表示上台阶相关动作。
    static uint8_t command_is_step_up(int cmd)
    {
        return (cmd == 3 || cmd == 4) ? 1U : 0U;
    }

    // 只查看队首命令是否需要跳过上台阶后的回中点，不弹出命令。
    static uint8_t next_command_skips_step_up_middle(void)
    {
        int cmd = 0;
        if (vision_command_peek(&cmd) != 1U)
        {
            return 0U;
        }

        return (command_is_step_up(cmd) != 0U) ? 1U : 0U;
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

    // 先锁定最终目标角。
    turn_final_yaw_ = normalize_yaw_deg(current_yaw + yaw_delta_deg);
    yaw_target = turn_final_yaw_;
    yaw_stable_count = 0;
    yaw_target_valid_ = 1U;
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
    relocation_position_sent_ = 0U;
    relocation_stop_stable_count_ = 0U;

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

    // 清空转弯最终目标角。
    turn_final_yaw_ = 0.0f;

    // 清空 1 区跑点底盘接管输出。
    path_loaded_ = 0U;
    find_kfs_positioning_ = 0U;
    find_kfs_position_stable_count_ = 0U;
    clear_path_output();

    // 清空 KFS 数量和机械臂取 KFS 子状态。
    number_KFS = 0U;
    arm_comm.resetPickKFS();

    // 清空视觉命令队列，防止复位后继续执行旧命令。
    vision_command_clear();
    vision_block_clear();
}

void ROUTE_TASK::clear_path_output(void)
{
    path_active_ = 0U;
    path_vx_target_ = 0.0f;
    path_vy_target_ = 0.0f;
    path_wz_target_ = 0.0f;
}

void ROUTE_TASK::update_number_KFS_by_cmd()
{
    const ArmComm::RxData &rx_data = arm_comm.getRxData();
    number_KFS = (uint8_t)(rx_data.car_kfs + rx_data.arm_kfs + 1);
}

uint8_t ROUTE_TASK::find_KFS1(void)

{
    if (path_loaded_ == 0U)
    {
        path_follow.loadPath(KFS1_area, KFS1_count);
        path_loaded_ = 1U;
    }
    return load_follow_plan();
}

uint8_t ROUTE_TASK::find_KFS2(void)
{
    if (path_loaded_ == 0U)
    {
        path_follow.loadPath(KFS2_area, KFS2_count);
        path_loaded_ = 1U;
    }
    return load_follow_plan();
}

uint8_t ROUTE_TASK::find_KFS3(void)
{
    if (path_loaded_ == 0U)
    {
        path_follow.loadPath(KFS3_area, KFS3_count);
        path_loaded_ = 1U;
    }
    return load_follow_plan();
}

uint8_t ROUTE_TASK::loadGeneratedPathToGoal(const BRPathPose &goal,
                                            const BRPathControlPoint *middle_points,
                                            std::size_t middle_point_count)
{
    if (path_loaded_ == 0U)
    {
        BRPathPose start;
        start.x_m = vision.x_diff;
        start.y_m = vision.y_diff;
        start.yaw_rad = vision.angle_x * 3.1415926f / 180.0f;

        std::size_t point_count = 0U;
        BRPathStatus status = generateBsplinePath(start,
                                                  middle_points,
                                                  middle_point_count,
                                                  goal,
                                                  ROUTE_FIND_KFS_GENERATE_PATH_MAX_VEL_M_S,
                                                  ROUTE_FIND_KFS_GENERATE_PATH_MAX_ACC_M_S2,
                                                  generated_path_,
                                                  ROUTE_GENERATE_PATH_MAX_POINTS,
                                                  &point_count,
                                                  ROUTE_FIND_KFS_GENERATE_PATH_GAP_M);
        if (status != BR_PATH_OK || point_count < 3U)
        {
            return 2U;
        }

        for (std::size_t i = 0U; i < point_count; ++i)
        {
            generated_follow_path_[i].vx = generated_path_[i].vx_mm_s;
            generated_follow_path_[i].vy = generated_path_[i].vy_mm_s;
            generated_follow_path_[i].x = generated_path_[i].x_mm;
            generated_follow_path_[i].y = generated_path_[i].y_mm;
            generated_follow_path_[i].theta = generated_path_[i].yaw_rad;
        }

        path_follow.loadPath(generated_follow_path_, (uint16_t)point_count);
        path_loaded_ = 1U;
    }

    return load_follow_plan();
}

/*
 * 二维位置 P 闭环速度规划。
 * 输入世界系 x/y 坐标误差，输出世界系目标速度，并限制最大速度和单周期速度变化量。
 */
void ROUTE_TASK::route_position_p_speed(float x_err,
                                        float y_err,
                                        float kp,
                                        float max_vel,
                                        float max_acc,
                                        float dt_s,
                                        float last_vx,
                                        float last_vy,
                                        float *vx,
                                        float *vy)
{
    if (vx == 0 || vy == 0)
    {
        return;
    }

    *vx = 0.0f;
    *vy = 0.0f;

    if (kp <= 0.0f || max_vel <= 0.0f)
    {
        return;
    }

    const float dist = sqrtf(x_err * x_err + y_err * y_err);
    if (dist <= 0.0f)
    {
        return;
    }

    float speed = kp * dist;
    if (speed > max_vel)
    {
        speed = max_vel;
    }

    const float target_vx = x_err / dist * speed;
    const float target_vy = y_err / dist * speed;

    if (max_acc <= 0.0f || dt_s <= 0.0f)
    {
        *vx = target_vx;
        *vy = target_vy;
        return;
    }

    // 用二维速度变化量统一限加速度，避免斜向运动时合速度突变。
    const float dvx = target_vx - last_vx;
    const float dvy = target_vy - last_vy;
    const float dv = sqrtf(dvx * dvx + dvy * dvy);
    const float max_dv = max_acc * dt_s;

    if (dv > max_dv && max_dv > 0.0f)
    {
        const float scale = max_dv / dv;
        *vx = last_vx + dvx * scale;
        *vy = last_vy + dvy * scale;
    }
    else
    {
        *vx = target_vx;
        *vy = target_vy;
    }
}

uint8_t ROUTE_TASK::runFindKfsPositionCloseLoop(const BRPathPose &goal)
{
    const float x_err = goal.x_m - vision.x_diff;
    const float y_err = goal.y_m - vision.y_diff;

    // x/y 都进入允许误差后，连续稳定一段时间才认为真正到位。
    const uint8_t arrived =
        (fabsf(x_err) < ROUTE_FIND_KFS_POSITION_X_TOL_M &&
         fabsf(y_err) < ROUTE_FIND_KFS_POSITION_Y_TOL_M)
            ? 1U
            : 0U;

    if (arrived != 0U)
    {
        if (find_kfs_position_stable_count_ < ROUTE_FIND_KFS_POSITION_STABLE_COUNT)
        {
            find_kfs_position_stable_count_++;
        }

        if (find_kfs_position_stable_count_ >= ROUTE_FIND_KFS_POSITION_STABLE_COUNT)
        {
            find_kfs_positioning_ = 0U;
            find_kfs_position_stable_count_ = 0U;
            clear_path_output();
            return 1U;
        }
    }
    else
    {
        find_kfs_position_stable_count_ = 0U;
    }

    // 限加速度需要以上一周期世界系速度为起点，因此先把底盘速度转回世界系。
    const float yaw_rad = vision.angle_x * 3.1415926f / 180.0f;
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);
    const float last_world_vx = cos_yaw * path_vx_target_ - sin_yaw * path_vy_target_;
    const float last_world_vy = sin_yaw * path_vx_target_ + cos_yaw * path_vy_target_;
    float world_vx = 0.0f;
    float world_vy = 0.0f;

    route_position_p_speed(x_err,
                           y_err,
                           ROUTE_FIND_KFS_POSITION_KP,
                           ROUTE_FIND_KFS_POSITION_MAX_VEL_M_S,
                           ROUTE_FIND_KFS_POSITION_MAX_ACC_M_S2,
                           0.001f,
                           last_world_vx,
                           last_world_vy,
                           &world_vx,
                           &world_vy);

    // 底盘接管接口使用车体系速度，P 闭环算完后再转回车体系输出。
    PathFollower::worldToBody(world_vx,
                              world_vy,
                              yaw_rad,
                              &path_vx_target_,
                              &path_vy_target_);
    path_wz_target_ = 0.0f;
    path_active_ = 1U;
    return 0U;
}

uint8_t ROUTE_TASK::runFindKfsToGoal(uint8_t index)
{
    if (index > 2U)
    {
        return 2U;
    }

    if (find_kfs_positioning_ == 0U)
    {
        // 第一步先跑 B 样条到 KFS 粗略终点。
        const uint8_t path_result = loadGeneratedPathToGoal(route_find_kfs_goals[index],
                                                            route_find_kfs_middle_points[index],
                                                            route_find_kfs_middle_point_counts[index]);
        if (path_result == 0U)
        {
            return 0U;
        }

        path_loaded_ = 0U;
        find_kfs_position_stable_count_ = 0U;

        if (path_result != 1U)
        {
            clear_path_output();
            return path_result;
        }

        // B 样条完成后，同一个状态里立刻接 KFS 终点 P 闭环精定位。
        find_kfs_positioning_ = 1U;
    }

    // 第二步持续做终点精定位，到位后返回 1，让外层切回视觉阶段。
    return runFindKfsPositionCloseLoop(route_find_kfs_goals[index]);
}

void ROUTE_TASK::vision_choice()
{
    // 等待视觉指令
    if (state != PHASE_VISION)
        return;

    int cmd;

#if ROUTE_DEBUG_MANUAL_STEP_CMD
    if (route_debug_step_next_cmd == 0U)
    {
        flag_vision = vision_command_has_pending();
        return;
    }
    route_debug_step_next_cmd = 0U;
#endif

    if (vision_command_pop(&cmd) != 1U)
    {
        flag_vision = 0;
        vision_plan_mark_consumed_if_empty();
        return;
    }

    vision.B = cmd;
    flag_vision = vision_command_has_pending();

    switch (cmd)
    {
    case 0:
    case 1:
    case 2: // 0 1 2都是寻找对应KFS的位置
        entrence_KFS = cmd;
        state = PHASE_FIND_KFS;
        break;

    case 3:
    {
        // 视觉指令 3：执行上台阶动作。
        // 从方块队列取编号，查表设置雷达目标坐标
        int block_num = 0;
        vision_block_pop(&block_num);                      // 取下一个台阶的方块
        float middle_x = block_vision_middle[block_num].x; // 取坐标
        float middle_y = block_vision_middle[block_num].y;
        lift_auto.setStepUpHeightMode(200U); // 设置200的高度
        lift_auto.setStepUpBlockNum(block_num);
        lift_auto.setStepUpRadarClimbDirection(last_turn_90_direction_);
        if (last_step_center_valid_ != 0U && (block_num < 1 || block_num > 3))
        {
            lift_auto.setStepUpLastMiddle(last_step_center_x_, last_step_center_y_);
        }
        lift_auto.setStepUpRadarTarget(middle_x, middle_y);
        lift_auto.setStepUpReturnMiddle((next_command_skips_step_up_middle() != 0U) ? 0U : 1U); // 是否上台阶后回中点
        last_step_center_x_ = middle_x;                                                         // 这次坐标变成下一次
        last_step_center_y_ = middle_y;
        last_step_center_valid_ = 1U; // 最近的坐标是否有效
        state = PHASE_STEP_UP;
        break;
    }

    case 4:
    {
        // 视觉命令 4：执行上 400 台阶动作。
        int block_num = 0;
        vision_block_pop(&block_num);
        float middle_x = block_vision_middle[block_num].x;
        float middle_y = block_vision_middle[block_num].y;
        lift_auto.setStepUpHeightMode(400U);
        lift_auto.setStepUpBlockNum(block_num);
        lift_auto.setStepUpRadarClimbDirection(last_turn_90_direction_);
        if (last_step_center_valid_ != 0U && (block_num < 1 || block_num > 3))
        {
            lift_auto.setStepUpLastMiddle(last_step_center_x_, last_step_center_y_);
        }
        lift_auto.setStepUpRadarTarget(middle_x, middle_y);
        lift_auto.setStepUpReturnMiddle((next_command_skips_step_up_middle() != 0U) ? 0U : 1U); // 是否上台阶后回中点
        last_step_center_x_ = middle_x;
        last_step_center_y_ = middle_y;
        last_step_center_valid_ = 1U;
        state = PHASE_STEP_UP;
        break;
    }

    case 5:
    case 6:
        // 指令 5 下200台阶，指令 6 下400台阶。
        // 从方块队列取编号，查表设置下台阶雷达目标坐标。
        {
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
            lift_step_down.setStepDownHeightMode((cmd == 6) ? 400U : 200U);
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
        state = PHASE_TURN_LEFT90;
        break;

    case 8:
        // 视觉指令 8：右转 90 度。
        yaw_stable_count = 0;
        yaw_target_valid_ = 0U;
        state = PHASE_TURN_RIGHT90;
        break;

    case 9:
        // 视觉指令 9：转 180 度。
        yaw_stable_count = 0;
        yaw_target_valid_ = 0U;
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
        state = FIRST_RELOCATION;

    switch (state)
    {

    case FIRST_RELOCATION:
    {
        const uint8_t relocation_sensor_mask = SENSOR_FRONT | SENSOR_LEFT;

        if (relocation_number == 0U)
        {
            // 本次进入该阶段时，重新累计一区重定位稳定周期。
            area_one_relocation.reset();
            relocation_stop_stable_count_ = 0U;
            relocation_number = 1U;
            break;
        }

        const uint8_t chassis_speed_in_limit =
            (fabsf(omni_chassis.now.Vx) <= ROUTE_RELOCATION_STOP_SPEED_LIMIT &&
             fabsf(omni_chassis.now.Vy) <= ROUTE_RELOCATION_STOP_SPEED_LIMIT &&
             fabsf(omni_chassis.now.Vz) <= ROUTE_RELOCATION_STOP_SPEED_LIMIT)
                ? 1U
                : 0U;

        if (chassis_speed_in_limit != 0U)
        {
            if (relocation_stop_stable_count_ < ROUTE_RELOCATION_STOP_STABLE_COUNT)
            {
                relocation_stop_stable_count_++;
            }
        }
        else
        {
            relocation_stop_stable_count_ = 0U;
        }

        const uint8_t chassis_speed_zero =
            (relocation_stop_stable_count_ >= ROUTE_RELOCATION_STOP_STABLE_COUNT) ? 1U : 0U;

        if (area_one_relocation.update(relocation_sensor_mask, chassis_speed_zero) == AreaOneRelocation::SENT)
        {
            relocation_number = 0U;
            relocation_stop_stable_count_ = 0U;
            relocation_position_sent_ = 1U;
            state = PHASE_VISION;
        }
        break;
    }

    case PHASE_FIND_KFS: // 寻找对应的KFS的位置
        if (runFindKfsToGoal((uint8_t)entrence_KFS) == 1U)
        {
            state = PHASE_VISION;
        }
        break;

    case SECOND_RELOCATION:
    {

        break;
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
            already_step_up_ = 1U; // 是否上过了台阶
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
    {
        if (yaw_target_valid_ == 0U)
        {
            start_turn_target(90.0f);
        }

        if (fabsf(normalize_yaw_deg(turn_final_yaw_ - vision.angle_x)) < 2.0f)
        {
            yaw_stable_count++;
        }
        else
        {
            yaw_stable_count = 0;
        }

        // yaw 误差连续稳定 200 个周期后，认为本次左转 90 度完成。
        if (yaw_stable_count >= 100)
        {
            yaw_stable_count = 0;
            yaw_target_valid_ = 0U;
            // 上/下台阶使用雷达实际 yaw 分类，避免连续转向后把动作方向误当成当前朝向。
            set_last_turn_flags_by_radar_yaw(vision.angle_x, &last_turn_90_direction_, &last_turn_180_);

            state = PHASE_VISION;
        }
        break;
    }

    case PHASE_TURN_RIGHT90:
    {
        if (yaw_target_valid_ == 0U)
        {
            start_turn_target(-90.0f);
        }

        if (fabsf(normalize_yaw_deg(turn_final_yaw_ - vision.angle_x)) < 2.0f)
        {
            yaw_stable_count++;
        }
        else
        {
            yaw_stable_count = 0;
        }

        // yaw 误差连续稳定 200 个周期后，认为本次右转 90 度完成。
        if (yaw_stable_count >= 100)
        {
            yaw_stable_count = 0;
            yaw_target_valid_ = 0U;
            // 上/下台阶使用雷达实际 yaw 分类，避免连续转向后把动作方向误当成当前朝向。
            set_last_turn_flags_by_radar_yaw(vision.angle_x, &last_turn_90_direction_, &last_turn_180_);

            state = PHASE_VISION;
        }
        break;
    }

    case PHASE_TURN180:
    {
        if (yaw_target_valid_ == 0U)
        {
            start_turn_target(180.0f);
        }

        if (fabsf(normalize_yaw_deg(turn_final_yaw_ - vision.angle_x)) < 2.0f)
        {
            yaw_stable_count++;
        }
        else
        {
            yaw_stable_count = 0;
        }

        if (yaw_stable_count >= 100)
        {
            yaw_stable_count = 0;
            yaw_target_valid_ = 0U;
            // 上/下台阶使用雷达实际 yaw 分类，避免连续转向后把动作方向误当成当前朝向。
            set_last_turn_flags_by_radar_yaw(vision.angle_x, &last_turn_90_direction_, &last_turn_180_);

            state = PHASE_VISION;
        }
        break;
    }

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

uint8_t ROUTE_TASK::load_follow_plan(void)
{
    PathFollower::Pose pose;
    pose.x = vision.x_diff * 1000.0f;                // m 转 mm
    pose.y = vision.y_diff * 1000.0f;                // m 转 mm
    pose.yaw = vision.angle_x * 3.1415926f / 180.0f; // deg 转 rad

    PathFollower::State state = path_follow.follow(pose);
    const PathFollower::Output &out = path_follow.getOutput();

    if (state == PathFollower::STATE_FINISHED || state == PathFollower::STATE_DEVIATED)
    {
        clear_path_output();
        if (state == PathFollower::STATE_FINISHED)
        {
            return 1U;
        }
        return 2U;
    }

    float body_vx = 0.0f;
    float body_vy = 0.0f;
    PathFollower::worldToBody(out.world_vx, out.world_vy, pose.yaw, &body_vx, &body_vy);

    // 底盘用 m/s，path_follow 输出 mm/s。
    path_vx_target_ = body_vx * 0.001f;
    path_vy_target_ = body_vy * 0.001f;
    path_wz_target_ = -out.wz;
    path_active_ = 1U;

    return 0U;
}

uint8_t ROUTE_TASK::getPathChassisTarget(float manual_vx,
                                         float manual_vy,
                                         float manual_wz,
                                         float *target_vx,
                                         float *target_vy,
                                         float *target_wz) const
{
    *target_vx = manual_vx;
    *target_vy = manual_vy;
    *target_wz = manual_wz;

    if (path_active_ == 0U)
    {
        return 0U;
    }

    *target_vx = path_vx_target_;
    *target_vy = path_vy_target_;
    *target_wz = path_wz_target_;

    return 1U;
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
uint16_t flag_step = 0;
extern "C" void plan_route(void *argument)
{
    uint8_t ftm_done_route_started = 0U;

    for (;;)
    {

        if (flag_meiling == 1)
        {
            route_t.route_reset();
            flag_meiling = 0;
            ftm_done_route_started = 0U;
            arm_comm.executeAction(ArmComm::ACTION_POWER_ON_INIT, 0);
            arm_comm.send();
        }

        
        // if ((FTM_GetMainState() == 4U) &&
        //     (route_t.state == PHASE_IDLE) &&
        //     (ftm_done_route_started == 0U) && (conbat_t.state == CONBAT_IDLE))
        // {
        //     route_t.flag_start = 1U;
        //     ftm_done_route_started = 1U;
        // }

        if (flag_step == 1)
        {
            area_one_relocation.update(SENSOR_FRONT | SENSOR_LEFT, 1);
        }

        if (flag_step == 2)
        {
            area_one_relocation.update(SENSOR_FRONT | SENSOR_RIGHT, 1);
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
