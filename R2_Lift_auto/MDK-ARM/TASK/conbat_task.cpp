#include "conbat_task.h"
#include "cmsis_os.h"
#include "usart_task.h"
#include "reolcation.h"
#include "arm_comm.h"
#include "DT35.h"
#include "laser_distance.h"
#include "lift_class.h"
#include "lift_step_up.h"
#include "omni_chassis.h"
#include <math.h>
#include "FTMTask.h"
extern float yaw_target;

// 通用参数
float CONBAT_DEG_TO_RAD = 3.1415926f / 180.0f; // 角度转弧度系数，用于视觉角度转换。

// 上坡状态的终点表，单位：x/y 为 m，yaw 为 rad；你后续直接改这里。
static BRPathPose conbat_ramp_up_goals[] = {
    // {3.43f, 1.6f, 0.0f},
    {2.93f, -1.6f, 0.0f}};

// 上坡状态的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint conbat_ramp_up_middle_points[] = {
    {0.84f, 0.07f},
    {1.69f, 0.04f},
    {3.46f, 0.06f},
    {3.36f, -0.53f},
    {3.18f, -1.5f},
    {3.18f, -1.6f},
};
static const std::size_t conbat_ramp_up_middle_point_count =
    sizeof(conbat_ramp_up_middle_points) / sizeof(conbat_ramp_up_middle_points[0]);

// 捡 KFS 状态的终点表，单位：x/y 为 m，yaw 为 rad；你后续直接改这里。
static BRPathPose conbat_pick_kfs_goals[] = {
    {3.05f, -1.63f, 0.0f},
    {3.00f, -2.33f, 0.0f},
    {3.05f, -3.03f, 0.0f},
};

// 捡最后一个 KFS 状态的中间点表，单位：x/y 为 m。
static BRPathControlPoint conbat_pick_kfs_middle_points[] = {
    {3.59f, -1.11f},
    {3.14f, -2.09f},
};
static const std::size_t conbat_pick_kfs_middle_point_count =
    sizeof(conbat_pick_kfs_middle_points) / sizeof(conbat_pick_kfs_middle_points[0]);

// 合体目标终点表，单位：x/y 为 m，yaw 为 rad；后续直接改这里。
static BRPathPose conbat_combine_goals[] = {
    {3.12f, -3.05f, 0.0f},
};

// 放 KFS 状态的终点表，单位：x/y 为 m，yaw 为 rad；按 kfs_place_index_ 选择。
static BRPathPose conbat_kfs_place_goals[] = {
    {3.84f, -4.43f, -1.5708f},
    {3.28f, -4.43f, -1.5708f},
    {2.71f, -4.43f, -1.5708f},
};

// 放 KFS 状态共用的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint conbat_kfs_place_middle_point_table[] = {
    {2.88, -4.20},
};

static const BRPathControlPoint *conbat_kfs_place_middle_points[] = {
    conbat_kfs_place_middle_point_table,
    conbat_kfs_place_middle_point_table,
    conbat_kfs_place_middle_point_table,
};

static const std::size_t conbat_kfs_place_middle_point_counts[] = {
    sizeof(conbat_kfs_place_middle_point_table) / sizeof(conbat_kfs_place_middle_point_table[0]),
    sizeof(conbat_kfs_place_middle_point_table) / sizeof(conbat_kfs_place_middle_point_table[0]),
    sizeof(conbat_kfs_place_middle_point_table) / sizeof(conbat_kfs_place_middle_point_table[0]),
};

// 放置完成后偏角等待路径的终点，单位：x/y 为 m，yaw 为 rad。
static BRPathPose conbat_kfs_wait_goal = {3.59f, -3.87f, -1.57f};

// 放置完成后偏角等待路径的中间点，单位：x/y 为 m。
static BRPathControlPoint conbat_kfs_wait_middle_points[] = {
    {3.58f, -4.08f},
};
static const std::size_t conbat_kfs_wait_middle_point_count =
    sizeof(conbat_kfs_wait_middle_points) / sizeof(conbat_kfs_wait_middle_points[0]);

CONBAT_TASK conbat_t;

// action_result: 0 继续当前状态，1 进入下一状态，2 回到空闲。
static void update_state_by_action_result(uint8_t action_result,
                                          ConbatState next_state,
                                          ConbatState *state)
{
    if (action_result == 1U)
    {
        *state = next_state;
    }
    else if (action_result == 2U)
    {
        *state = CONBAT_IDLE;
    }
}

static float conbat_speed_limit(float speed, float max)
{
    if (speed > max)
    {
        speed = max;
    }
    if (speed < -max)
    {
        speed = -max;
    }
    return speed;
}

static float conbat_trapezoid_speed(float error, float acc, float max)
{
    if (error <= 0.0f || acc <= 0.0f || max <= 0.0f)
    {
        return 0.0f;
    }

    return conbat_speed_limit(sqrtf(2.0f * error * acc), max);
}

/*
 * 二维位置 P 闭环速度规划。
 * 根据世界系 x/y 坐标误差计算目标速度，再限制最大速度和单周期速度变化量。
 */
void CONBAT_TASK::conbat_position_p_speed(float x_err,
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

    const float dvx = target_vx - last_vx;
    const float dvy = target_vy - last_vy;
    const float dv = sqrtf(dvx * dvx + dvy * dvy);
    // 用二维速度变化量统一限加速度，避免斜向运动时合速度突变。
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

static float conbat_min_abs_speed(float speed, float min_abs, float max_abs)
{
    if (speed == 0.0f || min_abs <= 0.0f)
    {
        return speed;
    }

    if (fabsf(speed) < min_abs)
    {
        speed = (speed > 0.0f) ? min_abs : -min_abs;
    }

    return conbat_speed_limit(speed, max_abs);
}

static uint8_t conbat_stable_confirm(uint8_t condition, uint8_t *stable_count, uint8_t target_count)
{
    if (condition == 0U)
    {
        *stable_count = 0U;
        return 0U;
    }

    if (*stable_count < target_count)
    {
        (*stable_count)++;
    }

    return (*stable_count >= target_count) ? 1U : 0U;
}

/*
 * 构造函数：上电创建全局 conbat_t 时调用。
 * 这里直接复位所有状态，保证任务启动前处于空闲、安全输出状态。
 */
CONBAT_TASK::CONBAT_TASK()
{
    reset();
}

/*
 * 复位战斗任务状态机。
 * 清空路径跟随、KFS 点位选择、yaw 目标和底盘速度输出，回到空闲状态。
 */
void CONBAT_TASK::reset(void)
{
    state = CONBAT_IDLE;
    last_state_ = CONBAT_IDLE;
    conbat_start = 0U;
    path_follower_.reset();
    path_loaded_ = 0U;
    ramp_up_waiting_ = 0U;
    ramp_up_zero_yaw_done_ = 0U;
    pick_kfs_step_ = PICK_KFS_LOCK_ZERO;
    place_kfs_step_ = PLACE_KFS_LOWER_ACTION;
    combine_step_ = COMBINE_PRE_LIFT;
    pick_kfs_meiling_active_ = 0U;
    pick_kfs_second_forward_done_ = 0U;
    pick_kfs_path_stable_count_ = 0U;
    kfs_place_stop_stable_count_ = 0U;
    kfs_place_index_ = 0U;
    kfs_place_arrived_ = 0U;
    kfs_place_precision_active_ = 0U;
    place_kfs_pick_precision_active_ = 0U;
    resetKfsPlaceLaserFlag();
    combine_pre_lift_ready_ = 0U;
    combine_crossed_finish_height_ = 0U;
    combine_stable_count_ = 0U;
    combine_pre_lift_command_seq_ = 0U;
    combine_climb_lift_command_seq_ = 0U;
    combine_final_lift_command_seq_ = 0U;
    lift_switch_target_ = 0U;
    lift_linear_speed_target_ = 0.0f;
    pick_kfs_first_back_start_x_m_ = 0.0f;
    pick_kfs_second_back_start_x_m_ = 0.0f;
    place_kfs_back_start_x_m_ = 0.0f;
    place_kfs_forward_start_x_m_ = 0.0f;
    place_kfs_forward_start_y_m_ = 0.0f;
    place_kfs_forward_start_yaw_deg_ = 0.0f;
    yaw_target_enabled = 0U;
    yaw_target_degree = 0.0f;
    clearPathOutput();
}

uint16_t flag = 0;
/*
 * 战斗任务主循环单次更新函数。
 * conbat_task 线程每 1ms 调用一次，负责检测状态切换，并分发到对应状态处理函数。
 * 还没有实现具体动作的状态会保持零速度输出，避免底盘被未知流程接管后乱动。
 */
void CONBAT_TASK::runOnce(void)
{
    /* 状态变化时先清理上一状态留下的路径和速度输出。 */
    if (state != last_state_)
    {
        handleStateChanged();
    }

    uint8_t action_result = 0U;

    switch (state)
    {
    case CONBAT_RAMP_UP:
        /* 上坡状态：加载并执行上坡路径。 */
        action_result = runRampUp();
        update_state_by_action_result(action_result, CONBAT_PICK_KFS, &state);
        break;

    case CONBAT_PICK_KFS: // 三层车略
        /* 捡 KFS 状态：生成到捡取点的路径并开始跟随。 */
        action_result = runPickKfs();

        update_state_by_action_result(action_result, CONBAT_COMBINE, &state);
        break;

    case CONBAT_PLACE_KFS: // 二层策略 放一个
        /* 放置 KFS 状态：按子状态依次取车内 KFS、跑放置点并放手。 */
        action_result = runPlaceKfs();
        update_state_by_action_result(action_result, CONBAT_IDLE, &state);
        break;

    case CONBAT_COMBINE: // 合体
    {

        if (vision.can_up == 1)
        {
            flag = 1;
        }
        if (flag == 1)
        {
            action_result = runCombine();
            update_state_by_action_result(action_result, CONBAT_IDLE, &state);
        }
        break;
    }

    case CONBAT_IDLE:
        if (conbat_start == 1U)
        {
            conbat_start = 0U;
            state = CONBAT_RAMP_UP;
        }
        if (conbat_start == 2U)
        {
            conbat_start = 0U;
            state = CONBAT_PLACE_KFS;
            osDelay(1000);
        }
        clearPathOutput();
        break;

    case CONBAT_AVOID:
    default:
        /* 未补具体动作的状态先保持安全停车。 */
        clearPathOutput();
        break;
    }
}

/*
 * 判断 conbat_task 是否正在接管底盘自动控制。
 * 非空闲状态都认为是激活状态，即使该状态暂时只输出零速度。
 */
uint8_t CONBAT_TASK::isActive(void) const
{
    return (state != CONBAT_IDLE) ? 1U : 0U;
}

/*
 * 获取 conbat_task 给底盘的目标速度。
 * 空闲时透传遥控速度；激活且有路径输出时返回路径跟随速度；激活但暂无具体动作时返回零速度。
 * chassis_task 通过这个函数把 conbat_task 纳入统一底盘控制权仲裁。
 */
uint8_t CONBAT_TASK::getChassisTarget(float manual_vx,
                                      float manual_vy,
                                      float manual_wz,
                                      float *target_vx,
                                      float *target_vy,
                                      float *target_wz) const
{
    /* 默认透传遥控目标，只有 conbat 激活时才覆盖底盘速度。 */
    *target_vx = manual_vx;
    *target_vy = manual_vy;
    *target_wz = manual_wz;

    if (isActive() == 0U)
    {
        return 0U;
    }

    if (pick_kfs_meiling_active_ != 0U)
    {
    }
    else if (path_active_ != 0U)
    {
        *target_vx = path_vx_target_;
        *target_vy = path_vy_target_;
        *target_wz = path_wz_target_;
    }
    else
    {
        /* 占位状态安全停车。 */
        *target_vx = 0.0f;
        *target_vy = 0.0f;
        *target_wz = 0.0f;
    }

    return 1U;
}

/*
 * 设置 KFS 放置点位编号。
 * 当前只支持 0、1、2 三个编号，超出范围时钳制到 2，避免数组路径选择越界。
 */
void CONBAT_TASK::setKfsPlaceIndex(uint8_t index)
{
    if (index > 2U)
    {
        index = 2U;
    }
    kfs_place_index_ = index;
}

/*
 * 获取当前选择的 KFS 放置点位编号。
 * 主要给调试观察或后续放置流程判断使用。
 */
uint8_t CONBAT_TASK::getKfsPlaceIndex(void) const
{
    return kfs_place_index_;
}

void CONBAT_TASK::resetKfsPlaceLaserFlag(void)
{
    // 重新开始一次放 KFS 检测，必须先挡住激光再松开才会重新置位。
    kfs_place_laser_release_flag = 0U;
    kfs_place_laser_blocked_ = 0U;
}

uint32_t CONBAT_KFS_PLACE_LASER_BLOCK_MM = 100U; // 左/右激光小于该距离时认为被 KFS 挡住，单位 mm。

uint8_t CONBAT_TASK::updateKfsPlaceLaserFlag(void)
{
    // 左右任意一侧激光有效且距离小于阈值，认为 KFS 正在挡住激光。
    const uint8_t left_blocked =
        (laser_left.data.valid != 0U &&
         laser_left.data.distance_mm < CONBAT_KFS_PLACE_LASER_BLOCK_MM)
            ? 1U
            : 0U;
    const uint8_t right_blocked =
        (laser_right.data.valid != 0U &&
         laser_right.data.distance_mm < CONBAT_KFS_PLACE_LASER_BLOCK_MM)
            ? 1U
            : 0U;
    const uint8_t laser_blocked = (left_blocked != 0U || right_blocked != 0U) ? 1U : 0U;

    if (laser_blocked != 0U)
    {
        // 挡住期间先清零标志位，保证下一次松开还能重新产生 0->1 变化。
        kfs_place_laser_release_flag = 0U;
        kfs_place_laser_blocked_ = 1U;
    }
    else if (kfs_place_laser_blocked_ != 0U)
    {
        // 曾经挡住后再次松开，说明 KFS 已离开激光束，置位放 KFS 标志。
        kfs_place_laser_release_flag = 1U;
        kfs_place_laser_blocked_ = 0U;
    }

    return kfs_place_laser_release_flag;
}

/*
 * 设置 conbat_task 专用 yaw 目标角。
 * 角度会归一化到 [-180, 180]，chassis_task 只有在 conbat_task 获得底盘控制权时才使用该目标。
 */
void CONBAT_TASK::setYawTarget(float yaw_degree)
{
    yaw_target_degree = normalizeYawDeg(yaw_degree);
    yaw_target_enabled = 1U;
}

uint8_t CONBAT_TASK::getLiftSwitch(uint8_t manual_switch) const
{
    if (state != CONBAT_COMBINE)
    {
        return manual_switch;
    }

    return lift_switch_target_;
}

float CONBAT_TASK::getLiftLinearSpeedTarget(float manual_target) const
{
    if (state != CONBAT_COMBINE)
    {
        return manual_target;
    }

    return lift_linear_speed_target_;
}

/*
 * 处理状态切换后的清理动作。
 * 每次进入新状态时重置路径跟随器和速度输出，避免沿用上一状态的路径进度或底盘目标。
 */
void CONBAT_TASK::handleStateChanged(void)
{
    /* 切状态时重新装载路径，避免沿用上一段路径跟随进度。 */
    path_follower_.reset();
    path_loaded_ = 0U;
    ramp_up_waiting_ = 0U;
    ramp_up_zero_yaw_done_ = 0U;
    pick_kfs_step_ = PICK_KFS_LOCK_ZERO;
    place_kfs_step_ = PLACE_KFS_LOWER_ACTION;
    combine_step_ = COMBINE_PRE_LIFT;
    pick_kfs_meiling_active_ = 0U;
    pick_kfs_second_forward_done_ = 0U;
    pick_kfs_path_stable_count_ = 0U;
    kfs_place_stop_stable_count_ = 0U;
    kfs_place_arrived_ = 0U;
    kfs_place_precision_active_ = 0U;
    place_kfs_pick_precision_active_ = 0U;
    combine_pre_lift_ready_ = 0U;
    combine_crossed_finish_height_ = 0U;
    combine_stable_count_ = 0U;
    combine_pre_lift_command_seq_ = 0U;
    combine_climb_lift_command_seq_ = 0U;
    combine_final_lift_command_seq_ = 0U;
    lift_switch_target_ = 0U;
    lift_linear_speed_target_ = 0.0f;
    pick_kfs_second_back_start_x_m_ = 0.0f;
    clearPathOutput();

    if (state == CONBAT_IDLE)
    {
        yaw_target_enabled = 0U;
    }

    if (state == CONBAT_COMBINE)
    {
        combine_pre_lift_command_seq_ = lift_calulate.command_seq;
    }

    last_state_ = state;
}

/*
 * 清空路径跟随输出。
 * 该函数只清速度接管结果，不改变主状态，用于结束跑点、状态占位和异常偏离后的安全停车。
 */
void CONBAT_TASK::clearPathOutput(void)
{
    path_active_ = 0U;
    path_vx_target_ = 0.0f;
    path_vy_target_ = 0.0f;
    path_wz_target_ = 0.0f;
}

/*
 * 上斜坡状态处理。
 */
static const float CONBAT_RAMP_UP_ZERO_YAW_TOL_DEG = 3.0f; // 上坡前锁 0 度的角度容差，单位度。

uint8_t CONBAT_TASK::runRampUp(void)
{
    if (ramp_up_zero_yaw_done_ == 0U)
    {
        clearPathOutput();
        setYawTarget(0.0f);
        if (fabsf(normalizeYawDeg(0.0f - vision.angle_x)) > CONBAT_RAMP_UP_ZERO_YAW_TOL_DEG)
        {
            return 0U;
        }

        ramp_up_zero_yaw_done_ = 1U;
    }

    /* 上坡完成后原地等待，避免重复生成路径后回追中间点。 */
    if (ramp_up_waiting_ != 0U)
    {
        clearPathOutput();
        return 0U;
    }

    /* 上坡状态：用当前雷达坐标生成到上坡终点的路径。 */
    uint8_t result = loadGeneratedPathToGoal(conbat_ramp_up_goals[0],
                                             conbat_ramp_up_middle_points,
                                             conbat_ramp_up_middle_point_count);
    if (result == 1U)
    {
        clearPathOutput();
        return 1U;
    }

    return result;
}

// 捡 KFS 流程参数。
float CONBAT_PICK_KFS_PATH_MAX_VEL_M_S = 1.5f;              // 捡 KFS 跑点的最大线速度，单位 m/s。
float CONBAT_PICK_KFS_PATH_MAX_ACC_M_S2 = 0.8f;             // 捡 KFS 跑点的最大加速度，单位 m/s2。
float CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_ACC_MPS2 = 0.9f;   // KFS 等待阶段车体前进加速度，单位 m/s2。
float CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_MAX_MPS = 1.0f;    // KFS 等待阶段车体前进最大速度，单位 m/s。
float CONBAT_PICK_KFS_FIRST_BACK_DISTANCE_M = 0.06f;        // KFS 吸取成功后沿 X 轴后退距离，单位 m。
float CONBAT_PICK_KFS_FIRST_BACK_SPEED_MPS = 0.4f;          // KFS 吸取成功后沿 X 轴后退速度，单位 m/s。
float CONBAT_PICK_GO_TO_COMBINE_KP = 1.6f;                  // 去合体目标点的二维位置 P 闭环系数。
float CONBAT_PICK_GO_TO_COMBINE_TOL_M = 0.03f;              // 去合体目标点的到位误差，单位 m。
float CONBAT_PICK_KFS_FIRST_WAIT_DT35_TARGET_MM = 455.0f;   // 第一个 KFS 边吸边前进的 DT35 目标距离。
float CONBAT_PICK_KFS_SECOND_WAIT_DT35_TARGET_MM = 440.0f;  // 第二个 KFS 边吸边前进的 DT35 目标距离。
static const float CONBAT_PICK_KFS_ZERO_YAW_TOL_DEG = 3.0f; // 捡 KFS 前锁 0 度的角度容差，单位度。

uint8_t CONBAT_TASK::runPickKfs(void)
{
    switch (pick_kfs_step_)
    {
    case PICK_KFS_LOCK_ZERO:
        /* 捡 KFS 前先停车锁 0 度，到位后再发送第一个机械臂动作。 */
        clearPathOutput();
        setYawTarget(0.0f);
        if (fabsf(normalizeYawDeg(0.0f - vision.angle_x)) > CONBAT_PICK_KFS_ZERO_YAW_TOL_DEG)
        {
            return 0U;
        }

        pick_kfs_step_ = PICK_KFS_FIRST_ACTION;
        return 0U;

    case PICK_KFS_FIRST_ACTION:
        /* 第一步：发送拾取第一个 KFS 的机械臂指令。 */
        if (arm_comm.executeAction(ArmComm::ACTION_PICK_FIRST_KFS, 1U) == 0U)
        {
            return 2U;
        }
        /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
        for (uint8_t i = 0U; i < 10U; ++i)
        {
            arm_comm.send();
        }

        pick_kfs_step_ = PICK_KFS_PATH_TO_AREA;
        pick_kfs_path_stable_count_ = 0U;

        return 0U;

    case PICK_KFS_PATH_TO_AREA:
    {
        /* 第二步：用梯形速度跑到 KFS 拾取区域的粗略点。 */
        if (arm_comm.rx_data_.event == 1U)
        {
            clearPathOutput();
            pick_kfs_second_forward_done_ = 0U;
            pick_kfs_path_stable_count_ = 0U;
            pick_kfs_step_ = PICK_KFS_SECOND_ACTION;
            return 0U;
        }

        pick_kfs_meiling_active_ = 0U;
        path_loaded_ = 0U;
        path_follower_.reset();

        const float x_err = conbat_pick_kfs_goals[0].x_m - vision.x_diff;
        const float y_err = conbat_pick_kfs_goals[0].y_m - vision.y_diff;
        if (conbat_stable_confirm((fabsf(x_err) < 0.08f && fabsf(y_err) < 0.03f) ? 1U : 0U,
                                  &pick_kfs_path_stable_count_,
                                  50U) != 0U)
        {
            clearPathOutput();

            pick_kfs_path_stable_count_ = 0U;
            pick_kfs_step_ = PICK_KFS_FIRST_WAIT_READY;

            return 0U;
        }

        /* 将上一周期车体系速度换回世界系，作为 P 闭环限加速度的起点。 */
        const float yaw_rad = vision.angle_x * CONBAT_DEG_TO_RAD;
        const float cos_yaw = cosf(yaw_rad);
        const float sin_yaw = sinf(yaw_rad);
        const float last_world_vx = cos_yaw * path_vx_target_ - sin_yaw * path_vy_target_;
        const float last_world_vy = sin_yaw * path_vx_target_ + cos_yaw * path_vy_target_;
        float world_vx = 0.0f;
        float world_vy = 0.0f;

        /* 根据二维位置误差做 P 闭环速度规划，同时限制最大速度和加速度。 */
        conbat_position_p_speed(x_err,
                                y_err,
                                CONBAT_PICK_GO_TO_COMBINE_KP,
                                CONBAT_PICK_KFS_PATH_MAX_VEL_M_S,
                                CONBAT_PICK_KFS_PATH_MAX_ACC_M_S2,
                                0.001f,
                                last_world_vx,
                                last_world_vy,
                                &world_vx,
                                &world_vy);

        /* 底盘接口使用车体系速度，这里再从世界系转回车体系。 */
        PathFollower::worldToBody(world_vx,
                                  world_vy,
                                  yaw_rad,
                                  &path_vx_target_,
                                  &path_vy_target_);
        path_wz_target_ = 0.0f;
        path_active_ = 1U;
        return 0U;
    }

    case PICK_KFS_FIRST_WAIT_READY:
        /* 第三步：等待机械臂回传 event=5，允许进入第一个 KFS 边走边吸取阶段。 */
        clearPathOutput();
        if (arm_comm.rx_data_.event == 1U)
        {
            pick_kfs_second_forward_done_ = 0U;
            pick_kfs_path_stable_count_ = 0U;
            pick_kfs_step_ = PICK_KFS_SECOND_ACTION;
            return 0U;
        }

        if (arm_comm.rx_data_.event == 5U)
        {
            pick_kfs_step_ = PICK_KFS_FIRST_WAIT_DONE;
        }
        return 0U;

    case PICK_KFS_FIRST_WAIT_DONE:
        /* 第四步：边向前走边吸取，机械臂回传 event=1 后立即停车并进入下一步。 */
        if (arm_comm.rx_data_.event == 1U)
        {
            clearPathOutput();
            pick_kfs_second_forward_done_ = 0U;
            pick_kfs_first_back_start_x_m_ = vision.x_diff;
            pick_kfs_step_ = PICK_KFS_FIRST_BACKWARD;
            return 0U;
        }

        clearPathOutput();
        if (CONBAT_PICK_KFS_FIRST_WAIT_DT35_TARGET_MM > 0.0f && dt35.ch2.valid != 0U)
        {
            const float laser_mm = dt35.ch2.distance_filtered;
            path_active_ = 1U;
            path_vx_target_ = conbat_trapezoid_speed((laser_mm - CONBAT_PICK_KFS_FIRST_WAIT_DT35_TARGET_MM) * 0.001f,
                                                     CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_ACC_MPS2,
                                                     CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_MAX_MPS);
            path_vy_target_ = 0.0f;
            path_wz_target_ = 0.0f;
        }
        return 0U;

    case PICK_KFS_FIRST_BACKWARD:
    {
        /* 第一个 KFS 吸取成功后，按视觉 X 坐标确认至少后退 6cm，再发送第二个 KFS 机械臂动作。 */
        const float x_back_m = pick_kfs_first_back_start_x_m_ - vision.x_diff;
        if (x_back_m >= CONBAT_PICK_KFS_FIRST_BACK_DISTANCE_M)
        {
            clearPathOutput();
            pick_kfs_path_stable_count_ = 0U;
            pick_kfs_step_ = PICK_KFS_SECOND_ACTION;
            return 0U;
        }

        /* 这里用世界系 X 负方向后退，再转换成底盘车体系速度输出。 */
        const float yaw_rad = vision.angle_x * CONBAT_DEG_TO_RAD;
        PathFollower::worldToBody(-CONBAT_PICK_KFS_FIRST_BACK_SPEED_MPS,
                                  0.0f,
                                  yaw_rad,
                                  &path_vx_target_,
                                  &path_vy_target_);
        path_wz_target_ = 0.0f;
        path_active_ = 1U;
        return 0U;
    }

    case PICK_KFS_SECOND_ACTION:
        /* 第五步：发送拾取第二个 KFS 的机械臂指令。 */
        if (arm_comm.executeAction(ArmComm::ACTION_PICK_SECOND_KFS, 2U) == 0U)
        {
            return 2U;
        }
        /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
        for (uint8_t i = 0U; i < 10U; ++i)
        {
            arm_comm.send();
        }
        pick_kfs_second_forward_done_ = 0U;
        pick_kfs_path_stable_count_ = 0U;
        pick_kfs_step_ = PICK_KFS_PATH_TO_SECOND_AREA;
        return 0U;

    case PICK_KFS_PATH_TO_SECOND_AREA:
    {
        /* 第六步：用梯形速度跑到 KFS 2 拾取区域的粗略点。 */
        pick_kfs_meiling_active_ = 0U;
        path_loaded_ = 0U;
        path_follower_.reset();

        const float x_err = conbat_pick_kfs_goals[1].x_m - vision.x_diff;
        const float y_err = conbat_pick_kfs_goals[1].y_m - vision.y_diff;
        if (conbat_stable_confirm((fabsf(x_err) < 0.08f && fabsf(y_err) < 0.03f) ? 1U : 0U,
                                  &pick_kfs_path_stable_count_,
                                  10U) != 0U)
        {
            clearPathOutput();
            pick_kfs_path_stable_count_ = 0U;
            pick_kfs_step_ = PICK_KFS_SECOND_WAIT_READY;
            return 0U;
        }

        /* 将上一周期车体系速度换回世界系，作为 P 闭环限加速度的起点。 */
        const float yaw_rad = vision.angle_x * CONBAT_DEG_TO_RAD;
        const float cos_yaw = cosf(yaw_rad);
        const float sin_yaw = sinf(yaw_rad);
        const float last_world_vx = cos_yaw * path_vx_target_ - sin_yaw * path_vy_target_;
        const float last_world_vy = sin_yaw * path_vx_target_ + cos_yaw * path_vy_target_;
        float world_vx = 0.0f;
        float world_vy = 0.0f;

        /* 根据二维位置误差做 P 闭环速度规划，同时限制最大速度和加速度。 */
        conbat_position_p_speed(x_err,
                                y_err,
                                CONBAT_PICK_GO_TO_COMBINE_KP,
                                CONBAT_PICK_KFS_PATH_MAX_VEL_M_S,
                                CONBAT_PICK_KFS_PATH_MAX_ACC_M_S2,
                                0.001f,
                                last_world_vx,
                                last_world_vy,
                                &world_vx,
                                &world_vy);

        /* 底盘接口使用车体系速度，这里再从世界系转回车体系。 */
        PathFollower::worldToBody(world_vx,
                                  world_vy,
                                  yaw_rad,
                                  &path_vx_target_,
                                  &path_vy_target_);
        path_wz_target_ = 0.0f;
        path_active_ = 1U;
        return 0U;
    }

    case PICK_KFS_SECOND_WAIT_READY:
        /* 第七步：等待 event=5 后，按前向 DT35 距离边走边吸取。 */
        if (pick_kfs_second_forward_done_ == 0U)
        {
            clearPathOutput();
            if (arm_comm.rx_data_.event == 5U)
            {
                pick_kfs_second_forward_done_ = 1U;
            }
            return 0U;
        }

        if (arm_comm.rx_data_.event == 1U)
        {
            pick_kfs_second_forward_done_ = 0U;
            clearPathOutput();
            pick_kfs_path_stable_count_ = 0U;
            pick_kfs_second_back_start_x_m_ = vision.x_diff;
            pick_kfs_step_ = PICK_KFS_SECOND_BACKWARD;
            return 0U;
        }

        clearPathOutput();
        if (CONBAT_PICK_KFS_SECOND_WAIT_DT35_TARGET_MM > 0.0f && dt35.ch2.valid != 0U)
        {
            const float laser_mm = dt35.ch2.distance_filtered;
            path_active_ = 1U;
            path_vx_target_ = conbat_trapezoid_speed((laser_mm - CONBAT_PICK_KFS_SECOND_WAIT_DT35_TARGET_MM) * 0.001f,
                                                     CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_ACC_MPS2,
                                                     CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_MAX_MPS);
            path_vy_target_ = 0.0f;
            path_wz_target_ = 0.0f;
        }
        return 0U;

    case PICK_KFS_SECOND_BACKWARD:
    {
        /* 第二个 KFS 吸取成功后，按视觉 X 坐标确认至少后退 6cm，再跑向合体目标点。 */
        const float x_back_m = pick_kfs_second_back_start_x_m_ - vision.x_diff;
        if (x_back_m >= CONBAT_PICK_KFS_FIRST_BACK_DISTANCE_M)
        {
            clearPathOutput();
            pick_kfs_path_stable_count_ = 0U;
            pick_kfs_step_ = PICK_GO_TO_COMBINE;
            return 0U;
        }

        /* 这里用世界系 X 负方向后退，再转换成底盘车体系速度输出。 */
        const float yaw_rad = vision.angle_x * CONBAT_DEG_TO_RAD;
        PathFollower::worldToBody(-CONBAT_PICK_KFS_FIRST_BACK_SPEED_MPS,
                                  0.0f,
                                  yaw_rad,
                                  &path_vx_target_,
                                  &path_vy_target_);
        path_wz_target_ = 0.0f;
        path_active_ = 1U;
        return 0U;
    }

    case PICK_GO_TO_COMBINE:
    {
        const float x_err = conbat_combine_goals[0].x_m - vision.x_diff;
        const float y_err = conbat_combine_goals[0].y_m - vision.y_diff;
        const uint8_t position_reached = (fabsf(x_err) < CONBAT_PICK_GO_TO_COMBINE_TOL_M &&
                                          fabsf(y_err) < CONBAT_PICK_GO_TO_COMBINE_TOL_M)
                                             ? 1U
                                             : 0U;

        if (position_reached != 0U)
        {
            setYawTarget(-90.0f);
        }

        if (conbat_stable_confirm(position_reached,
                                  &pick_kfs_path_stable_count_,
                                  10U) != 0U)
        {
            pick_kfs_path_stable_count_ = 0U;
            clearPathOutput();
            return 1U;
        }

        const float yaw_rad = vision.angle_x * CONBAT_DEG_TO_RAD;
        const float cos_yaw = cosf(yaw_rad);
        const float sin_yaw = sinf(yaw_rad);
        const float last_world_vx = cos_yaw * path_vx_target_ - sin_yaw * path_vy_target_;
        const float last_world_vy = sin_yaw * path_vx_target_ + cos_yaw * path_vy_target_;
        float world_vx = 0.0f;
        float world_vy = 0.0f;

        conbat_position_p_speed(x_err,
                                y_err,
                                CONBAT_PICK_GO_TO_COMBINE_KP,
                                CONBAT_PICK_KFS_PATH_MAX_VEL_M_S,
                                CONBAT_PICK_KFS_PATH_MAX_ACC_M_S2,
                                0.001f,
                                last_world_vx,
                                last_world_vy,
                                &world_vx,
                                &world_vy);

        PathFollower::worldToBody(world_vx,
                                  world_vy,
                                  yaw_rad,
                                  &path_vx_target_,
                                  &path_vy_target_);
        path_wz_target_ = 0.0f;
        path_active_ = 1U;
        return 0U;
    }

    default:
        /* 异常子状态：释放底盘控制并让上层回到空闲。 */
        pick_kfs_meiling_active_ = 0U;
        clearPathOutput();
        return 2U;
    }
}

// 放 KFS 流程中的取车内 KFS 参数。
float CONBAT_PLACE_KFS_PICK_PATH_MAX_VEL_M_S = 1.6f;  // 取最后一个车内 KFS 跑点的最大线速度，单位 m/s。
float CONBAT_PLACE_KFS_PICK_PATH_MAX_ACC_M_S2 = 1.2f; // 取最后一个车内 KFS 跑点的最大加速度，单位 m/s2。
static const float CONBAT_KFS_PLACE_STOP_SPEED_LIMIT = 0.01f;
static const uint16_t CONBAT_KFS_PLACE_STOP_STABLE_COUNT = 100U;
static const float CONBAT_KFS_PLACE_YAW_TOL_DEG = 3.0f;
float CONBAT_PLACE_KFS_FORWARD_DISTANCE_M = 0.15f; // 放置 KFS 后沿车头方向前进距离，单位 m。
float CONBAT_PLACE_KFS_FORWARD_SPEED_MPS = 0.4f;   // 放置 KFS 后沿车头方向前进速度，单位 m/s。

uint8_t CONBAT_TASK::runPlaceKfs(void)
{
    switch (place_kfs_step_)
    {
    case PLACE_KFS_LOWER_ACTION:
        /* 第一步：发送机械臂放车内底层 KFS 的动作，随后去取 KFS 的精确点。 */

        clearPathOutput();

        if (arm_comm.executeAction(ArmComm::ACTION_PICK_THIRD_KFS, 0U) == 0U)
        {
            return 2U;
        }
        arm_comm.send();
        pick_kfs_path_stable_count_ = 0U;
        place_kfs_pick_precision_active_ = 0U;
        place_kfs_step_ = PLACE_KFS_PATH_TO_PICK;
        return 0U;

    case PLACE_KFS_PATH_TO_PICK:
    {
        /* 第二步：先用 B 样条跑到取 KFS 粗略点，再用二维 P 闭环精定位。 */
        if (place_kfs_pick_precision_active_ == 0U)
        {
            uint8_t result = loadGeneratedPathToGoal(conbat_pick_kfs_goals[2],
                                                     conbat_pick_kfs_middle_points,
                                                     conbat_pick_kfs_middle_point_count,
                                                     CONBAT_PLACE_KFS_PICK_PATH_MAX_VEL_M_S,
                                                     CONBAT_PLACE_KFS_PICK_PATH_MAX_ACC_M_S2);
            if (result != 1U)
            {
                return result;
            }

            place_kfs_pick_precision_active_ = 1U;
            pick_kfs_path_stable_count_ = 0U;
        }

        const float x_err = conbat_pick_kfs_goals[2].x_m - vision.x_diff;
        const float y_err = conbat_pick_kfs_goals[2].y_m - vision.y_diff;
        const uint8_t position_reached = (fabsf(x_err) < CONBAT_PICK_GO_TO_COMBINE_TOL_M &&
                                          fabsf(y_err) < CONBAT_PICK_GO_TO_COMBINE_TOL_M)
                                             ? 1U
                                             : 0U;

        if (conbat_stable_confirm(position_reached,
                                  &pick_kfs_path_stable_count_,
                                  10U) != 0U)
        {
            clearPathOutput();
            place_kfs_pick_precision_active_ = 0U;
            pick_kfs_path_stable_count_ = 0U;
            place_kfs_step_ = PLACE_KFS_DT35_FORWARD;
            return 0U;
        }

        /* 用上一周期车体系速度换算出的世界系速度作为限加速起点。 */
        const float yaw_rad = vision.angle_x * CONBAT_DEG_TO_RAD;
        const float cos_yaw = cosf(yaw_rad);
        const float sin_yaw = sinf(yaw_rad);
        const float last_world_vx = cos_yaw * path_vx_target_ - sin_yaw * path_vy_target_;
        const float last_world_vy = sin_yaw * path_vx_target_ + cos_yaw * path_vy_target_;
        float world_vx = 0.0f;
        float world_vy = 0.0f;

        conbat_position_p_speed(x_err,
                                y_err,
                                CONBAT_PICK_GO_TO_COMBINE_KP,
                                CONBAT_PLACE_KFS_PICK_PATH_MAX_VEL_M_S,
                                CONBAT_PLACE_KFS_PICK_PATH_MAX_ACC_M_S2,
                                0.001f,
                                last_world_vx,
                                last_world_vy,
                                &world_vx,
                                &world_vy);

        /* 底盘接口使用车体系速度，这里从世界系转回车体系输出。 */
        PathFollower::worldToBody(world_vx,
                                  world_vy,
                                  yaw_rad,
                                  &path_vx_target_,
                                  &path_vy_target_);
        path_wz_target_ = 0.0f;
        path_active_ = 1U;
        return 0U;
    }

    case PLACE_KFS_DT35_FORWARD:
        /* 第三步：按 DT35 距离边向前走边吸取，机械臂回传 event=1 后先后退一小段。 */
        if (arm_comm.rx_data_.event == 1U)
        {
            clearPathOutput();
            place_kfs_back_start_x_m_ = vision.x_diff;
            place_kfs_step_ = PLACE_KFS_BACKWARD;
            return 0U;
        }

        clearPathOutput();
        if (CONBAT_PICK_KFS_SECOND_WAIT_DT35_TARGET_MM > 0.0f && dt35.ch2.valid != 0U)
        {
            const float laser_mm = dt35.ch2.distance_filtered;
            path_active_ = 1U;
            path_vx_target_ = conbat_trapezoid_speed((laser_mm - CONBAT_PICK_KFS_SECOND_WAIT_DT35_TARGET_MM) * 0.001f,
                                                     CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_ACC_MPS2,
                                                     CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_MAX_MPS);
            path_vy_target_ = 0.0f;
            path_wz_target_ = 0.0f;
        }
        return 0U;

    case PLACE_KFS_BACKWARD:
    {
        /* KFS 吸取成功后，确认至少后退 6cm，再进入放置路径。 */
        const float x_back_m = place_kfs_back_start_x_m_ - vision.x_diff;
        if (x_back_m >= CONBAT_PICK_KFS_FIRST_BACK_DISTANCE_M)
        {
            clearPathOutput();
            setYawTarget(-90.0f);
            if (fabsf(normalizeYawDeg(-90.0f - vision.angle_x)) > CONBAT_KFS_PLACE_YAW_TOL_DEG)
            {
                return 0U;
            }
            path_loaded_ = 0U;
            kfs_place_arrived_ = 0U;
            kfs_place_stop_stable_count_ = 0U;
            place_kfs_step_ = PLACE_KFS_PATH_TO_PLACE;
            return 0U;
        }

        /* 这里用世界系 X 负方向后退，再转换成底盘车体系速度输出。 */
        const float yaw_rad = vision.angle_x * CONBAT_DEG_TO_RAD;
        PathFollower::worldToBody(-CONBAT_PICK_KFS_FIRST_BACK_SPEED_MPS,
                                  0.0f,
                                  yaw_rad,
                                  &path_vx_target_,
                                  &path_vy_target_);
        path_wz_target_ = 0.0f;
        path_active_ = 1U;
        return 0U;
    }

    case PLACE_KFS_PATH_TO_PLACE:
    {
        /* 第四步：跑到当前选择的放 KFS 终点，到位并停稳后发送放置动作。 */

        if (kfs_place_arrived_ == 0U)
        {
            const uint8_t path_result = runSelectKfsPlace();
            if (path_result == 1U)
            {
                clearPathOutput();
                kfs_place_arrived_ = 1U;
            }
            else if (path_result == 2U)
            {
                return 2U;
            }
            else
            {
                return 0U;
            }
        }

        const uint8_t chassis_stopped =
            (fabsf(omni_chassis.now.Vx) < CONBAT_KFS_PLACE_STOP_SPEED_LIMIT &&
             fabsf(omni_chassis.now.Vy) < CONBAT_KFS_PLACE_STOP_SPEED_LIMIT &&
             fabsf(omni_chassis.now.Vz) < CONBAT_KFS_PLACE_STOP_SPEED_LIMIT)
                ? 1U
                : 0U;

        clearPathOutput();
        if (chassis_stopped != 0U)
        {
            if (kfs_place_stop_stable_count_ < CONBAT_KFS_PLACE_STOP_STABLE_COUNT)
            {
                ++kfs_place_stop_stable_count_;
            }
        }
        else
        {
            kfs_place_stop_stable_count_ = 0U;
        }

        if (kfs_place_stop_stable_count_ < CONBAT_KFS_PLACE_STOP_STABLE_COUNT)
        {
            return 0U;
        }

        arm_comm.executeAction(ArmComm::ACTION_ZONE3_PLACE_FINALL, 1U);
        /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
        for (uint8_t i = 0U; i < 10U; ++i)
        {
            arm_comm.send();
        }
        kfs_place_arrived_ = 0U;
        place_kfs_forward_start_x_m_ = vision.x_diff;
        place_kfs_forward_start_y_m_ = vision.y_diff;
        place_kfs_forward_start_yaw_deg_ = vision.angle_x;
        place_kfs_step_ = PLACE_KFS_FORWARD_AFTER_PLACE;
        return 0U;
    }

    case PLACE_KFS_FORWARD_AFTER_PLACE:
    {
        /* 放置完成后沿车头方向前进，超过 15cm 即认为结束。 */
        const float yaw_rad = place_kfs_forward_start_yaw_deg_ * CONBAT_DEG_TO_RAD;
        const float dx = vision.x_diff - place_kfs_forward_start_x_m_;
        const float dy = vision.y_diff - place_kfs_forward_start_y_m_;
        const float forward_m = cosf(yaw_rad) * dx + sinf(yaw_rad) * dy;
        if (forward_m >= CONBAT_PLACE_KFS_FORWARD_DISTANCE_M)
        {
            clearPathOutput();
            path_loaded_ = 0U;
            place_kfs_step_ = PLACE_KFS_WAIT;
            return 0U;
        }

        path_vx_target_ = CONBAT_PLACE_KFS_FORWARD_SPEED_MPS;
        path_vy_target_ = 0.0f;
        path_wz_target_ = 0.0f;
        path_active_ = 1U;
        return 0U;
    }

    case PLACE_KFS_WAIT:
    {
        /* 放置后跑到偏角等待点。 */
        const uint8_t path_result = loadGeneratedPathToGoal(conbat_kfs_wait_goal,
                                                            conbat_kfs_wait_middle_points,
                                                            conbat_kfs_wait_middle_point_count);
        if (path_result == 1U)
        {
            clearPathOutput();
            return 1U;
        }
        if (path_result == 2U)
        {
            return 2U;
        }
        return 0U;
    }

    default:
        clearPathOutput();
        return 2U;
    }
}

// 合体流程参数。
float CONBAT_COMBINE_CLIMB_FINISH_MM = 255.0f;         // 合体爬升结束时 DT35 判定距离，单位 mm。
float CONBAT_COMBINE_FORWARD_TARGET_MM = 65.0f;        // 合体最后车体前向靠近的 DT35 目标距离，0 表示跳过。
float CONBAT_COMBINE_FORWARD_TOL_MM = 8.0f;            // 合体最后前向靠近的 DT35 到位允许误差，单位 mm。
float CONBAT_COMBINE_FORWARD_ACC_MPS2 = 0.5f;          // 合体最后前向靠近的加速度，单位 m/s2。
float CONBAT_COMBINE_FORWARD_MAX_MPS = 0.9f;           // 合体最后前向靠近的最大速度，单位 m/s。
float CONBAT_COMBINE_LIFT_ACC_MPS2 = 0.4f;             // 合体举升阶段速度规划加速度，单位 m/s2。
float CONBAT_COMBINE_LIFT_MAX_MPS = 0.5f;              // 合体举升阶段最大速度，单位 m/s。
float CONBAT_COMBINE_LIFT_MIN_MPS = 0.2f;              // 合体举升阶段最小速度，单位 m/s。
float CONBAT_COMBINE_LIFT_HEIGHT_TOLERANCE_MM = 10.0f; // 合体举升目标高度允许误差，单位 mm。
uint8_t CONBAT_COMBINE_STABLE_COUNT = 10U;             // 合体到位判定需要连续稳定的次数。

/*
 * 合体状态处理。
 * 预抬、等待 2 档和 2006 爬升阶段底盘不动，最后才按 ch2 车体前向靠近。
 */
uint8_t CONBAT_TASK::runCombine(void)
{
    clearPathOutput();

    switch (combine_step_)
    {
    case COMBINE_PRE_LIFT:
        /*
         * 阶段1：先把升降机构切到 1 档预抬。
         * 这里不让底盘动，确认 lift_task 已接收到 1 档并完成后，再打开气缸。
         */
        lift_switch_target_ = 1U;
        lift_linear_speed_target_ = 0.0f;

        if (lift_calulate.command_seq != combine_pre_lift_command_seq_)
        {
            if (lift_calulate.finished != 0U)
            {
                combine_pre_lift_ready_ = 1U;
            }
        }

        if (combine_pre_lift_ready_ != 0U)
        {
            STEP_UP_CYLINDER_OPEN();
            combine_climb_lift_command_seq_ = lift_calulate.command_seq;
            combine_stable_count_ = 0U;

            combine_step_ = COMBINE_WAIT_CLIMB_HEIGHT;

            osDelay(500); // 等待气缸完毕
        }
        return 0U;

    case COMBINE_WAIT_CLIMB_HEIGHT:
    {
        /*
         * 阶段2：打开气缸后切到 2 档。
         * 必须等左右两侧实际高度都接近目标高度，才允许进入 2006 爬升阶段。
         */
        lift_switch_target_ = 2U;
        lift_linear_speed_target_ = 0.0f;

        const uint8_t climb_height_reached =
            (fabsf(lift_class.left.height - lift_calulate.target_height) <= CONBAT_COMBINE_LIFT_HEIGHT_TOLERANCE_MM &&
             fabsf(lift_class.right.height - lift_calulate.target_height) <= CONBAT_COMBINE_LIFT_HEIGHT_TOLERANCE_MM)
                ? 1U
                : 0U;

        if (lift_calulate.command_seq != combine_climb_lift_command_seq_ &&
            climb_height_reached != 0U)
        {
            combine_stable_count_ = 0U;
            combine_crossed_finish_height_ = 0U;

            combine_step_ = COMBINE_CLIMB_FORWARD;
        }
        return 0U;
    }

    case COMBINE_CLIMB_FORWARD:
    {
        /*
         * 阶段3：底盘继续保持不动，只用 ch2 激光逻辑驱动 2006。
         * 逻辑参考 STEP_UP_CLIMB_FORWARD：先看到距离越过完成阈值，再回落到阈值内才算爬升完成。
         */
        lift_switch_target_ = 2U;

        if (dt35.ch2.valid != 0U && dt35.ch2.distance_filtered > CONBAT_COMBINE_CLIMB_FINISH_MM &&
            combine_crossed_finish_height_ == 0U)
        {
            combine_crossed_finish_height_ = 1U;
        }

        const uint8_t laser_reached =
            (combine_crossed_finish_height_ != 0U &&
             dt35.ch2.valid != 0U &&
             dt35.ch2.distance_filtered <= CONBAT_COMBINE_CLIMB_FINISH_MM)
                ? 1U
                : 0U;

        if (laser_reached != 0U)
        {
            lift_linear_speed_target_ = 0.0f;
        }
        else if (dt35.ch2.valid != 0U && combine_crossed_finish_height_ != 0U)
        {
            const float err = (dt35.ch2.distance_filtered - CONBAT_COMBINE_CLIMB_FINISH_MM) * 0.001f;
            lift_linear_speed_target_ = conbat_trapezoid_speed(err,
                                                               CONBAT_COMBINE_LIFT_ACC_MPS2,
                                                               CONBAT_COMBINE_LIFT_MAX_MPS);
            lift_linear_speed_target_ = conbat_min_abs_speed(lift_linear_speed_target_,
                                                             CONBAT_COMBINE_LIFT_MIN_MPS,
                                                             CONBAT_COMBINE_LIFT_MAX_MPS);
        }
        else
        {
            lift_linear_speed_target_ = 0.0f;
        }

        if (conbat_stable_confirm(laser_reached, &combine_stable_count_, CONBAT_COMBINE_STABLE_COUNT) != 0U)
        {
            lift_linear_speed_target_ = 0.0f;
            combine_stable_count_ = 0U;

            combine_final_lift_command_seq_ = lift_calulate.command_seq;
            combine_step_ = COMBINE_ZONE3_READY;
        }
        return 0U;
    }

    case COMBINE_ZONE3_READY:
        /*
         * 阶段4：爬升完成后先让机械臂进入九宫格预备位，再接原来的回 1 档等待流程。
         */
        lift_linear_speed_target_ = 0.0f;
        lift_switch_target_ = 1U;
        if (arm_comm.executeAction(ArmComm::ACTION_ZONE3_READY, 0U) == 0U)
        {
            return 2U;
        }
        /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
        arm_comm.send();
        STEP_UP_CYLINDER_CLOSE();
        combine_stable_count_ = 0U;

        combine_step_ = COMBINE_WAIT_FINAL_LIFT_HEIGHT;

        return 0U;

    case COMBINE_WAIT_FINAL_LIFT_HEIGHT:
    {
        /*
         * 阶段5-1：爬升完成后收气缸，并等待升降实际回到 1 档高度。
         * 这里不允许底盘前向，避免升降还没到位时车体先动。
         */
        lift_switch_target_ = 1U;
        lift_linear_speed_target_ = 0.0f;

        const uint8_t final_lift_height_reached =
            (fabsf(lift_class.left.height - lift_calulate.target_height) <= CONBAT_COMBINE_LIFT_HEIGHT_TOLERANCE_MM &&
             fabsf(lift_class.right.height - lift_calulate.target_height) <= CONBAT_COMBINE_LIFT_HEIGHT_TOLERANCE_MM)
                ? 1U
                : 0U;

        if (lift_calulate.command_seq != combine_final_lift_command_seq_ &&
            final_lift_height_reached != 0U)
        {
            combine_stable_count_ = 0U;

            combine_step_ = COMBINE_FINAL_FORWARD;
        }
        return 0U;
    }

    case COMBINE_FINAL_FORWARD:
        /*
         * 阶段5-2：1 档高度到位后，底盘按车体前向 vx 靠 ch2 继续走到目标距离。
         */
        lift_switch_target_ = 1U;
        lift_linear_speed_target_ = 0.0f;

        if (CONBAT_COMBINE_FORWARD_TARGET_MM <= 0.0f)
        {
            combine_step_ = COMBINE_WAIT_PLACE_HAND_DT35;
            return 0U;
        }

        if (dt35.ch2.valid != 0U)
        {
            const float laser_mm = dt35.ch2.distance_filtered;
            const uint8_t arrived =
                (laser_mm <= (CONBAT_COMBINE_FORWARD_TARGET_MM + CONBAT_COMBINE_FORWARD_TOL_MM)) ? 1U : 0U;

            if (conbat_stable_confirm(arrived, &combine_stable_count_, CONBAT_COMBINE_STABLE_COUNT) != 0U)
            {
                clearPathOutput();
                combine_step_ = COMBINE_WAIT_PLACE_HAND_DT35;

                return 0U;
            }

            path_active_ = 1U;
            path_vx_target_ = conbat_trapezoid_speed((laser_mm - CONBAT_COMBINE_FORWARD_TARGET_MM) * 0.001f,
                                                     CONBAT_COMBINE_FORWARD_ACC_MPS2,
                                                     CONBAT_COMBINE_FORWARD_MAX_MPS);
            path_vy_target_ = 0.0f;
            path_wz_target_ = 0.0f;
        }
        return 0U;

    case COMBINE_WAIT_PLACE_HAND_DT35:
        /*
         * 阶段5-3：COMBINE_FINAL_FORWARD 结束后，先确认前方 DT35 大于 10cm，
         * 再进入放手持 KFS 流程。
         */
        lift_switch_target_ = 1U;
        lift_linear_speed_target_ = 0.0f;
        clearPathOutput();

        if (dt35.ch2.valid != 0U && dt35.ch2.distance_filtered > 100.0f)
        {
            resetKfsPlaceLaserFlag();
            combine_step_ = COMBINE_PLACE_HAND;
        }
        return 0U;

    case COMBINE_PLACE_HAND:
        /*
         * 阶段5-4：车体靠近完成后，等待手持 KFS 挡住侧激光后再松开，
         * 再发送机械臂放手持 KFS 命令。
         */
        lift_switch_target_ = 1U;
        lift_linear_speed_target_ = 0.0f;
        clearPathOutput();

        updateKfsPlaceLaserFlag();
        if (kfs_place_laser_release_flag != 0U)
        {
            if (arm_comm.executeAction(ArmComm::ACTION_ZONE3_PLACE_HAND, 0U) == 0U)
            {
                return 0U;
            }
            /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
            for (uint8_t i = 0U; i < 10U; ++i)
            {
                arm_comm.send();
            }
            resetKfsPlaceLaserFlag();
            combine_step_ = COMBINE_PLACE_LOWER_KFS;
            return 0U;
        }
        return 0U;

    case COMBINE_PLACE_LOWER_KFS:
        /*
         * 阶段5-4：等待车内底层 KFS 挡住侧激光后再松开，
         * 再发送机械臂放车内底层 KFS 命令。
         */
        lift_switch_target_ = 1U;
        lift_linear_speed_target_ = 0.0f;
        clearPathOutput();

        updateKfsPlaceLaserFlag();
        if (kfs_place_laser_release_flag != 0U)
        {
            if (arm_comm.executeAction(ArmComm::ACTION_ZONE3_PLACE_LOWER, 0U) == 0U)
            {
                return 0U;
            }
            /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
            for (uint8_t i = 0U; i < 10U; ++i)
            {
                arm_comm.send();
            }
            resetKfsPlaceLaserFlag();
            return 1U;
        }
        return 0U;

    default:
        lift_linear_speed_target_ = 0.0f;
        clearPathOutput();
        return 2U;
    }
}

float CONBAT_KFS_PLACE_PATH_MAX_VEL_M_S = 1.5f;  // KFS 放置点路径的最大线速度，单位 m/s。
float CONBAT_KFS_PLACE_PATH_MAX_ACC_M_S2 = 0.8f; // KFS 放置点路径的最大加速度，单位 m/s2。

/*
 * 选择 KFS 放置点位状态处理。
 * 根据 kfs_place_index_ 选择对应路径，并把路径跟随速度交给底盘。
 */
uint8_t CONBAT_TASK::runSelectKfsPlace(void)
{
    /* 根据选择的 KFS 点位加载对应路径，默认使用 KFS1。 */
    uint8_t selected_index = 0U;
    if (kfs_place_index_ == 1U || kfs_place_index_ == 2U)
    {
        selected_index = kfs_place_index_;
    }

    const BRPathPose &goal = conbat_kfs_place_goals[selected_index];
    const BRPathControlPoint *middle_points = conbat_kfs_place_middle_points[selected_index];
    std::size_t middle_point_count = conbat_kfs_place_middle_point_counts[selected_index];

    if (kfs_place_precision_active_ == 0U)
    {
        if (middle_point_count > 0U &&
            ((goal.y_m - vision.y_diff) * (middle_points[0].y_m - vision.y_diff)) <= 0.0f)
        {
            /* 中间点已经在车后面时跳过，避免路径起步反向绕回去。 */
            middle_points = 0;
            middle_point_count = 0U;
        }

        /* 第一步先跑 B 样条到放置点附近，完成后再进入终点 P 闭环。 */
        const uint8_t path_result = loadGeneratedPathToGoal(goal,
                                                            middle_points,
                                                            middle_point_count,
                                                            CONBAT_KFS_PLACE_PATH_MAX_VEL_M_S,
                                                            CONBAT_KFS_PLACE_PATH_MAX_ACC_M_S2);
        if (path_result == 0U)
        {
            return 0U;
        }

        path_loaded_ = 0U;

        if (path_result != 1U)
        {
            clearPathOutput();
            return path_result;
        }

        kfs_place_precision_active_ = 1U;
    }

    const float x_err = goal.x_m - vision.x_diff;
    const float y_err = goal.y_m - vision.y_diff;
    if (fabsf(x_err) <= 0.05f && fabsf(y_err) <= 0.05f)
    {
        clearPathOutput();
        kfs_place_precision_active_ = 0U;
        return 1U;
    }

    /* B 样条完成后，继续用二维 P 精确跑到 5cm 误差内。 */
    const float yaw_rad = vision.angle_x * CONBAT_DEG_TO_RAD;
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);
    const float last_world_vx = cos_yaw * path_vx_target_ - sin_yaw * path_vy_target_;
    const float last_world_vy = sin_yaw * path_vx_target_ + cos_yaw * path_vy_target_;
    float world_vx = 0.0f;
    float world_vy = 0.0f;

    conbat_position_p_speed(x_err,
                            y_err,
                            CONBAT_PICK_GO_TO_COMBINE_KP,
                            CONBAT_KFS_PLACE_PATH_MAX_VEL_M_S,
                            CONBAT_KFS_PLACE_PATH_MAX_ACC_M_S2,
                            0.001f,
                            last_world_vx,
                            last_world_vy,
                            &world_vx,
                            &world_vy);

    PathFollower::worldToBody(world_vx,
                              world_vy,
                              yaw_rad,
                              &path_vx_target_,
                              &path_vy_target_);
    path_wz_target_ = 0.0f;
    path_active_ = 1U;
    return 0U;
}

float CONBAT_GENERATE_PATH_MAX_VEL_M_S = 2.0f;  // 自动生成路径的最大线速度，单位 m/s。
float CONBAT_GENERATE_PATH_MAX_ACC_M_S2 = 1.4f; // 自动生成路径的最大加速度，单位 m/s2。
float CONBAT_GENERATE_PATH_GAP_M = 0.03f;       // 自动生成路径点的间距，单位 m。

/*
 * 加载并执行当前状态对应的路径。
 * 同一个状态内只加载一次路径，之后每个周期继续调用 load_follow_plan() 推进路径跟随。
 */
uint8_t CONBAT_TASK::loadGeneratedPathToGoal(const BRPathPose &goal,
                                             const BRPathControlPoint *middle_points,
                                             std::size_t middle_point_count,
                                             float max_vel_m_s,
                                             float max_acc_m_s2)
{
    if (path_loaded_ == 0U)
    {
        BRPathPose start;
        start.x_m = vision.x_diff;
        start.y_m = vision.y_diff;
        start.yaw_rad = vision.angle_x * CONBAT_DEG_TO_RAD;

        std::size_t point_count = 0U;
        BRPathStatus status = generateBsplinePath(start,
                                                  middle_points,
                                                  middle_point_count,
                                                  goal,
                                                  (max_vel_m_s > 0.0f) ? max_vel_m_s : CONBAT_GENERATE_PATH_MAX_VEL_M_S,
                                                  (max_acc_m_s2 > 0.0f) ? max_acc_m_s2 : CONBAT_GENERATE_PATH_MAX_ACC_M_S2,
                                                  generated_path_,
                                                  CONBAT_GENERATE_PATH_MAX_POINTS,
                                                  &point_count,
                                                  CONBAT_GENERATE_PATH_GAP_M);
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

        path_follower_.loadPath(generated_follow_path_, (uint16_t)point_count);
        path_loaded_ = 1U;
    }

    return load_follow_plan();
}

/* 每个状态第一次进入时只加载一次路径。 */
/*
 * 路径跟随核心函数。
 * 功能参考 ROUTE_TASK::load_follow_plan()：读取视觉/雷达位姿，调用 PathFollower 计算世界系速度，
 * 再转换成底盘车体系速度，供 chassis_task 统一下发。
 */
uint8_t CONBAT_TASK::load_follow_plan(void)
{
    /* 雷达/视觉位姿单位转换：PathFollower 使用 mm 和 rad。 */
    PathFollower::Pose pose;
    pose.x = vision.x_diff * 1000.0f;
    pose.y = vision.y_diff * 1000.0f;
    pose.yaw = vision.angle_x * 3.1415926f / 180.0f;

    PathFollower::State follow_state = path_follower_.follow(pose);
    const PathFollower::Output &out = path_follower_.getOutput();

    if (follow_state == PathFollower::STATE_FINISHED || follow_state == PathFollower::STATE_DEVIATED)
    {
        /* 跑点结束或偏离时释放底盘接管。 */
        path_loaded_ = 0U;
        path_follower_.reset();
        clearPathOutput();
        return (follow_state == PathFollower::STATE_FINISHED) ? 1U : 2U;
    }

    float body_vx = 0.0f;
    float body_vy = 0.0f;
    PathFollower::worldToBody(out.world_vx, out.world_vy, pose.yaw, &body_vx, &body_vy);

    /* 底盘使用 m/s，PathFollower 输出 mm/s。 */
    path_vx_target_ = body_vx * 0.001f;
    path_vy_target_ = body_vy * 0.001f;
    path_wz_target_ = -out.wz;
    path_active_ = 1U;

    return 0U;
}

/*
 * yaw 角归一化工具函数。
 * 把任意角度折算到 [-180, 180]，方便 yaw PID 按最短角度误差控制。
 */
float CONBAT_TASK::normalizeYawDeg(float yaw_degree)
{
    while (yaw_degree > 180.0f)
    {
        yaw_degree -= 360.0f;
    }
    while (yaw_degree < -180.0f)
    {
        yaw_degree += 360.0f;
    }
    return yaw_degree;
}

/*
 * FreeRTOS 任务入口。
 * freertos.c 通过 C 链接名创建该任务；任务内部每 1ms 更新一次 CONBAT_TASK 状态机。
 */
uint16_t flag_beh = 0;
uint16_t numo = 0;
extern "C" void conbat_task(void *argument)
{
    (void)argument;

    for (;;)
    {

        conbat_t.runOnce();
        conbat_t.setKfsPlaceIndex(numo);
        if (flag_beh == 1)
        {
            arm_comm.executeAction(ArmComm::ACTION_PICK_FIRST_KFS, 1U);
            /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
            for (uint8_t i = 0U; i < 10U; ++i)
            {
                arm_comm.send();
            }
        }

        if (flag_beh == 2)
        {
            arm_comm.executeAction(ArmComm::ACTION_PICK_SECOND_KFS, 1U);
            /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
            for (uint8_t i = 0U; i < 10U; ++i)
            {
                arm_comm.send();
            }
        }

        if (flag_beh == 3)
        {
            arm_comm.executeAction(ArmComm::ACTION_ZONE3_READY, 2U);
            /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
            for (uint8_t i = 0U; i < 10U; ++i)
            {
                arm_comm.send();
            }
        }

        if (flag_beh == 4)
        {
            arm_comm.executeAction(ArmComm::ACTION_ZONE3_PLACE_HAND, 1U);
            /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
            for (uint8_t i = 0U; i < 10U; ++i)
            {
                arm_comm.send();
            }
            flag_beh = 0;
        }

        if (flag_beh == 5)
        {
            arm_comm.executeAction(ArmComm::ACTION_ZONE3_PLACE_LOWER, 1U);
            /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
            for (uint8_t i = 0U; i < 10U; ++i)
            {
                arm_comm.send();
                flag_beh = 0;
            }
        }

        if (flag_beh == 6)
        {
            arm_comm.executeAction(ArmComm::ACTION_PICK_THIRD_KFS, 1U);
            /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
            for (uint8_t i = 0U; i < 10U; ++i)
            {
                arm_comm.send();
                flag_beh = 0;
            }
        }

        if (flag_beh == 7)
        {
            arm_comm.executeAction(ArmComm::ACTION_ZONE3_PLACE_FINALL, 1U);
            /* 连续发送 10 次，降低机械臂漏收单帧命令的概率。 */
            for (uint8_t i = 0U; i < 10U; ++i)
            {
                arm_comm.send();
                flag_beh = 0;
            }
        }

        osDelay(1);
    }
}
