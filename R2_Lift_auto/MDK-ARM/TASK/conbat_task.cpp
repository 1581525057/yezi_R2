#include "conbat_task.h"
#include "cmsis_os.h"
#include "usart_task.h"
#include "mieling.h"
#include "arm_comm.h"
#include "DT35.h"
#include "lift_class.h"
#include "lift_step_up.h"
#include <math.h>

float CONBAT_DEG_TO_RAD = 3.1415926f / 180.0f;  // 角度转弧度系数，用于视觉角度转换。
float CONBAT_GENERATE_PATH_MAX_VEL_M_S = 2.0f;  // 自动生成路径的最大线速度，单位 m/s。
float CONBAT_GENERATE_PATH_MAX_ACC_M_S2 = 1.8f; // 自动生成路径的最大加速度，单位 m/s2。
float CONBAT_GENERATE_PATH_GAP_M = 0.1f;        // 自动生成路径点的间距，单位 m。

float CONBAT_PICK_KFS_FIRST_WAIT_DT35_TARGET_MM = 0.0f;  // 第一个 KFS 边吸边前进的 DT35 目标距离，在这里填写。
float CONBAT_PICK_KFS_SECOND_WAIT_DT35_TARGET_MM = 0.0f; // 等机械臂准备第二次拾取前的 DT35 目标距离，在这里填写。
float CONBAT_PICK_KFS_SECOND_WAIT_DT35_TOL_MM = 10.0f;   // 第二次拾取等待时 DT35 到位允许误差，单位 mm。

float CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_ACC_MPS2 = 0.7f; // KFS 等待阶段车体前进加速度，单位 m/s2。
float CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_MAX_MPS = 0.4f;  // KFS 等待阶段车体前进最大速度，单位 m/s。

float CONBAT_COMBINE_CLIMB_FINISH_MM = 255.0f;  // 合体爬升结束时 DT35 判定距离，单位 mm。
float CONBAT_COMBINE_FORWARD_TARGET_MM = 65.0f; // 合体最后车体前向靠近的 DT35 目标距离，0 表示跳过。
float CONBAT_COMBINE_FORWARD_TOL_MM = 8.0f;     // 合体最后前向靠近的 DT35 到位允许误差，单位 mm。

float CONBAT_COMBINE_FORWARD_ACC_MPS2 = 0.7f; // 合体最后前向靠近的加速度，单位 m/s2。
float CONBAT_COMBINE_FORWARD_MAX_MPS = 0.4f;  // 合体最后前向靠近的最大速度，单位 m/s。

float CONBAT_COMBINE_LIFT_ACC_MPS2 = 0.5f;             // 合体举升阶段速度规划加速度，单位 m/s2。
float CONBAT_COMBINE_LIFT_MAX_MPS = 1.0f;              // 合体举升阶段最大速度，单位 m/s。
float CONBAT_COMBINE_LIFT_MIN_MPS = 0.25f;             // 合体举升阶段最小速度，单位 m/s。
float CONBAT_COMBINE_LIFT_HEIGHT_TOLERANCE_MM = 40.0f; // 合体举升目标高度允许误差，单位 mm。
uint8_t CONBAT_COMBINE_STABLE_COUNT = 10U;             // 合体到位判定需要连续稳定的次数。

// 上坡状态的终点表，单位：x/y 为 m，yaw 为 rad；你后续直接改这里。
static BRPathPose conbat_ramp_up_goals[] = {
    // {2.63f, 1.99f, 0.0f},
    {2.51f, 2.58f, 0.0f},
};

// 上坡状态的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint conbat_ramp_up_middle_points[] = {
    {0.84f, -0.07f},
    {1.69f, -0.04f},
    {3.24f, -0.08f},
    {3.24f, 1.23f},

};
static const std::size_t conbat_ramp_up_middle_point_count =
    sizeof(conbat_ramp_up_middle_points) / sizeof(conbat_ramp_up_middle_points[0]);

// 捡 KFS 状态的终点表，单位：x/y 为 m，yaw 为 rad；你后续直接改这里。
static BRPathPose conbat_pick_kfs_goals[] = {
    {0.0f, 0.0f, 0.0f},
};

// 捡 KFS 状态的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint conbat_pick_kfs_middle_points[] = {
    {0.0f, 0.0f},
};
static const std::size_t conbat_pick_kfs_middle_point_count =
    sizeof(conbat_pick_kfs_middle_points) / sizeof(conbat_pick_kfs_middle_points[0]);

// 拾取 KFS 的右激光精定位目标，单位 mm；现场只需要改 R_ref。
static MeilingTarget_t conbat_pick_kfs_first_laser_target = {
    .preset_id = 0,
    .L_ref = 0.0f,
    .R_ref = 0.0f,
    .F_ref = 0.0f,
    .tol_lat = 20.0f,
    .tol_lon = 20.0f,
    .timeout_ms = 500000U,
    .sensor_mask = SENSOR_RIGHT,
};

static MeilingTarget_t conbat_pick_kfs_second_laser_target = {
    .preset_id = 0,
    .L_ref = 0.0f,
    .R_ref = 0.0f,
    .F_ref = 0.0f,
    .tol_lat = 20.0f,
    .tol_lon = 20.0f,
    .timeout_ms = 500000U,
    .sensor_mask = SENSOR_RIGHT,
};

// 放 KFS 状态的终点表，单位：x/y 为 m，yaw 为 rad；按 kfs_place_index_ 选择。
static BRPathPose conbat_kfs_place_goals[] = {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
};

// 放 KFS1 状态的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint conbat_kfs_place_0_middle_points[] = {
    {0.0f, 0.0f},
};

// 放 KFS2 状态的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint conbat_kfs_place_1_middle_points[] = {
    {0.0f, 0.0f},
};

// 放 KFS3 状态的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint conbat_kfs_place_2_middle_points[] = {
    {0.0f, 0.0f},
};

static const BRPathControlPoint *conbat_kfs_place_middle_points[] = {
    conbat_kfs_place_0_middle_points,
    conbat_kfs_place_1_middle_points,
    conbat_kfs_place_2_middle_points,
};

static const std::size_t conbat_kfs_place_middle_point_counts[] = {
    sizeof(conbat_kfs_place_0_middle_points) / sizeof(conbat_kfs_place_0_middle_points[0]),
    sizeof(conbat_kfs_place_1_middle_points) / sizeof(conbat_kfs_place_1_middle_points[0]),
    sizeof(conbat_kfs_place_2_middle_points) / sizeof(conbat_kfs_place_2_middle_points[0]),
};

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
    pick_kfs_step_ = PICK_KFS_PATH_TO_AREA;
    combine_step_ = COMBINE_PRE_LIFT;
    pick_kfs_meiling_active_ = 0U;
    pick_kfs_second_forward_done_ = 0U;
    kfs_place_index_ = 0U;
    combine_pre_lift_ready_ = 0U;
    combine_crossed_finish_height_ = 0U;
    combine_stable_count_ = 0U;
    combine_pre_lift_command_seq_ = 0U;
    combine_climb_lift_command_seq_ = 0U;
    lift_switch_target_ = 0U;
    lift_linear_speed_target_ = 0.0f;
    yaw_target_enabled = 0U;
    yaw_target_degree = 0.0f;
    clearPathOutput();
}

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

    case CONBAT_PICK_KFS:
        /* 捡 KFS 状态：生成到捡取点的路径并开始跟随。 */
        action_result = runPickKfs();

        update_state_by_action_result(action_result, CONBAT_SELECT_KFS_PLACE, &state);
        break;

    case CONBAT_SELECT_KFS_PLACE:
        /* 选择 KFS 放置点：按 kfs_place_index_ 选择对应路径。 */
        action_result = runSelectKfsPlace();
        update_state_by_action_result(action_result, CONBAT_IDLE, &state);
        break;

    case CONBAT_COMBINE:
        action_result = runCombine();
        update_state_by_action_result(action_result, CONBAT_IDLE, &state);
        break;

    case CONBAT_IDLE:
        if (conbat_start == 1U)
        {
            conbat_start = 0U;
            state = CONBAT_RAMP_UP;
        }
        clearPathOutput();
        break;

    case CONBAT_PLACE_KFS:
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
        *target_vx = meiling.getChassisVxTarget(manual_vx);
        *target_vy = meiling.getChassisVyTarget(manual_vy);
        *target_wz = meiling.getChassisVzTarget(manual_wz);
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
    pick_kfs_step_ = PICK_KFS_PATH_TO_AREA;
    combine_step_ = COMBINE_PRE_LIFT;
    pick_kfs_meiling_active_ = 0U;
    pick_kfs_second_forward_done_ = 0U;
    combine_pre_lift_ready_ = 0U;
    combine_crossed_finish_height_ = 0U;
    combine_stable_count_ = 0U;
    combine_pre_lift_command_seq_ = 0U;
    combine_climb_lift_command_seq_ = 0U;
    lift_switch_target_ = 0U;
    lift_linear_speed_target_ = 0.0f;
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
 * 当前先复用 first_area 路径跑点；路径结束或偏离后停止输出，后续可以在这里接入真正的上斜坡动作。
 */
uint8_t CONBAT_TASK::runRampUp(void)
{
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
        ramp_up_waiting_ = 1U;
        clearPathOutput();
        return 0U;
    }

    return result;
}

uint8_t CONBAT_TASK::runPickKfs(void)
{
    switch (pick_kfs_step_)
    {
    case PICK_KFS_PATH_TO_AREA:
    {
        /* 第一步：先按路径跑到 KFS 拾取区域的粗略点。 */
        pick_kfs_meiling_active_ = 0U;
        uint8_t result = loadGeneratedPathToGoal(conbat_pick_kfs_goals[0],
                                                 conbat_pick_kfs_middle_points,
                                                 conbat_pick_kfs_middle_point_count);
        if (result == 1U)
        {
            clearPathOutput();
            pick_kfs_step_ = PICK_KFS_FIRST_LOC_START;
            return 0U;
        }
        return result;
    }

    case PICK_KFS_FIRST_LOC_START:
        /* 第二步：启动第一次右激光精定位，准备对准第一个 KFS。 */
        clearPathOutput();
        meiling.start(conbat_pick_kfs_first_laser_target);
        pick_kfs_meiling_active_ = 1U;
        pick_kfs_step_ = PICK_KFS_FIRST_LOC_RUN;
        return 0U;

    case PICK_KFS_FIRST_LOC_RUN:
    {
        /* 第三步：持续更新梅林定位，成功后进入第一个 KFS 拾取动作。 */
        uint8_t result = meiling.update();
        if (result == MeilingLocator::SUCCESS)
        {
            pick_kfs_meiling_active_ = 0U;
            clearPathOutput();
            pick_kfs_step_ = PICK_KFS_FIRST_ACTION;
        }
        else if (result == MeilingLocator::TIMEOUT)
        {
            pick_kfs_meiling_active_ = 0U;
            clearPathOutput();
            return 2U;
        }
        return 0U;
    }

    case PICK_KFS_FIRST_ACTION:
        /* 第四步：发送拾取第一个 KFS 的机械臂指令。 */

        if (arm_comm.executeAction(ArmComm::ACTION_PICK_FIRST_KFS, 1U) == 0U)
        {
            return 2U;
        }
        arm_comm.send();
        pick_kfs_step_ = PICK_KFS_FIRST_WAIT_DONE;
        return 0U;

    case PICK_KFS_FIRST_WAIT_DONE:
        /* 第五步：边向前走边吸取，机械臂回传 event=1 后立即停车并进入下一步。 */
        if (arm_comm.rx_data_.event == 1U)
        {
            clearPathOutput();
            pick_kfs_step_ = PICK_KFS_SECOND_LOC_START;
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

    case PICK_KFS_SECOND_LOC_START:
        /* 第六步：启动第二次右激光精定位，准备移动到第二个 KFS。 */
        clearPathOutput();
        meiling.start(conbat_pick_kfs_second_laser_target);
        pick_kfs_meiling_active_ = 1U;
        pick_kfs_step_ = PICK_KFS_SECOND_LOC_RUN;
        return 0U;

    case PICK_KFS_SECOND_LOC_RUN:
    {
        /* 第七步：持续更新梅林定位，成功后等待机械臂准备好接收下一条指令。 */
        uint8_t result = meiling.update();
        if (result == MeilingLocator::SUCCESS)
        {
            pick_kfs_meiling_active_ = 0U;
            clearPathOutput();
            pick_kfs_second_forward_done_ = 0U;
            pick_kfs_step_ = PICK_KFS_SECOND_WAIT_READY;
        }
        else if (result == MeilingLocator::TIMEOUT)
        {
            pick_kfs_meiling_active_ = 0U;
            clearPathOutput();
            return 2U;
        }
        return 0U;
    }

    case PICK_KFS_SECOND_WAIT_READY:
        /* 第八步：先按前向 DT35 距离向前走，到位停车后再等待机械臂回传 event=3。 */
        if (pick_kfs_second_forward_done_ == 0U)
        {
            clearPathOutput();
            if (CONBAT_PICK_KFS_SECOND_WAIT_DT35_TARGET_MM > 0.0f && dt35.ch2.valid != 0U)
            {
                const float laser_mm = dt35.ch2.distance_filtered;
                const float arrive_mm = CONBAT_PICK_KFS_SECOND_WAIT_DT35_TARGET_MM + CONBAT_PICK_KFS_SECOND_WAIT_DT35_TOL_MM;
                if (laser_mm <= arrive_mm)
                {
                    pick_kfs_second_forward_done_ = 1U;
                }
                else
                {
                    path_active_ = 1U;
                    path_vx_target_ = conbat_trapezoid_speed((laser_mm - CONBAT_PICK_KFS_SECOND_WAIT_DT35_TARGET_MM) * 0.001f,
                                                             CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_ACC_MPS2,
                                                             CONBAT_PICK_KFS_FIRST_WAIT_FORWARD_MAX_MPS);
                    path_vy_target_ = 0.0f;
                    path_wz_target_ = 0.0f;
                }
            }
        }

        if (pick_kfs_second_forward_done_ != 0U && arm_comm.rx_data_.event == 3U)
        {
            clearPathOutput();
            pick_kfs_second_forward_done_ = 0U;
            pick_kfs_step_ = PICK_KFS_SECOND_ACTION;
        }
        return 0U;

    case PICK_KFS_SECOND_ACTION:
        /* 第九步：发送拾取第二个 KFS 的机械臂指令。 */
        if (arm_comm.executeAction(ArmComm::ACTION_PICK_SECOND_KFS, 2U) == 0U)
        {
            return 2U;
        }
        arm_comm.send();
        pick_kfs_step_ = PICK_KFS_SECOND_WAIT_DONE;
        return 0U;

    case PICK_KFS_SECOND_WAIT_DONE:
        /* 第十步：等待机械臂再次回传 event=1，表示第二个 KFS 吸取成功。 */
        if (arm_comm.rx_data_.event == 1U)
        {
            pick_kfs_meiling_active_ = 0U;
            clearPathOutput();
            return 1U;
        }
        return 0U;

    default:
        /* 异常子状态：释放底盘控制并让上层回到空闲。 */
        pick_kfs_meiling_active_ = 0U;
        clearPathOutput();
        return 2U;
    }
}

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
            lift_switch_target_ = 1U;
            STEP_UP_CYLINDER_CLOSE();
            combine_stable_count_ = 0U;
            combine_step_ = COMBINE_FINAL_FORWARD;
        }
        return 0U;
    }

    case COMBINE_FINAL_FORWARD:
        /*
         * 阶段4：爬升完成后收气缸，并把升降档位保持到 1 档。
         * 如果配置了 CONBAT_COMBINE_FORWARD_TARGET_MM，则底盘按车体前向 vx 靠 ch2 继续走到目标距离。
         */
        lift_switch_target_ = 1U;
        lift_linear_speed_target_ = 0.0f;

        if (CONBAT_COMBINE_FORWARD_TARGET_MM <= 0.0f)
        {
            return 1U;
        }

        if (dt35.ch2.valid != 0U)
        {
            const float laser_mm = dt35.ch2.distance_filtered;
            const uint8_t arrived =
                (laser_mm <= (CONBAT_COMBINE_FORWARD_TARGET_MM + CONBAT_COMBINE_FORWARD_TOL_MM)) ? 1U : 0U;

            if (conbat_stable_confirm(arrived, &combine_stable_count_, CONBAT_COMBINE_STABLE_COUNT) != 0U)
            {
                clearPathOutput();
                return 1U;
            }

            path_active_ = 1U;
            path_vx_target_ = conbat_trapezoid_speed((laser_mm - CONBAT_COMBINE_FORWARD_TARGET_MM) * 0.001f,
                                                     CONBAT_COMBINE_FORWARD_ACC_MPS2,
                                                     CONBAT_COMBINE_FORWARD_MAX_MPS);
            path_vy_target_ = 0.0f;
            path_wz_target_ = 0.0f;
        }
        return 0U;

    default:
        lift_linear_speed_target_ = 0.0f;
        clearPathOutput();
        return 2U;
    }
}

/*
 * 选择 KFS 放置点位状态处理。
 * 根据 kfs_place_index_ 选择对应路径，并把路径跟随速度交给底盘。
 */
uint8_t CONBAT_TASK::runSelectKfsPlace(void)
{
    /* 根据选择的 KFS 点位加载对应路径，默认使用 KFS1。 */
    if (kfs_place_index_ == 1U)
    {
        return loadGeneratedPathToGoal(conbat_kfs_place_goals[1],
                                       conbat_kfs_place_middle_points[1],
                                       conbat_kfs_place_middle_point_counts[1]);
    }

    if (kfs_place_index_ == 2U)
    {
        return loadGeneratedPathToGoal(conbat_kfs_place_goals[2],
                                       conbat_kfs_place_middle_points[2],
                                       conbat_kfs_place_middle_point_counts[2]);
    }

    return loadGeneratedPathToGoal(conbat_kfs_place_goals[0],
                                   conbat_kfs_place_middle_points[0],
                                   conbat_kfs_place_middle_point_counts[0]);
}

/*
 * 加载并执行当前状态对应的路径。
 * 同一个状态内只加载一次路径，之后每个周期继续调用 load_follow_plan() 推进路径跟随。
 */
uint8_t CONBAT_TASK::loadGeneratedPathToGoal(const BRPathPose &goal,
                                             const BRPathControlPoint *middle_points,
                                             std::size_t middle_point_count)
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
                                                  CONBAT_GENERATE_PATH_MAX_VEL_M_S,
                                                  CONBAT_GENERATE_PATH_MAX_ACC_M_S2,
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
extern "C" void conbat_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        conbat_t.runOnce();
        osDelay(1);
    }
}
