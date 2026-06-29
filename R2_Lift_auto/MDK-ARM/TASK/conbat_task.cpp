#include "conbat_task.h"
#include "cmsis_os.h"
#include "usart_task.h"

static const float CONBAT_DEG_TO_RAD = 3.1415926f / 180.0f;
static const float CONBAT_GENERATE_PATH_MAX_VEL_M_S = 1.2f;
static const float CONBAT_GENERATE_PATH_MAX_ACC_M_S2 = 1.0f;
static const float CONBAT_GENERATE_PATH_GAP_M = 0.1f;

// 上坡状态的终点表，单位：x/y 为 m，yaw 为 rad；你后续直接改这里。
static BRPathPose conbat_ramp_up_goals[] = {
    {0.0f, 0.0f, 0.0f},
};

// 上坡状态的中间点表，单位：x/y 为 m；按顺序依次经过。
static BRPathControlPoint conbat_ramp_up_middle_points[] = {
    {0.0f, 0.0f},
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
    path_follower_.reset();
    path_loaded_ = 0U;
    kfs_place_index_ = 0U;
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
        if (action_result == 1U)
        {
            state = CONBAT_PICK_KFS;
        }
        else if (action_result == 2U)
        {
            state = CONBAT_IDLE;
        }
        break;

    case CONBAT_PICK_KFS:
        /* 捡 KFS 状态：生成到捡取点的路径并开始跟随。 */
        action_result = loadGeneratedPathToGoal(conbat_pick_kfs_goals[0],
                                                conbat_pick_kfs_middle_points,
                                                conbat_pick_kfs_middle_point_count);
        if (action_result == 1U)
        {
            state = CONBAT_SELECT_KFS_PLACE;
        }
        else if (action_result == 2U)
        {
            state = CONBAT_IDLE;
        }
        break;

    case CONBAT_SELECT_KFS_PLACE:
        /* 选择 KFS 放置点：按 kfs_place_index_ 选择对应路径。 */
        action_result = runSelectKfsPlace();
        if (action_result == 1U)
        {
            state = CONBAT_IDLE;
        }
        else if (action_result == 2U)
        {
            state = CONBAT_IDLE;
        }
        break;

    case CONBAT_IDLE:
    case CONBAT_PLACE_KFS:
    case CONBAT_COMBINE:
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

    if (path_active_ != 0U)
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

/*
 * 处理状态切换后的清理动作。
 * 每次进入新状态时重置路径跟随器和速度输出，避免沿用上一状态的路径进度或底盘目标。
 */
void CONBAT_TASK::handleStateChanged(void)
{
    /* 切状态时重新装载路径，避免沿用上一段路径跟随进度。 */
    path_follower_.reset();
    path_loaded_ = 0U;
    clearPathOutput();

    if (state == CONBAT_IDLE)
    {
        yaw_target_enabled = 0U;
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
    /* 上斜坡阶段暂用 first_area 路径。 */
    /* 上坡状态：用当前雷达坐标生成到上坡终点的路径。 */
    return loadGeneratedPathToGoal(conbat_ramp_up_goals[0],
                                   conbat_ramp_up_middle_points,
                                   conbat_ramp_up_middle_point_count);
}

/*
 * 选择 KFS 放置点位状态处理。
 * 根据 kfs_place_index_ 选择 KFS1、KFS2 或 KFS3 路径，并把路径跟随速度交给底盘。
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
