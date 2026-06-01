#include "FTMTask.h"
#include "CommData.h"
#include "M2006Follower.h"
#include "StepMotorTask.h"
#include "RS05.h"
#include "GripPush.h"
#include "usart_task.h"
#include "cmsis_os.h"
#include <math.h>

extern "C" void WuqiquTask_Start(void);
extern "C" void WuqiquTask_Stop(void);
extern "C" uint8_t WuqiquTask_RunOnce(void);
extern "C" uint8_t WuqiquTask_IsActive(void);

namespace
{
/* 微调机构状态枚举 */
enum FTMState
{
    FTM_STATE_INIT = 0,                 // 初始化状态
    FTM_STATE_MOVE = 1,                 // 跟随移动状态
    FTM_STATE_GRIP = 2,                 // 抓取状态
    FTM_STATE_RELEASE = 3,              // 释放状态
    FTM_STATE_PLATFORM_FORWARD = 4,     // 推杆伸出状态（平台前进）
    FTM_STATE_PLATFORM_BACKWARD = 5,    // 推杆缩回状态（平台后退）
    FTM_STATE_CLAW_OPEN = 6,            // 夹爪打开状态
    FTM_STATE_CLAW_CLOSE = 7,           // 夹爪关闭状态
    FTM_STATE_RELEASE_TO_ZERO = 8,      // 释放回零状态
    FTM_STATE_PLATFORM_STOP = 9,        // 推杆停止状态
    FTM_STATE_WUQIQU_ROUTE = 10,        // 武器区路径状态
    FTM_STATE_WUQIQU_ZERO = 11          // 武器区路径前发送视觉置零
};

constexpr uint8_t kM2006StopDebounceCycles = 2U;   // M2006 停止去抖周期数
constexpr uint16_t kGripLiftSpeedRpm = 60U;        // 抓取抬升速度，单位：RPM
constexpr uint32_t kFollowerControlPeriodMs = 10U; // 随动控制周期，单位：ms
constexpr uint32_t kLiftMotionMarginMs = 200U;     // 抬升动作容差时间，单位：ms
constexpr float kReleaseDropDistanceMm = 80.0f; // 释放时的下降距离，单位：mm
constexpr float kGripClawFinalAngleRad = 1.575f;   // 夹爪最终角度，单位：rad
constexpr float kGripClawPreOpenAngleRad = kGripClawFinalAngleRad / 2.0f; // 夹爪预张开角度
constexpr float kGripClawReleaseFinalAngleRad = 0.0f; // 释放结束角度
constexpr float kRs05AngleToleranceRad = 0.05f;    // RS05 角度容差，单位：rad
constexpr uint32_t kRs05CommandSettleMs = 200U;    // RS05 指令稳定时间，单位：ms
constexpr uint32_t kRs05MoveTimeoutMs = 2000U;     // RS05 动作超时时间，单位：ms
constexpr uint32_t kWuqiquZeroSendIntervalMs = 20U;
constexpr uint32_t kWuqiquZeroSettleMs = 200U;

/* 释放阶段的 RS05 参数 */
constexpr float kReleaseRs05Speed = 1.5f;
constexpr uint32_t kRs05ReinforceIntervalMs = 100U;
constexpr float kReleaseRs05LimitSpd = 1.5f;
constexpr float kReleaseRs05LimitCur = 1.0f;

/* 抓取翻转阶段的 RS05 参数 */
constexpr float kGripRs05Speed = 3.0f;
constexpr float kGripRs05LimitSpd = 3.0f;
constexpr float kGripRs05LimitCur = 1.5f;
constexpr float kRs05DefaultLimitSpd = 10.0f;
constexpr float kRs05DefaultLimitCur = 23.0f;

void RestoreRs05Limits(void);

uint8_t g_m2006_nonpositive_cycles = kM2006StopDebounceCycles; // M2006 非正向运动周期计数

/* 释放上下文 */
struct ReleaseContext
{
    uint8_t phase;                // 当前阶段
    uint32_t stage_start_tick;    // 阶段起始时刻
    float release_target_mm;      // 释放目标距离，单位：mm
    uint32_t last_rs05_cmd_tick;  // 上次发送 RS05 指令的时刻
};

/* 释放回零上下文 */
struct ReleaseToZeroContext
{
    uint8_t phase;             // 当前阶段
    uint32_t stage_start_tick; // 阶段起始时刻
    uint32_t last_rs05_cmd_tick;
};

/* 释放阶段枚举 */
enum ReleasePhase
{
    RELEASE_PHASE_IDLE = 0,            // 空闲
    RELEASE_PHASE_WAIT_RELEASE_ANGLE,  // 等待释放角度到位
    RELEASE_PHASE_DROP_PARTIAL         // 部分下降
};

/* 释放回零阶段枚举 */
enum ReleaseToZeroPhase
{
    RELEASE_TO_ZERO_PHASE_IDLE = 0,            // 空闲
    RELEASE_TO_ZERO_PHASE_WAIT_RELEASE_ANGLE,  // 等待释放角度到位
    RELEASE_TO_ZERO_PHASE_DROP_TO_ZERO         // 下降到零点
};

struct MoveEntryZeroContext
{
    uint8_t phase;
    uint32_t stage_start_tick;
    uint32_t last_rs05_cmd_tick;
};

struct GripContext
{
    uint8_t phase;
    uint32_t stage_start_tick;
    uint32_t last_rs05_cmd_tick;
};

enum MoveEntryZeroPhase
{
    MOVE_ENTRY_ZERO_PHASE_IDLE = 0,
    MOVE_ENTRY_ZERO_PHASE_SEND_COMMAND,
    MOVE_ENTRY_ZERO_PHASE_WAIT_COMPLETED
};

enum GripPhase
{
    GRIP_PHASE_IDLE = 0,
    GRIP_PHASE_WAIT_PREOPEN,
    GRIP_PHASE_WAIT_CLAW_OPEN,
    GRIP_PHASE_WAIT_FINAL_ANGLE
};

ReleaseContext g_release = {RELEASE_PHASE_IDLE, 0U, 0.0f, 0U};
ReleaseToZeroContext g_release_to_zero = {RELEASE_TO_ZERO_PHASE_IDLE, 0U, 0U};
MoveEntryZeroContext g_move_entry_zero = {MOVE_ENTRY_ZERO_PHASE_IDLE, 0U, 0U};
GripContext g_grip = {GRIP_PHASE_IDLE, 0U, 0U};

struct WuqiquZeroContext
{
    uint8_t active;
    uint32_t start_tick;
    uint32_t last_send_tick;
};

WuqiquZeroContext g_wuqiqu_zero = {0U, 0U, 0U};

void ResetWuqiquZeroContext(void)
{
    g_wuqiqu_zero.active = 0U;
    g_wuqiqu_zero.start_tick = 0U;
    g_wuqiqu_zero.last_send_tick = 0U;
}

/**
 * @brief 估算抓取抬升动作的执行时间
 *
 * @param distance_mm 抬升距离，单位：mm
 * @return uint32_t 估算执行时间，单位：ms
 */
uint32_t EstimateGripLiftMoveTimeMs(float distance_mm)
{
    if (distance_mm <= 0.0f)
    {
        return kLiftMotionMarginMs;
    }

    const float revolutions = distance_mm / (PI_VALUE * Z_ROLLER_DIAMETER);
    const float move_time_minutes = revolutions / static_cast<float>(kGripLiftSpeedRpm);
    const float move_time_ms = move_time_minutes * 60000.0f;

    return static_cast<uint32_t>(move_time_ms + 0.5f) + kLiftMotionMarginMs;
}

/**
 * @brief 复位释放状态机
 *
 * @return 无
 */
void ResetReleaseStateMachine(void)
{
    g_release.phase = RELEASE_PHASE_IDLE;
    g_release.stage_start_tick = 0U;
    g_release.release_target_mm = 0.0f;
    g_release.last_rs05_cmd_tick = 0U;
    g_release_to_zero.phase = RELEASE_TO_ZERO_PHASE_IDLE;
    g_release_to_zero.stage_start_tick = 0U;
    g_release_to_zero.last_rs05_cmd_tick = 0U;
}

void RequestMoveEntryZero(void)
{
    g_move_entry_zero.phase = MOVE_ENTRY_ZERO_PHASE_SEND_COMMAND;
    g_move_entry_zero.stage_start_tick = 0U;
    g_move_entry_zero.last_rs05_cmd_tick = 0U;
}

void ResetGripStateMachine(void)
{
    g_grip.phase = GRIP_PHASE_IDLE;
    g_grip.stage_start_tick = 0U;
    g_grip.last_rs05_cmd_tick = 0U;
}

/**
 * @brief 判断指定时长是否已经过去
 *
 * @param start_tick 起始时刻
 * @param duration_ms 持续时间，单位：ms
 * @return bool 已超时返回 true
 */
bool HasElapsed(uint32_t start_tick, uint32_t duration_ms)
{
    return static_cast<uint32_t>(HAL_GetTick() - start_tick) >= duration_ms;
}

/**
 * @brief 判断 RS05 是否到达目标角度
 *
 * @param target_angle_rad 目标角度，单位：rad
 * @return bool 到达目标返回 true
 */
bool IsRs05AtTarget(float target_angle_rad)
{
    return fabsf(RS05_GetMotor().Pos_Info.Angle - target_angle_rad) <= kRs05AngleToleranceRad;
}

/**
 * @brief 维护释放阶段的 M2006 跟随控制
 *
 * @return 无
 */
void ServiceReleaseFollower(void)
{
    if (StepMotor_IsRecoveryActive() != false)
    {
        StepMotor_ServiceRecovery();
        g_m2006_nonpositive_cycles = 0U;
        g_m2006.SetStepReference(StepMotor_GetRecoveryDirection(), StepMotor_GetRecoverySpeedRpm());
        g_m2006.ControlTick(HAL_GetTick());
    }
    else
    {
        if (g_m2006_nonpositive_cycles < kM2006StopDebounceCycles)
        {
            ++g_m2006_nonpositive_cycles;
            g_m2006.ControlTick(HAL_GetTick());
        }
        else
        {
            g_m2006.Stop();
        }
    }
}

/**
 * @brief 执行释放状态机
 *
 * @return bool 状态机完成时返回 true
 */
bool RunReleaseLoop(void)
{
    ServiceReleaseFollower();

    switch (g_release.phase)
    {
    case RELEASE_PHASE_IDLE:
        g_m2006.Stop();
        (void)StepMotorCommandDelayTimeout(20U);
        g_release.release_target_mm =
            (g_grip_distance_mm > kReleaseDropDistanceMm) ? (g_grip_distance_mm - kReleaseDropDistanceMm) : 0.0f;

        // 限速并限流，降低带载释放时的翻转冲击。
        g_rs05_motor.Set_RobStride_Motor_parameter(0x7017, kReleaseRs05LimitSpd, Set_parameter);
        osDelay(5);
        g_rs05_motor.Set_RobStride_Motor_parameter(0x7018, kReleaseRs05LimitCur, Set_parameter);
        osDelay(5);
        RS05_PositionControl(kReleaseRs05Speed, kGripClawPreOpenAngleRad);
        g_release.stage_start_tick = HAL_GetTick();
        g_release.last_rs05_cmd_tick = HAL_GetTick();
        g_release.phase = RELEASE_PHASE_WAIT_RELEASE_ANGLE;
        return false;

    case RELEASE_PHASE_WAIT_RELEASE_ANGLE:
        if (HasElapsed(g_release.stage_start_tick, kRs05CommandSettleMs) == false)
        {
            return false;
        }

        // 翻转过程中周期性补发指令，避免中途扰动导致角度偏离。
        if (HasElapsed(g_release.last_rs05_cmd_tick, kRs05ReinforceIntervalMs))
        {
            RS05_PositionControl(kReleaseRs05Speed, kGripClawPreOpenAngleRad);
            g_release.last_rs05_cmd_tick = HAL_GetTick();
        }

        if (IsRs05AtTarget(kGripClawPreOpenAngleRad) == false &&
            HasElapsed(g_release.stage_start_tick, kRs05MoveTimeoutMs) == false)
        {
            return false;
        }

        ReturnLiftToPosition(g_release.release_target_mm, kReleaseDropDistanceMm);
        g_release.phase = RELEASE_PHASE_DROP_PARTIAL;
        return false;

    case RELEASE_PHASE_DROP_PARTIAL:
        // 下降过程中继续维持 RS05 的释放姿态。
        if (HasElapsed(g_release.last_rs05_cmd_tick, kRs05ReinforceIntervalMs))
        {
            RS05_PositionControl(kReleaseRs05Speed, kGripClawPreOpenAngleRad);
            g_release.last_rs05_cmd_tick = HAL_GetTick();
        }

        if (StepMotor_IsRecoveryActive() != false)
        {
            return false;
        }

        RestoreRs05Limits();
        ResetReleaseStateMachine();
        return true;

    default:
        ResetReleaseStateMachine();
        g_m2006.Stop();
        return true;
    }
}

/**
 * @brief 执行释放回零状态
 *
 * @return bool 状态机完成时返回 true
 */
bool RunReleaseToZeroLoop(void)
{
    ServiceReleaseFollower();

    switch (g_release_to_zero.phase)
    {
    case RELEASE_TO_ZERO_PHASE_IDLE:
        g_m2006.Stop();
        (void)StepMotorCommandDelayTimeout(20U);
        RS05_PositionControl(Speed, kGripClawReleaseFinalAngleRad);
        g_release_to_zero.stage_start_tick = HAL_GetTick();
        g_release_to_zero.last_rs05_cmd_tick = HAL_GetTick();
        g_release_to_zero.phase = RELEASE_TO_ZERO_PHASE_WAIT_RELEASE_ANGLE;
        return false;

    case RELEASE_TO_ZERO_PHASE_WAIT_RELEASE_ANGLE:
        // 等待期间持续维护 M2006 状态，避免丢失同步
        g_m2006.ControlTick(HAL_GetTick());

        if (HasElapsed(g_release_to_zero.last_rs05_cmd_tick, kRs05ReinforceIntervalMs))
        {
            RS05_PositionControl(Speed, kGripClawReleaseFinalAngleRad);
            g_release_to_zero.last_rs05_cmd_tick = HAL_GetTick();
        }

        if (HasElapsed(g_release_to_zero.stage_start_tick, kRs05CommandSettleMs) == false)
        {
            return false;
        }

        if (IsRs05AtTarget(kGripClawReleaseFinalAngleRad) == false &&
            HasElapsed(g_release_to_zero.stage_start_tick, kRs05MoveTimeoutMs) == false)
        {
            return false;
        }

        ReturnLiftToPosition(0.0f, g_grip_distance_mm);
        g_release_to_zero.phase = RELEASE_TO_ZERO_PHASE_DROP_TO_ZERO;
        return false;

    case RELEASE_TO_ZERO_PHASE_DROP_TO_ZERO:
        if (HasElapsed(g_release_to_zero.last_rs05_cmd_tick, kRs05ReinforceIntervalMs))
        {
            RS05_PositionControl(Speed, kGripClawReleaseFinalAngleRad);
            g_release_to_zero.last_rs05_cmd_tick = HAL_GetTick();
        }

        if (StepMotor_IsRecoveryActive() != false)
        {
            return false;
        }

        ResetReleaseStateMachine();
        return true;

    default:
        ResetReleaseStateMachine();
        g_m2006.Stop();
        return true;
    }
}

/**
 * @brief 在指定时长内持续维护步进电机恢复动作
 *
 * @param duration_ms 持续时间，单位：ms
 * @return 无
 */
/**
 * @brief 执行 MOVE 状态入口的一次性 RS05 回零动作
 *
 * @return bool 回零完成时返回 true
 */
bool RunMoveEntryZeroLoop(void)
{
    switch (g_move_entry_zero.phase)
    {
    case MOVE_ENTRY_ZERO_PHASE_IDLE:
        return true;

    case MOVE_ENTRY_ZERO_PHASE_SEND_COMMAND:
        g_m2006.Stop();
        RS05_PositionControl(Speed, kGripClawReleaseFinalAngleRad);
        g_move_entry_zero.stage_start_tick = HAL_GetTick();
        g_move_entry_zero.last_rs05_cmd_tick = HAL_GetTick();
        g_move_entry_zero.phase = MOVE_ENTRY_ZERO_PHASE_WAIT_COMPLETED;
        return false;

    case MOVE_ENTRY_ZERO_PHASE_WAIT_COMPLETED:
        g_m2006.Stop();

        if (HasElapsed(g_move_entry_zero.last_rs05_cmd_tick, kRs05ReinforceIntervalMs))
        {
            RS05_PositionControl(Speed, kGripClawReleaseFinalAngleRad);
            g_move_entry_zero.last_rs05_cmd_tick = HAL_GetTick();
        }

        if (HasElapsed(g_move_entry_zero.stage_start_tick, kRs05CommandSettleMs) == false)
        {
            return false;
        }

        if (IsRs05AtTarget(kGripClawReleaseFinalAngleRad) == false &&
            HasElapsed(g_move_entry_zero.stage_start_tick, kRs05MoveTimeoutMs) == false)
        {
            return false;
        }

        g_move_entry_zero.phase = MOVE_ENTRY_ZERO_PHASE_IDLE;
        g_move_entry_zero.stage_start_tick = 0U;
        g_move_entry_zero.last_rs05_cmd_tick = 0U;
        return true;

    default:
        g_move_entry_zero.phase = MOVE_ENTRY_ZERO_PHASE_IDLE;
        g_move_entry_zero.stage_start_tick = 0U;
        g_move_entry_zero.last_rs05_cmd_tick = 0U;
        g_m2006.Stop();
        return true;
    }
}

/**
 * @brief 在指定时长内持续维护步进电机恢复动作
 *
 * @param duration_ms 持续时间，单位：ms
 * @return 无
 */
void StepMotorRecoveryDelay(uint32_t duration_ms)
{
    const uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < duration_ms)
    {
        StepMotor_ServiceRecovery();
        osDelay(kFollowerControlPeriodMs);
    }
}

/**
 * @brief 初始化微调机构相关模块
 *
 * @return 无
 */
bool RunGripLoop(void)
{
    switch (g_grip.phase)
    {
    case GRIP_PHASE_IDLE:
        ResetReleaseStateMachine();
        g_m2006.Stop();
        (void)StepMotorCommandDelayTimeout(20U);
        FineTuneLiftForWeaponGrip();
        StepMotorRecoveryDelay(10000U);

        RS05_PositionControl(Speed, kGripClawPreOpenAngleRad);
        g_grip.stage_start_tick = HAL_GetTick();
        g_grip.last_rs05_cmd_tick = HAL_GetTick();
        g_grip.phase = GRIP_PHASE_WAIT_PREOPEN;
        return false;

    case GRIP_PHASE_WAIT_PREOPEN:
        if (HasElapsed(g_grip.last_rs05_cmd_tick, kRs05ReinforceIntervalMs))
        {
            RS05_PositionControl(Speed, kGripClawPreOpenAngleRad);
            g_grip.last_rs05_cmd_tick = HAL_GetTick();
        }

        if (HasElapsed(g_grip.stage_start_tick, 500U) == false)
        {
            return false;
        }

        claw_open();
        g_grip.stage_start_tick = HAL_GetTick();
        g_grip.phase = GRIP_PHASE_WAIT_CLAW_OPEN;
        return false;

    case GRIP_PHASE_WAIT_CLAW_OPEN:
        if (HasElapsed(g_grip.stage_start_tick, 100U) == false)
        {
            return false;
        }

        RS05_PositionControl(Speed, kGripClawFinalAngleRad);
        g_grip.stage_start_tick = HAL_GetTick();
        g_grip.last_rs05_cmd_tick = HAL_GetTick();
        g_grip.phase = GRIP_PHASE_WAIT_FINAL_ANGLE;
        return false;

    case GRIP_PHASE_WAIT_FINAL_ANGLE:
        if (HasElapsed(g_grip.last_rs05_cmd_tick, kRs05ReinforceIntervalMs))
        {
            RS05_PositionControl(Speed, kGripClawFinalAngleRad);
            g_grip.last_rs05_cmd_tick = HAL_GetTick();
        }

        if (HasElapsed(g_grip.stage_start_tick, kRs05CommandSettleMs) == false)
        {
            return false;
        }

        if (IsRs05AtTarget(kGripClawFinalAngleRad) == false &&
            HasElapsed(g_grip.stage_start_tick, kRs05MoveTimeoutMs) == false)
        {
            return false;
        }

        ResetGripStateMachine();
        return true;

    default:
        ResetGripStateMachine();
        return true;
    }
}

void FTM_InitModules(void)
{
    cmd_init();
    RS05_Init();
    RequestMoveEntryZero();
    StepMotor_Init();
    g_m2006.Init();
    g_m2006.SetDescendDirection(Z_MOTOR_DIR);
    g_m2006_nonpositive_cycles = kM2006StopDebounceCycles;
}

/**
 * @brief 执行移动跟随主循环
 *
 * @return 无
 */
void FTM_RunMoveLoop(void)
{
    int16_t visual_z = 0;

    __disable_irq();
    visual_z = g_visual_data.z;
    __enable_irq();

    StepMotor_Z.SetError(visual_z);

    Motor_Ctrl(StepMotor_Z);
    (void)StepMotorCommandDelayTimeout(5U);

    if (StepMotor_Z.GetError() > 0)
    {
        g_m2006_nonpositive_cycles = 0U;
        g_m2006.SetStepReference(StepMotor_Z.GetDirection(), StepMotor_Z.GetSpeedRpm());
        g_m2006.ControlTick(HAL_GetTick());
    }
    else
    {
        if (g_m2006_nonpositive_cycles < kM2006StopDebounceCycles)
        {
            ++g_m2006_nonpositive_cycles;
            g_m2006.ControlTick(HAL_GetTick());
        }
        else
        {
            g_m2006.Stop();
        }
    }

    StepMotor_Z.QueueResetClogProtection();
    StepMotor_Z.QueueEnable(true, false);
}

/**
 * @brief 直接调整夹爪角度
 *
 * @param angle_rad 目标角度，单位：rad
 * @return 无
 */
void AdjustGripClawPosition(float angle_rad)
{
    RS05_PositionControl(Speed, angle_rad);
}

/**
 * @brief 以限速限流方式调整夹爪角度
 *
 * @param angle_rad 目标角度，单位：rad
 * @return 无
 */
void AdjustGripClawPositionDamped(float angle_rad)
{
    g_rs05_motor.Set_RobStride_Motor_parameter(0x7017, kGripRs05LimitSpd, Set_parameter);
    osDelay(5);
    g_rs05_motor.Set_RobStride_Motor_parameter(0x7018, kGripRs05LimitCur, Set_parameter);
    osDelay(5);
    RS05_PositionControl(kGripRs05Speed, angle_rad);
}

/**
 * @brief 恢复 RS05 默认速度和电流上限
 *
 * @return 无
 */
void RestoreRs05Limits(void)
{
    g_rs05_motor.Set_RobStride_Motor_parameter(0x7017, kRs05DefaultLimitSpd, Set_parameter);
    osDelay(5);
    g_rs05_motor.Set_RobStride_Motor_parameter(0x7018, kRs05DefaultLimitCur, Set_parameter);
    osDelay(5);
}
}

extern "C" volatile uint8_t g_ftm_state = FTM_STATE_INIT;

/**
 * @brief 获取当前微调机构状态
 *
 * @return uint8_t 当前状态值
 */
extern "C" uint8_t FTM_GetState(void)
{
    return g_ftm_state;
}


/**
 * @brief 微调机构主控制任务
 *
 * @param argument 任务参数
 * @return 无
 */
extern "C" void ftm_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        if (g_ftm_state != FTM_STATE_WUQIQU_ROUTE && WuqiquTask_IsActive() != 0U)
        {
            WuqiquTask_Stop();
        }

        if (g_ftm_state != FTM_STATE_WUQIQU_ZERO && g_wuqiqu_zero.active != 0U)
        {
            ResetWuqiquZeroContext();
        }

        switch (g_ftm_state)
        {
        /* 初始化状态 */
        case FTM_STATE_INIT:
            ResetReleaseStateMachine();
            FTM_InitModules();
            g_ftm_state = FTM_STATE_MOVE;
            break;
        /* 微调机构调整状态 */
        case FTM_STATE_MOVE:
            if (RunMoveEntryZeroLoop() == false)
            {
                break;
            }
            FTM_RunMoveLoop();
            break;
        /* 抓取武器头状态 */
        case FTM_STATE_GRIP:
            if (RunGripLoop() != false)
            {
                g_ftm_state = FTM_STATE_MOVE;
            }
            break;

            // 先翻转到半开位，减小后续夹持时的瞬态冲击。
            RS05_PositionControl(Speed, kGripClawPreOpenAngleRad);

            // 再翻转到最终角度。
            RS05_PositionControl(Speed, kGripClawFinalAngleRad);
        /* 抓取武器头后回收爪子一半 平台下降80状态 */
        case FTM_STATE_RELEASE:
            if (RunReleaseLoop() != false)
            {
                g_ftm_state = FTM_STATE_RELEASE_TO_ZERO;
            }
            break;
        /* 抓取武器头后回收爪子 平台下降到零点状态 */
        case FTM_STATE_RELEASE_TO_ZERO:
            if (RunReleaseToZeroLoop() != false)
            {
                g_ftm_state = FTM_STATE_MOVE;
            }
            break;
        /* 推杆伸出状态 */
        case FTM_STATE_PLATFORM_FORWARD:
            ResetReleaseStateMachine();
            platform_forward();
            osDelay(3750); // 30mm / 8mm/s = 3.75s
            g_ftm_state = FTM_STATE_PLATFORM_STOP;
            break;
        /* 推杆缩回状态 */
        case FTM_STATE_PLATFORM_BACKWARD:
            ResetReleaseStateMachine();
            platform_backward();
            osDelay(3750); // 30mm / 8mm/s = 3.75s
            g_ftm_state = FTM_STATE_PLATFORM_STOP;
            break;
        /* 推杆停止状态 */
        case FTM_STATE_PLATFORM_STOP:
            ResetReleaseStateMachine();
            platform_stop();
            break;
        /* 夹爪打开状态 */
        case FTM_STATE_CLAW_OPEN:
            ResetReleaseStateMachine();
            claw_open();
            break;
        /* 夹爪关闭状态 */
        case FTM_STATE_CLAW_CLOSE:
            ResetReleaseStateMachine();
            claw_close();
            break;
        case FTM_STATE_WUQIQU_ZERO:
            ResetReleaseStateMachine();
            WuqiquTask_Stop();
            if (g_wuqiqu_zero.active == 0U)
            {
                g_wuqiqu_zero.active = 1U;
                g_wuqiqu_zero.start_tick = HAL_GetTick();
                g_wuqiqu_zero.last_send_tick = 0U;
            }

            if (g_wuqiqu_zero.last_send_tick == 0U ||
                HasElapsed(g_wuqiqu_zero.last_send_tick, kWuqiquZeroSendIntervalMs))
            {
                send_position_to_pc(0, 1, 0, 0, 0);
                g_wuqiqu_zero.last_send_tick = HAL_GetTick();
            }

            if (HasElapsed(g_wuqiqu_zero.start_tick, kWuqiquZeroSettleMs))
            {
                ResetWuqiquZeroContext();
               // g_ftm_state = FTM_STATE_WUQIQU_ROUTE;
            }
            break;
        case FTM_STATE_WUQIQU_ROUTE:
            if (WuqiquTask_IsActive() == 0U)
            {
                WuqiquTask_Start();
            }

            if (WuqiquTask_RunOnce() != 0U)
            {
                WuqiquTask_Stop();
                g_ftm_state = FTM_STATE_MOVE;
            }
            break;
        /* 默认状态 */
        default:
            ResetReleaseStateMachine();
            g_m2006.Stop();
            g_ftm_state = FTM_STATE_INIT;
            break;
        }
        osDelay(1);
    }
}
