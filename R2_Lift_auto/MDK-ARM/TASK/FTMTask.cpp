#include "FTMTask.h"
#include "CommData.h"
#include "FTMLiftAction.h"
#include "M2006AngleMotor.h"
#include "RS05.h"
#include "GripPush.h"
#include "usart_task.h"
#include "cmsis_os.h"
#include <math.h>

extern "C" void WuqiquTask_Start(void);
extern "C" void WuqiquTask_Stop(void);
extern "C" uint8_t WuqiquTask_RunOnce(void);
extern "C" uint8_t WuqiquTask_IsActive(void);
extern "C" void WuqiquTask_AdvanceToNext(void);
extern "C" uint8_t WuqiquTask_IsAllFinished(void);

enum FTMState
{
    FTM_STATE_INIT = 0,               // 初始化/空闲准备状态。
    FTM_STATE_WUQIQU_ROUTE = 1,       // 武器区跑点状态，只负责底盘跑点。
    FTM_STATE_IDLE = 2,               // 手动调试等待状态；不执行动作，等待 Keil Watch 写入下一状态。
    FTM_STATE_RS05_TO_90 = 3,         // RS05 转到 Keil Watch 中 Angle 指定的角度。
    FTM_STATE_LIFT_UP = 4,            // 抬升机构上升到 81mm。
    FTM_STATE_CLAW_OPEN = 5,          // 夹爪张开。
    FTM_STATE_CLAW_CLOSE = 6,         // 夹爪闭合。
    FTM_STATE_LIFT_UP_220 = 7,        // 抬升机构上升到 217mm。
    FTM_STATE_M2006_TURN_180 = 8,     // M2006 从当前角度相对翻转 180 度。
    FTM_STATE_RS05_TO_0 = 9,          // RS05 回到 0 度。
    FTM_STATE_LIFT_DOWN = 10,         // 抬升机构下降到 70mm。
    FTM_STATE_DONE = 11,              // 全流程完成保持状态。
    FTM_STATE_WUQIQU_ZERO = 12,       // 武器区跑点前发送视觉置零。
    FTM_STATE_M2006_TURN_BACK_180 = 13, // M2006 从当前角度反向翻转 180 度。
    FTM_STATE_SEQUENCE_5_4_3 = 14,    // 组合执行：5 -> 4 -> 3。
    FTM_STATE_SEQUENCE_7_3_13_9 = 15, // 组合执行：7 -> 3 -> 13 -> 9。
    FTM_STATE_WUQIQU_ROUTE_2 = 16,
    FTM_STATE_WUQIQU_ROUTE_3 = 17,
    FTM_STATE_WUQIQU_ROUTE_4 = 18,
    FTM_STATE_SEQUENCE_16_17_18_15 = 19
};

namespace
{
constexpr float kDegToRad = 0.01745329251994329577f;
constexpr float kRs05AngleToleranceRad = 0.02f;
constexpr uint32_t kRs05CommandIntervalMs = 20U;
constexpr uint32_t kRs05SettleMs = 300U;
constexpr uint32_t kRs05TimeoutMs = 3500U;

constexpr float kLiftToleranceMm = 5.0f;
constexpr float kLiftMoveTimeS = 0.7f;
constexpr float kLiftUpHighTargetMm = 217.0f;

constexpr uint32_t kClawActionDelayMs = 200U;
constexpr float kM2006TurnAngleDeg = 180.0f;
constexpr float kM2006ToleranceDeg = 3.0f;
constexpr uint32_t kM2006TimeoutMs = 3000U;

constexpr uint32_t kWuqiquZeroSendIntervalMs = 20U;
constexpr uint32_t kWuqiquZeroSettleMs = 200U;

struct TimedStep
{
    uint8_t active;
    uint8_t finished;
    uint32_t start_tick;
    uint32_t last_cmd_tick;
};

TimedStep g_step = {0U, 0U, 0U, 0U};
uint8_t g_lift_commanded = 0U;
float g_lift_last_target_height_mm = 0.0f;
uint8_t g_wuqiqu_route_finished = 0U;
uint8_t g_active_state = 0xFFU;
uint8_t g_modules_initialized = 0U;
uint8_t g_m2006_angle_lock_active = 1U;
uint8_t g_sequence_step_index = 0U;
uint8_t g_wuqiqu_sequence_step_index = 0U;

const uint8_t kSequence543[] = {
    FTM_STATE_CLAW_OPEN,
    FTM_STATE_LIFT_UP,
    FTM_STATE_RS05_TO_90
};

const uint8_t kSequence73139[] = {
    FTM_STATE_LIFT_UP_220,
    FTM_STATE_RS05_TO_90,
    FTM_STATE_M2006_TURN_BACK_180,
    FTM_STATE_RS05_TO_0
};

uint32_t g_wuqiqu_zero_start_tick = 0U;
uint32_t g_wuqiqu_zero_last_send_tick = 0U;
uint8_t g_wuqiqu_zero_active = 0U;

bool HasElapsed(uint32_t start_tick, uint32_t duration_ms)
{
    return static_cast<uint32_t>(HAL_GetTick() - start_tick) >= duration_ms;
}

void ResetTimedStep(void)
{
    g_step.active = 0U;
    g_step.finished = 0U;
    g_step.start_tick = 0U;
    g_step.last_cmd_tick = 0U;
}

void ResetWuqiquZero(void)
{
    g_wuqiqu_zero_active = 0U;
    g_wuqiqu_zero_start_tick = 0U;
    g_wuqiqu_zero_last_send_tick = 0U;
}

void ResetMechanismStep(void)
{
    ResetTimedStep();
    g_lift_commanded = 0U;
    g_lift_last_target_height_mm = 0.0f;
}

void ServiceM2006AngleLock(uint8_t state)
{
    if (g_m2006_angle_lock_active == 0U)
    {
        return;
    }

    if ((state == FTM_STATE_M2006_TURN_180) ||
        (state == FTM_STATE_M2006_TURN_BACK_180))
    {
        return;
    }

    M2006Angle_ControlTick();
}

uint8_t IsMechanismState(uint8_t state)
{
    return (((state >= FTM_STATE_RS05_TO_90) && (state <= FTM_STATE_DONE)) ||
            (state == FTM_STATE_M2006_TURN_BACK_180) ||
            (state == FTM_STATE_SEQUENCE_5_4_3) ||
            (state == FTM_STATE_SEQUENCE_7_3_13_9) ||
            (state == FTM_STATE_SEQUENCE_16_17_18_15)) ? 1U : 0U;
}

uint8_t IsWuqiquRouteState(uint8_t state)
{
    return ((state == FTM_STATE_WUQIQU_ROUTE) ||
            (state == FTM_STATE_WUQIQU_ROUTE_2) ||
            (state == FTM_STATE_WUQIQU_ROUTE_3) ||
            (state == FTM_STATE_WUQIQU_ROUTE_4) ||
            (state == FTM_STATE_SEQUENCE_16_17_18_15)) ? 1U : 0U;
}

void PrepareState(uint8_t state)
{
    ResetMechanismStep();
    g_sequence_step_index = 0U;
    g_wuqiqu_sequence_step_index = 0U;

    if (IsWuqiquRouteState(state) != 0U)
    {
        WuqiquTask_Stop();
        g_wuqiqu_route_finished = 0U;
    }

    if (state != FTM_STATE_WUQIQU_ZERO)
    {
        ResetWuqiquZero();
    }

    if (IsMechanismState(state) == 0U)
    {
        FTMLiftAction_SetTakeover(0U);
    }

    if (state == FTM_STATE_INIT)
    {
        M2006Angle_SetTarget(0.0f);
        g_m2006_angle_lock_active = 1U;
    }
    else if ((IsWuqiquRouteState(state) != 0U) ||
             (state == FTM_STATE_WUQIQU_ZERO))
    {
        g_m2006_angle_lock_active = 1U;
    }
    else if ((state != FTM_STATE_M2006_TURN_180) &&
             (state != FTM_STATE_M2006_TURN_BACK_180))
    {
        g_m2006_angle_lock_active = 1U;
    }
}

void EnterState(uint8_t state)
{
    g_ftm_state = state;
    g_active_state = state;
    PrepareState(state);
}

void SyncExternalState(void)
{
    const uint8_t state = g_ftm_state;
    if (state != g_active_state)
    {
        g_active_state = state;
        PrepareState(state);
    }
}

float DegToRad(float degree)
{
    return degree * kDegToRad;
}

bool IsRs05AtTarget(float target_rad)
{
    return fabsf(RS05_GetMotor().Pos_Info.Angle - target_rad) <= kRs05AngleToleranceRad;
}

void SendRs05Target(float target_rad, uint8_t zero_lock_enabled)
{
    RS05_SetZeroLock(zero_lock_enabled);
    RS05_PositionControl(Speed, target_rad);
}

bool RunRs05State(float target_degree, uint8_t zero_lock_enabled = 0U)
{
    const float target_rad = DegToRad(target_degree);

    if (g_step.finished != 0U)
    {
        if (HasElapsed(g_step.last_cmd_tick, kRs05CommandIntervalMs))
        {
            SendRs05Target(target_rad, zero_lock_enabled);
            g_step.last_cmd_tick = HAL_GetTick();
        }
        return true;
    }

    if (g_step.active == 0U)
    {
        SendRs05Target(target_rad, zero_lock_enabled);
        g_step.active = 1U;
        g_step.start_tick = HAL_GetTick();
        g_step.last_cmd_tick = g_step.start_tick;
        return false;
    }

    if (HasElapsed(g_step.last_cmd_tick, kRs05CommandIntervalMs))
    {
        SendRs05Target(target_rad, zero_lock_enabled);
        g_step.last_cmd_tick = HAL_GetTick();
    }

    if (HasElapsed(g_step.start_tick, kRs05SettleMs) == false)
    {
        return false;
    }

    if ((IsRs05AtTarget(target_rad) != false) || HasElapsed(g_step.start_tick, kRs05TimeoutMs))
    {
        g_step.finished = 1U;
        return true;
    }

    return false;
}

bool RunLiftState(float target_height_mm)
{
    if (g_step.finished != 0U)
    {
        return true;
    }

    if ((g_lift_commanded == 0U) ||
        (fabsf(target_height_mm - g_lift_last_target_height_mm) > 0.001f))
    {
        FTMLiftAction_MoveTo(target_height_mm, kLiftMoveTimeS);
        g_lift_commanded = 1U;
        g_lift_last_target_height_mm = target_height_mm;
        return false;
    }

    if (FTMLiftAction_IsFinished(kLiftToleranceMm) != 0U)
    {
        g_step.finished = 1U;
        return true;
    }

    return false;
}

bool RunClawDelay(void (*action)(void))
{
    if (g_step.finished != 0U)
    {
        return true;
    }

    if (g_step.active == 0U)
    {
        action();
        g_step.active = 1U;
        g_step.start_tick = HAL_GetTick();
        return false;
    }

    if (HasElapsed(g_step.start_tick, kClawActionDelayMs))
    {
        g_step.finished = 1U;
        return true;
    }

    return false;
}

bool RunM2006Turn(float relative_angle_degree)
{
    if (g_step.finished != 0U)
    {
        g_m2006_angle_lock_active = 1U;
        M2006Angle_ControlTick();
        return true;
    }

    if (g_step.active == 0U)
    {
        const float start_angle = M2006Angle_GetAngleDegree();
        g_m2006_angle_lock_active = 0U;
        M2006Angle_SetTarget(start_angle + relative_angle_degree);
        M2006Angle_ControlTick();
        g_step.active = 1U;
        g_step.start_tick = HAL_GetTick();
        return false;
    }

    M2006Angle_ControlTick();
    if ((M2006Angle_IsAtTarget(kM2006ToleranceDeg) != 0U) || HasElapsed(g_step.start_tick, kM2006TimeoutMs))
    {
        g_m2006_angle_lock_active = 1U;
        g_step.finished = 1U;
        return true;
    }

    return false;
}

bool RunMechanismStep(uint8_t state)
{
    switch (state)
    {
    case FTM_STATE_RS05_TO_90:
        return RunRs05State(Angle);

    case FTM_STATE_LIFT_UP:
        return RunLiftState(g_ftm_lift_up_target_mm);

    case FTM_STATE_CLAW_OPEN:
        return RunClawDelay(claw_open);

    case FTM_STATE_CLAW_CLOSE:
        return RunClawDelay(claw_close);

    case FTM_STATE_LIFT_UP_220:
        return RunLiftState(kLiftUpHighTargetMm);

    case FTM_STATE_M2006_TURN_180:
        return RunM2006Turn(kM2006TurnAngleDeg);

    case FTM_STATE_RS05_TO_0:
        return RunRs05State(0.0f, 1U);

    case FTM_STATE_LIFT_DOWN:
        return RunLiftState(g_ftm_lift_down_target_mm);

    case FTM_STATE_M2006_TURN_BACK_180:
        return RunM2006Turn(-kM2006TurnAngleDeg);

    default:
        return true;
    }
}

bool RunMechanismSequence(const uint8_t *sequence, uint8_t sequence_count)
{
    if (g_sequence_step_index >= sequence_count)
    {
        return true;
    }

    if (RunMechanismStep(sequence[g_sequence_step_index]) == false)
    {
        return false;
    }

    g_sequence_step_index++;
    ResetMechanismStep();

    return (g_sequence_step_index >= sequence_count);
}

void StartWuqiquRoutePoint(uint8_t waypoint_index)
{
    WuqiquTask_Start();
    for (uint8_t i = 0U; i < waypoint_index; ++i)
    {
        WuqiquTask_AdvanceToNext();
    }
}

void ServiceWuqiquRoutePoint(uint8_t waypoint_index)
{
    if (g_wuqiqu_route_finished != 0U)
    {
        return;
    }

    if (WuqiquTask_IsActive() == 0U)
    {
        StartWuqiquRoutePoint(waypoint_index);
        if (WuqiquTask_IsAllFinished() != 0U)
        {
            WuqiquTask_Stop();
            g_wuqiqu_route_finished = 1U;
            EnterState(FTM_STATE_INIT);
            return;
        }
    }

    if (WuqiquTask_RunOnce() != 0U)
    {
        WuqiquTask_Stop();
        g_wuqiqu_route_finished = 1U;
        EnterState(FTM_STATE_INIT);
    }
}

uint8_t RunWuqiquRoutePoint(uint8_t waypoint_index)
{
    if (WuqiquTask_IsActive() == 0U)
    {
        StartWuqiquRoutePoint(waypoint_index);
        if (WuqiquTask_IsAllFinished() != 0U)
        {
            WuqiquTask_Stop();
            return 1U;
        }
    }

    if (WuqiquTask_RunOnce() != 0U)
    {
        WuqiquTask_Stop();
        return 1U;
    }

    return 0U;
}

bool RunWuqiquAndMechanismSequence(void)
{
    if (g_wuqiqu_sequence_step_index < 3U)
    {
        const uint8_t waypoint_index = static_cast<uint8_t>(g_wuqiqu_sequence_step_index + 1U);
        if (RunWuqiquRoutePoint(waypoint_index) == 0U)
        {
            return false;
        }

        ++g_wuqiqu_sequence_step_index;
        return false;
    }

    return RunMechanismSequence(kSequence73139, static_cast<uint8_t>(sizeof(kSequence73139) / sizeof(kSequence73139[0])));
}

void ServiceWuqiquZero(void)
{
    WuqiquTask_Stop();

    if (g_wuqiqu_zero_active == 0U)
    {
        g_wuqiqu_zero_active = 1U;
        g_wuqiqu_zero_start_tick = HAL_GetTick();
        g_wuqiqu_zero_last_send_tick = 0U;
    }

    if ((g_wuqiqu_zero_last_send_tick == 0U) ||
        HasElapsed(g_wuqiqu_zero_last_send_tick, kWuqiquZeroSendIntervalMs))
    {
        send_position_to_pc(0, 1, 0, 0, 0);
        g_wuqiqu_zero_last_send_tick = HAL_GetTick();
    }

    if (HasElapsed(g_wuqiqu_zero_start_tick, kWuqiquZeroSettleMs))
    {
        EnterState(FTM_STATE_IDLE);
    }
}

void InitModules(void)
{
    if (g_modules_initialized != 0U)
    {
        return;
    }

    vis_init();
    cmd_init();
    RS05_Init();
    M2006Angle_Init();
    FTMLiftAction_Reset();
    g_modules_initialized = 1U;
}
}

extern "C" volatile uint8_t g_ftm_state = FTM_STATE_INIT;
extern "C" volatile float g_ftm_lift_up_target_mm = 81.0f;
extern "C" volatile float g_ftm_lift_down_target_mm = 70.0f;

extern "C" uint8_t FTM_GetState(void)
{
    return g_ftm_state;
}

extern "C" uint8_t FTM_IsWuqiquDone(void)
{
    return (g_ftm_state == FTM_STATE_DONE) ? 1U : 0U;
}

extern "C" uint8_t FTM_IsYawTargetCorrectionEnabled(void)
{
    if ((g_ftm_state == FTM_STATE_WUQIQU_ROUTE) ||
        (g_ftm_state == FTM_STATE_WUQIQU_ROUTE_2) ||
        (g_ftm_state == FTM_STATE_WUQIQU_ROUTE_3) ||
        (g_ftm_state == FTM_STATE_WUQIQU_ROUTE_4) ||
        ((g_ftm_state == FTM_STATE_SEQUENCE_16_17_18_15) &&
         (g_wuqiqu_sequence_step_index < 3U)) ||
        (g_ftm_state == FTM_STATE_IDLE) ||
        (g_ftm_state == FTM_STATE_INIT) ||
        (g_ftm_state == FTM_STATE_DONE) ||
        (g_ftm_state == FTM_STATE_WUQIQU_ZERO))
    {
        return 0U;
    }

    return 1U;
}

extern "C" void ftm_task(void *argument)
{
    (void)argument;

    InitModules();

    for (;;)
    {
        SyncExternalState();

        if ((IsWuqiquRouteState(g_ftm_state) == 0U) && (WuqiquTask_IsActive() != 0U))
        {
            WuqiquTask_Stop();
        }

        switch (g_ftm_state)
        {
        // 状态 0：初始化通信、电机和动作封装；初始化完成后保持空闲，等待外部写状态。
        case FTM_STATE_INIT:
            InitModules();
            EnterState(FTM_STATE_IDLE);
            break;

        // 状态 1：执行武器区跑点；跑点完成后回到状态 0。
        case FTM_STATE_WUQIQU_ROUTE:
            ServiceWuqiquRoutePoint(0U);
            break;

        // 状态 2：手动调试等待状态；不执行动作，等待 Keil Watch 写入下一状态。
        case FTM_STATE_IDLE:
            break;

        // 状态 3：RS05 转到 Keil Watch 中 Angle 指定的角度；到位或超时后回到状态 2。
        case FTM_STATE_RS05_TO_90:
            if (RunRs05State(Angle))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 状态 4：FTM 接管抬升机构，上升到 86mm；完成后回到状态 2。
        case FTM_STATE_LIFT_UP:
            if (RunLiftState(g_ftm_lift_up_target_mm))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 状态 5：夹爪张开并等待动作稳定；完成后回到状态 2。
        case FTM_STATE_CLAW_OPEN:
            if (RunClawDelay(claw_open))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 状态 6：夹爪闭合并等待动作稳定；完成后回到状态 2。
        case FTM_STATE_CLAW_CLOSE:
            if (RunClawDelay(claw_close))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 状态 7：FTM 接管抬升机构，上升到 217mm；完成后回到状态 2。
        case FTM_STATE_LIFT_UP_220:
            if (RunLiftState(kLiftUpHighTargetMm))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 状态 8：M2006 以当前角度为起点相对翻转 180 度；完成后回到状态 2。
        case FTM_STATE_M2006_TURN_180:
            if (RunM2006Turn(kM2006TurnAngleDeg))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 状态 9：RS05 回到 0 度；到位或超时后回到状态 2。
        case FTM_STATE_RS05_TO_0:
            if (RunRs05State(0.0f, 1U))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 状态 10：FTM 接管抬升机构，下降到 70mm；完成后回到状态 2。
        case FTM_STATE_LIFT_DOWN:
            if (RunLiftState(g_ftm_lift_down_target_mm))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 状态 11：全流程完成保持；M2006 停止，RS05 周期补发 0 度保持命令。
        case FTM_STATE_DONE:
            if (g_m2006_angle_lock_active == 0U)
            {
                g_m2006_angle_lock_active = 1U;
            }
            (void)RunRs05State(0.0f, 1U);
            break;

        // 状态 12：向视觉发送置零命令，稳定后回到状态 2。
        case FTM_STATE_WUQIQU_ZERO:
            ServiceWuqiquZero();
            break;

        // 状态 13：M2006 以当前角度为起点反向翻转 180 度；完成后回到状态 2。
        case FTM_STATE_M2006_TURN_BACK_180:
            if (RunM2006Turn(-kM2006TurnAngleDeg))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 状态 14：组合执行 5 -> 4 -> 3；完成后回到状态 2。
        case FTM_STATE_SEQUENCE_5_4_3:
            if (RunMechanismSequence(kSequence543, static_cast<uint8_t>(sizeof(kSequence543) / sizeof(kSequence543[0]))))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 状态 15：组合执行 7 -> 3 -> 13 -> 9；完成后回到状态 2。
        case FTM_STATE_SEQUENCE_7_3_13_9:
            if (RunMechanismSequence(kSequence73139, static_cast<uint8_t>(sizeof(kSequence73139) / sizeof(kSequence73139[0]))))
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        case FTM_STATE_WUQIQU_ROUTE_2:
            ServiceWuqiquRoutePoint(1U);
            break;

        case FTM_STATE_WUQIQU_ROUTE_3:
            ServiceWuqiquRoutePoint(2U);
            break;

        case FTM_STATE_WUQIQU_ROUTE_4:
            ServiceWuqiquRoutePoint(3U);
            break;

        case FTM_STATE_SEQUENCE_16_17_18_15:
            if (RunWuqiquAndMechanismSequence())
            {
                EnterState(FTM_STATE_IDLE);
            }
            break;

        // 异常状态：回到初始化/空闲状态，等待重新触发。
        default:
            EnterState(FTM_STATE_INIT);
            break;
        }

        ServiceM2006AngleLock(g_ftm_state);
        osDelay(1);
    }
}
