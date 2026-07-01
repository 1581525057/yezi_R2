#include "FTMTask.h"
#include "FTMLiftAction.h"
#include "M2006AngleMotor.h"
#include "RS05.h"
#include "GripPush.h"
#include "usart_task.h"
#include "cmsis_os.h"
#include <math.h>

extern "C" void WuqiquTask_Start(void);
extern "C" void WuqiquTask_StartAt(uint8_t waypoint_index);
extern "C" void WuqiquTask_Stop(void);
extern "C" uint8_t WuqiquTask_IsActive(void);
extern "C" uint8_t WuqiquTask_IsFinished(void);
extern "C" float WuqiquTask_GetWaypointYawDeg(uint8_t waypoint_index);
extern "C" void WuqiquTask_StartDockAdjust(void);

enum FTMMainState
{
    FTM_MAIN_INIT = 0,            // 初始化各功能模块，完成后进入空闲。
    FTM_MAIN_IDLE = 1,            // 空闲/手动调试状态，等待 Watch 写入动作状态。
    FTM_MAIN_WUQIQU_ROUTE = 2,    // 独立执行武器区第 1 个跑点。
    FTM_MAIN_WUQIQU_ZERO = 3,     // 向视觉发送置零命令。
    FTM_MAIN_DONE = 4,            // 全流程完成保持状态。
    FTM_MAIN_AUTO_PICK_ROUTE = 5, // 武器区综合取物流程：跑第 1 点时同步开爪、预抬和 RS05 对位，到点后下降闭爪并抬到对接高度，后续跑点同步回位。
    FTM_MAIN_AUTO_TURN_READY = 6, // 武器区姿态准备流程：张爪、对位、M2006 翻转并让 RS05 回 0。
    FTM_MAIN_DOCKING = 7,         // 对接调试状态：MiniPC 松手和对接高度微调只在此状态生效。
    FTM_MAIN_GO_MEILIN = 8,             // 前往梅林：先修正航向到 0 度，再跑梅林目标点。
    FTM_MAIN_AUTO_FULL_FLOW = 9,        // 完整自动流程入口：切入 5，之后依次执行 7、8、4。
    FTM_MAIN_PRELIM_AUTO_FULL_FLOW = 10 // 预选赛三武器头流程入口：按 exec=2/3/4 选择当前武器头，依次夹取三次后进梅林。
};

enum FTMActionState
{
    FTM_ACTION_NONE = 0,                             // 不执行小动作。
    FTM_ACTION_RS05_TO_TARGET = 1,                   // RS05 转到 Angle 指定角度。
    FTM_ACTION_LIFT_UP_GRAB = 2,                     // 抬升机构上升到抓取高度。
    FTM_ACTION_CLAW_OPEN = 3,                        // 夹爪张开。
    FTM_ACTION_CLAW_CLOSE = 4,                       // 夹爪闭合。
    FTM_ACTION_LIFT_UP_WEAPON_HEAD_TAKEOUT_DOCK = 5, // 抬升机构上升到取出武器头对接高度。
    FTM_ACTION_M2006_TURN_180 = 6,                   // M2006 正向翻转 180 度。
    FTM_ACTION_RS05_TO_RETURN = 7,                   // RS05 回到可调目标角度。
    FTM_ACTION_LIFT_DOWN = 8,                        // 抬升机构下降。
    FTM_ACTION_M2006_TURN_BACK_180 = 9,              // M2006 反向翻转 180 度。
    FTM_ACTION_SEQUENCE_OPEN_LIFT_RS05 = 10,         // 夹爪张开、抬升到取出武器头对接高度、RS05 对位。
    FTM_ACTION_SEQUENCE_BACKTURN_RS05 = 11,          // M2006 反向翻转 180 度、RS05 回位。
    FTM_ACTION_WUQIQU_ROUTE_2 = 12,                  // 武器区第 2 个跑点。
    FTM_ACTION_WUQIQU_YAW_TURN_180 = 13,             // 武器区第 2 点到第 3 点之间原地转向 180 度。
    FTM_ACTION_WUQIQU_ROUTE_3 = 14,                  // 武器区第 3 个跑点。
    FTM_ACTION_SEQUENCE_ROUTE_BACKTURN = 15,         // 完成武器区 2/3 号跑点和中间转向后，执行 M2006 反转和 RS05 回位。
    FTM_ACTION_SEQUENCE_ROUTE_CLOSE_LIFT = 16,       // 跑武器区 1 号点时同步开爪、预抬和 RS05 对位，到点后下降闭爪并抬到对接高度，后续跑点同步回位。
    FTM_ACTION_SEQUENCE_OPEN_RS05_TURN = 17,         // 夹爪张开、RS05 对位、M2006 正向翻转 180 度。
    FTM_ACTION_LIFT_UP_GRAB_APPROACH = 18,           // 抬升机构上升到抓取预备高度：抓取高度 + 20mm。
    FTM_ACTION_SEQUENCE_OPEN_GRAB_APPROACH_RS05 = 19 // 自动夹取专用：夹爪张开、抬升到抓取高度+20、RS05 对位。
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
    constexpr float kLiftGrabApproachOffsetMm = 20.0f;

    constexpr uint32_t kClawActionDelayMs = 200U;
    constexpr float kM2006TurnAngleDeg = 180.0f;
    constexpr float kM2006ToleranceDeg = 3.0f;
    constexpr uint32_t kM2006TimeoutMs = 3000U;

    constexpr uint32_t kWuqiquZeroSendIntervalMs = 20U;
    constexpr uint32_t kWuqiquZeroSettleMs = 200U;
    constexpr float kWuqiquZeroRelocalizeX = 0.0f;
    constexpr float kWuqiquZeroRelocalizeY = 0.0f;
    constexpr float kWuqiquZeroRelocalizeYawDeg = -90.0f;
    constexpr float kWuqiquYawTurnToleranceDeg = 1.5f;
    constexpr uint16_t kWuqiquYawTurnStableCycles = 200U;
    constexpr uint8_t kWuqiquSecondWaypointIndex = 1U;
    constexpr uint8_t kWuqiquYawTargetWaypointIndex = 2U;
    constexpr uint8_t kWuqiquMeilinWaypointIndex = 3U;
    constexpr float kMiniPcLiftDockAdjustStepMm = 1.0f;
    constexpr uint8_t kFtmActionDockingPreAdjust = 20U;
    constexpr int kPrelimExecGoMeilin = 1;
    constexpr int kPrelimExecFirstWeapon = 2;
    constexpr int kPrelimExecSecondWeapon = 3;
    constexpr int kPrelimExecThirdWeapon = 4;
    constexpr uint8_t kPrelimWeaponCount = 3U;

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
    uint8_t g_wuqiqu_route_started = 0U;
    uint8_t g_wuqiqu_parallel_route_finished = 0U;
    uint8_t g_wuqiqu_parallel_mechanism_finished = 0U;
    uint8_t g_active_main_state = 0xFFU;
    uint8_t g_active_action_state = 0xFFU;
    uint8_t g_modules_initialized = 0U;
    uint8_t g_m2006_angle_lock_active = 1U;
    constexpr uint8_t kYawTargetCorrectionOff = 0U;
    constexpr uint8_t kYawTargetCorrectionZero = 1U;
    constexpr uint8_t kYawTargetCorrectionWuqiquTurn = 2U;
    uint8_t g_action_sequence_step_index = 0U;
    uint8_t g_wuqiqu_route_sequence_step_index = 0U;
    uint8_t g_route_action_sequence_step_index = 0U;
    uint8_t g_go_meilin_step_index = 0U;
    uint8_t g_main_action_step_index = 0U;
    uint8_t g_auto_full_flow_active = 0U;
    uint8_t g_prelim_auto_full_flow_active = 0U;
    uint8_t g_prelim_weapon_index = 0U;
    uint8_t g_prelim_docking_release_latched = 0U;
    uint32_t g_last_minipc_control_seq = 0U;
    int16_t g_last_lift_adjust_unused_mark = 0;
    uint8_t g_docking_lift_adjust_active = 0U;
    uint8_t g_wuqiqu_yaw_turn_active = 0U;
    uint16_t g_wuqiqu_yaw_turn_stable_count = 0U;
    uint8_t g_docking_pre_adjust_started = 0U;

    const uint8_t kSequenceOpenLiftRs05[] = {
        FTM_ACTION_CLAW_OPEN,
        FTM_ACTION_LIFT_UP_WEAPON_HEAD_TAKEOUT_DOCK,
        FTM_ACTION_RS05_TO_TARGET};

    const uint8_t kSequenceOpenGrabApproachRs05[] = {
        FTM_ACTION_CLAW_OPEN,
        FTM_ACTION_LIFT_UP_GRAB_APPROACH,
        FTM_ACTION_RS05_TO_TARGET};

    const uint8_t kSequenceBackturnRs05[] = {
        FTM_ACTION_M2006_TURN_BACK_180,
        FTM_ACTION_RS05_TO_RETURN};

    const uint8_t kSequenceGrabClose[] = {
        FTM_ACTION_LIFT_UP_GRAB,
        FTM_ACTION_CLAW_CLOSE,
        FTM_ACTION_LIFT_UP_WEAPON_HEAD_TAKEOUT_DOCK};

    const uint8_t kSequenceOpenRs05Turn[] = {
        FTM_ACTION_CLAW_OPEN,
        FTM_ACTION_RS05_TO_TARGET,
        FTM_ACTION_M2006_TURN_180};

    const uint8_t kMainSequencePickRoute[] = {
        FTM_ACTION_SEQUENCE_ROUTE_CLOSE_LIFT};

    const uint8_t kMainSequenceTurnReady[] = {
        FTM_ACTION_SEQUENCE_OPEN_RS05_TURN,
        FTM_ACTION_RS05_TO_RETURN};

    uint32_t g_wuqiqu_zero_start_tick = 0U;
    uint32_t g_wuqiqu_zero_last_send_tick = 0U;
    uint8_t g_wuqiqu_zero_active = 0U;

    void SetDockingBrake(uint8_t active);

    bool HasElapsed(uint32_t start_tick, uint32_t duration_ms)
    {
        return static_cast<uint32_t>(HAL_GetTick() - start_tick) >= duration_ms;
    }

    float NormalizeYawDeg(float yaw_deg)
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

    float GetLiftGrabApproachTargetMm(void)
    {
        return g_ftm_lift_up_target_mm + kLiftGrabApproachOffsetMm;
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

    void ResetYawTargetTurnRuntime(void)
    {
        g_wuqiqu_yaw_turn_active = 0U;
        g_wuqiqu_yaw_turn_stable_count = 0U;

        if (g_ftm_yaw_target_correction_state == kYawTargetCorrectionWuqiquTurn)
        {
            g_ftm_yaw_target_correction_state = kYawTargetCorrectionOff;
            g_ftm_yaw_target_degree = 0.0f;
        }
    }

    void ResetActionRuntime(void)
    {
        ResetMechanismStep();
        ResetYawTargetTurnRuntime();
        g_action_sequence_step_index = 0U;
        g_wuqiqu_route_sequence_step_index = 0U;
        g_route_action_sequence_step_index = 0U;
        g_go_meilin_step_index = 0U;
        g_wuqiqu_route_started = 0U;
        g_wuqiqu_parallel_route_finished = 0U;
        g_wuqiqu_parallel_mechanism_finished = 0U;
        g_docking_pre_adjust_started = 0U;
    }

    uint8_t IsMechanismAction(uint8_t action_state)
    {
        return (((action_state >= FTM_ACTION_RS05_TO_TARGET) &&
                 (action_state <= FTM_ACTION_M2006_TURN_BACK_180)) ||
                (action_state == FTM_ACTION_SEQUENCE_OPEN_LIFT_RS05) ||
                (action_state == FTM_ACTION_SEQUENCE_BACKTURN_RS05) ||
                (action_state == FTM_ACTION_SEQUENCE_ROUTE_BACKTURN) ||
                (action_state == FTM_ACTION_SEQUENCE_ROUTE_CLOSE_LIFT) ||
                (action_state == FTM_ACTION_SEQUENCE_OPEN_RS05_TURN) ||
                (action_state == FTM_ACTION_LIFT_UP_GRAB_APPROACH) ||
                (action_state == FTM_ACTION_SEQUENCE_OPEN_GRAB_APPROACH_RS05))
                   ? 1U
                   : 0U;
    }

    uint8_t IsWuqiquRouteMainState(uint8_t main_state)
    {
        return ((main_state == FTM_MAIN_WUQIQU_ROUTE) ||
                (main_state == FTM_MAIN_AUTO_PICK_ROUTE) ||
                (main_state == FTM_MAIN_GO_MEILIN))
                   ? 1U
                   : 0U;
    }

    uint8_t IsAutoFullFlowCarryState(uint8_t main_state)
    {
        return ((main_state == FTM_MAIN_AUTO_FULL_FLOW) ||
                (main_state == FTM_MAIN_AUTO_PICK_ROUTE) ||
                (main_state == FTM_MAIN_DOCKING) ||
                (main_state == FTM_MAIN_GO_MEILIN) ||
                (main_state == FTM_MAIN_AUTO_TURN_READY) ||
                (main_state == FTM_MAIN_PRELIM_AUTO_FULL_FLOW))
                   ? 1U
                   : 0U;
    }

    uint8_t GetPrelimWeaponIndexFromExec(int exec)
    {
        if (exec == kPrelimExecFirstWeapon)
        {
            return 0U;
        }
        if (exec == kPrelimExecSecondWeapon)
        {
            return 1U;
        }
        if (exec == kPrelimExecThirdWeapon)
        {
            return 2U;
        }
        return 0xFFU;
    }

    uint8_t GetCurrentPickWaypointIndex(void)
    {
        return (g_prelim_auto_full_flow_active != 0U) ? g_prelim_weapon_index : 0U;
    }

    void ResetPrelimAutoFullFlow(void)
    {
        g_prelim_auto_full_flow_active = 0U;
        g_prelim_weapon_index = 0U;
        g_prelim_docking_release_latched = 0U;
    }

    uint8_t IsWuqiquRouteAction(uint8_t action_state)
    {
        return ((action_state == FTM_ACTION_WUQIQU_ROUTE_2) ||
                (action_state == FTM_ACTION_WUQIQU_YAW_TURN_180) ||
                (action_state == FTM_ACTION_WUQIQU_ROUTE_3) ||
                (action_state == FTM_ACTION_SEQUENCE_ROUTE_BACKTURN) ||
                (action_state == FTM_ACTION_SEQUENCE_ROUTE_CLOSE_LIFT) ||
                (action_state == kFtmActionDockingPreAdjust))
                   ? 1U
                   : 0U;
    }

    uint8_t IsWuqiquRouteActiveContext(void)
    {
        return ((IsWuqiquRouteMainState(g_ftm_main_state) != 0U) ||
                (IsWuqiquRouteAction(g_ftm_action_state) != 0U))
                   ? 1U
                   : 0U;
    }

    void UpdateYawTargetCorrectionState(uint8_t main_state)
    {
        if (main_state == FTM_MAIN_DONE)
        {
            g_ftm_yaw_target_degree = 0.0f;
            g_ftm_yaw_target_correction_state = kYawTargetCorrectionZero;
        }
        else
        {
            g_ftm_yaw_target_correction_state = kYawTargetCorrectionOff;
        }
    }

    void ServiceM2006AngleLock(uint8_t action_state)
    {
        if (g_m2006_angle_lock_active == 0U)
        {
            return;
        }

        if ((action_state == FTM_ACTION_M2006_TURN_180) ||
            (action_state == FTM_ACTION_M2006_TURN_BACK_180))
        {
            return;
        }

        M2006Angle_ControlTick();
    }

    void PrepareActionState(uint8_t action_state)
    {
        ResetActionRuntime();

        if (IsWuqiquRouteAction(action_state) != 0U)
        {
            WuqiquTask_Stop();
            g_wuqiqu_route_finished = 0U;
            g_wuqiqu_route_started = 0U;
        }

        if ((IsMechanismAction(action_state) == 0U) &&
            (g_ftm_main_state != FTM_MAIN_DONE))
        {
            FTMLiftAction_SetTakeover(0U);
        }

        if ((action_state != FTM_ACTION_M2006_TURN_180) &&
            (action_state != FTM_ACTION_M2006_TURN_BACK_180))
        {
            g_m2006_angle_lock_active = 1U;
        }
    }

    void PrepareMainState(uint8_t main_state)
    {
        UpdateYawTargetCorrectionState(main_state);
        ResetActionRuntime();
        g_main_action_step_index = 0U;

        if (IsAutoFullFlowCarryState(main_state) == 0U)
        {
            g_auto_full_flow_active = 0U;
            ResetPrelimAutoFullFlow();
        }

        if (main_state != FTM_MAIN_IDLE)
        {
            g_ftm_action_state = FTM_ACTION_NONE;
            g_active_action_state = FTM_ACTION_NONE;
        }

        if (IsWuqiquRouteMainState(main_state) != 0U)
        {
            WuqiquTask_Stop();
            g_wuqiqu_route_finished = 0U;
            g_wuqiqu_route_started = 0U;
        }

        if (main_state != FTM_MAIN_WUQIQU_ZERO)
        {
            ResetWuqiquZero();
        }

        if ((main_state != FTM_MAIN_DONE) &&
            (IsMechanismAction(g_ftm_action_state) == 0U))
        {
            FTMLiftAction_SetTakeover(0U);
        }

        if (main_state == FTM_MAIN_INIT)
        {
            M2006Angle_SetTarget(0.0f);
        }

        if ((main_state == FTM_MAIN_INIT) ||
            (main_state == FTM_MAIN_WUQIQU_ROUTE) ||
            (main_state == FTM_MAIN_WUQIQU_ZERO) ||
            (main_state == FTM_MAIN_AUTO_PICK_ROUTE) ||
            (main_state == FTM_MAIN_AUTO_TURN_READY) ||
            (main_state == FTM_MAIN_GO_MEILIN))
        {
            g_m2006_angle_lock_active = 1U;
        }

        if (main_state == FTM_MAIN_DOCKING)
        {
            if (g_prelim_auto_full_flow_active != 0U)
            {
                g_prelim_docking_release_latched = 0U;
            }
            SetDockingBrake(1U);
        }
        else
        {
            SetDockingBrake(0U);
        }
    }

    void EnterMainState(uint8_t main_state)
    {
        g_ftm_main_state = main_state;
        g_active_main_state = main_state;
        PrepareMainState(main_state);
    }

    void EnterActionState(uint8_t action_state)
    {
        g_ftm_action_state = action_state;
        g_active_action_state = action_state;
        PrepareActionState(action_state);
    }

    uint8_t TryEnterGoMeilinFromDocking(void)
    {
        if (g_prelim_auto_full_flow_active != 0U)
        {
            return 0U;
        }

        if ((g_ftm_main_state == FTM_MAIN_DOCKING) && (vision.exec == 1))
        {
            SetDockingBrake(0U);
            EnterMainState(FTM_MAIN_GO_MEILIN);
            return 1U;
        }

        return 0U;
    }

    void SetDockingBrake(uint8_t active)
    {
        g_ftm_docking_brake_active = (active != 0U) ? 1U : 0U;
    }

    void SyncExternalState(void)
    {
        const uint8_t main_state = g_ftm_main_state;
        if (main_state != g_active_main_state)
        {
            g_active_main_state = main_state;
            PrepareMainState(main_state);
        }

        const uint8_t action_state = g_ftm_action_state;
        if (action_state != g_active_action_state)
        {
            g_active_action_state = action_state;
            PrepareActionState(action_state);
        }
    }

    void ServiceMiniPcFtmCommands(void)
    {
        if (g_ftm_main_state != FTM_MAIN_DOCKING)
        {
            g_last_minipc_control_seq = g_ftm_minipc_control_seq;
            g_last_lift_adjust_unused_mark = g_ftm_minipc_unused_mark;
            g_docking_lift_adjust_active = 0U;
            return;
        }

        if (g_last_minipc_control_seq == g_ftm_minipc_control_seq)
        {
            return;
        }
        g_last_minipc_control_seq = g_ftm_minipc_control_seq;

        if (g_ftm_minipc_unused_mark != g_last_lift_adjust_unused_mark)
        {
            g_last_lift_adjust_unused_mark = g_ftm_minipc_unused_mark;

            const uint8_t lift_adjust_cmd = g_ftm_minipc_lift_dock_adjust_cmd;
            uint8_t lift_adjust_valid = 0U;
            if (lift_adjust_cmd == 1U)
            {
                g_ftm_lift_weapon_head_takeout_dock_target_mm += kMiniPcLiftDockAdjustStepMm;
                lift_adjust_valid = 1U;
            }
            else if (lift_adjust_cmd == 2U)
            {
                g_ftm_lift_weapon_head_takeout_dock_target_mm -= kMiniPcLiftDockAdjustStepMm;
                lift_adjust_valid = 1U;
            }

            if (lift_adjust_valid != 0U)
            {
                g_docking_lift_adjust_active = 1U;
                ResetMechanismStep();
            }
        }

        if (g_ftm_minipc_claw_release_cmd == 1U)
        {
            claw_open();
            SetDockingBrake(0U);
            if (g_prelim_auto_full_flow_active != 0U)
            {
                g_prelim_docking_release_latched = 1U;
            }
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

    bool RunMechanismStep(uint8_t action_state)
    {
        switch (action_state)
        {
        case FTM_ACTION_RS05_TO_TARGET:
            return RunRs05State(Angle);

        case FTM_ACTION_LIFT_UP_GRAB:
            return RunLiftState(g_ftm_lift_up_target_mm);

        case FTM_ACTION_LIFT_UP_GRAB_APPROACH:
            return RunLiftState(GetLiftGrabApproachTargetMm());

        case FTM_ACTION_CLAW_OPEN:
            return RunClawDelay(claw_open);

        case FTM_ACTION_CLAW_CLOSE:
            return RunClawDelay(claw_close);

        case FTM_ACTION_LIFT_UP_WEAPON_HEAD_TAKEOUT_DOCK:
            return RunLiftState(g_ftm_lift_weapon_head_takeout_dock_target_mm);

        case FTM_ACTION_M2006_TURN_180:
            return RunM2006Turn(kM2006TurnAngleDeg);

        case FTM_ACTION_RS05_TO_RETURN:
            return RunRs05State(g_ftm_rs05_return_target_degree, 1U);

        case FTM_ACTION_LIFT_DOWN:
            return RunLiftState(g_ftm_lift_down_target_mm);

        case FTM_ACTION_M2006_TURN_BACK_180:
            return RunM2006Turn(-kM2006TurnAngleDeg);

        default:
            return true;
        }
    }

    bool RunMechanismSequence(const uint8_t *sequence, uint8_t sequence_count)
    {
        if (g_action_sequence_step_index >= sequence_count)
        {
            return true;
        }

        if (RunMechanismStep(sequence[g_action_sequence_step_index]) == false)
        {
            return false;
        }

        g_action_sequence_step_index++;
        ResetMechanismStep();

        return (g_action_sequence_step_index >= sequence_count);
    }

    void StartWuqiquRoutePoint(uint8_t waypoint_index)
    {
        WuqiquTask_StartAt(waypoint_index);
        g_wuqiqu_route_started = 1U;
    }

    void ServiceWuqiquRoutePoint(uint8_t waypoint_index)
    {
        if (g_wuqiqu_route_finished != 0U)
        {
            return;
        }

        if (WuqiquTask_IsFinished() != 0U)
        {
            WuqiquTask_Stop();
            g_wuqiqu_route_started = 0U;
            g_wuqiqu_route_finished = 1U;
            EnterMainState(FTM_MAIN_INIT);
            return;
        }

        if (g_wuqiqu_route_started == 0U)
        {
            StartWuqiquRoutePoint(waypoint_index);
            if (WuqiquTask_IsFinished() != 0U)
            {
                WuqiquTask_Stop();
                g_wuqiqu_route_started = 0U;
                g_wuqiqu_route_finished = 1U;
                EnterMainState(FTM_MAIN_INIT);
            }
        }
    }

    uint8_t RunWuqiquRoutePoint(uint8_t waypoint_index)
    {
        if (WuqiquTask_IsFinished() != 0U)
        {
            WuqiquTask_Stop();
            g_wuqiqu_route_started = 0U;
            return 1U;
        }

        if (g_wuqiqu_route_started == 0U)
        {
            StartWuqiquRoutePoint(waypoint_index);
            if (WuqiquTask_IsFinished() != 0U)
            {
                WuqiquTask_Stop();
                g_wuqiqu_route_started = 0U;
                return 1U;
            }
        }

        return 0U;
    }

    bool RunWuqiquRoutePointAndMechanismSequence(uint8_t waypoint_index, const uint8_t *sequence, uint8_t sequence_count)
    {
        if (g_wuqiqu_parallel_route_finished == 0U)
        {
            if (RunWuqiquRoutePoint(waypoint_index) != 0U)
            {
                g_wuqiqu_parallel_route_finished = 1U;
            }
        }

        const bool mechanism_finished = RunMechanismSequence(sequence, sequence_count);

        return ((g_wuqiqu_parallel_route_finished != 0U) && (mechanism_finished != false));
    }

    bool RunParallelMechanismSequence(const uint8_t *sequence, uint8_t sequence_count)
    {
        if (g_wuqiqu_parallel_mechanism_finished != 0U)
        {
            return true;
        }

        if (RunMechanismSequence(sequence, sequence_count) != false)
        {
            g_wuqiqu_parallel_mechanism_finished = 1U;
            return true;
        }

        return false;
    }

    uint8_t RunWuqiquYawTurn180(void)
    {
        if (g_wuqiqu_yaw_turn_active == 0U)
        {
            WuqiquTask_Stop();
            g_ftm_yaw_target_degree = NormalizeYawDeg(WuqiquTask_GetWaypointYawDeg(kWuqiquYawTargetWaypointIndex));
            g_ftm_yaw_target_correction_state = kYawTargetCorrectionWuqiquTurn;
            g_wuqiqu_yaw_turn_stable_count = 0U;
            g_wuqiqu_yaw_turn_active = 1U;
        }

        const float yaw_error_deg = NormalizeYawDeg(g_ftm_yaw_target_degree - vision.angle_x);
        if (fabsf(yaw_error_deg) < kWuqiquYawTurnToleranceDeg)
        {
            if (g_wuqiqu_yaw_turn_stable_count < kWuqiquYawTurnStableCycles)
            {
                ++g_wuqiqu_yaw_turn_stable_count;
            }
        }
        else
        {
            g_wuqiqu_yaw_turn_stable_count = 0U;
        }

        if (g_wuqiqu_yaw_turn_stable_count >= kWuqiquYawTurnStableCycles)
        {
            ResetYawTargetTurnRuntime();
            return 1U;
        }

        return 0U;
    }

    bool RunWuqiquYawTurnAndMechanismSequence(void)
    {
        const bool yaw_turn_finished = (RunWuqiquYawTurn180() != 0U);
        const bool mechanism_finished =
            RunParallelMechanismSequence(kSequenceBackturnRs05,
                                         static_cast<uint8_t>(sizeof(kSequenceBackturnRs05) / sizeof(kSequenceBackturnRs05[0])));

        return ((yaw_turn_finished != false) && (mechanism_finished != false));
    }

    bool RunDockingPreAdjust(void)
    {
        if (g_docking_pre_adjust_started == 0U)
        {
            WuqiquTask_StartDockAdjust();
            g_docking_pre_adjust_started = 1U;
        }

        if (WuqiquTask_IsFinished() == 0U)
        {
            return false;
        }

        WuqiquTask_Stop();
        g_docking_pre_adjust_started = 0U;
        SetDockingBrake(1U);
        return true;
    }

    uint8_t RunGoMeilinYawZero(void)
    {
        if (g_wuqiqu_yaw_turn_active == 0U)
        {
            WuqiquTask_Stop();
            g_ftm_yaw_target_degree = 0.0f;
            g_ftm_yaw_target_correction_state = kYawTargetCorrectionWuqiquTurn;
            g_wuqiqu_yaw_turn_stable_count = 0U;
            g_wuqiqu_yaw_turn_active = 1U;
        }

        const float yaw_error_deg = NormalizeYawDeg(g_ftm_yaw_target_degree - vision.angle_x);
        if (fabsf(yaw_error_deg) < kWuqiquYawTurnToleranceDeg)
        {
            if (g_wuqiqu_yaw_turn_stable_count < kWuqiquYawTurnStableCycles)
            {
                ++g_wuqiqu_yaw_turn_stable_count;
            }
        }
        else
        {
            g_wuqiqu_yaw_turn_stable_count = 0U;
        }

        if (g_wuqiqu_yaw_turn_stable_count >= kWuqiquYawTurnStableCycles)
        {
            ResetYawTargetTurnRuntime();
            return 1U;
        }

        return 0U;
    }

    uint8_t RunGoMeilinSequence(void)
    {
        switch (g_go_meilin_step_index)
        {
        case 0:
            if (RunGoMeilinYawZero() == 0U)
            {
                return 0U;
            }
            ++g_go_meilin_step_index;
            return 0U;

        case 1:
            if (RunWuqiquRoutePoint(kWuqiquMeilinWaypointIndex) == 0U)
            {
                return 0U;
            }
            ++g_go_meilin_step_index;
            return 1U;

        default:
            return 1U;
        }
    }

    uint8_t ServicePrelimDockingFlow(void)
    {
        if (g_prelim_auto_full_flow_active == 0U)
        {
            return 0U;
        }

        if (g_prelim_docking_release_latched == 0U)
        {
            return 0U;
        }

        if (g_prelim_weapon_index < static_cast<uint8_t>(kPrelimWeaponCount - 1U))
        {
            const int next_exec = kPrelimExecFirstWeapon + static_cast<int>(g_prelim_weapon_index) + 1;
            if (vision.exec == next_exec)
            {
                ++g_prelim_weapon_index;
                g_prelim_docking_release_latched = 0U;
                EnterMainState(FTM_MAIN_AUTO_TURN_READY);
                return 1U;
            }
        }
        else if (vision.exec == kPrelimExecGoMeilin)
        {
            g_prelim_docking_release_latched = 0U;
            SetDockingBrake(0U);
            EnterMainState(FTM_MAIN_GO_MEILIN);
            return 1U;
        }

        return 0U;
    }

    bool RunWuqiquAndMechanismSequence(void)
    {
        switch (g_wuqiqu_route_sequence_step_index)
        {
        case 0:
            if (RunWuqiquRoutePoint(kWuqiquSecondWaypointIndex) == 0U)
            {
                return false;
            }
            ++g_wuqiqu_route_sequence_step_index;
            return false;

        case 1:
            if (RunWuqiquYawTurnAndMechanismSequence() == false)
            {
                return false;
            }
            ++g_wuqiqu_route_sequence_step_index;
            return false;

        case 2:
            if (RunWuqiquRoutePoint(kWuqiquYawTargetWaypointIndex) == 0U)
            {
                return false;
            }

            ++g_wuqiqu_route_sequence_step_index;
            return false;

        case 3:
            return RunDockingPreAdjust();

        default:
            return true;
        }
    }

    bool RunRouteAndMechanismSequence(void)
    {
        switch (g_route_action_sequence_step_index)
        {
        case 0:
            if (RunWuqiquRoutePointAndMechanismSequence(GetCurrentPickWaypointIndex(),
                                                        kSequenceOpenGrabApproachRs05,
                                                        static_cast<uint8_t>(sizeof(kSequenceOpenGrabApproachRs05) / sizeof(kSequenceOpenGrabApproachRs05[0]))) == false)
            {
                return false;
            }
            ++g_route_action_sequence_step_index;
            g_wuqiqu_parallel_route_finished = 0U;
            g_action_sequence_step_index = 0U;
            ResetMechanismStep();
            return false;

        case 1:
            if (RunMechanismSequence(kSequenceGrabClose, static_cast<uint8_t>(sizeof(kSequenceGrabClose) / sizeof(kSequenceGrabClose[0]))) == false)
            {
                return false;
            }
            ++g_route_action_sequence_step_index;
            g_action_sequence_step_index = 0U;
            ResetMechanismStep();
            return false;

        case 2:
            return RunWuqiquAndMechanismSequence();

        default:
            return true;
        }
    }

    bool RunActionState(uint8_t action_state)
    {
        switch (action_state)
        {
        case FTM_ACTION_NONE:
            return true;

        case FTM_ACTION_RS05_TO_TARGET:
        case FTM_ACTION_LIFT_UP_GRAB:
        case FTM_ACTION_CLAW_OPEN:
        case FTM_ACTION_CLAW_CLOSE:
        case FTM_ACTION_LIFT_UP_WEAPON_HEAD_TAKEOUT_DOCK:
        case FTM_ACTION_M2006_TURN_180:
        case FTM_ACTION_RS05_TO_RETURN:
        case FTM_ACTION_LIFT_DOWN:
        case FTM_ACTION_M2006_TURN_BACK_180:
        case FTM_ACTION_LIFT_UP_GRAB_APPROACH:
            return RunMechanismStep(action_state);

        case FTM_ACTION_SEQUENCE_OPEN_LIFT_RS05:
            return RunMechanismSequence(kSequenceOpenLiftRs05,
                                        static_cast<uint8_t>(sizeof(kSequenceOpenLiftRs05) / sizeof(kSequenceOpenLiftRs05[0])));

        case FTM_ACTION_SEQUENCE_BACKTURN_RS05:
            return RunMechanismSequence(kSequenceBackturnRs05,
                                        static_cast<uint8_t>(sizeof(kSequenceBackturnRs05) / sizeof(kSequenceBackturnRs05[0])));

        case FTM_ACTION_WUQIQU_ROUTE_2:
            return (RunWuqiquRoutePoint(1U) != 0U);

        case FTM_ACTION_WUQIQU_YAW_TURN_180:
            return (RunWuqiquYawTurn180() != 0U);

        case FTM_ACTION_WUQIQU_ROUTE_3:
            return (RunWuqiquRoutePoint(2U) != 0U);

        case FTM_ACTION_SEQUENCE_ROUTE_BACKTURN:
            return RunWuqiquAndMechanismSequence();

        case FTM_ACTION_SEQUENCE_ROUTE_CLOSE_LIFT:
            return RunRouteAndMechanismSequence();

        case kFtmActionDockingPreAdjust:
            return RunDockingPreAdjust();

        case FTM_ACTION_SEQUENCE_OPEN_RS05_TURN:
            return RunMechanismSequence(kSequenceOpenRs05Turn,
                                        static_cast<uint8_t>(sizeof(kSequenceOpenRs05Turn) / sizeof(kSequenceOpenRs05Turn[0])));

        case FTM_ACTION_SEQUENCE_OPEN_GRAB_APPROACH_RS05:
            return RunMechanismSequence(kSequenceOpenGrabApproachRs05,
                                        static_cast<uint8_t>(sizeof(kSequenceOpenGrabApproachRs05) / sizeof(kSequenceOpenGrabApproachRs05[0])));

        default:
            return true;
        }
    }

    bool RunMainActionSequence(const uint8_t *sequence, uint8_t sequence_count)
    {
        if (g_main_action_step_index >= sequence_count)
        {
            return true;
        }

        const uint8_t expected_action = sequence[g_main_action_step_index];
        if (g_ftm_action_state != expected_action)
        {
            EnterActionState(expected_action);
        }

        if (RunActionState(expected_action) == false)
        {
            return false;
        }

        ++g_main_action_step_index;
        EnterActionState(FTM_ACTION_NONE);

        return (g_main_action_step_index >= sequence_count);
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
            send_position_to_pc(0,
                                1,
                                kWuqiquZeroRelocalizeX,
                                kWuqiquZeroRelocalizeY,
                                kWuqiquZeroRelocalizeYawDeg);
            g_wuqiqu_zero_last_send_tick = HAL_GetTick();
        }

        if (HasElapsed(g_wuqiqu_zero_start_tick, kWuqiquZeroSettleMs))
        {
            EnterMainState(FTM_MAIN_IDLE);
        }
    }

    void InitModules(void)
    {
        if (g_modules_initialized != 0U)
        {
            return;
        }

        RS05_Init();
        M2006Angle_Init();
        FTMLiftAction_Reset();
        g_modules_initialized = 1U;
    }
}

extern "C" volatile uint8_t g_ftm_main_state = FTM_MAIN_INIT;
extern "C" volatile uint8_t g_ftm_action_state = FTM_ACTION_NONE;
extern "C" volatile uint8_t wuqiqu_done = 0U;
extern "C" volatile uint8_t g_ftm_yaw_target_correction_state = 0U;
extern "C" volatile float g_ftm_yaw_target_degree = 0.0f;
extern "C" volatile float g_ftm_lift_up_target_mm = 78.0f;
extern "C" volatile float g_ftm_lift_weapon_head_takeout_dock_target_mm = 214.0f;
extern "C" volatile float g_ftm_lift_down_target_mm = 68.0f;
extern "C" volatile float g_ftm_rs05_return_target_degree = 0.0f;
extern "C" volatile uint8_t g_ftm_minipc_claw_release_cmd = 0U;
extern "C" volatile uint8_t g_ftm_minipc_lift_dock_adjust_cmd = 0U;
extern "C" volatile int16_t g_ftm_minipc_unused_mark = 0;
extern "C" volatile uint32_t g_ftm_minipc_control_seq = 0U;
extern "C" volatile uint8_t g_ftm_docking_brake_active = 0U;
extern "C" volatile int32_t g_ftm_docking_brake_current_mA = 5000;

extern "C" uint8_t FTM_GetState(void)
{
    return g_ftm_main_state;
}

extern "C" uint8_t FTM_GetMainState(void)
{
    return g_ftm_main_state;
}

extern "C" uint8_t FTM_GetActionState(void)
{
    return g_ftm_action_state;
}

extern "C" uint8_t FTM_IsWuqiquDone(void)
{
    return ((g_ftm_main_state == FTM_MAIN_DONE) ||
            (g_ftm_main_state == FTM_MAIN_GO_MEILIN) ||
            (g_ftm_yaw_target_correction_state == kYawTargetCorrectionWuqiquTurn))
               ? 1U
               : 0U;
}

extern "C" uint8_t FTM_IsYawTargetCorrectionEnabled(void)
{
    return (g_ftm_yaw_target_correction_state != kYawTargetCorrectionOff) ? 1U : 0U;
}

extern "C" uint8_t FTM_IsYawTargetTurnActive(void)
{
    return (g_ftm_yaw_target_correction_state == kYawTargetCorrectionWuqiquTurn) ? 1U : 0U;
}

extern "C" float FTM_GetYawTargetDegree(void)
{
    return g_ftm_yaw_target_degree;
}

extern "C" uint8_t FTM_IsDockingBrakeActive(void)
{
    return g_ftm_docking_brake_active;
}

extern "C" int32_t FTM_GetDockingBrakeCurrentmA(void)
{
    return (g_ftm_docking_brake_current_mA > 0) ? g_ftm_docking_brake_current_mA : 0;
}

extern "C" void ftm_task(void *argument)
{
    (void)argument;

    InitModules();

    for (;;)
    {
        SyncExternalState();
        (void)TryEnterGoMeilinFromDocking();
        ServiceMiniPcFtmCommands();

        if ((IsWuqiquRouteActiveContext() == 0U) && (WuqiquTask_IsActive() != 0U))
        {
            WuqiquTask_Stop();
        }

        switch (g_ftm_main_state)
        {
        // 主状态 0：初始化通信、电机和动作封装；完成后进入主状态 1。
        case FTM_MAIN_INIT:
            InitModules();
            EnterMainState(FTM_MAIN_IDLE);
            break;

        // 主状态 1：空闲/手动调试；写 g_ftm_action_state 可单独执行小动作。
        case FTM_MAIN_IDLE:
            if (g_ftm_action_state != FTM_ACTION_NONE)
            {
                if (RunActionState(g_ftm_action_state))
                {
                    EnterActionState(FTM_ACTION_NONE);
                }
            }
            break;

        // 主状态 2：独立执行武器区第 1 个跑点；完成后回到主状态 0。
        case FTM_MAIN_WUQIQU_ROUTE:
            ServiceWuqiquRoutePoint(0U);
            break;

        // 主状态 3：向视觉发送置零命令；稳定后回到主状态 1。
        case FTM_MAIN_WUQIQU_ZERO:
            ServiceWuqiquZero();
            break;

        // 主状态 4：全流程完成保持；RS05 周期补发回位角度，底盘可开启航向保持。
        case FTM_MAIN_DONE:
            wuqiqu_done = 1U;
            if (g_m2006_angle_lock_active == 0U)
            {
                g_m2006_angle_lock_active = 1U;
            }
            (void)RunRs05State(g_ftm_rs05_return_target_degree, 1U);
            break;

        // 主状态 5：武器区综合取物流程；完成后回到主状态 1。
        case FTM_MAIN_AUTO_PICK_ROUTE:
            if (RunMainActionSequence(kMainSequencePickRoute,
                                      static_cast<uint8_t>(sizeof(kMainSequencePickRoute) / sizeof(kMainSequencePickRoute[0]))))
            {
                EnterMainState(FTM_MAIN_DOCKING);
            }
            break;

        // 主状态 6：武器区姿态准备流程；完成后回到主状态 1。
        case FTM_MAIN_AUTO_TURN_READY:
            if (RunMainActionSequence(kMainSequenceTurnReady,
                                      static_cast<uint8_t>(sizeof(kMainSequenceTurnReady) / sizeof(kMainSequenceTurnReady[0]))))
            {
                if (g_prelim_auto_full_flow_active != 0U)
                {
                    EnterMainState(FTM_MAIN_AUTO_PICK_ROUTE);
                }
                else
                {
                    EnterMainState(FTM_MAIN_IDLE);
                }
            }
            break;

        // 主状态 7：对接调试；只处理 MiniPC 的松手和对接高度微调，状态保持不自动退出。
        case FTM_MAIN_DOCKING:
            if (TryEnterGoMeilinFromDocking() != 0U)
            {
                break;
            }

            if (ServicePrelimDockingFlow() != 0U)
            {
                break;
            }

            if (g_docking_lift_adjust_active != 0U)
            {
                if (RunLiftState(g_ftm_lift_weapon_head_takeout_dock_target_mm))
                {
                    g_docking_lift_adjust_active = 0U;
                    ResetMechanismStep();
                }
            }
            break;

        // 主状态 8：前往梅林；先修正航向到 0 度，再跑梅林目标点。
        case FTM_MAIN_GO_MEILIN:
            if (RunGoMeilinSequence() != 0U)
            {
                if (g_auto_full_flow_active != 0U)
                {
                    if (RunWuqiquRoutePoint(kWuqiquYawTargetWaypointIndex) == 0U)
                    {
                        break;
                    }
                    g_auto_full_flow_active = 0U;
                }
                EnterMainState(FTM_MAIN_DONE);
            }
            break;

        // 主状态 9：完整自动流程入口；进入 5，之后按 5->7->8->4 自动衔接。
        case FTM_MAIN_AUTO_FULL_FLOW:
            g_auto_full_flow_active = 1U;
            EnterMainState(FTM_MAIN_AUTO_PICK_ROUTE);
            break;

        case FTM_MAIN_PRELIM_AUTO_FULL_FLOW:
        {
            const uint8_t weapon_index = GetPrelimWeaponIndexFromExec(vision.exec);
            if (weapon_index != 0xFFU)
            {
                g_auto_full_flow_active = 0U;
                g_prelim_auto_full_flow_active = 1U;
                g_prelim_weapon_index = weapon_index;
                g_prelim_docking_release_latched = 0U;
                EnterMainState(FTM_MAIN_AUTO_PICK_ROUTE);
            }
            break;
        }

        // 异常主状态：回到初始化，等待重新触发。
        default:
            EnterMainState(FTM_MAIN_INIT);
            break;
        }

        ServiceM2006AngleLock(g_ftm_action_state);
        osDelay(1);
    }
}
