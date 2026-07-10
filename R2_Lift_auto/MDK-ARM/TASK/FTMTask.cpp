#include "FTMTask.h"
#include "wuqiqu_task.h"
#include "FTMLiftAction.h"
#include "M2006AngleMotor.h"
#include "RS05.h"
#include "GripPush.h"
#include "usart_task.h"
#include "led_task.h"
#include "laser_distance.h"
#include "cmsis_os.h"
#include <math.h>

extern "C" volatile uint32_t g_ftm_grab_settle_delay_ms;

enum FTMMainState
{
    FTM_MAIN_INIT = 0,                 // 初始化各功能模块，完成后进入空闲。
    FTM_MAIN_IDLE = 1,                 // 空闲/手动调试状态，等待 Watch 写入动作状态。
    FTM_MAIN_WUQIQU_ROUTE = 2,         // 独立执行武器区第 1 个跑点。
    FTM_MAIN_WUQIQU_ZERO = 3,          // 向视觉发送置零命令。 0 0 90
    FTM_MAIN_DONE = 4,                 // 全流程完成保持状态。
    FTM_MAIN_AUTO_PICK_ROUTE = 5,      // 武器区综合取物流程：跑第 1 点时同步开爪、预抬和 RS05 对位，到点后下降闭爪并抬到对接高度，后续跑点同步回位。
    FTM_MAIN_AUTO_TURN_READY = 6,      // 武器区调整姿态 再次取武器头流程：张爪、对位、M2006 翻转。
    FTM_MAIN_DOCKING = 7,              // 对接调试状态：MiniPC 松手和对接高度微调只在此状态生效。
    FTM_MAIN_GO_MEILIN = 8,            // 前往梅林：先回第三点，再修正航向到 0 度，最后跑梅林目标点。
    FTM_MAIN_PRELIM_AUTO_FULL_FLOW = 9 // 预选赛三武器头流程入口：按 exec=2/3/4 选择当前武器头，依次夹取三次后进梅林。
};

enum FTMActionState
{
    FTM_ACTION_NONE = 0,                              // 不执行小动作。
    FTM_ACTION_RS05_TO_TARGET = 1,                    // RS05 转到 Angle 指定角度。
    FTM_ACTION_LIFT_UP_GRAB = 2,                      // 抬升机构上升到抓取高度。
    FTM_ACTION_CLAW_OPEN = 3,                         // 夹爪张开。
    FTM_ACTION_CLAW_CLOSE = 4,                        // 夹爪闭合。
    FTM_ACTION_LIFT_UP_WEAPON_HEAD_TAKEOUT_DOCK = 5,  // 抬升机构上升到取出武器头对接高度。
    FTM_ACTION_M2006_TURN_180 = 6,                    // M2006 正向翻转 180 度。
    FTM_ACTION_RS05_TO_RETURN = 7,                    // RS05 回0度。
    FTM_ACTION_LIFT_DOWN = 8,                         // 抬升机构下降。
    FTM_ACTION_M2006_TURN_BACK_180 = 9,               // M2006 反向翻转 180 度。
    FTM_ACTION_SEQUENCE_OPEN_LIFT_RS05 = 10,          // 夹爪张开、抬升到取出武器头对接高度、RS05 对位。
    FTM_ACTION_SEQUENCE_BACKTURN_RS05 = 11,           // M2006 反向翻转 180 度、RS05 回位。
    FTM_ACTION_WUQIQU_ROUTE_2 = 12,                   // 武器区第 2 个跑点。
    FTM_ACTION_WUQIQU_YAW_TURN_180 = 13,              // 武器区第 2 点到第 3 点之间原地转向 180 度。
    FTM_ACTION_WUQIQU_ROUTE_3 = 14,                   // 武器区第 3 个跑点。
    FTM_ACTION_SEQUENCE_ROUTE_BACKTURN = 15,          // 完成武器区 2/3 号跑点和中间转向后，执行 M2006 反转和 RS05 回位。
    FTM_ACTION_SEQUENCE_ROUTE_CLOSE_LIFT = 16,        // 跑武器区 1 号点时同步开爪、预抬和 RS05 对位，到点后下降闭爪并抬到对接高度，后续跑点同步回位。
    FTM_ACTION_SEQUENCE_OPEN_RS05_TURN = 17,          // 夹爪张开、RS05 对位、M2006 正向翻转 180 度。
    FTM_ACTION_LIFT_UP_GRAB_APPROACH = 18,            // 抬升机构上升到抓取预备高度：抓取高度 + 20mm。
    FTM_ACTION_SEQUENCE_OPEN_GRAB_APPROACH_RS05 = 19, // 自动夹取专用：夹爪张开、抬升到抓取高度+20、RS05 对位。
    FTM_ACTION_GRAB_SETTLE_DELAY = 20                 // 到抓取高度后等待稳定，再闭合夹爪。
};

namespace
{
    constexpr float kDegToRad = 0.01745329251994329577f;          // 角度转弧度的固定换算系数。
    constexpr float kRs05AngleToleranceRad = 0.02f;               // RS05 角度到位容差，单位 rad。
    constexpr uint32_t kRs05CommandIntervalMs = 20U;              // RS05 发送控制指令的周期，单位 ms。
    constexpr uint32_t kRs05SettleMs = 300U;                      // RS05 到位后等待稳定时间，单位 ms。
    constexpr uint32_t kRs05TimeoutMs = 3500U;                    // RS05 转角动作超时时间，单位 ms。

    constexpr float kLiftToleranceMm = 5.0f;                      // 抬升高度到位容差，单位 mm。
    constexpr float kLiftGrabApproachOffsetMm = 20.0f;            // 抓取预备高度：在抓取高度基础上上抬 20mm。

    constexpr uint32_t kClawActionDelayMs = 200U;                 // 夹爪动作后的等待时间，单位 ms。
    constexpr float kM2006TurnAngleDeg = 180.0f;                  // M2006 翻转目标角度，单位 deg。
    constexpr float kM2006ToleranceDeg = 3.0f;                    // M2006 翻转到位容差，单位 deg。
    constexpr uint32_t kM2006TimeoutMs = 3000U;                   // M2006 翻转动作超时时间，单位 ms。

    constexpr uint32_t kWuqiquZeroSendIntervalMs = 20U;           // 武器区置零指令发送周期，单位 ms。
    constexpr uint32_t kWuqiquZeroSettleMs = 200U;                // 置零后等待视觉/底盘稳定时间，单位 ms。
    constexpr float kWuqiquZeroRelocalizeX = 0.0f;                // 置零后重定位 X 坐标。
    constexpr float kWuqiquZeroRelocalizeY = 0.0f;                // 置零后重定位 Y 坐标。
    constexpr float kWuqiquZeroRelocalizeYawDeg = 90.0f;          // 置零后重定位航向角，单位 deg。
    constexpr float kWuqiquYawTurnToleranceDeg = 1.5f;            // 武器区原地转向到位容差，单位 deg。
    constexpr uint16_t kWuqiquYawTurnStableCycles = 200U;         // 航向满足容差后需连续稳定的周期数。
    constexpr uint8_t kWuqiquSecondWaypointIndex = 1U;            // 武器区第 2 个跑点索引。
    constexpr uint8_t kWuqiquYawTargetWaypointIndex = 2U;         // 武器区第 3 点前的转向目标点索引。
    constexpr uint8_t kWuqiquMeilinWaypointIndex = 3U;            // 前往梅林使用的目标点索引。
    constexpr float kMiniPcLiftDockAdjustStepMm = 1.0f;           // 对接调试时 MiniPC 每次微调的高度步进，单位 mm。
    constexpr int kPrelimExecGoMeilin = 1;           // 预选赛流程：exec=1 表示跳转梅林。
    constexpr int kPrelimExecFirstWeapon = 2;        // 预选赛流程：exec=2 表示第 1 个武器头。
    constexpr int kPrelimExecSecondWeapon = 3;       // 预选赛流程：exec=3 表示第 2 个武器头。
    constexpr int kPrelimExecThirdWeapon = 4;        // 预选赛流程：exec=4 表示第 3 个武器头。
    constexpr uint8_t kPrelimWeaponCount = 3U;       // 预选赛连续夹取的武器头数量。
    constexpr uint8_t kPrelimPickWaypointIndex = 0U; // 预选赛每次夹取从第 1 个跑点开始。

    // 激光修正参数
    static const float kLaserCorrTargetM[3] = {1.14f, 0.94f, 0.74f}; // 各武器头到位后 laser_left 目标值（m）
    constexpr float kLaserCorrToleranceM = 0.030f;       // 到位容差 ±30mm
    constexpr float kLaserCorrKp = 1.2f;               // 激光距离闭环比例增益，误差 0.10m 时目标速度约 0.12m/s。
    constexpr float kLaserCorrMinSpeedMps = 0.07f;       // 底盘最小有效修正速度，避免小误差时推不动车。
    constexpr float kLaserCorrMaxSpeedMps = 0.25f;       // 激光闭环最大修正速度，限制贴近阶段速度。
    constexpr float kLaserCorrYHoldToleranceM = 0.020f;  // laser_left 修正时，vision.y_diff 回正容差 ±10mm。
    constexpr float kLaserCorrYHoldKp = 1.0f;            // vision.y_diff 保持比例增益，误差 0.10m 时目标速度约 0.10m/s。
    constexpr float kLaserCorrYHoldMaxSpeedMps = 0.20f;  // vision.y_diff 保持最大修正速度。
    constexpr uint32_t kLaserCorrTotalTimeoutMs = 5000U; // 激光修正总超时，超时后跳过直接夹
    constexpr uint32_t kLaserCorrInvalidTimeoutMs = 1000U; // 激光无效等待超时，超时后跳过直接夹

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
    int16_t g_last_claw_vertical_adjust_count = 0;
    int g_last_docking_exec = 0;
    uint8_t g_docking_exec_changed = 0U;
    uint8_t g_docking_lift_adjust_active = 0U;
    uint8_t g_led_prelim_green_hold_active = 0U;
    LedTask_Segment g_led_prelim_green_hold_segment = LED_TASK_SEG_ALL;
    uint8_t g_led_go_meilin_if_go_latched = 0U;
    uint8_t g_led_go_meilin_rainbow_latched = 0U;
    uint8_t g_led_angle_x_seen = 0U;
    uint8_t g_wuqiqu_yaw_turn_active = 0U;
    uint8_t g_wuqiqu_yaw_turn_target_waypoint_index = kWuqiquYawTargetWaypointIndex;
    uint16_t g_wuqiqu_yaw_turn_stable_count = 0U;
    uint8_t g_turn_ready_yaw_turn_finished = 0U;
    uint8_t g_prelim_turn_ready_step_index = 0U;

    // 激光修正运行时状态
    uint8_t g_laser_corr_active = 0U;           // 修正步骤是否已进入
    uint32_t g_laser_corr_start_tick = 0U;      // 修正步骤起始时间戳
    float g_laser_corr_y_hold_ref_m = 0.0f;     // 进入修正瞬间锁存的 vision.y_diff 基准值

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
        FTM_ACTION_GRAB_SETTLE_DELAY,
        FTM_ACTION_CLAW_CLOSE,
        FTM_ACTION_LIFT_UP_WEAPON_HEAD_TAKEOUT_DOCK};

    const uint8_t kSequenceOpenRs05Turn[] = {
        FTM_ACTION_CLAW_OPEN,
        FTM_ACTION_RS05_TO_TARGET,
        FTM_ACTION_M2006_TURN_180};

    const uint8_t kMainSequencePickRoute[] = {
        FTM_ACTION_SEQUENCE_ROUTE_CLOSE_LIFT};

    const uint8_t kMainSequenceTurnReady[] = {
        FTM_ACTION_SEQUENCE_OPEN_RS05_TURN};

    uint32_t g_wuqiqu_zero_start_tick = 0U;
    uint32_t g_wuqiqu_zero_last_send_tick = 0U;
    uint8_t g_wuqiqu_zero_active = 0U;

    constexpr uint8_t kLedStateAllWhiteOn = 1U;
    constexpr uint8_t kLedStateAllRedOn = 16U;
    constexpr uint8_t kLedStateAllGreenOn = 17U;

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
        g_wuqiqu_yaw_turn_target_waypoint_index = kWuqiquYawTargetWaypointIndex;
        g_wuqiqu_yaw_turn_stable_count = 0U;

        if (g_ftm_yaw_target_correction_state == kYawTargetCorrectionWuqiquTurn)
        {
            g_ftm_yaw_target_correction_state = kYawTargetCorrectionOff;
            g_ftm_yaw_target_degree = 0.0f;
        }
    }

    void SelectWuqiquYawTurnTarget(uint8_t waypoint_index)
    {
        if (g_wuqiqu_yaw_turn_active == 0U)
        {
            g_wuqiqu_yaw_turn_target_waypoint_index = waypoint_index;
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
        g_laser_corr_active = 0U;
        g_laser_corr_start_tick = 0U;
        g_laser_corr_y_hold_ref_m = 0.0f;
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
                (action_state == FTM_ACTION_SEQUENCE_OPEN_GRAB_APPROACH_RS05) ||
                (action_state == FTM_ACTION_GRAB_SETTLE_DELAY))
                   ? 1U
                   : 0U;
    }

    uint8_t IsWuqiquRouteMainState(uint8_t main_state)
    {
        return ((main_state == FTM_MAIN_WUQIQU_ROUTE) ||
                (main_state == FTM_MAIN_AUTO_PICK_ROUTE) ||
                (main_state == FTM_MAIN_AUTO_TURN_READY) ||
                (main_state == FTM_MAIN_GO_MEILIN))
                   ? 1U
                   : 0U;
    }

    uint8_t IsAutoFullFlowCarryState(uint8_t main_state)
    {
        return ((main_state == FTM_MAIN_AUTO_PICK_ROUTE) ||
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
        return kPrelimPickWaypointIndex;
    }

    uint8_t IsPrelimPickRoutePoint(uint8_t waypoint_index)
    {
        return ((g_prelim_auto_full_flow_active != 0U) &&
                (waypoint_index == kPrelimPickWaypointIndex))
                   ? 1U
                   : 0U;
    }

    LedTask_Segment GetCurrentLedSegment(void)
    {
        switch (g_prelim_weapon_index)
        {
        case 1U:
            return LED_TASK_SEG_MIDDLE;
        case 2U:
            return LED_TASK_SEG_BACK;
        case 0U:
        default:
            return LED_TASK_SEG_FRONT;
        }
    }

    LedTask_Segment GetLedSegmentFromExec(int exec)
    {
        switch (exec)
        {
        case kPrelimExecFirstWeapon:
            return LED_TASK_SEG_FRONT;
        case kPrelimExecSecondWeapon:
            return LED_TASK_SEG_MIDDLE;
        case kPrelimExecThirdWeapon:
            return LED_TASK_SEG_BACK;
        default:
            return LED_TASK_SEG_ALL;
        }
    }

    LedTask_Segment GetActiveWeaponLedSegment(void)
    {
        const LedTask_Segment exec_segment = GetLedSegmentFromExec(vision.exec);
        if (exec_segment != LED_TASK_SEG_ALL)
        {
            return exec_segment;
        }

        return GetCurrentLedSegment();
    }

    uint8_t GetRedOnLedState(LedTask_Segment segment)
    {
        switch (segment)
        {
        case LED_TASK_SEG_MIDDLE:
            return 3U;
        case LED_TASK_SEG_BACK:
            return 4U;
        case LED_TASK_SEG_FRONT:
        default:
            return 2U;
        }
    }

    uint8_t GetRedFlashLedState(LedTask_Segment segment)
    {
        switch (segment)
        {
        case LED_TASK_SEG_MIDDLE:
            return 6U;
        case LED_TASK_SEG_BACK:
            return 7U;
        case LED_TASK_SEG_FRONT:
        default:
            return 5U;
        }
    }

    uint8_t GetGreenOnLedState(LedTask_Segment segment)
    {
        switch (segment)
        {
        case LED_TASK_SEG_MIDDLE:
            return 9U;
        case LED_TASK_SEG_BACK:
            return 10U;
        case LED_TASK_SEG_FRONT:
        default:
            return 8U;
        }
    }

    void UpdateLedForMainState(uint8_t main_state)
    {
        const LedTask_Segment segment = GetActiveWeaponLedSegment();

        if ((g_led_angle_x_seen == 0U) && (vision.angle_x != 0.0f))
        {
            // 视觉角度首次有效后锁存白灯，后续由 exec 和状态 3 提升优先级。
            g_led_angle_x_seen = 1U;
        }

        switch (main_state)
        {
        case FTM_MAIN_INIT:
            LED_state = 16U;
            break;
        case FTM_MAIN_IDLE:
            if (vision.exec == 0)
            {
                LED_state = 15U;
            }
            else
            {
                LED_state = 1U;
            }
            break;
        case FTM_MAIN_AUTO_PICK_ROUTE:
            if (g_led_prelim_green_hold_active != 0U)
            {
                g_led_prelim_green_hold_active = 0U;
            }
            LED_state = GetRedOnLedState(segment);
            break;
        case FTM_MAIN_PRELIM_AUTO_FULL_FLOW:
            LED_state = GetRedOnLedState(segment);
            break;
        case FTM_MAIN_AUTO_TURN_READY:
            if (g_led_prelim_green_hold_active != 0U)
            {
                LED_state = GetGreenOnLedState(g_led_prelim_green_hold_segment);
            }
            else
            {
                LED_state = GetRedOnLedState(segment);
            }
            break;
        case FTM_MAIN_DOCKING:
            LED_state = GetRedFlashLedState(segment);
            break;
        case FTM_MAIN_GO_MEILIN:
            if (vision.if_go == 1)
            {
                g_led_go_meilin_if_go_latched = 1U;
                LED_state = 12U;
            }
            else
            {
                LED_state = 11U;
            }
            break;
        case FTM_MAIN_WUQIQU_ZERO:
            LED_state = 16U;
            break;
        case FTM_MAIN_DONE:
            LED_state = (g_led_go_meilin_rainbow_latched != 0U) ? 14U : 12U;
            break;
        default:
            break;
        }

        if ((main_state == FTM_MAIN_IDLE) && (vision.exec == 0) && (g_led_angle_x_seen != 0U))
        {
            LED_state = kLedStateAllWhiteOn;
        }
        if ((main_state == FTM_MAIN_IDLE) && (vision.exec != 0))
        {
            LED_state = kLedStateAllRedOn;
        }
        if (main_state == FTM_MAIN_WUQIQU_ZERO)
        {
            LED_state = kLedStateAllGreenOn;
        }
    }

    void ResetPrelimAutoFullFlow(void)
    {
        g_prelim_auto_full_flow_active = 0U;
        g_prelim_weapon_index = 0U;
        g_prelim_docking_release_latched = 0U;
        g_prelim_turn_ready_step_index = 0U;
        g_led_prelim_green_hold_active = 0U;
        g_led_prelim_green_hold_segment = LED_TASK_SEG_ALL;
    }

    void ResetDockingExecEdge(void)
    {
        g_last_docking_exec = vision.exec;
        g_docking_exec_changed = 0U;
    }

    void UpdateDockingExecEdge(void)
    {
        const int current_exec = vision.exec;
        if (g_ftm_main_state == FTM_MAIN_DOCKING)
        {
            g_docking_exec_changed = (current_exec != g_last_docking_exec) ? 1U : 0U;
        }
        else
        {
            g_docking_exec_changed = 0U;
        }
        g_last_docking_exec = current_exec;
    }

    uint8_t IsDockingExecChangedTo(int target_exec)
    {
        return ((g_ftm_main_state == FTM_MAIN_DOCKING) &&
                (g_docking_exec_changed != 0U) &&
                (vision.exec == target_exec))
                   ? 1U
                   : 0U;
    }

    uint8_t IsWuqiquRouteAction(uint8_t action_state)
    {
        return ((action_state == FTM_ACTION_WUQIQU_ROUTE_2) ||
                (action_state == FTM_ACTION_WUQIQU_YAW_TURN_180) ||
                (action_state == FTM_ACTION_WUQIQU_ROUTE_3) ||
                (action_state == FTM_ACTION_SEQUENCE_ROUTE_BACKTURN) ||
                (action_state == FTM_ACTION_SEQUENCE_ROUTE_CLOSE_LIFT))
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
        UpdateLedForMainState(main_state);
        ResetActionRuntime();
        g_main_action_step_index = 0U;
        g_turn_ready_yaw_turn_finished = 0U;
        if (main_state == FTM_MAIN_AUTO_TURN_READY)
        {
            g_prelim_turn_ready_step_index = 0U;
            g_wuqiqu_parallel_mechanism_finished = 0U;
        }

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
            ResetDockingExecEdge();
            if (g_prelim_auto_full_flow_active != 0U)
            {
                g_prelim_docking_release_latched = 0U;
            }
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

        if (IsDockingExecChangedTo(kPrelimExecGoMeilin) != 0U)
        {
            EnterMainState(FTM_MAIN_GO_MEILIN);
            return 1U;
        }

        return 0U;
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
            g_last_claw_vertical_adjust_count = g_ftm_minipc_claw_vertical_adjust_count;
            g_docking_lift_adjust_active = 0U;
            return;
        }

        if (g_last_minipc_control_seq == g_ftm_minipc_control_seq)
        {
            return;
        }
        g_last_minipc_control_seq = g_ftm_minipc_control_seq;

        if (g_ftm_minipc_claw_vertical_adjust_count != g_last_claw_vertical_adjust_count)
        {
            g_last_claw_vertical_adjust_count = g_ftm_minipc_claw_vertical_adjust_count;

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
            // 松爪只放开夹爪；底盘转向由后续 exec 变化触发。
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
            FTMLiftAction_MoveTo(target_height_mm);
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

    bool RunTimedDelay(uint32_t delay_ms)
    {
        if (g_step.finished != 0U)
        {
            return true;
        }

        if (g_step.active == 0U)
        {
            g_step.active = 1U;
            g_step.start_tick = HAL_GetTick();
            return false;
        }

        if (HasElapsed(g_step.start_tick, delay_ms))
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

        case FTM_ACTION_GRAB_SETTLE_DELAY:
            return RunTimedDelay(g_ftm_grab_settle_delay_ms);

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
        if (IsPrelimPickRoutePoint(waypoint_index) != 0U)
        {
            WuqiquTask_StartAtPrelimWeaponHead(g_prelim_weapon_index);
        }
        else
        {
            WuqiquTask_StartAt(waypoint_index);
        }
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
            g_ftm_yaw_target_degree = NormalizeYawDeg(WuqiquTask_GetWaypointYawDeg(g_wuqiqu_yaw_turn_target_waypoint_index));
            g_ftm_yaw_target_correction_state = kYawTargetCorrectionWuqiquTurn;
            g_wuqiqu_yaw_turn_stable_count = 0U;
            g_wuqiqu_yaw_turn_active = 1U;
        }

        const float yaw_error_deg = NormalizeYawDeg(g_ftm_yaw_target_degree - vision.angle_x);
        if (fabsf(yaw_error_deg) < (2.0f * kWuqiquYawTurnToleranceDeg))
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
        SelectWuqiquYawTurnTarget(kWuqiquYawTargetWaypointIndex);
        const bool yaw_turn_finished = (RunWuqiquYawTurn180() != 0U);
        const bool mechanism_finished =
            RunParallelMechanismSequence(kSequenceBackturnRs05,
                                         static_cast<uint8_t>(sizeof(kSequenceBackturnRs05) / sizeof(kSequenceBackturnRs05[0])));

        return ((yaw_turn_finished != false) && (mechanism_finished != false));
    }

    bool RunPrelimTurnReadyAndYawTurnSequence(void)
    {
        switch (g_prelim_turn_ready_step_index)
        {
        case 0:
            if (RunWuqiquRoutePoint(kWuqiquYawTargetWaypointIndex) == 0U)
            {
                return false;
            }
            ++g_prelim_turn_ready_step_index;
            return false;

        case 1:
            if (g_wuqiqu_parallel_mechanism_finished == 0U)
            {
                if (g_ftm_action_state != FTM_ACTION_SEQUENCE_OPEN_RS05_TURN)
                {
                    EnterActionState(FTM_ACTION_SEQUENCE_OPEN_RS05_TURN);
                }

                if (RunMechanismSequence(kSequenceOpenRs05Turn,
                                         static_cast<uint8_t>(sizeof(kSequenceOpenRs05Turn) / sizeof(kSequenceOpenRs05Turn[0]))) != false)
                {
                    g_wuqiqu_parallel_mechanism_finished = 1U;
                }
            }

            if (g_turn_ready_yaw_turn_finished == 0U)
            {
                // 预选赛再次取头前，先到第三点，再执行机械准备和底盘转向。
                SelectWuqiquYawTurnTarget(kWuqiquSecondWaypointIndex);
                if (RunWuqiquYawTurn180() != 0U)
                {
                    g_turn_ready_yaw_turn_finished = 1U;
                }
            }

            return ((g_wuqiqu_parallel_mechanism_finished != 0U) &&
                    (g_turn_ready_yaw_turn_finished != 0U));

        default:
            return true;
        }
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
            if (RunWuqiquRoutePoint(kWuqiquYawTargetWaypointIndex) == 0U)
            {
                return 0U;
            }
            ++g_go_meilin_step_index;
            return 0U;

        case 1:
            if (RunGoMeilinYawZero() == 0U)
            {
                return 0U;
            }
            ++g_go_meilin_step_index;
            return 0U;

        case 2:
            if (RunWuqiquRoutePoint(kWuqiquMeilinWaypointIndex) == 0U)
            {
                return 0U;
            }
            if ((g_led_go_meilin_if_go_latched != 0U) || (vision.if_go == 1))
            {
                g_led_go_meilin_rainbow_latched = 1U;
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
            if (IsDockingExecChangedTo(next_exec) != 0U)
            {
                g_led_prelim_green_hold_active = 1U;
                g_led_prelim_green_hold_segment = GetCurrentLedSegment();
                ++g_prelim_weapon_index;
                g_prelim_docking_release_latched = 0U;
                EnterMainState(FTM_MAIN_AUTO_TURN_READY);
                return 1U;
            }
        }
        else if (IsDockingExecChangedTo(kPrelimExecGoMeilin) != 0U)
        {
            g_prelim_docking_release_latched = 0U;
            g_led_go_meilin_if_go_latched = 0U;
            g_led_go_meilin_rainbow_latched = 0U;
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

            return true;

        default:
            return true;
        }
    }

    // 返回当前武器头对应的 laser_left 目标值（m）。
    // 预选赛流程按 g_prelim_weapon_index 索引，非预选赛流程固定使用第 0 个。
    float GetCurrentWeaponHeadLaserTargetM(void)
    {
        uint8_t idx = 0U;
        if ((g_prelim_auto_full_flow_active != 0U) && (g_prelim_weapon_index < 3U))
        {
            idx = g_prelim_weapon_index;
        }
        return kLaserCorrTargetM[idx];
    }

    float ClampLaserCorrSpeed(float vx_mps)
    {
        if (vx_mps > kLaserCorrMaxSpeedMps)
        {
            return kLaserCorrMaxSpeedMps;
        }
        if (vx_mps < -kLaserCorrMaxSpeedMps)
        {
            return -kLaserCorrMaxSpeedMps;
        }

        if (fabsf(vx_mps) < kLaserCorrMinSpeedMps)
        {
            return (vx_mps >= 0.0f) ? kLaserCorrMinSpeedMps : -kLaserCorrMinSpeedMps;
        }

        return vx_mps;
    }

    float ClampLaserCorrYHoldSpeed(float vx_mps)
    {
        if (vx_mps > kLaserCorrYHoldMaxSpeedMps)
        {
            return kLaserCorrYHoldMaxSpeedMps;
        }
        if (vx_mps < -kLaserCorrYHoldMaxSpeedMps)
        {
            return -kLaserCorrYHoldMaxSpeedMps;
        }

        return vx_mps;
    }

    // 到位后用 laser_left 修正贴边距离，同时用 vision.y_diff 保持进入修正时的 Y 基准。
    // 每次 tick 调用一次；返回 true 表示修正完成（含超时跳过情况），返回 false 表示仍在进行。
    bool RunLaserCorrectionStep(void)
    {
        if (g_laser_corr_active == 0U)
        {
            g_laser_corr_active = 1U;
            g_laser_corr_start_tick = HAL_GetTick();
            g_laser_corr_y_hold_ref_m = vision.y_diff;
        }

        // 总超时保护：超时后跳过修正直接进入夹取
        if (HasElapsed(g_laser_corr_start_tick, kLaserCorrTotalTimeoutMs))
        {
            WuqiquTask_Stop();
            return true;
        }

        // 激光无效时等待；超时则跳过
        if (laser_left.data.valid == 0U)
        {
            if (HasElapsed(g_laser_corr_start_tick, kLaserCorrInvalidTimeoutMs))
            {
                WuqiquTask_Stop();
                return true;
            }
            WuqiquTask_SetChassisTarget(0.0f, 0.0f, 0.0f);
            return false;
        }

        const float actual_m = laser_left.data.distance_m;
        const float target_m = GetCurrentWeaponHeadLaserTargetM();
        const float delta_m = actual_m - target_m;
        const float y_delta_m = g_laser_corr_y_hold_ref_m - vision.y_diff;

        // 已在容差范围内，修正完成
        if ((fabsf(delta_m) <= kLaserCorrToleranceM) &&
            (fabsf(y_delta_m) <= kLaserCorrYHoldToleranceM))
        {
            WuqiquTask_Stop();
            return true;
        }

        // 到点后不再改路线点，直接下发底盘车体系速度做闭环微调。
        // laser_left 偏大时向左平移（+Vy），偏小时向右平移（-Vy）。
        // Vx 用来把 vision.y_diff 拉回进入修正瞬间的基准，抑制贴边修正时车身走歪。
        const float vx_mps = ClampLaserCorrYHoldSpeed(kLaserCorrYHoldKp * y_delta_m);
        const float vy_mps = ClampLaserCorrSpeed(kLaserCorrKp * delta_m);
        WuqiquTask_SetChassisTarget(vx_mps, vy_mps, 0.0f);

        return false;
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

        // case 1：激光修正 —— 到位后根据 laser_left 和 vision.y_diff 微调底盘，直到进入容差或超时。
        case 1:
            if (RunLaserCorrectionStep() == false)
            {
                return false;
            }
            ++g_route_action_sequence_step_index;
            return false;

        case 2:
            if (RunMechanismSequence(kSequenceGrabClose, static_cast<uint8_t>(sizeof(kSequenceGrabClose) / sizeof(kSequenceGrabClose[0]))) == false)
            {
                return false;
            }
            ++g_route_action_sequence_step_index;
            g_action_sequence_step_index = 0U;
            ResetMechanismStep();
            return false;

        case 3:
            if (RunWuqiquAndMechanismSequence() == false)
            {
                return false;
            }
            ++g_route_action_sequence_step_index;
            return false;

        case 4:
            if (RunWuqiquRoutePoint(4U) == 0U)
            {
                return false;
            }
            return true;

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
        case FTM_ACTION_GRAB_SETTLE_DELAY:
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
extern "C" volatile float g_ftm_lift_up_target_mm = 74.0f;
extern "C" volatile float g_ftm_lift_weapon_head_takeout_dock_target_mm = 214.0f;
extern "C" volatile float g_ftm_lift_down_target_mm = 68.0f;
extern "C" volatile float g_ftm_rs05_return_target_degree = 0.0f;
extern "C" volatile uint32_t g_ftm_grab_settle_delay_ms = 200U;
extern "C" volatile uint8_t g_ftm_minipc_claw_release_cmd = 0U;
extern "C" volatile uint8_t g_ftm_minipc_lift_dock_adjust_cmd = 0U;
extern "C" volatile int16_t g_ftm_minipc_claw_vertical_adjust_count = 0;
extern "C" volatile uint32_t g_ftm_minipc_control_seq = 0U;

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

extern "C" void ftm_task(void *argument)
{
    (void)argument;

    InitModules();

    for (;;)
    {
        SyncExternalState();
        UpdateDockingExecEdge();
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
            if (g_prelim_auto_full_flow_active != 0U)
            {
                if (RunPrelimTurnReadyAndYawTurnSequence() != false)
                {
                    EnterMainState(FTM_MAIN_AUTO_PICK_ROUTE);
                }
            }
            else
            {
                if (RunMainActionSequence(kMainSequenceTurnReady,
                                          static_cast<uint8_t>(sizeof(kMainSequenceTurnReady) / sizeof(kMainSequenceTurnReady[0]))))
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

        // 主状态 8：前往梅林；先回第三点，再修正航向到 0 度，最后跑梅林目标点。
        case FTM_MAIN_GO_MEILIN:
            if (RunGoMeilinSequence() != 0U)
            {
                if (g_auto_full_flow_active != 0U)
                {
                    g_auto_full_flow_active = 0U;
                }
                EnterMainState(FTM_MAIN_DONE);
            }
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
                g_led_prelim_green_hold_active = 0U;
                g_led_prelim_green_hold_segment = LED_TASK_SEG_ALL;
                g_led_go_meilin_if_go_latched = 0U;
                g_led_go_meilin_rainbow_latched = 0U;
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
        UpdateLedForMainState(g_ftm_main_state);
        osDelay(1);
    }
}
