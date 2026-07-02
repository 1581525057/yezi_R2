#ifndef __FTMTASK_H__
#define __FTMTASK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Keil Watch 可直接写入：主流程状态。
// 0 初始化，1 空闲/动作调试，2 武器区跑点1，3 视觉置零，4 完成保持，5 武器区综合取物流程，6 武器区姿态准备流程，7 对接调试，8 前往梅林，9 完整自动流程，10 预选赛三武器头流程。
extern volatile uint8_t g_ftm_main_state;
extern volatile uint8_t wuqiqu_done;

// Keil Watch 可直接写入：独立小动作状态。
// 0 无动作，1~19 对应 FTMTask.cpp 中 FTMActionState；12 第 2 点，13 中间 yaw 转向，14 第 3 点，18 抓取预备高度，19 自动夹取预备流程。
extern volatile uint8_t g_ftm_action_state;
extern volatile uint8_t g_ftm_yaw_target_correction_state;
extern volatile float g_ftm_yaw_target_degree;
extern volatile float g_ftm_lift_up_target_mm;
extern volatile float g_ftm_lift_weapon_head_takeout_dock_target_mm;
extern volatile float g_ftm_lift_down_target_mm;
extern volatile float g_ftm_rs05_return_target_degree;
extern volatile uint8_t g_ftm_minipc_claw_release_cmd;
extern volatile uint8_t g_ftm_minipc_lift_dock_adjust_cmd;
extern volatile int16_t g_ftm_minipc_unused_mark;
extern volatile uint32_t g_ftm_minipc_control_seq;

void ftm_task(void *argument);
uint8_t FTM_GetState(void);
uint8_t FTM_GetMainState(void);
uint8_t FTM_GetActionState(void);
uint8_t FTM_IsWuqiquDone(void);
uint8_t FTM_IsYawTargetCorrectionEnabled(void);
uint8_t FTM_IsYawTargetTurnActive(void);
float FTM_GetYawTargetDegree(void);

#ifdef __cplusplus
}
#endif

#endif
