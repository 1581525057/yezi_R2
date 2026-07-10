#ifndef __WUQIQU_TASK_H__
#define __WUQIQU_TASK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 武器区路径任务对外接口，供 FreeRTOS 入口和其他任务统一引用。
void wuqiqu_task(void *argument);
void WuqiquTask_SetTeamSide(uint8_t team_side);
void WuqiquTask_Start(void);
void WuqiquTask_StartAt(uint8_t waypoint_index);
void WuqiquTask_StartAtPrelimWeaponHead(uint8_t weapon_index);
void WuqiquTask_SetChassisTarget(float vx_mps, float vy_mps, float wz_radps);
void WuqiquTask_Stop(void);
uint8_t WuqiquTask_RunOnce(void);
uint8_t WuqiquTask_IsActive(void);
uint8_t WuqiquTask_IsFinished(void);
float WuqiquTask_GetChassisVxTarget(float manual);
float WuqiquTask_GetChassisVyTarget(float manual);
float WuqiquTask_GetChassisVzTarget(float manual);
void WuqiquTask_AdvanceToNext(void);
uint8_t WuqiquTask_IsAllFinished(void);
float WuqiquTask_GetWaypointYawDeg(uint8_t waypoint_index);

#ifdef __cplusplus
}
#endif

#endif
