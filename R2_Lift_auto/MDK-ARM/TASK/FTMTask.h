#ifndef __FTMTASK_H__
#define __FTMTASK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint8_t g_ftm_state;
extern volatile float g_ftm_lift_up_target_mm;
extern volatile float g_ftm_lift_down_target_mm;

void ftm_task(void *argument);
uint8_t FTM_GetState(void);
uint8_t FTM_IsWuqiquDone(void);
uint8_t FTM_IsYawTargetCorrectionEnabled(void);

#ifdef __cplusplus
}
#endif

#endif
