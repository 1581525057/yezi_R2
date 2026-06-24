#ifndef __FTM_LIFT_ACTION_H__
#define __FTM_LIFT_ACTION_H__

#include <stdint.h>

void FTMLiftAction_Reset(void);
void FTMLiftAction_MoveTo(float target_height_mm, float move_time_s);
uint8_t FTMLiftAction_IsFinished(float tolerance_mm);
void FTMLiftAction_SetTakeover(uint8_t enable);
uint8_t FTMLiftAction_IsTakeover(void);

#endif
