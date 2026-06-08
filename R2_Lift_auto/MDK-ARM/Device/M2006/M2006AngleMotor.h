#ifndef __M2006_ANGLE_MOTOR_H__
#define __M2006_ANGLE_MOTOR_H__

#include <stdint.h>

void M2006Angle_Init(void);
void M2006Angle_UpdateFeedback(const uint8_t data[8]);
void M2006Angle_SetTarget(float target_angle_degree);
void M2006Angle_ControlTick(void);
void M2006Angle_Stop(void);
uint8_t M2006Angle_IsAtTarget(float tolerance_degree);
int16_t M2006Angle_GetCurrentCommand(void);
float M2006Angle_GetAngleDegree(void);

#endif
