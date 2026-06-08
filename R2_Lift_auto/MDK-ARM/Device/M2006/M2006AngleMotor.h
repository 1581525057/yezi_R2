#ifndef __M2006_ANGLE_MOTOR_H__
#define __M2006_ANGLE_MOTOR_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;
    int16_t speed_rpm;
    float angle_degree;
    float target_angle_degree;
    float angle_kp;
    float angle_ki;
    float angle_kd;
    float angle_integral_limit;
    float angle_speed_limit_dps;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float speed_integral_limit;
    float current_limit;
    float position_integral;
    float speed_integral;
    float last_position_error;
    float last_speed_error;
    uint32_t last_tick;
    int16_t current_cmd;
    uint8_t initialized;
} M2006AngleContext;

extern M2006AngleContext g_m2006_angle;

void M2006Angle_Init(void);
void M2006Angle_UpdateFeedback(const uint8_t data[8]);
void M2006Angle_SetTarget(float target_angle_degree);
void M2006Angle_ControlTick(void);
void M2006Angle_Stop(void);
uint8_t M2006Angle_IsAtTarget(float tolerance_degree);
int16_t M2006Angle_GetCurrentCommand(void);
float M2006Angle_GetAngleDegree(void);

#ifdef __cplusplus
}
#endif

#endif
