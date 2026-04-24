#ifndef __RS05_H
#define __RS05_H

#include "fdcan.h"
#include "RobStride.h"
#include <stdint.h>

/* RS05 电机默认 CAN ID */
#define RS05_CANID 0x01

/* RS05 电机实例及调试参数 */
extern RobStride_Motor g_rs05_motor;
extern float Angle;
extern float Speed;

void RS05_Init(void);
void RS05_HandleCanMessage(uint32_t can_id, uint8_t *data_frame);
void RS05_PositionControl(float speed, float angle);
void RS05_PositionControlDegree(float speed, float angle_degree);
RobStride_Motor &RS05_GetMotor(void);

#endif
