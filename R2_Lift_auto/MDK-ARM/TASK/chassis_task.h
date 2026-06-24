#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdint.h"

// 抬升角度环
#define OUTPUT_LIFT     3
#define INTERLIMIT_LIFT 0
#define DEBAND_LIFT     0
#define KP_LIFT         0.5
#define KI_LIFT         0
#define KD_LIFT         0

// 抬升前进
#define OUTPUT_LIFT_MOVE     10000
#define INTERLIMIT_LIFT_MOVE 0
#define DEBAND_LIFT_MOVE     1
#define KP_LIFT_MOVE         30
#define KI_LIFT_MOVE         0.8
#define KD_LIFT_MOVE         0

// 底盘整体
#define OUTPUT_CHASSIS_LINEAR     4
#define INTERLIMIT_CHASSIS_LINEAR 0
#define DEBAND_CHASSIS_LINEAR     0
#define KP_CHASSIS_LINEAR         20
#define KI_CHASSIS_LINEAR         0
#define KD_CHASSIS_LINEAR         0

#define OUTPUT_CHASSIS_3508       450
#define INTERLIMIT_CHASSIS_3508   0
#define DEBAND_CHASSIS_3508       0.5
#define KP_CHASSIS_3508           1
#define KI_CHASSIS_3508           0
#define KD_CHASSIS_3508           0

struct chassis_pid {
    /* data */
    float kp;
    float ki;
    float kd;
    float output_max;
    float limit;
};

#ifdef __cplusplus
}
#endif

#endif
