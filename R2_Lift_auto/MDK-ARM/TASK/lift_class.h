#ifndef _LIFTCLASS_H
#define _LIFTCLASS_H

#include "main.h"
#include "stdint.h"

// 抬升卷轮直径（单位：mm）
#define HEIGHT_DIAMETER (40.59f)

// 从动轮直径（单位：mm）
#define WHEEL_DIAMETERr_MM (30.0f)

// 直驱轮齿数
#define Z_MOTOR (24.0f)

// 从动轮齿数
#define Z_WHEEL (32.0f)

typedef struct
{
    float height;
    float angle;
} Lift_Class_Single; // 单侧抬升状态

typedef struct
{
    float rpm_left;
    float rpm_right;
    float vel_2006_left;
    float vel_2006_right;
} lift_speed;

typedef struct
{
    Lift_Class_Single left;
    Lift_Class_Single right;

    lift_speed now;    // 当前反馈
    lift_speed target; // 目标值
} Lift_Class;          // 总体状态

// 一次线性抬升高度轨迹
typedef struct
{
    float start_height;
    float target_height;
    float current_height;
    float total_time;
    float start_time;
    uint8_t started;
    uint8_t finished;
} LiftHeight_t;

typedef struct debug_lift {
    float height_calulate;
    float height_target;
    float flag;
    float vel;
    float posi;
} debug_lift;

extern Lift_Class lift_class;
extern LiftHeight_t lift_calulate;
extern debug_lift lift_debug;




#endif