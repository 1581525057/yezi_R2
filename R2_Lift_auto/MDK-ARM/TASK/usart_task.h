#ifndef _USART_TASK_H
#define _USART_TASK_H

#include "main.h"
#include "stdint.h"

typedef struct
{
    float x_diff;
    float y_diff;
    float angle_x;
} VisionData_t;



// 结束标志：00 00 80 7F （对应 float 的 NaN/特殊值标记常见用法）
#define CURVE_END_0         0x00
#define CURVE_END_1         0x00
#define CURVE_END_2         0x80
#define CURVE_END_3         0x7F

#define CURVE_TX_MAX_FLOATS 5

extern uint8_t data_usb[30];

#endif
