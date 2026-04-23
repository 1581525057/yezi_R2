#ifndef _USART_TASK_H
#define _USART_TASK_H

#include "main.h"
#include <stdint.h>

/* 视觉数据结构体：存储上位机发送的视觉坐标偏差和角度信息 */
typedef struct
{
    int   B;         /* 整数标志位 */
    float x_diff;    /* X 轴偏差 */
    float y_diff;    /* Y 轴偏差 */
    float angle_x;   /* X 轴角度 */
} VisionData_t;

/*
 * LoRa 帧结束标志：0x00 0x00 0x80 0x7F
 * 对应 float 的正无穷大（0x7F800000），用作帧同步分隔符，
 * 因为正常传感器数据不会出现这个值。
 */
#define CURVE_END_0 0x00
#define CURVE_END_1 0x00
#define CURVE_END_2 0x80
#define CURVE_END_3 0x7F

/* LoRa 单帧最大可携带的 float 数量 */
#define CURVE_TX_MAX_FLOATS 5

/* USB 串口接收缓冲区（中断回调写入，任务循环读取） */
extern uint8_t data_usb[30];

/* 全局视觉数据实例 */
extern VisionData_t vision;

/*
 * 解析视觉帧，提取坐标偏差和角度
 * 帧格式：S,<x_diff>,<y_diff>,<angle_x>,<B>E
 * 返回 1 成功，0 失败
 */
int parse_vision_frame_computer(uint8_t *data, uint16_t len, VisionData_t *out);

#endif
