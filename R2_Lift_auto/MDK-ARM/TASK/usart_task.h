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

/* PID 参数结构体：存储串口 8 接收的 PID 参数 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float limit_inter;
    float outputmax;
} pid_data;

#define CURVE_END_0 0x00
#define CURVE_END_1 0x00
#define CURVE_END_2 0x80
#define CURVE_END_3 0x7F

#define CURVE_TX_MAX_FLOATS 10

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

/*
 * 解析 PID 参数帧
 * 帧格式：S,<kp>,<ki>,<kd>,<inter>,<outmax>E
 * 返回 0 成功，-1 失败，失败时不修改输出结构体
 */
int parse_vision_frame_pid(uint8_t *data, uint16_t len, pid_data *out);
void send_position_to_pc(int16_t behaivor, uint8_t p_diff, float X_diff, float Y_diff, float yaw);
#endif
