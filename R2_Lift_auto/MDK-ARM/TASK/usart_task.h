#ifndef _USART_TASK_H
#define _USART_TASK_H

#include "main.h"
#include <stdint.h>

/* 视觉数据结构体：存储上位机发送的视觉坐标偏差和角度信息 */
typedef struct
{
    int exec;      /* 执行程 */
    float x_diff;  /* X 轴偏差 */
    float y_diff;  /* Y 轴偏差 */
    float angle_x; /* X 轴角度 */
    int B;         /* 整数标志位（保留兼容） */
} VisionData_t;

typedef struct {
    /* data */
    float x;
    float y;
} Block_Vision;

/* PID 参数结构体：存储串口 8 接收的 PID 参数 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float limit_inter;
    float outputmax;
} pid_data;

#define CURVE_END_0         0x00
#define CURVE_END_1         0x00
#define CURVE_END_2         0x80
#define CURVE_END_3         0x7F

#define CURVE_TX_MAX_FLOATS 10

/* USB 串口接收缓冲区（中断回调写入，任务循环读取） */
#define USB_RX_BUFFER_SIZE 128U
extern uint8_t data_usb[USB_RX_BUFFER_SIZE];

/* 全局视觉数据实例 */
extern VisionData_t vision;
extern Block_Vision block_vision[10];
extern Block_Vision block_vision_middle[10];
extern Block_Vision block_vision_climb[10];
/*
 * 解析视觉帧，提取坐标偏差和角度，并把不定长 B 指令压入队列。
 * 帧格式：S,<x_diff>,<y_diff>,<yaw>E 或 S,<x_diff>,<y_diff>,<yaw>,<B>[,<B>...]E
 * 返回 1 成功，0 失败。
 */
int parse_vision_frame_computer(uint8_t *data, uint16_t len, VisionData_t *out);

/* 动作队列（原有） */
uint8_t vision_command_push(int cmd);
uint8_t vision_command_pop(int *out);
uint8_t vision_command_has_pending(void);
void vision_command_clear(void);

/* 方块队列（新增） */
uint8_t vision_block_push(int val);
uint8_t vision_block_pop(int *out);
uint8_t vision_block_has_pending(void);
void vision_block_clear(void);

/*
 * 解析 PID 参数帧
 * 帧格式：S,<kp>,<ki>,<kd>,<inter>,<outmax>E
 * 返回 0 成功，-1 失败，失败时不修改输出结构体
 */
int parse_vision_frame_pid(uint8_t *data, uint16_t len, pid_data *out);
void send_position_to_pc(int16_t behaivor, uint8_t p_diff, float X_diff, float Y_diff, float yaw);
#endif
