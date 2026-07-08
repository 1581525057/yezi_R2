#ifndef _USART_TASK_H
#define _USART_TASK_H

#include "main.h"
#include <stdint.h>
#define Blue HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)
#define Yellow HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)
#define Green HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)
#define Orange HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)
#define Whihe HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)
#define Red HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)
#define Red2 HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)
#define Yellow2 HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)
#define Blue2 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)

/* 视觉数据结构体：存储上位机发送的视觉坐标、角度和标定位 */
typedef struct
{
    int exec;                       /* 是否前往第二区标志 */
    float x_diff;                   /* X 坐标 */
    float y_diff;                   /* Y 坐标 */
    float angle_x;                  /* 航向角 */
    int B;                          /* 当前梅花林动作，保留现有路线接口 */
    int release_flag;               /* 是否松手标定位 */
    int claw_vertical_flag;         /* 夹爪上下标定位 */
    int claw_vertical_adjust_count; /* 夹爪上下调整接收次数 */
    int if_go;                      /* 重定位完成后是否允许继续 */
    int16_t can_up;                 /* CAN 上升标定位 */
} VisionData_t;

typedef struct
{
    /* data */
    float x;
    float y;
} Block_Vision;

/* USB 串口接收缓冲区（中断回调写入，任务循环读取） */
#define USB_RX_BUFFER_SIZE 256U
extern uint8_t data_usb[USB_RX_BUFFER_SIZE];
extern uint16_t usb_rx_idx; /* USB接收缓冲区的当前写入位置，由CDC_Receive_HS追加，usart_task消费后清零 */

/* 全局视觉数据实例 */
extern VisionData_t vision;
extern Block_Vision block_vision[10];
extern Block_Vision block_vision_middle[16];
/*
 * 解析视觉帧，提取坐标、角度和标定位，并把不定长动作及格子编号压入队列。
 * 帧格式：S,<exec>,<x>,<y>,<yaw>,C,<action...>,B,<block...>,A,<release>,<claw_vertical>,<claw_vertical_adjust_count>,P,<if_go>,<can_up>,E
 * 返回 1 成功，0 失败。
 */
int parse_vision_frame_computer(uint8_t *data, uint16_t len, VisionData_t *out);

/* 动作队列（原有） */
uint8_t vision_command_push(int cmd);
uint8_t vision_command_pop(int *out);
uint8_t vision_command_peek(int *out);
uint8_t vision_command_has_pending(void);
void vision_command_clear(void);

/* 方块队列（新增） */
uint8_t vision_block_push(int val);
uint8_t vision_block_pop(int *out);
uint8_t vision_block_has_pending(void);
void vision_block_clear(void);
void vision_plan_mark_consumed_if_empty(void);

void send_position_to_pc(int16_t behaivor, uint8_t p_diff, float X_diff, float Y_diff, float yaw);

#endif
