#ifndef GRIP_PUSH_H
#define GRIP_PUSH_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 夹爪控制引脚 (PA1) */
#define CLAW_Pin GPIO_PIN_7
#define CLAW_GPIO_Port GPIOD

/* 推杆继电器 K1 控制引脚 (PA2) */
#define PUSH_K1_Pin GPIO_PIN_3
#define PUSH_K1_GPIO_Port GPIOB

/* 推杆继电器 K2 控制引脚 (PA3) */
#define PUSH_K2_Pin GPIO_PIN_4
#define PUSH_K2_GPIO_Port GPIOB

/* 推杆状态枚举 */
typedef enum
{
    PUSH_STATE_STOP = 0,     // 停止
    PUSH_STATE_EXTEND,       // 伸出
    PUSH_STATE_RETRACT       // 缩回
} PushState_e;

void platform_forward(void);
void platform_backward(void);
void platform_stop(void);
void claw_open(void);
void claw_close(void);

#ifdef __cplusplus
}
#endif

#endif
