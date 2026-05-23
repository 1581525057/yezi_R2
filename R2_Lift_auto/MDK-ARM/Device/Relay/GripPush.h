#ifndef GRIP_PUSH_H
#define GRIP_PUSH_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 夹爪控制引脚 (PA1) */
#define CLAW_Pin GPIO_PIN_7
#define CLAW_GPIO_Port GPIOD

void claw_open(void);
void claw_close(void);

#ifdef __cplusplus
}
#endif

#endif
