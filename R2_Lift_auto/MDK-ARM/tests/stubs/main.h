#ifndef TEST_STUB_MAIN_H
#define TEST_STUB_MAIN_H

#include <stdint.h>

#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_0 0U
#define GPIO_PIN_4 4U

typedef void GPIO_TypeDef;

static GPIO_TypeDef *const GPIOB = 0;

static inline void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, uint8_t)
{
}

#endif
