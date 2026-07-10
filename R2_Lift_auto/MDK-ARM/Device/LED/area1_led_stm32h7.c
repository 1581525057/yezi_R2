#include "area1_led_stm32h7.h"
#include "stm32h7_ws281x.h"

static void Area1_STM32H7_SetPixel(void *user, uint16_t index, Area1Led_Color color)
{
    STM32H7_WS281X_Color c;
    (void)user;

    c.r = color.r;
    c.g = color.g;
    c.b = color.b;

    STM32H7_WS281X_SetPixel(index, c);
}

static void Area1_STM32H7_Show(void *user)
{
    (void)user;
    (void)STM32H7_WS281X_Show();
}

static uint32_t Area1_STM32H7_GetMs(void *user)
{
    (void)user;
    return HAL_GetTick();
}

void Area1Led_STM32H7_Init(void)
{
    Area1Led_Config config;

    STM32H7_WS281X_Init();

    config.led_count = STM32H7_WS281X_GetLedCount();
    config.set_pixel = Area1_STM32H7_SetPixel;
    config.show = Area1_STM32H7_Show;
    config.get_ms = Area1_STM32H7_GetMs;
    config.user = 0;

    Area1Led_Init(&config);
}
