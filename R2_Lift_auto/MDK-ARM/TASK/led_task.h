#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdint.h>

typedef enum
{
    LED_TASK_MODE_POWER_ON_RED = 0,
    LED_TASK_MODE_RED_BREATH,
    LED_TASK_MODE_WEAPON_WHITE,
    LED_TASK_MODE_DOCKING_WHITE_BREATH,
    LED_TASK_MODE_DOCKING_DONE_WHITE,
    LED_TASK_MODE_ALL_WHITE,
    LED_TASK_MODE_WHITE_FLASH,
    LED_TASK_MODE_RED_SEGMENT_ON,
    LED_TASK_MODE_RED_SEGMENT_FLASH,
    LED_TASK_MODE_RELOCATION_BLUE_FLASH,
    LED_TASK_MODE_RELOCATION_BLUE_ON,
    LED_TASK_MODE_ALL_GREEN,
    LED_TASK_MODE_GREEN_SEGMENT_ON,
    LED_TASK_MODE_COLORFUL_FLOW,
    LED_TASK_MODE_COLORFUL_SOLID,
    LED_TASK_MODE_ALL_OFF
} LedTask_Mode;

typedef enum
{
    LED_TASK_SEG_ALL = 0,
    LED_TASK_SEG_FRONT,
    LED_TASK_SEG_MIDDLE,
    LED_TASK_SEG_BACK
} LedTask_Segment;

#ifdef __cplusplus
extern "C" {
#endif

void led_task(void *argument);
void LedTask_SetMode(LedTask_Mode mode, LedTask_Segment segment);

/* Keil Watch 手动测试入口：
 * 0 自动跟随 FTM 逻辑
 * 1 全部白色常亮
 * 2/3/4 第一/二/三组红色常亮
 * 5/6/7 第一/二/三组红色闪烁
 * 8/9/10 第一/二/三组绿色常亮
 * 11 蓝色闪烁，12 蓝色常亮，13 全灭
 * 14 整条同色彩虹渐变
 * 15 全部白色闪烁，16 全部红色常亮
 * 17 全部绿色常亮
 */
extern volatile uint8_t LED_state;

#ifdef __cplusplus
}
#endif

#endif
