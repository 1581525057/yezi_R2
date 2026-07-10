#include "led_task.h"
#include "area1_led_stm32h7.h"
#include "area1_led_portable.h"
#include "usart_task.h"
#include "cmsis_os.h"

namespace
{
    constexpr uint32_t kLedTaskPeriodMs = 10U; // 动态灯效刷新周期。
    constexpr uint8_t kLedStateAuto = 0U;

    volatile LedTask_Mode g_requested_mode = LED_TASK_MODE_POWER_ON_RED;
    volatile LedTask_Segment g_requested_segment = LED_TASK_SEG_ALL;

    LedTask_Mode g_rendered_mode = LED_TASK_MODE_POWER_ON_RED;
    LedTask_Segment g_rendered_segment = LED_TASK_SEG_ALL;
    uint8_t g_static_rendered = 0U;

    Area1Led_Segment ToAreaSegment(LedTask_Segment segment)
    {
        switch (segment)
        {
        case LED_TASK_SEG_FRONT:
            return AREA1_LED_SEG_FRONT;
        case LED_TASK_SEG_MIDDLE:
            return AREA1_LED_SEG_MIDDLE;
        case LED_TASK_SEG_BACK:
            return AREA1_LED_SEG_BACK;
        case LED_TASK_SEG_ALL:
        default:
            return AREA1_LED_SEG_ALL;
        }
    }

    void ApplyLedStateOverride(LedTask_Mode *mode, LedTask_Segment *segment)
    {
        if ((mode == 0) || (segment == 0) || (LED_state == kLedStateAuto))
        {
            return;
        }

        switch (LED_state)
        {
        case 1U:
            *mode = LED_TASK_MODE_ALL_WHITE;
            *segment = LED_TASK_SEG_ALL;
            break;
        case 2U:
            *mode = LED_TASK_MODE_RED_SEGMENT_ON;
            *segment = LED_TASK_SEG_FRONT;
            break;
        case 3U:
            *mode = LED_TASK_MODE_RED_SEGMENT_ON;
            *segment = LED_TASK_SEG_MIDDLE;
            break;
        case 4U:
            *mode = LED_TASK_MODE_RED_SEGMENT_ON;
            *segment = LED_TASK_SEG_BACK;
            break;
        case 5U:
            *mode = LED_TASK_MODE_RED_SEGMENT_FLASH;
            *segment = LED_TASK_SEG_FRONT;
            break;
        case 6U:
            *mode = LED_TASK_MODE_RED_SEGMENT_FLASH;
            *segment = LED_TASK_SEG_MIDDLE;
            break;
        case 7U:
            *mode = LED_TASK_MODE_RED_SEGMENT_FLASH;
            *segment = LED_TASK_SEG_BACK;
            break;
        case 8U:
            *mode = LED_TASK_MODE_GREEN_SEGMENT_ON;
            *segment = LED_TASK_SEG_FRONT;
            break;
        case 9U:
            *mode = LED_TASK_MODE_GREEN_SEGMENT_ON;
            *segment = LED_TASK_SEG_MIDDLE;
            break;
        case 10U:
            *mode = LED_TASK_MODE_GREEN_SEGMENT_ON;
            *segment = LED_TASK_SEG_BACK;
            break;
        case 11U:
            *mode = LED_TASK_MODE_RELOCATION_BLUE_FLASH;
            *segment = LED_TASK_SEG_ALL;
            break;
        case 12U:
            *mode = LED_TASK_MODE_RELOCATION_BLUE_ON;
            *segment = LED_TASK_SEG_ALL;
            break;
        case 13U:
            *mode = LED_TASK_MODE_ALL_OFF;
            *segment = LED_TASK_SEG_ALL;
            break;
        case 14U:
            *mode = LED_TASK_MODE_COLORFUL_SOLID;
            *segment = LED_TASK_SEG_ALL;
            break;
        case 15U:
            *mode = LED_TASK_MODE_WHITE_FLASH;
            *segment = LED_TASK_SEG_ALL;
            break;
        case 16U:
            *mode = LED_TASK_MODE_POWER_ON_RED;
            *segment = LED_TASK_SEG_ALL;
            break;
        case 17U:
            *mode = LED_TASK_MODE_ALL_GREEN;
            *segment = LED_TASK_SEG_ALL;
            break;
        default:
            break;
        }
    }

    uint8_t IsDynamicMode(LedTask_Mode mode)
    {
        return ((mode == LED_TASK_MODE_RED_BREATH) ||
                (mode == LED_TASK_MODE_WHITE_FLASH) ||
                (mode == LED_TASK_MODE_RED_SEGMENT_FLASH) ||
                (mode == LED_TASK_MODE_DOCKING_WHITE_BREATH) ||
                (mode == LED_TASK_MODE_RELOCATION_BLUE_FLASH) ||
                (mode == LED_TASK_MODE_COLORFUL_FLOW) ||
                (mode == LED_TASK_MODE_COLORFUL_SOLID))
                   ? 1U
                   : 0U;
    }

    void RenderStaticMode(LedTask_Mode mode, LedTask_Segment segment)
    {
        const Area1Led_Segment area_segment = ToAreaSegment(segment);

        switch (mode)
        {
        case LED_TASK_MODE_POWER_ON_RED:
            Area1Led_PowerOn_AllRed();
            break;
        case LED_TASK_MODE_ALL_WHITE:
            Area1Led_AllWhite();
            break;
        case LED_TASK_MODE_RED_SEGMENT_ON:
            Area1Led_RedSegmentOn(area_segment);
            break;
        case LED_TASK_MODE_WEAPON_WHITE:
            Area1Led_WeaponHead_White(area_segment);
            break;
        case LED_TASK_MODE_DOCKING_DONE_WHITE:
            Area1Led_DockingDone_White(area_segment);
            break;
        case LED_TASK_MODE_RELOCATION_BLUE_ON:
            Area1Led_Relocation_BlueOn();
            break;
        case LED_TASK_MODE_ALL_GREEN:
            Area1Led_AllGreen();
            break;
        case LED_TASK_MODE_GREEN_SEGMENT_ON:
            Area1Led_GreenSegmentOn(area_segment);
            break;
        case LED_TASK_MODE_ALL_OFF:
            Area1Led_AllOff();
            break;
        default:
            break;
        }
    }

    void RenderDynamicMode(LedTask_Mode mode, LedTask_Segment segment)
    {
        const Area1Led_Segment area_segment = ToAreaSegment(segment);

        switch (mode)
        {
        case LED_TASK_MODE_RED_BREATH:
            Area1Led_RedBreath();
            break;
        case LED_TASK_MODE_WHITE_FLASH:
            Area1Led_WhiteFlash();
            break;
        case LED_TASK_MODE_RED_SEGMENT_FLASH:
            Area1Led_RedSegmentFlash(area_segment);
            break;
        case LED_TASK_MODE_DOCKING_WHITE_BREATH:
            Area1Led_Docking_WhiteBreath(area_segment);
            break;
        case LED_TASK_MODE_RELOCATION_BLUE_FLASH:
            Area1Led_Relocation_BlueFlash();
            break;
        case LED_TASK_MODE_COLORFUL_FLOW:
            Area1Led_ColorfulFlow();
            break;
        case LED_TASK_MODE_COLORFUL_SOLID:
            Area1Led_ColorfulSolid();
            break;
        default:
            break;
        }
    }

    void ServiceLedRender(void)
    {
        LedTask_Mode mode = g_requested_mode;
        LedTask_Segment segment = g_requested_segment;
        ApplyLedStateOverride(&mode, &segment);

        if ((mode != g_rendered_mode) || (segment != g_rendered_segment))
        {
            g_rendered_mode = mode;
            g_rendered_segment = segment;
            g_static_rendered = 0U;
        }

        if (IsDynamicMode(mode) != 0U)
        {
            RenderDynamicMode(mode, segment);
            return;
        }

        if (g_static_rendered == 0U)
        {
            RenderStaticMode(mode, segment);
            g_static_rendered = 1U;
        }
    }
}

extern "C" volatile uint8_t LED_state = 0U;

extern "C" void LedTask_SetMode(LedTask_Mode mode, LedTask_Segment segment)
{
    g_requested_mode = mode;
    g_requested_segment = segment;
}

extern "C" void led_task(void *argument)
{
    (void)argument;

    Area1Led_STM32H7_Init();
    Area1Led_PowerOn_AllRed();

    for (;;)
    {
        ServiceLedRender();
        osDelay(kLedTaskPeriodMs);
    }
}
