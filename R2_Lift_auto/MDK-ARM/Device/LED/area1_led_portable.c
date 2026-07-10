#include "area1_led_portable.h"

#define AREA1_BREATH_INTERVAL_MS 10U
#define AREA1_BREATH_STEP        5
#define AREA1_FLASH_INTERVAL_MS  300U
#define AREA1_FLOW_INTERVAL_MS   80U
#define AREA1_FLOW_STEP          8U
#define AREA1_FLOW_HUE_SPACING   26U

static const Area1Led_Color AREA1_COLOR_RED   = {255, 0, 0};
static const Area1Led_Color AREA1_COLOR_WHITE = {255, 255, 255};
static const Area1Led_Color AREA1_COLOR_BLUE  = {0, 0, 255};
static const Area1Led_Color AREA1_COLOR_GREEN = {0, 255, 0};
static const Area1Led_Color AREA1_COLOR_BLACK = {0, 0, 0};

typedef struct
{
    uint32_t last_tick;
    int16_t brightness;
    int8_t direction;
} Area1_BreathState;

static Area1Led_Config g_area1;
static Area1_BreathState g_red_breath;
static Area1_BreathState g_white_breath;
static uint32_t g_red_flash_last_tick;
static uint8_t g_red_flash_on;
static uint32_t g_white_flash_last_tick;
static uint8_t g_white_flash_on;
static uint32_t g_blue_flash_last_tick;
static uint8_t g_blue_flash_on;
static uint32_t g_flow_last_tick;
static uint8_t g_flow_hue_offset;

static Area1_BreathState Area1_NewBreathState(void)
{
    Area1_BreathState state = {0, 0, 1};

    return state;
}

static void Area1_ResetEffectStates(void)
{
    g_red_breath = Area1_NewBreathState();
    g_white_breath = Area1_NewBreathState();
    g_red_flash_last_tick = 0U;
    g_red_flash_on = 0U;
    g_white_flash_last_tick = 0U;
    g_white_flash_on = 0U;
    g_blue_flash_last_tick = 0U;
    g_blue_flash_on = 0U;
    g_flow_last_tick = 0U;
    g_flow_hue_offset = 0U;
}

static uint8_t Area1_IsReady(void)
{
    return (g_area1.led_count > 0U) &&
           (g_area1.set_pixel != 0) &&
           (g_area1.show != 0) &&
           (g_area1.get_ms != 0);
}

void Area1Led_Init(const Area1Led_Config *config)
{
    Area1_ResetEffectStates();

    if (config == 0)
    {
        g_area1.led_count = 0;
        g_area1.set_pixel = 0;
        g_area1.show = 0;
        g_area1.get_ms = 0;
        g_area1.user = 0;
        return;
    }

    g_area1 = *config;
}

static void Area1_FillRange(uint16_t start, uint16_t end, Area1Led_Color color)
{
    uint16_t i;

    for (i = start; i < end; i++)
    {
        g_area1.set_pixel(g_area1.user, i, color);
    }
}

static void Area1_RenderAll(Area1Led_Color color)
{
    if (!Area1_IsReady())
    {
        return;
    }

    Area1_FillRange(0U, g_area1.led_count, color);
    g_area1.show(g_area1.user);
}

static uint8_t Area1_GetSegmentRange(Area1Led_Segment segment,
                                     uint16_t *start,
                                     uint16_t *end)
{
    if (!Area1_IsReady() || start == 0 || end == 0)
    {
        return 0;
    }

    if (segment == AREA1_LED_SEG_ALL)
    {
        *start = 0;
        *end = g_area1.led_count;
        return 1;
    }

    if (segment != AREA1_LED_SEG_FRONT &&
        segment != AREA1_LED_SEG_MIDDLE &&
        segment != AREA1_LED_SEG_BACK)
    {
        return 0;
    }

    *start = (uint16_t)(((uint32_t)(segment - 1) * g_area1.led_count) / 3U);
    *end = (uint16_t)(((uint32_t)segment * g_area1.led_count) / 3U);

    if (segment == AREA1_LED_SEG_BACK)
    {
        *end = g_area1.led_count;
    }

    return 1;
}

static void Area1_RenderSegmentOnly(Area1Led_Segment segment, Area1Led_Color color)
{
    uint16_t start;
    uint16_t end;

    if (!Area1_GetSegmentRange(segment, &start, &end))
    {
        return;
    }

    Area1_FillRange(0U, g_area1.led_count, AREA1_COLOR_BLACK);
    Area1_FillRange(start, end, color);
    g_area1.show(g_area1.user);
}

static Area1Led_Color Area1_ColorWheel(uint8_t hue)
{
    Area1Led_Color color;

    if (hue < 85U)
    {
        color.r = (uint8_t)(255U - (hue * 3U));
        color.g = (uint8_t)(hue * 3U);
        color.b = 0U;
    }
    else if (hue < 170U)
    {
        hue = (uint8_t)(hue - 85U);
        color.r = 0U;
        color.g = (uint8_t)(255U - (hue * 3U));
        color.b = (uint8_t)(hue * 3U);
    }
    else
    {
        hue = (uint8_t)(hue - 170U);
        color.r = (uint8_t)(hue * 3U);
        color.g = 0U;
        color.b = (uint8_t)(255U - (hue * 3U));
    }

    return color;
}

static uint8_t Area1_UpdateBreath(Area1_BreathState *state, uint8_t *brightness)
{
    uint32_t now;

    if (!Area1_IsReady() || state == 0 || brightness == 0)
    {
        return 0;
    }

    now = g_area1.get_ms(g_area1.user);

    if ((state->last_tick != 0U) && ((now - state->last_tick) < AREA1_BREATH_INTERVAL_MS))
    {
        return 0;
    }

    state->last_tick = now;
    state->brightness += state->direction * AREA1_BREATH_STEP;

    if (state->brightness >= 255)
    {
        state->brightness = 255;
        state->direction = -1;
    }
    else if (state->brightness <= 0)
    {
        state->brightness = 0;
        state->direction = 1;
    }

    *brightness = (uint8_t)state->brightness;
    return 1;
}

static Area1Led_Color Area1_ScaleColor(Area1Led_Color color, uint8_t brightness)
{
    color.r = (uint8_t)(((uint16_t)color.r * brightness) / 255U);
    color.g = (uint8_t)(((uint16_t)color.g * brightness) / 255U);
    color.b = (uint8_t)(((uint16_t)color.b * brightness) / 255U);

    return color;
}

void Area1Led_PowerOn_AllRed(void)
{
    Area1_RenderAll(AREA1_COLOR_RED);
}

void Area1Led_AllWhite(void)
{
    Area1_RenderAll(AREA1_COLOR_WHITE);
}

void Area1Led_WhiteFlash(void)
{
    uint32_t now;

    if (!Area1_IsReady())
    {
        return;
    }

    now = g_area1.get_ms(g_area1.user);

    if ((g_white_flash_last_tick != 0U) &&
        ((now - g_white_flash_last_tick) < AREA1_FLASH_INTERVAL_MS))
    {
        return;
    }

    g_white_flash_last_tick = now;
    g_white_flash_on = !g_white_flash_on;

    if (g_white_flash_on)
    {
        Area1_RenderAll(AREA1_COLOR_WHITE);
    }
    else
    {
        Area1_RenderAll(AREA1_COLOR_BLACK);
    }
}

void Area1Led_RedSegmentOn(Area1Led_Segment segment)
{
    Area1_RenderSegmentOnly(segment, AREA1_COLOR_RED);
}

void Area1Led_RedSegmentFlash(Area1Led_Segment segment)
{
    uint32_t now;

    if (!Area1_IsReady())
    {
        return;
    }

    now = g_area1.get_ms(g_area1.user);

    if ((g_red_flash_last_tick != 0U) &&
        ((now - g_red_flash_last_tick) < AREA1_FLASH_INTERVAL_MS))
    {
        return;
    }

    g_red_flash_last_tick = now;
    g_red_flash_on = !g_red_flash_on;

    if (g_red_flash_on)
    {
        Area1_RenderSegmentOnly(segment, AREA1_COLOR_RED);
    }
    else
    {
        Area1_RenderAll(AREA1_COLOR_BLACK);
    }
}

void Area1Led_RedBreath(void)
{
    uint8_t brightness;

    if (!Area1_UpdateBreath(&g_red_breath, &brightness))
    {
        return;
    }

    /* 红色呼吸：按当前控制点配置整条同步明暗变化。 */
    Area1_RenderAll(Area1_ScaleColor(AREA1_COLOR_RED, brightness));
}

void Area1Led_WeaponHead_White(Area1Led_Segment segment)
{
    Area1_RenderSegmentOnly(segment, AREA1_COLOR_WHITE);
}

void Area1Led_WeaponHead_WhiteFront(void)
{
    Area1Led_WeaponHead_White(AREA1_LED_SEG_FRONT);
}

void Area1Led_WeaponHead_WhiteMiddle(void)
{
    Area1Led_WeaponHead_White(AREA1_LED_SEG_MIDDLE);
}

void Area1Led_WeaponHead_WhiteBack(void)
{
    Area1Led_WeaponHead_White(AREA1_LED_SEG_BACK);
}

void Area1Led_Docking_WhiteBreath(Area1Led_Segment segment)
{
    uint8_t brightness;

    if (!Area1_UpdateBreath(&g_white_breath, &brightness))
    {
        return;
    }

    Area1_RenderSegmentOnly(segment, Area1_ScaleColor(AREA1_COLOR_WHITE, brightness));
}

void Area1Led_Docking_WhiteBreathFront(void)
{
    Area1Led_Docking_WhiteBreath(AREA1_LED_SEG_FRONT);
}

void Area1Led_Docking_WhiteBreathMiddle(void)
{
    Area1Led_Docking_WhiteBreath(AREA1_LED_SEG_MIDDLE);
}

void Area1Led_Docking_WhiteBreathBack(void)
{
    Area1Led_Docking_WhiteBreath(AREA1_LED_SEG_BACK);
}

void Area1Led_DockingDone_White(Area1Led_Segment segment)
{
    Area1_RenderSegmentOnly(segment, AREA1_COLOR_WHITE);
}

void Area1Led_DockingDone_WhiteFront(void)
{
    Area1Led_DockingDone_White(AREA1_LED_SEG_FRONT);
}

void Area1Led_DockingDone_WhiteMiddle(void)
{
    Area1Led_DockingDone_White(AREA1_LED_SEG_MIDDLE);
}

void Area1Led_DockingDone_WhiteBack(void)
{
    Area1Led_DockingDone_White(AREA1_LED_SEG_BACK);
}

void Area1Led_Relocation_BlueFlash(void)
{
    uint32_t now;

    if (!Area1_IsReady())
    {
        return;
    }

    now = g_area1.get_ms(g_area1.user);

    if ((g_blue_flash_last_tick != 0U) &&
        ((now - g_blue_flash_last_tick) < AREA1_FLASH_INTERVAL_MS))
    {
        return;
    }

    g_blue_flash_last_tick = now;
    g_blue_flash_on = !g_blue_flash_on;

    if (g_blue_flash_on)
    {
        Area1_RenderAll(AREA1_COLOR_BLUE);
    }
    else
    {
        Area1_RenderAll(AREA1_COLOR_BLACK);
    }
}

void Area1Led_Relocation_BlueOn(void)
{
    Area1_RenderAll(AREA1_COLOR_BLUE);
}

void Area1Led_AllGreen(void)
{
    /* 纯绿常亮：按当前控制点配置整条一起变绿。 */
    Area1_RenderAll(AREA1_COLOR_GREEN);
}

void Area1Led_GreenSegmentOn(Area1Led_Segment segment)
{
    Area1_RenderSegmentOnly(segment, AREA1_COLOR_GREEN);
}

void Area1Led_ColorfulSolid(void)
{
    uint32_t now;

    if (!Area1_IsReady())
    {
        return;
    }

    now = g_area1.get_ms(g_area1.user);

    if ((g_flow_last_tick != 0U) &&
        ((now - g_flow_last_tick) < AREA1_FLOW_INTERVAL_MS))
    {
        return;
    }

    g_flow_last_tick = now;
    g_flow_hue_offset = (uint8_t)(g_flow_hue_offset + AREA1_FLOW_STEP);

    /* 同色彩虹渐变：所有控制点保持同一颜色，只随时间整体变色。 */
    Area1_RenderAll(Area1_ColorWheel(g_flow_hue_offset));
}

void Area1Led_ColorfulFlow(void)
{
    uint32_t now;
    uint16_t i;
    uint8_t hue;

    if (!Area1_IsReady())
    {
        return;
    }

    now = g_area1.get_ms(g_area1.user);

    if ((g_flow_last_tick != 0U) &&
        ((now - g_flow_last_tick) < AREA1_FLOW_INTERVAL_MS))
    {
        return;
    }

    g_flow_last_tick = now;
    g_flow_hue_offset = (uint8_t)(g_flow_hue_offset + AREA1_FLOW_STEP);

    /* 炫彩流水：每个控制点取不同色相，整体色相偏移形成流动效果。 */
    for (i = 0; i < g_area1.led_count; i++)
    {
        hue = (uint8_t)(g_flow_hue_offset + (uint8_t)(i * AREA1_FLOW_HUE_SPACING));
        g_area1.set_pixel(g_area1.user, i, Area1_ColorWheel(hue));
    }

    g_area1.show(g_area1.user);
}

void Area1Led_AllOff(void)
{
    Area1_RenderAll(AREA1_COLOR_BLACK);
}
