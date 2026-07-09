#ifndef AREA1_LED_PORTABLE_H
#define AREA1_LED_PORTABLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} Area1Led_Color;

typedef enum
{
    AREA1_LED_SEG_ALL = 0,
    AREA1_LED_SEG_FRONT = 1,
    AREA1_LED_SEG_MIDDLE = 2,
    AREA1_LED_SEG_BACK = 3
} Area1Led_Segment;

typedef void (*Area1Led_SetPixelFn)(void *user, uint16_t index, Area1Led_Color color);
typedef void (*Area1Led_ShowFn)(void *user);
typedef uint32_t (*Area1Led_GetMsFn)(void *user);

typedef struct
{
    uint16_t led_count;
    Area1Led_SetPixelFn set_pixel;
    Area1Led_ShowFn show;
    Area1Led_GetMsFn get_ms;
    void *user;
} Area1Led_Config;

/* 所有对外函数都可以多次调用。
 * Area1Led_Init() 重复调用会重新绑定配置并复位动态效果状态。
 * 常亮类函数重复调用会刷新同一效果，动态类函数重复调用会推进动画。
 */
void Area1Led_Init(const Area1Led_Config *config);

void Area1Led_PowerOn_AllRed(void);
void Area1Led_AllWhite(void);
void Area1Led_WhiteFlash(void);
void Area1Led_RedSegmentOn(Area1Led_Segment segment);
void Area1Led_RedSegmentFlash(Area1Led_Segment segment);

/* 红色呼吸灯：一区整条灯带红色明暗呼吸，需要在主循环中反复调用。 */
void Area1Led_RedBreath(void);

void Area1Led_WeaponHead_White(Area1Led_Segment segment);
void Area1Led_WeaponHead_WhiteFront(void);
void Area1Led_WeaponHead_WhiteMiddle(void);
void Area1Led_WeaponHead_WhiteBack(void);

void Area1Led_Docking_WhiteBreath(Area1Led_Segment segment);
void Area1Led_Docking_WhiteBreathFront(void);
void Area1Led_Docking_WhiteBreathMiddle(void);
void Area1Led_Docking_WhiteBreathBack(void);

void Area1Led_DockingDone_White(Area1Led_Segment segment);
void Area1Led_DockingDone_WhiteFront(void);
void Area1Led_DockingDone_WhiteMiddle(void);
void Area1Led_DockingDone_WhiteBack(void);

void Area1Led_Relocation_BlueFlash(void);
void Area1Led_Relocation_BlueOn(void);

/* 纯绿：一区整条灯带绿色常亮，可多次调用。 */
void Area1Led_AllGreen(void);
void Area1Led_GreenSegmentOn(Area1Led_Segment segment);

/* 炫彩流水：一区整条灯带按彩虹色流动，需要在主循环中反复调用。 */
void Area1Led_ColorfulFlow(void);
void Area1Led_ColorfulSolid(void);

void Area1Led_AllOff(void);

#ifdef __cplusplus
}
#endif

#endif
