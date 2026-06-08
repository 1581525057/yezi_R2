#include "FTMLiftAction.h"
#include "lift_class.h"
#include <math.h>

namespace
{
uint32_t g_last_command_seq = 0U;
uint8_t g_ftm_lift_takeover = 0U;

float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}
}

void FTMLiftAction_Reset(void)
{
    g_last_command_seq = lift_calulate.command_seq;
    g_ftm_lift_takeover = 0U;
}

void FTMLiftAction_MoveTo(float target_height_mm, float move_time_s)
{
    g_ftm_lift_takeover = 1U;
    lift_debug.height_target = target_height_mm;
    lift_debug.flag = 1.0f;
    (void)move_time_s;
    g_last_command_seq = lift_calulate.command_seq;
}

uint8_t FTMLiftAction_IsFinished(float tolerance_mm)
{
    const float target = lift_debug.height_target;

    if (lift_calulate.command_seq == g_last_command_seq)
    {
        return 0U;
    }

    if (lift_calulate.finished != 0U)
    {
        return 1U;
    }

    if (AbsFloat(lift_class.left.height - target) <= tolerance_mm &&
        AbsFloat(lift_class.right.height - target) <= tolerance_mm)
    {
        return 1U;
    }

    return 0U;
}

void FTMLiftAction_SetTakeover(uint8_t enable)
{
    g_ftm_lift_takeover = (enable != 0U) ? 1U : 0U;
}

uint8_t FTMLiftAction_IsTakeover(void)
{
    return g_ftm_lift_takeover;
}
