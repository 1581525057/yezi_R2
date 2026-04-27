#ifndef CHASSIS_AUTO_SOURCE_H
#define CHASSIS_AUTO_SOURCE_H

#include <stdint.h>

typedef enum
{
    CHASSIS_AUTO_NONE = 0,
    CHASSIS_AUTO_MEILING,
    CHASSIS_AUTO_WUQIQU,
    CHASSIS_AUTO_CONFLICT
} ChassisAutoSource;

static inline ChassisAutoSource ChassisAuto_SelectSource(uint8_t wuqiqu_active, uint8_t meiling_area_active)
{
    if (wuqiqu_active != 0U && meiling_area_active != 0U)
    {
        return CHASSIS_AUTO_CONFLICT;
    }
    if (wuqiqu_active != 0U)
    {
        return CHASSIS_AUTO_WUQIQU;
    }
    if (meiling_area_active != 0U)
    {
        return CHASSIS_AUTO_MEILING;
    }
    return CHASSIS_AUTO_NONE;
}

#endif
