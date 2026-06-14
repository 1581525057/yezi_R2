#ifndef ROUTE_TEST_STUB_MAIN_H
#define ROUTE_TEST_STUB_MAIN_H

#include <stdint.h>

extern uint32_t route_test_tick_ms;

static inline uint32_t HAL_GetTick(void)
{
    return route_test_tick_ms;
}

#endif
