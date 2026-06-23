#ifndef ROUTE_TEST_STUB_LIFT_STEP_UP_H
#define ROUTE_TEST_STUB_LIFT_STEP_UP_H

#include <stdint.h>

class LiftAuto
{
public:
    int8_t climb_direction;
    uint8_t return_middle;

    LiftAuto()
        : climb_direction(0)
        , return_middle(1U)
    {
    }

    void setStepUpBlockNum(int) {}
    void setStepUpRadarClimbDirection(int8_t direction) { climb_direction = direction; }
    void setStepUpRadarTarget(float, float) {}
    void setStepUpHeightMode(uint16_t) {}
    void setStepUpReturnMiddle(uint8_t enable) { return_middle = enable; }
    void startStepUp(void) {}
    uint8_t isStepUpFinished(void) const { return 0U; }
    void stopStepUp(void) {}
    void update(void) {}
};

extern LiftAuto lift_auto;

#endif
