#ifndef LIFT_AUTO_H
#define LIFT_AUTO_H

#include "main.h"
#include <stdint.h>

class LiftAuto
{
public:
    LiftAuto();

    void update(void);
    uint8_t getLiftSwitch(uint8_t manual_switch) const;
    float getLiftLinearSpeedTarget(float manual_target) const;
    float getChassisVyTarget(float manual_target) const;

private:
    void reset(void);

    enum StepState {
        STEP_IDLE = 0,
        STEP_APPROACH_Y,
        STEP_WAIT_NEW_HEIGHT,
        STEP_CLIMB_FORWARD,
        STEP_FINISHED,
    };

    StepState state_;
    uint8_t lift_switch_target_;
    float lift_linear_speed_target_;
    uint8_t chassis_vy_override_;
    float chassis_vy_target_;
    uint8_t stable_count_;
    uint32_t last_laser_mm_;
};

extern LiftAuto lift_auto;

#endif
