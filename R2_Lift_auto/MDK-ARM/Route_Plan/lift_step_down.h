#ifndef LIFT_STEP_DOWN_H
#define LIFT_STEP_DOWN_H

#include "main.h"
#include <stdint.h>

class LiftStepDown
{
public:
    LiftStepDown();

    void startStepDown(void);
    void stopStepDown(void);
    uint8_t isStepDownFinished(void) const;
    void update(void);

    uint8_t getLiftSwitch(uint8_t manual_switch) const;
    float getLiftLinearSpeedTarget(float manual_target) const;
    float getChassisVxTarget(float manual_target) const;
    float getChassisVyTarget(float manual_target) const;

    void setStepDownRadarTarget(float x_ref_prepare,
                                float x_ref_descend,
                                float x_ref_finish,
                                float y_ref_finish);
    void setStepDownBlockNum(int num);

private:
    enum StepDownState {
        STEP_DOWN_IDLE = 0,
        STEP_DOWN_MOVE_TO_PREPARE,
        STEP_DOWN_DESCEND,
        STEP_DOWN_MOVE_TO_FINISH,
        STEP_DOWN_FINISHED
    };

    void resetStepDown(void);
    float speed_limit(float speed, float max);
    float trapezoid_speed(float error, float acc, float max);
    uint8_t step_down_stable_confirm(uint8_t condition);

    uint8_t step_down_started_;
    StepDownState step_down_state_;

    uint8_t lift_switch_target_;
    float lift_linear_speed_target_;
    float chassis_vx_target_;
    float chassis_vy_target_;

    uint8_t step_down_stable_count_;
    int step_down_block_num_;
    float step_down_radar_x_ref_prepare_;
    float step_down_radar_x_ref_descend_;
    float step_down_radar_x_ref_finish_;
    float step_down_radar_y_ref_finish_;
};

extern LiftStepDown lift_step_down;

#endif
