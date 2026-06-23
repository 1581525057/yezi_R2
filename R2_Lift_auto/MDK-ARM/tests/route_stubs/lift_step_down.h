#ifndef ROUTE_TEST_STUB_LIFT_STEP_DOWN_H
#define ROUTE_TEST_STUB_LIFT_STEP_DOWN_H

#include <stdint.h>

class LiftStepDown
{
public:
    float prepare_base_x;
    float prepare_base_y;
    float finish_x;
    float finish_y;
    int block_num;
    uint16_t height_mode_mm;
    uint8_t turn_left_90;
    uint8_t turn_right_90;
    uint8_t turn_180;

    LiftStepDown()
        : prepare_base_x(0.0f),
          prepare_base_y(0.0f),
          finish_x(0.0f),
          finish_y(0.0f),
          block_num(0),
          height_mode_mm(0U),
          turn_left_90(0U),
          turn_right_90(0U),
          turn_180(0U)
    {
    }

    void setStepDownBlockNum(int num) { block_num = num; }
    void setStepDownHeightMode(uint16_t height_mm) { height_mode_mm = height_mm; }

    void setStepDownRadarTarget(float x_ref_prepare_base,
                                float y_ref_prepare_base,
                                float x_ref_finish,
                                float y_ref_finish,
                                uint8_t turn_left,
                                uint8_t turn_right,
                                uint8_t turn_back)
    {
        prepare_base_x = x_ref_prepare_base;
        prepare_base_y = y_ref_prepare_base;
        finish_x       = x_ref_finish;
        finish_y       = y_ref_finish;
        turn_left_90   = turn_left;
        turn_right_90  = turn_right;
        turn_180       = turn_back;
    }

    void startStepDown(void) {}
    uint8_t isStepDownFinished(void) const { return 0U; }
    void stopStepDown(void) {}
    void update(void) {}
};

extern LiftStepDown lift_step_down;

#endif
