#include "lift_step_down.h"
#include "usart_task.h"
#include <math.h>

extern VisionData_t vision;

const float STEP_DOWN_AUTO_CHASSIS_SPEED_MPS = 0.8f;
const float STEP_DOWN_AUTO_LIFT_SPEED_MPS    = 0.65f;
const float STEP_DOWN_CHASSIS_ACC_SPEED      = 0.8f;
const float STEP_DOWN_LIFT_ACC_SPEED         = 0.4f;
const uint8_t STEP_DOWN_AUTO_STABLE_COUNT    = 10U;

LiftStepDown lift_step_down;

LiftStepDown::LiftStepDown()
{
    resetStepDown();
}

float LiftStepDown::speed_limit(float speed, float max)
{
    if (speed > max) {
        speed = max;
    }
    if (speed < -max) {
        speed = -max;
    }
    return speed;
}

float LiftStepDown::trapezoid_speed(float error, float acc, float max)
{
    if (error == 0.0f || acc <= 0.0f || max <= 0.0f) {
        return 0.0f;
    }

    float speed = sqrtf(2.0f * fabsf(error) * acc);
    if (error < 0.0f) {
        speed = -speed;
    }

    return speed_limit(speed, max);
}

uint8_t LiftStepDown::step_down_stable_confirm(uint8_t condition)
{
    if (condition == 0U) {
        step_down_stable_count_ = 0U;
        return 0U;
    }

    if (step_down_stable_count_ < STEP_DOWN_AUTO_STABLE_COUNT) {
        step_down_stable_count_++;
    }

    return (step_down_stable_count_ >= STEP_DOWN_AUTO_STABLE_COUNT) ? 1U : 0U;
}

void LiftStepDown::resetStepDown(void)
{
    step_down_started_             = 0U;
    step_down_state_               = STEP_DOWN_IDLE;
    lift_switch_target_            = 0U;
    lift_linear_speed_target_      = 0.0f;
    chassis_vx_target_             = 0.0f;
    chassis_vy_target_             = 0.0f;
    step_down_stable_count_        = 0U;
    step_down_block_num_           = 0;
    step_down_radar_x_ref_prepare_ = 0.0f;
    step_down_radar_x_ref_descend_ = 0.0f;
    step_down_radar_x_ref_finish_  = 0.0f;
    step_down_radar_y_ref_finish_  = 0.0f;
}

void LiftStepDown::startStepDown(void)
{
    step_down_started_ = 1U;
}

void LiftStepDown::stopStepDown(void)
{
    resetStepDown();
}

uint8_t LiftStepDown::isStepDownFinished(void) const
{
    return (step_down_state_ == STEP_DOWN_FINISHED) ? 1U : 0U;
}

void LiftStepDown::update(void)
{
    if (step_down_started_ == 0U) {
        return;
    }

    if (step_down_state_ == STEP_DOWN_IDLE) {
        step_down_state_ = STEP_DOWN_MOVE_TO_PREPARE;
    }

    switch (step_down_state_) {
        case STEP_DOWN_MOVE_TO_PREPARE: {
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;
            chassis_vy_target_        = 0.0f;

            float x_err        = step_down_radar_x_ref_prepare_ - vision.x_diff;
            chassis_vx_target_ = trapezoid_speed(x_err,
                                                  STEP_DOWN_CHASSIS_ACC_SPEED,
                                                  STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);

            if (step_down_stable_confirm((fabsf(x_err) < 0.050f) ? 1U : 0U) != 0U) {
                chassis_vx_target_      = 0.0f;
                step_down_stable_count_ = 0U;
                step_down_state_        = STEP_DOWN_DESCEND;
            }
            break;
        }

        case STEP_DOWN_DESCEND: {
            lift_switch_target_ = 2U;
            chassis_vx_target_  = 0.0f;
            chassis_vy_target_  = 0.0f;

            float x_err               = step_down_radar_x_ref_descend_ - vision.x_diff;
            lift_linear_speed_target_ = trapezoid_speed(x_err,
                                                         STEP_DOWN_LIFT_ACC_SPEED,
                                                         STEP_DOWN_AUTO_LIFT_SPEED_MPS);

            if (step_down_stable_confirm((fabsf(x_err) < 0.050f) ? 1U : 0U) != 0U) {
                lift_switch_target_       = 1U;
                lift_linear_speed_target_ = 0.0f;
                step_down_stable_count_   = 0U;
                step_down_state_          = STEP_DOWN_MOVE_TO_FINISH;
            }
            break;
        }

        case STEP_DOWN_MOVE_TO_FINISH: {
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;

            float x_err        = step_down_radar_x_ref_finish_ - vision.x_diff;
            float y_err        = step_down_radar_y_ref_finish_ - vision.y_diff;
            chassis_vx_target_ = trapezoid_speed(x_err,
                                                  STEP_DOWN_CHASSIS_ACC_SPEED,
                                                  STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);
            chassis_vy_target_ = trapezoid_speed(y_err,
                                                  STEP_DOWN_CHASSIS_ACC_SPEED,
                                                  STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);

            if (step_down_stable_confirm((fabsf(x_err) < 0.050f &&
                                          fabsf(y_err) < 0.050f) ? 1U : 0U) != 0U) {
                chassis_vx_target_      = 0.0f;
                chassis_vy_target_      = 0.0f;
                step_down_stable_count_ = 0U;
                step_down_state_        = STEP_DOWN_FINISHED;
            }
            break;
        }

        case STEP_DOWN_FINISHED:
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;
            chassis_vx_target_        = 0.0f;
            chassis_vy_target_        = 0.0f;
            break;

        default:
            resetStepDown();
            break;
    }
}

uint8_t LiftStepDown::getLiftSwitch(uint8_t manual_switch) const
{
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_switch;
    }

    return lift_switch_target_;
}

float LiftStepDown::getLiftLinearSpeedTarget(float manual_target) const
{
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_target;
    }

    return lift_linear_speed_target_;
}

float LiftStepDown::getChassisVxTarget(float manual_target) const
{
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_target;
    }

    return chassis_vx_target_;
}

float LiftStepDown::getChassisVyTarget(float manual_target) const
{
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_target;
    }

    return chassis_vy_target_;
}

void LiftStepDown::setStepDownRadarTarget(float x_ref_prepare,
                                          float x_ref_descend,
                                          float x_ref_finish,
                                          float y_ref_finish)
{
    step_down_radar_x_ref_prepare_ = x_ref_prepare;
    step_down_radar_x_ref_descend_ = x_ref_descend;
    step_down_radar_x_ref_finish_  = x_ref_finish;
    step_down_radar_y_ref_finish_  = y_ref_finish;
}

void LiftStepDown::setStepDownBlockNum(int num)
{
    step_down_block_num_ = num;
}
