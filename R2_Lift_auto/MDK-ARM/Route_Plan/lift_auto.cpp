#include "lift_auto.h"
#include "DT35.h"
#include "mieling.h"

namespace
{
    static const float LIFT_AUTO_APPROACH_VY_MPS = 0.6f;
    static const float LIFT_AUTO_CLIMB_SPEED_MPS = 0.32f;
    static const uint32_t LIFT_AUTO_PREPARE_MM = 45U;
    static const uint32_t LIFT_AUTO_NEW_HEIGHT_MM = 1230U;
    static const uint32_t LIFT_AUTO_FINISH_MM = 630U;
    static const uint32_t LIFT_AUTO_HEIGHT_TOL_MM = 70U;
    static const uint8_t LIFT_AUTO_STABLE_COUNT = 10U;
}

LiftAuto lift_auto;

LiftAuto::LiftAuto()
{
    reset();
}

void LiftAuto::start(void)
{
    flag_start = 1U;
}

void LiftAuto::stop(void)
{
    reset();
}

uint8_t LiftAuto::isFinished(void) const
{
    return (state_ == STEP_FINISHED) ? 1U : 0U;
}

void LiftAuto::reset(void)
{
    flag_start = 0U;
    state_ = STEP_IDLE;
    lift_switch_target_ = 0U;
    lift_linear_speed_target_ = 0.0f;
    chassis_vy_override_ = 0U;
    chassis_vy_target_ = 0.0f;
    stable_count_ = 0U;
    last_laser_mm_ = UINT32_MAX;
}

void LiftAuto::update(void)
{
    const uint32_t laser_mm = (uint32_t)dt35.ch0.distance_mm;
    const uint8_t laser_valid = dt35.ch0.valid;

    if (flag_start == 0U)
    {
        reset();
        return;
    }

    if (state_ == STEP_IDLE)
    {
        state_ = STEP_APPROACH_Y;
    }

    switch (state_)
    {
    case STEP_APPROACH_Y:
        chassis_vy_override_ = 1U;
        lift_switch_target_ = 1U;
        lift_linear_speed_target_ = 0.0f;

        if (laser_valid != 0U && laser_mm > LIFT_AUTO_PREPARE_MM)
        {
            chassis_vy_target_ = 3.5f * (laser_mm - LIFT_AUTO_PREPARE_MM) * 0.001f;
            chassis_vy_target_ = abs_limit(chassis_vy_target_, 0.5f, LIFT_AUTO_APPROACH_VY_MPS);
        }
        else
        {
            chassis_vy_target_ = 0.0f;
        }

        if (laser_valid != 0U)
        {
            last_laser_mm_ = laser_mm;
        }

        if (laser_valid != 0U && laser_mm <= LIFT_AUTO_PREPARE_MM)
        {
            chassis_vy_target_ = 0.0f;
            lift_switch_target_ = 2U;
            stable_count_ = 0U;
            state_ = STEP_WAIT_NEW_HEIGHT;
        }
        break;

    case STEP_WAIT_NEW_HEIGHT:
        chassis_vy_override_ = 1U;
        chassis_vy_target_ = 0.0f;
        lift_switch_target_ = 2U;
        lift_linear_speed_target_ = 0.0f;

        if (laser_valid != 0U &&
            laser_mm >= (LIFT_AUTO_NEW_HEIGHT_MM - LIFT_AUTO_HEIGHT_TOL_MM) &&
            laser_mm <= (LIFT_AUTO_NEW_HEIGHT_MM + LIFT_AUTO_HEIGHT_TOL_MM))
        {
            if (stable_count_ < LIFT_AUTO_STABLE_COUNT)
            {
                stable_count_++;
            }
        }
        else
        {
            stable_count_ = 0U;
        }

        if (stable_count_ >= LIFT_AUTO_STABLE_COUNT)
        {
            state_ = STEP_CLIMB_FORWARD;
        }
        break;

    case STEP_CLIMB_FORWARD:
        chassis_vy_override_ = 1U;
        chassis_vy_target_ = 0.0f;
        lift_switch_target_ = 2U;
        lift_linear_speed_target_ = LIFT_AUTO_CLIMB_SPEED_MPS;

        if (laser_valid != 0U && laser_mm <= LIFT_AUTO_FINISH_MM)
        {
            lift_switch_target_ = 1U;
            lift_linear_speed_target_ = 0.0f;
            state_ = STEP_FINISHED;
        }
        break;

    case STEP_FINISHED:
        chassis_vy_override_ = 0U;
        chassis_vy_target_ = 0.0f;
        lift_switch_target_ = 1U;
        lift_linear_speed_target_ = 0.0f;
        break;

    default:
        reset();
        break;
    }
}

uint8_t LiftAuto::getLiftSwitch(uint8_t manual_switch) const
{
    if (state_ == STEP_IDLE)
    {
        return manual_switch;
    }

    return lift_switch_target_;
}

float LiftAuto::getLiftLinearSpeedTarget(float manual_target) const
{
    if (state_ == STEP_IDLE)
    {
        return manual_target;
    }

    return lift_linear_speed_target_;
}

float LiftAuto::getChassisVyTarget(float manual_target) const
{
    if (chassis_vy_override_ == 0U)
    {
        return manual_target;
    }

    return chassis_vy_target_;
}
