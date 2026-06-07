#include "lift_step_down.h"
#include "usart_task.h"
#include <assert.h>
#include <math.h>

extern float STEP_DOWN_PREPARE_DISTANCE_L;
extern float STEP_DOWN_DESCEND_DISTANCE_D;
extern uint8_t STEP_DOWN_AUTO_STABLE_COUNT;

VisionData_t vision = {};

static void reset_test_config(void)
{
    STEP_DOWN_AUTO_STABLE_COUNT  = 1U;
    STEP_DOWN_PREPARE_DISTANCE_L = 0.83f;
    STEP_DOWN_DESCEND_DISTANCE_D = 0.59f;
    vision.exec                  = 0;
    vision.x_diff                = 0.0f;
    vision.y_diff                = 0.0f;
    vision.angle_x               = 0.0f;
    vision.B                     = 0;
}

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

static void finish_turn_180_step_down(LiftStepDown &step, float x_finish, float y_finish)
{
    step.setStepDownRadarTarget(x_finish, y_finish, x_finish, y_finish, 0U, 0U, 1U);
    vision.x_diff = x_finish;
    vision.y_diff = y_finish;
    step.startStepDown();

    step.update();
    vision.x_diff = x_finish + STEP_DOWN_PREPARE_DISTANCE_L;
    step.update();
    step.update();
    vision.x_diff = x_finish + STEP_DOWN_PREPARE_DISTANCE_L + STEP_DOWN_DESCEND_DISTANCE_D;
    step.update();
    vision.x_diff = x_finish;
    vision.y_diff = y_finish;
    step.update();

    assert(step.isStepDownFinished() != 0U);
    step.stopStepDown();
}

static void test_prepare_base_uses_latest_config(void)
{
    reset_test_config();
    LiftStepDown step;
    step.setStepDownRadarTarget(0.0f, 0.0f, 0.0f, 0.0f, 0U, 0U, 1U);
    step.setStepDownRadarTarget(2.0f, 2.0f, 2.0f, 2.0f, 0U, 0U, 1U);
    vision.x_diff = 2.0f;
    vision.y_diff = 2.0f;
    step.startStepDown();

    step.update();

    assert(step.getChassisVxTarget(0.0f) > 0.0f);
    assert_near(step.getChassisVyTarget(0.0f), 0.0f);
}

static void test_prepare_base_uses_current_config_not_previous_finish(void)
{
    reset_test_config();
    LiftStepDown step;
    finish_turn_180_step_down(step, 0.0f, 0.0f);
    step.setStepDownRadarTarget(2.0f, 2.0f, 5.0f, 5.0f, 0U, 0U, 1U);
    vision.x_diff = 2.0f;
    vision.y_diff = 2.0f;
    step.startStepDown();

    step.update();

    assert(step.getChassisVxTarget(0.0f) > 0.0f);
    assert_near(step.getChassisVyTarget(0.0f), 0.0f);
}

static void test_turn_180_prepare_and_descend(void)
{
    reset_test_config();
    LiftStepDown step;
    finish_turn_180_step_down(step, 0.0f, 0.0f);
    step.setStepDownRadarTarget(0.0f, 0.0f, 2.0f, 2.0f, 0U, 0U, 1U);
    step.startStepDown();

    step.update();
    assert(step.getChassisVxTarget(0.0f) > 0.0f);
    assert_near(step.getChassisVyTarget(0.0f), 0.0f);

    vision.x_diff = STEP_DOWN_PREPARE_DISTANCE_L;
    step.update();
    step.update();
    assert(step.getLiftLinearSpeedTarget(0.0f) < 0.0f);
}

static void test_left_90_prepare_and_descend(void)
{
    reset_test_config();
    LiftStepDown step;
    finish_turn_180_step_down(step, 0.0f, 0.0f);
    step.setStepDownRadarTarget(0.0f, 0.0f, 2.0f, 2.0f, 1U, 0U, 0U);
    step.startStepDown();

    step.update();
    assert_near(step.getChassisVxTarget(0.0f), 0.0f);
    assert(step.getChassisVyTarget(0.0f) < 0.0f);

    vision.y_diff = -STEP_DOWN_PREPARE_DISTANCE_L;
    step.update();
    step.update();
    assert(step.getLiftLinearSpeedTarget(0.0f) < 0.0f);
}

static void test_right_90_prepare_and_descend(void)
{
    reset_test_config();
    LiftStepDown step;
    finish_turn_180_step_down(step, 0.0f, 0.0f);
    step.setStepDownRadarTarget(0.0f, 0.0f, 2.0f, 2.0f, 0U, 1U, 0U);
    step.startStepDown();

    step.update();
    assert_near(step.getChassisVxTarget(0.0f), 0.0f);
    assert(step.getChassisVyTarget(0.0f) > 0.0f);

    vision.y_diff = STEP_DOWN_PREPARE_DISTANCE_L;
    step.update();
    step.update();
    assert(step.getLiftLinearSpeedTarget(0.0f) < 0.0f);
}

int main(void)
{
    test_prepare_base_uses_latest_config();
    test_prepare_base_uses_current_config_not_previous_finish();
    test_turn_180_prepare_and_descend();
    test_left_90_prepare_and_descend();
    test_right_90_prepare_and_descend();
    return 0;
}
