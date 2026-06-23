#include "lift_step_down.h"
#include "lift_class.h"
#include "usart_task.h"
#include <assert.h>
#include <math.h>

extern float STEP_DOWN_PREPARE_DISTANCE_L;
extern float STEP_DOWN_DESCEND_DISTANCE_D;
extern uint8_t STEP_DOWN_AUTO_STABLE_COUNT;

VisionData_t vision = {};
Lift_Class lift_class = {};
LiftHeight_t lift_calulate = {};
debug_lift lift_debug = {};

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
    lift_calulate.command_seq    = 0U;
    lift_calulate.finished       = 1U;
}

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

static void finish_step_down_pre_lift(LiftStepDown &step)
{
    lift_calulate.command_seq++;
    lift_calulate.finished = 1U;
    step.update();
}

static void finish_step_down_post_lift(LiftStepDown &step)
{
    lift_calulate.command_seq++;
    lift_calulate.finished = 1U;
    step.update();
}

static void enter_step_down_descend(LiftStepDown &step, float x_prepare, float y_prepare)
{
    vision.x_diff = x_prepare;
    vision.y_diff = y_prepare;
    step.update();
    step.update();
}

static void finish_turn_180_step_down(LiftStepDown &step, float x_finish, float y_finish)
{
    step.setStepDownRadarTarget(x_finish, y_finish, x_finish, y_finish, 0U, 0U, 1U);
    vision.x_diff = x_finish;
    vision.y_diff = y_finish;
    step.startStepDown();

    step.update();
    enter_step_down_descend(step, x_finish + STEP_DOWN_PREPARE_DISTANCE_L, y_finish);
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

    enter_step_down_descend(step, STEP_DOWN_PREPARE_DISTANCE_L, 0.0f);
    assert(step.getLiftLinearSpeedTarget(0.0f) < 0.0f);
}

static void test_step_down_200_does_not_wait_for_lift_height(void)
{
    reset_test_config();
    LiftStepDown step;
    step.setStepDownHeightMode(200U);
    step.setStepDownRadarTarget(0.0f, 0.0f, 2.0f, 2.0f, 0U, 0U, 1U);
    lift_calulate.command_seq = 100U;
    lift_calulate.finished = 0U;

    step.startStepDown();
    step.update();
    vision.x_diff = STEP_DOWN_PREPARE_DISTANCE_L;
    step.update();
    step.update();

    assert(step.getLiftSwitch(0U) == 2U);
    assert(step.getLiftLinearSpeedTarget(0.0f) < 0.0f);
}

static void test_turn_180_prepare_keeps_converted_body_components(void)
{
    reset_test_config();
    LiftStepDown step;
    step.setStepDownRadarTarget(0.0f, 0.0f, 2.0f, 2.0f, 0U, 0U, 1U);
    vision.angle_x = 177.0f;
    step.startStepDown();

    step.update();

    assert(fabsf(step.getChassisVxTarget(0.0f)) > 0.1000f);
    assert(fabsf(step.getChassisVyTarget(0.0f)) > 0.0100f);
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

    enter_step_down_descend(step, 0.0f, -STEP_DOWN_PREPARE_DISTANCE_L);
    assert(step.getLiftLinearSpeedTarget(0.0f) < 0.0f);
}

static void test_left_90_prepare_converts_world_y_to_body_x(void)
{
    reset_test_config();
    LiftStepDown step;
    step.setStepDownRadarTarget(0.0f, 0.0f, 2.0f, 2.0f, 1U, 0U, 0U);
    vision.angle_x = 87.0f;
    step.startStepDown();

    step.update();

    assert(fabsf(step.getChassisVxTarget(0.0f)) > 0.1000f);
    assert(fabsf(step.getChassisVyTarget(0.0f)) > 0.0100f);
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

    enter_step_down_descend(step, 0.0f, STEP_DOWN_PREPARE_DISTANCE_L);
    assert(step.getLiftLinearSpeedTarget(0.0f) < 0.0f);
}

static void test_right_90_prepare_converts_world_y_to_body_x(void)
{
    reset_test_config();
    LiftStepDown step;
    step.setStepDownRadarTarget(0.0f, 0.0f, 2.0f, 2.0f, 0U, 1U, 0U);
    vision.angle_x = -87.0f;
    step.startStepDown();

    step.update();

    assert(fabsf(step.getChassisVxTarget(0.0f)) > 0.1000f);
    assert(fabsf(step.getChassisVyTarget(0.0f)) > 0.0100f);
}

static void test_step_down_400_waits_for_lift_heights(void)
{
    reset_test_config();
    LiftStepDown step;
    step.setStepDownHeightMode(400U);
    step.setStepDownRadarTarget(0.0f, 0.0f, 2.0f, 2.0f, 0U, 0U, 1U);
    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    lift_calulate.command_seq = 100U;
    lift_calulate.finished = 1U;

    step.startStepDown();
    step.update();
    vision.x_diff = STEP_DOWN_PREPARE_DISTANCE_L;
    step.update();

    lift_calulate.finished = 0U;
    step.update();
    assert(step.getLiftSwitch(0U) == 2U);
    assert_near(step.getLiftLinearSpeedTarget(1.0f), 0.0f);
    assert_near(step.getChassisVxTarget(1.0f), 0.0f);
    assert_near(step.getChassisVyTarget(1.0f), 0.0f);

    lift_calulate.command_seq = 101U;
    lift_calulate.finished = 1U;
    step.update();
    step.update();
    assert(step.getLiftLinearSpeedTarget(0.0f) < 0.0f);

    vision.x_diff = STEP_DOWN_PREPARE_DISTANCE_L + STEP_DOWN_DESCEND_DISTANCE_D;
    step.update();
    lift_calulate.finished = 0U;
    step.update();
    assert(step.getLiftSwitch(0U) == 1U);
    assert_near(step.getLiftLinearSpeedTarget(1.0f), 0.0f);
    assert_near(step.getChassisVxTarget(1.0f), 0.0f);
    assert_near(step.getChassisVyTarget(1.0f), 0.0f);

    lift_calulate.command_seq = 102U;
    lift_calulate.finished = 1U;
    step.update();
    step.update();
    assert(step.getLiftSwitch(0U) == 3U);
    assert(step.getChassisVxTarget(0.0f) > 0.0f);

    vision.x_diff = 2.0f;
    vision.y_diff = 2.0f;
    step.update();
    assert(step.isStepDownFinished() != 0U);
}

int main(void)
{
    test_prepare_base_uses_latest_config();
    test_prepare_base_uses_current_config_not_previous_finish();
    test_turn_180_prepare_and_descend();
    test_step_down_200_does_not_wait_for_lift_height();
    test_turn_180_prepare_keeps_converted_body_components();
    test_left_90_prepare_and_descend();
    test_left_90_prepare_converts_world_y_to_body_x();
    test_right_90_prepare_and_descend();
    test_right_90_prepare_converts_world_y_to_body_x();
    test_step_down_400_waits_for_lift_heights();
    return 0;
}
