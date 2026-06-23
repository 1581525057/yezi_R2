#include "lift_step_up.h"
#include "DT35.h"
#include "laser_distance.h"
#include "lift_class.h"
#include "usart_task.h"

#include <math.h>

extern uint8_t STEP_UP_AUTO_STABLE_COUNT;
extern uint32_t STEP_UP_AUTO_PREPARE_MM;
extern uint32_t STEP_UP_AUTO_FINISH_MM;
extern float STEP_UP_RADAR_CLIMB_DISTANCE_M;

VisionData_t vision = {};
Block_Vision block_vision[10] = {};
Block_Vision block_vision_middle[13] = {};
DT35 dt35;
LaserDistance laser_left;
LaserDistance laser_right;
Lift_Class lift_class = {};
LiftHeight_t lift_calulate = {};
debug_lift lift_debug = {};

static void enterClimbForward(uint8_t block_num)
{
    STEP_UP_AUTO_STABLE_COUNT = 1U;
    block_vision_middle[block_num].x = 1.17f;

    lift_auto.stopStepUp();
    lift_auto.setStepUpBlockNum(block_num);
    lift_auto.setStepUpRadarTarget(block_vision_middle[block_num].x, block_vision_middle[block_num].y);
    lift_auto.setStepUpRadarClimbDirection(0);

    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 0.0f;
    dt35.ch2.valid = 1U;
    dt35.ch2.distance_filtered = (float)STEP_UP_AUTO_PREPARE_MM;
    lift_calulate.command_seq = 10U;
    lift_calulate.finished = 1U;

    lift_auto.startStepUp();
    lift_auto.update();

    lift_calulate.command_seq = 11U;
    lift_calulate.finished = 1U;
    lift_auto.update();
}

static int expectClimbWaitsForLiftSwitch2Finished(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;
    STEP_UP_AUTO_STABLE_COUNT = 1U;

    lift_auto.stopStepUp();
    lift_auto.setStepUpBlockNum(1);
    block_vision_middle[1].x = 1.17f;
    lift_auto.setStepUpRadarTarget(block_vision_middle[1].x, block_vision_middle[1].y);

    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 0.0f;
    dt35.ch2.valid = 1U;
    dt35.ch2.distance_filtered = (float)STEP_UP_AUTO_PREPARE_MM;
    lift_calulate.command_seq = 10U;
    lift_calulate.finished = 1U;

    lift_auto.startStepUp();
    lift_auto.update();

    lift_calulate.finished = 0U;
    lift_auto.update();

    if (lift_auto.getLiftSwitch(0U) != 2U) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 1;
    }
    if (fabsf(lift_auto.getLiftLinearSpeedTarget(1.0f)) > 0.000001f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 2;
    }
    if (fabsf(lift_auto.getChassisVxTarget(1.0f)) > 0.000001f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 3;
    }
    if (fabsf(lift_auto.getChassisVyTarget(1.0f)) > 0.000001f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 4;
    }

    lift_calulate.command_seq = 11U;
    lift_calulate.finished = 1U;
    lift_auto.update();
    lift_auto.update();

    if (lift_auto.getLiftLinearSpeedTarget(0.0f) <= 0.0f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 5;
    }

    STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
    return 0;
}

static int expectStepUp400WaitsForSwitch1BeforeSwitch2(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;
    STEP_UP_AUTO_STABLE_COUNT = 1U;

    lift_auto.stopStepUp();
    lift_auto.setStepUpBlockNum(1);
    block_vision_middle[1].x = 1.17f;
    lift_auto.setStepUpRadarTarget(block_vision_middle[1].x, block_vision_middle[1].y);
    lift_auto.setStepUpRadarClimbDirection(0);
    lift_auto.setStepUpHeightMode(400U);

    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 0.0f;
    dt35.ch2.valid = 1U;
    dt35.ch2.distance_filtered = (float)(STEP_UP_AUTO_PREPARE_MM + 100U);
    lift_calulate.command_seq = 20U;
    lift_calulate.finished = 1U;

    lift_auto.startStepUp();
    lift_auto.update();

    if (lift_auto.getLiftSwitch(0U) != 1U) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 1;
    }

    lift_calulate.command_seq = 21U;
    lift_calulate.finished = 0U;
    lift_auto.update();

    lift_calulate.finished = 1U;
    lift_auto.update();

    dt35.ch2.distance_filtered = (float)STEP_UP_AUTO_PREPARE_MM;
    lift_auto.update();

    if (lift_auto.getLiftSwitch(0U) != 1U) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 2;
    }

    lift_auto.update();

    if (lift_auto.getLiftSwitch(0U) != 2U) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 3;
    }
    if (fabsf(lift_auto.getLiftLinearSpeedTarget(1.0f)) > 0.000001f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 4;
    }
    if (fabsf(lift_auto.getChassisVxTarget(1.0f)) > 0.000001f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 5;
    }
    if (fabsf(lift_auto.getChassisVyTarget(1.0f)) > 0.000001f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 6;
    }

    STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
    return 0;
}

static int expectStepUp400KeepsApproachingBeforeSwitch1Ready(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;
    STEP_UP_AUTO_STABLE_COUNT = 1U;

    lift_auto.stopStepUp();
    lift_auto.setStepUpBlockNum(1);
    block_vision_middle[1].x = 1.17f;
    lift_auto.setStepUpRadarTarget(block_vision_middle[1].x, block_vision_middle[1].y);
    lift_auto.setStepUpRadarClimbDirection(0);
    lift_auto.setStepUpHeightMode(400U);

    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 0.0f;
    dt35.ch2.valid = 1U;
    dt35.ch2.distance_filtered = (float)(STEP_UP_AUTO_PREPARE_MM + 100U);
    lift_calulate.command_seq = 40U;
    lift_calulate.finished = 1U;

    lift_auto.startStepUp();
    lift_auto.update();

    lift_calulate.command_seq = 41U;
    lift_calulate.finished = 0U;
    dt35.ch2.distance_filtered = (float)STEP_UP_AUTO_PREPARE_MM;
    lift_auto.update();

    dt35.ch2.distance_filtered = (float)(STEP_UP_AUTO_PREPARE_MM + 100U);
    lift_auto.update();

    if (lift_auto.getLiftSwitch(0U) != 1U) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 1;
    }
    if (lift_auto.getChassisVxTarget(0.0f) <= 0.0f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 2;
    }

    STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
    return 0;
}

static int expectStepUp400IgnoresFinishedWithoutObservedMove(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;
    STEP_UP_AUTO_STABLE_COUNT = 1U;

    lift_auto.stopStepUp();
    lift_auto.setStepUpBlockNum(1);
    block_vision_middle[1].x = 1.17f;
    lift_auto.setStepUpRadarTarget(block_vision_middle[1].x, block_vision_middle[1].y);
    lift_auto.setStepUpRadarClimbDirection(0);
    lift_auto.setStepUpHeightMode(400U);

    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 0.0f;
    dt35.ch2.valid = 1U;
    dt35.ch2.distance_filtered = (float)(STEP_UP_AUTO_PREPARE_MM + 100U);
    lift_calulate.command_seq = 30U;
    lift_calulate.finished = 1U;

    lift_auto.startStepUp();
    lift_auto.update();

    lift_calulate.command_seq = 31U;
    lift_calulate.finished = 1U;
    dt35.ch2.distance_filtered = (float)STEP_UP_AUTO_PREPARE_MM;
    lift_auto.update();
    lift_auto.update();
    lift_auto.update();

    if (lift_auto.getLiftSwitch(0U) != 1U) {
        lift_auto.stopStepUp();
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 1;
    }

    lift_auto.stopStepUp();
    STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
    return 0;
}

static int expectClimbStopsInsideRadarFinishBand(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;

    enterClimbForward(1U);
    STEP_UP_AUTO_STABLE_COUNT = 3U;

    vision.x_diff = block_vision_middle[1].x - STEP_UP_RADAR_CLIMB_DISTANCE_M;
    lift_auto.update();

    if (lift_auto.getLiftSwitch(0U) != 2U) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 1;
    }
    if (fabsf(lift_auto.getLiftLinearSpeedTarget(1.0f)) > 0.000001f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 2;
    }

    STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
    return 0;
}

static int expectEntryStepUpUsesOwnBlockXMinusDistance(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;
    LiftAuto local_auto;

    STEP_UP_AUTO_STABLE_COUNT = 1U;
    block_vision_middle[1].x = 0.25f;
    block_vision_middle[2].x = 1.25f;

    local_auto.setStepUpBlockNum(1);
    local_auto.setStepUpRadarTarget(block_vision_middle[1].x, block_vision_middle[1].y);
    local_auto.setStepUpRadarClimbDirection(0);

    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 0.0f;
    dt35.ch2.valid = 1U;
    dt35.ch2.distance_filtered = (float)STEP_UP_AUTO_PREPARE_MM;
    lift_calulate.command_seq = 50U;
    lift_calulate.finished = 1U;

    local_auto.startStepUp();
    local_auto.update();

    lift_calulate.command_seq = 51U;
    lift_calulate.finished = 1U;
    local_auto.update();

    vision.x_diff = block_vision_middle[1].x - STEP_UP_RADAR_CLIMB_DISTANCE_M;
    local_auto.update();

    if (fabsf(local_auto.getLiftLinearSpeedTarget(1.0f)) > 0.000001f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 1;
    }

    STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
    return 0;
}

static int expectEntryStepUpIgnoresStalePreviousCenter(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;
    LiftAuto local_auto;

    STEP_UP_AUTO_STABLE_COUNT = 1U;
    block_vision_middle[1].x = 0.33f;
    block_vision_middle[2].x = 1.33f;

    local_auto.setStepUpBlockNum(4);
    local_auto.setStepUpRadarTarget(8.0f, 0.0f);
    local_auto.stopStepUp();

    local_auto.setStepUpBlockNum(1);
    local_auto.setStepUpRadarTarget(block_vision_middle[1].x, block_vision_middle[1].y);
    local_auto.setStepUpRadarClimbDirection(0);

    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 0.0f;
    dt35.ch2.valid = 1U;
    dt35.ch2.distance_filtered = (float)STEP_UP_AUTO_PREPARE_MM;
    lift_calulate.command_seq = 60U;
    lift_calulate.finished = 1U;

    local_auto.startStepUp();
    local_auto.update();

    lift_calulate.command_seq = 61U;
    lift_calulate.finished = 1U;
    local_auto.update();

    vision.x_diff = block_vision_middle[1].x - STEP_UP_RADAR_CLIMB_DISTANCE_M;
    local_auto.update();

    if (fabsf(local_auto.getLiftLinearSpeedTarget(1.0f)) > 0.000001f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 1;
    }

    STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
    return 0;
}

static int expectClimbKeepsMinimumSpeedNearLaserFinish(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;

    enterClimbForward(2U);
    STEP_UP_AUTO_STABLE_COUNT = 3U;

    dt35.ch2.valid = 1U;
    dt35.ch2.distance_filtered = (float)(STEP_UP_AUTO_FINISH_MM + 2U);
    lift_auto.update();

    if (lift_auto.getLiftLinearSpeedTarget(0.0f) < 0.15f) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 1;
    }

    STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
    return 0;
}

static int expectClimbCompletionReturnsToSwitch3(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;

    enterClimbForward(1U);
    STEP_UP_AUTO_STABLE_COUNT = 1U;

    vision.x_diff = block_vision_middle[1].x - STEP_UP_RADAR_CLIMB_DISTANCE_M;
    lift_auto.update();
    lift_auto.update();

    if (lift_auto.getLiftSwitch(0U) != 3U) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 1;
    }

    STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
    return 0;
}

static int expectClimbCompletionCanSkipReturnMiddle(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;

    enterClimbForward(1U);
    STEP_UP_AUTO_STABLE_COUNT = 1U;
    lift_auto.setStepUpReturnMiddle(0U);

    vision.x_diff = block_vision_middle[1].x - STEP_UP_RADAR_CLIMB_DISTANCE_M;
    lift_auto.update();

    lift_calulate.command_seq = 12U;
    lift_calulate.finished = 1U;
    lift_auto.update();

    if (lift_auto.isStepUpFinished() != 1U) {
        STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
        return 1;
    }

    STEP_UP_AUTO_STABLE_COUNT = old_stable_count;
    return 0;
}

int main(void)
{
    const int wait_result = expectClimbWaitsForLiftSwitch2Finished();
    if (wait_result != 0) {
        return wait_result;
    }

    const int step_up_400_result = expectStepUp400WaitsForSwitch1BeforeSwitch2();
    if (step_up_400_result != 0) {
        return 30 + step_up_400_result;
    }

    const int step_up_400_approach_result = expectStepUp400KeepsApproachingBeforeSwitch1Ready();
    if (step_up_400_approach_result != 0) {
        return 80 + step_up_400_approach_result;
    }

    const int step_up_400_stale_result = expectStepUp400IgnoresFinishedWithoutObservedMove();
    if (step_up_400_stale_result != 0) {
        return 70 + step_up_400_stale_result;
    }

    const int radar_result = expectClimbStopsInsideRadarFinishBand();
    if (radar_result != 0) {
        return 10 + radar_result;
    }

    const int first_step_result = expectEntryStepUpUsesOwnBlockXMinusDistance();
    if (first_step_result != 0) {
        return 60 + first_step_result;
    }

    const int entry_stale_result = expectEntryStepUpIgnoresStalePreviousCenter();
    if (entry_stale_result != 0) {
        return 90 + entry_stale_result;
    }

    const int laser_result = expectClimbKeepsMinimumSpeedNearLaserFinish();
    if (laser_result != 0) {
        return 20 + laser_result;
    }

    const int return_height_result = expectClimbCompletionReturnsToSwitch3();
    if (return_height_result != 0) {
        return 40 + return_height_result;
    }

    const int skip_middle_result = expectClimbCompletionCanSkipReturnMiddle();
    if (skip_middle_result != 0) {
        return 50 + skip_middle_result;
    }

    return 0;
}
