#include "lift_step_up.h"
#include "DT35.h"
#include "laser_distance.h"
#include "lift_class.h"
#include "usart_task.h"

#include <math.h>

extern uint8_t STEP_UP_AUTO_STABLE_COUNT;
extern uint32_t STEP_UP_AUTO_PREPARE_MM;
extern uint32_t STEP_UP_AUTO_FINISH_MM;

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

    lift_auto.stopStepUp();
    lift_auto.setStepUpBlockNum(block_num);
    lift_auto.setStepUpRadarTarget(0.0f, 0.0f);
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
    lift_auto.setStepUpRadarTarget(0.0f, 0.0f);

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

static int expectClimbStopsInsideRadarFinishBand(void)
{
    const uint8_t old_stable_count = STEP_UP_AUTO_STABLE_COUNT;

    enterClimbForward(1U);
    STEP_UP_AUTO_STABLE_COUNT = 3U;

    vision.x_diff = 0.85f;
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

int main(void)
{
    const int wait_result = expectClimbWaitsForLiftSwitch2Finished();
    if (wait_result != 0) {
        return wait_result;
    }

    const int radar_result = expectClimbStopsInsideRadarFinishBand();
    if (radar_result != 0) {
        return 10 + radar_result;
    }

    const int laser_result = expectClimbKeepsMinimumSpeedNearLaserFinish();
    if (laser_result != 0) {
        return 20 + laser_result;
    }

    return 0;
}
