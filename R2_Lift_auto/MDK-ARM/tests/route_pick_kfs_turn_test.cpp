#include "route_task.h"
#include "lift_step_down.h"
#include "lift_step_up.h"
#include "mieling.h"
#include "PID.h"
#include "usart_task.h"
#include "usart.h"
#include "arm_comm.h"

#include <math.h>

extern uint8_t ARM_COMM_PICK_KFS_STABLE_COUNT;
extern float PICK_KFS_BEFORE_STEP_ADVANCE_CM;

float yaw_target = 0.0f;
uint32_t route_test_tick_ms = 0U;
PID pid_yaw      = {};
VisionData_t vision = {};
Block_Vision block_vision_middle[13] = {};
LiftAuto lift_auto;
LiftStepDown lift_step_down;
MeilingLocator meiling;
UART_HandleTypeDef huart7 = {};

static int command_queue[8];
static uint8_t command_head = 0U;
static uint8_t command_tail = 0U;
static int block_queue[8];
static uint8_t block_head = 0U;
static uint8_t block_tail = 0U;

uint8_t vision_command_push(int cmd)
{
    const uint8_t next_tail = static_cast<uint8_t>((command_tail + 1U) % 8U);
    if (next_tail == command_head) {
        return 0U;
    }

    command_queue[command_tail] = cmd;
    command_tail                = next_tail;
    return 1U;
}

uint8_t vision_command_pop(int *out)
{
    if (out == 0 || command_head == command_tail) {
        return 0U;
    }

    *out         = command_queue[command_head];
    command_head = static_cast<uint8_t>((command_head + 1U) % 8U);
    return 1U;
}

uint8_t vision_command_peek(int *out)
{
    if (out == 0 || command_head == command_tail) {
        return 0U;
    }

    *out = command_queue[command_head];
    return 1U;
}

uint8_t vision_command_has_pending(void)
{
    return (command_head != command_tail) ? 1U : 0U;
}

void vision_command_clear(void)
{
    command_head = 0U;
    command_tail = 0U;
}

uint8_t vision_block_push(int val)
{
    const uint8_t next_tail = static_cast<uint8_t>((block_tail + 1U) % 8U);
    if (next_tail == block_head) {
        return 0U;
    }

    block_queue[block_tail] = val;
    block_tail              = next_tail;
    return 1U;
}

uint8_t vision_block_pop(int *out)
{
    if (out == 0 || block_head == block_tail) {
        return 0U;
    }

    *out       = block_queue[block_head];
    block_head = static_cast<uint8_t>((block_head + 1U) % 8U);
    return 1U;
}

uint8_t vision_block_has_pending(void)
{
    return (block_head != block_tail) ? 1U : 0U;
}

void vision_block_clear(void)
{
    block_head = 0U;
    block_tail = 0U;
}

int parse_vision_frame_computer(uint8_t *, uint16_t, VisionData_t *)
{
    return 0;
}

void send_position_to_pc(int16_t, uint8_t, float, float, float)
{
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t)
{
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
    return HAL_UART_Transmit(huart, pData, Size, 0U);
}

static void clear_queues(void)
{
    vision_command_clear();
    vision_block_clear();
}

static int floatNear(float actual, float expected)
{
    return (fabsf(actual - expected) < 0.001f) ? 1 : 0;
}

static void restorePickKFSParams(uint8_t stable_count, float before_advance_cm)
{
    ARM_COMM_PICK_KFS_STABLE_COUNT = stable_count;
    PICK_KFS_BEFORE_STEP_ADVANCE_CM = before_advance_cm;
}

static void cacheCurrentBlockCenterWithStepUp(void)
{
    block_vision_middle[1].x = 1.0f;
    block_vision_middle[1].y = 2.0f;

    route_t.state = PHASE_VISION;
    vision_command_push(3);
    vision_block_push(1);
    route_t.vision_choice();

    route_t.state = PHASE_VISION;
}

static int expectPickKFSReturnsToCenterBeforeTurn(int cmd, Route_state kfs_state, int result_base)
{
    const uint8_t old_stable_count = ARM_COMM_PICK_KFS_STABLE_COUNT;
    const float old_before_advance_cm = PICK_KFS_BEFORE_STEP_ADVANCE_CM;
    ARM_COMM_PICK_KFS_STABLE_COUNT = 1U;
    PICK_KFS_BEFORE_STEP_ADVANCE_CM = 0.0f;

    route_t.route_reset();
    clear_queues();

    cacheCurrentBlockCenterWithStepUp();

    block_vision_middle[2].x = 5.0f;
    block_vision_middle[2].y = 2.0f;
    vision.x_diff = 1.2f;
    vision.y_diff = 2.0f;
    vision.angle_x = 0.0f;

    vision_command_push(cmd);
    vision_command_push(7);
    vision_block_push(2);
    route_t.vision_choice();
    if (route_t.state != kfs_state) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return result_base + 1;
    }

    int next_block = 0;
    if (vision_block_pop(&next_block) != 1U || next_block != 2) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return result_base + 2;
    }

    route_t.flag_start = 1U;
    route_t.meiling_route();
    arm_comm.rx_data_.event = 1U;
    route_t.meiling_route();
    if (route_t.state != kfs_state) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return result_base + 3;
    }

    route_t.meiling_route();
    if (arm_comm.getChassisVxTarget(0.0f) >= 0.0f) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return result_base + 4;
    }

    vision.x_diff = 1.0f;
    vision.y_diff = 2.0f;
    route_t.meiling_route();
    if (route_t.state != PHASE_TURN_LEFT90) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return result_base + 5;
    }

    restorePickKFSParams(old_stable_count, old_before_advance_cm);
    return 0;
}

static int expectPickKFSWithoutCenterEntersTurnOnEvent1(void)
{
    const uint8_t old_stable_count = ARM_COMM_PICK_KFS_STABLE_COUNT;
    const float old_before_advance_cm = PICK_KFS_BEFORE_STEP_ADVANCE_CM;
    ARM_COMM_PICK_KFS_STABLE_COUNT = 1U;
    PICK_KFS_BEFORE_STEP_ADVANCE_CM = 0.0f;

    route_t.route_reset();
    clear_queues();

    vision.x_diff = 1.2f;
    vision.y_diff = 2.0f;
    vision.angle_x = 0.0f;

    route_t.flag_start = 1U;
    route_t.state = PHASE_VISION;
    vision_command_push(11);
    vision_command_push(7);
    route_t.vision_choice();
    if (route_t.state != PHASE_GET_KFS_HEIGHT_200) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return 31;
    }

    route_t.meiling_route();
    arm_comm.rx_data_.event = 1U;
    route_t.meiling_route();
    if (route_t.state != PHASE_TURN_LEFT90) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return 32;
    }

    restorePickKFSParams(old_stable_count, old_before_advance_cm);
    return 0;
}

static int expectPickKFSSlowTurnRampsYawTarget(void)
{
    const uint8_t old_stable_count = ARM_COMM_PICK_KFS_STABLE_COUNT;
    const float old_before_advance_cm = PICK_KFS_BEFORE_STEP_ADVANCE_CM;
    ARM_COMM_PICK_KFS_STABLE_COUNT = 1U;
    PICK_KFS_BEFORE_STEP_ADVANCE_CM = 0.0f;
    route_test_tick_ms = 1000U;

    route_t.route_reset();
    clear_queues();

    cacheCurrentBlockCenterWithStepUp();

    vision.x_diff = 1.2f;
    vision.y_diff = 2.0f;
    vision.angle_x = 10.0f;

    vision_command_push(11);
    vision_command_push(7);
    route_t.vision_choice();

    route_t.flag_start = 1U;
    route_t.meiling_route();
    arm_comm.rx_data_.event = 1U;
    route_t.meiling_route();
    vision.x_diff = 1.0f;
    vision.y_diff = 2.0f;
    route_t.meiling_route();
    if (route_t.state != PHASE_TURN_LEFT90) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return 41;
    }

    route_t.meiling_route();
    if (floatNear(yaw_target, 25.0f) == 0) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return 42;
    }

    route_test_tick_ms = 1500U;
    route_t.meiling_route();
    if (floatNear(yaw_target, 65.0f) == 0) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return 43;
    }

    route_test_tick_ms = 2500U;
    route_t.meiling_route();
    if (floatNear(yaw_target, 100.0f) == 0) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return 44;
    }
    if (route_t.state != PHASE_TURN_LEFT90) {
        restorePickKFSParams(old_stable_count, old_before_advance_cm);
        return 45;
    }

    restorePickKFSParams(old_stable_count, old_before_advance_cm);
    return 0;
}

int main(void)
{
    int result = expectPickKFSReturnsToCenterBeforeTurn(10, PHASE_GET_KFS_SHORT_200, 0);
    if (result != 0) {
        return result;
    }

    result = expectPickKFSReturnsToCenterBeforeTurn(11, PHASE_GET_KFS_HEIGHT_200, 10);
    if (result != 0) {
        return result;
    }

    result = expectPickKFSReturnsToCenterBeforeTurn(12, PHASE_GET_KFS_HEIGHT_400, 20);
    if (result != 0) {
        return result;
    }

    result = expectPickKFSWithoutCenterEntersTurnOnEvent1();
    if (result != 0) {
        return result;
    }

    return expectPickKFSSlowTurnRampsYawTarget();
}
