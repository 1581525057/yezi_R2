#include "route_task.h"
#include "lift_step_down.h"
#include "lift_step_up.h"
#include "mieling.h"
#include "PID.h"
#include "usart_task.h"
#include "usart.h"
#include "path_follow.h"
#include "path.h"

#include <math.h>

float yaw_target = 0.0f;
uint32_t route_test_tick_ms = 0U;
PID pid_yaw = {};
VisionData_t vision = {};
Block_Vision block_vision_middle[13] = {};
LiftAuto lift_auto;
LiftStepDown lift_step_down;
MeilingLocator meiling;
UART_HandleTypeDef huart7 = {};

static int command_queue[4];
static uint8_t command_head = 0U;
static uint8_t command_tail = 0U;
static int block_queue[4];
static uint8_t block_head = 0U;
static uint8_t block_tail = 0U;

uint8_t vision_command_push(int cmd)
{
    const uint8_t next_tail = static_cast<uint8_t>((command_tail + 1U) % 4U);
    if (next_tail == command_head) {
        return 0U;
    }

    command_queue[command_tail] = cmd;
    command_tail = next_tail;
    return 1U;
}

uint8_t vision_command_pop(int *out)
{
    if (out == 0 || command_head == command_tail) {
        return 0U;
    }

    *out = command_queue[command_head];
    command_head = static_cast<uint8_t>((command_head + 1U) % 4U);
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
    const uint8_t next_tail = static_cast<uint8_t>((block_tail + 1U) % 4U);
    if (next_tail == block_head) {
        return 0U;
    }

    block_queue[block_tail] = val;
    block_tail = next_tail;
    return 1U;
}

uint8_t vision_block_pop(int *out)
{
    if (out == 0 || block_head == block_tail) {
        return 0U;
    }

    *out = block_queue[block_head];
    block_head = static_cast<uint8_t>((block_head + 1U) % 4U);
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

static int floatNear(float actual, float expected)
{
    return (fabsf(actual - expected) < 0.001f) ? 1 : 0;
}

static int expectInactivePathPassesManualTargets(void)
{
    route_t.route_reset();
    vision.exec = 0;
    route_t.state = PHASE_FIRST_PATH;
    route_t.meiling_route();

    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_wz = 0.0f;
    uint8_t active = route_t.getPathChassisTarget(1.2f, -0.4f, 0.3f, &target_vx, &target_vy, &target_wz);

    if (active != 0U) {
        return 1;
    }
    if (floatNear(target_vx, 1.2f) == 0) {
        return 2;
    }
    if (floatNear(target_vy, -0.4f) == 0) {
        return 3;
    }
    if (floatNear(target_wz, 0.3f) == 0) {
        return 4;
    }

    return 0;
}

static int expectActivePathOverridesTargets(void)
{
    route_t.route_reset();

    route_t.flag_start = 1U;
    route_t.state = PHASE_FIRST_PATH;
    vision.exec = 1;
    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 0.0f;
    route_t.meiling_route();

    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_wz = 0.0f;
    uint8_t active = route_t.getPathChassisTarget(9.0f, 9.0f, 9.0f, &target_vx, &target_vy, &target_wz);

    if (active == 0U) {
        return 11;
    }
    if (floatNear(target_vx, 0.119796f) == 0) {
        return 12;
    }
    if (floatNear(target_vy, 0.195829f) == 0) {
        return 13;
    }

    vision.exec = 0;
    route_t.meiling_route();
    active = route_t.getPathChassisTarget(1.2f, -0.4f, 0.3f, &target_vx, &target_vy, &target_wz);
    if (active != 0U) {
        return 14;
    }
    if (floatNear(target_vx, 1.2f) == 0) {
        return 15;
    }

    return 0;
}

static int expectPathDeviationReleasesTargets(void)
{
    route_t.route_reset();

    route_t.flag_start = 1U;
    route_t.state = PHASE_FIRST_PATH;
    vision.exec = 1;
    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 0.0f;
    route_t.meiling_route();

    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_wz = 0.0f;
    uint8_t active = route_t.getPathChassisTarget(1.0f, 1.0f, 1.0f, &target_vx, &target_vy, &target_wz);

    if (active == 0U) {
        return 21;
    }

    vision.x_diff = 100.0f;
    vision.y_diff = 100.0f;
    route_t.meiling_route();
    active = route_t.getPathChassisTarget(1.2f, -0.4f, 0.7f, &target_vx, &target_vy, &target_wz);
    if (active != 0U) {
        return 22;
    }
    if (floatNear(target_wz, 0.7f) == 0) {
        return 23;
    }

    return 0;
}

static int expectCommandZeroEntersFindKFS(void)
{
    route_t.route_reset();
    vision_command_clear();
    vision_command_push(0);

    route_t.state = PHASE_VISION;
    route_t.vision_choice();

    if (route_t.state != PHASE_FIND_KFS) {
        return 31;
    }

    return 0;
}

static int expectPathFinishEntersFirstRelocation(void)
{
    route_t.route_reset();

    route_t.flag_start = 1U;
    route_t.state = PHASE_FIRST_PATH;
    vision.exec = 1;
    vision.x_diff = 0.94f;
    vision.y_diff = 1.63f;
    vision.angle_x = 0.0f;

    for (int i = 0; i < 200 && route_t.state == PHASE_FIRST_PATH; ++i) {
        route_t.meiling_route();
    }

    if (route_t.state != FIRST_RELOCATION) {
        return 41;
    }

    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_wz = 0.0f;
    uint8_t active = route_t.getPathChassisTarget(1.0f, 2.0f, 3.0f, &target_vx, &target_vy, &target_wz);
    if (active != 0U) {
        return 42;
    }

    return 0;
}

static int expectFindKFSRunsPath(void)
{
    route_t.route_reset();
    vision_command_clear();
    vision_command_push(1);

    route_t.flag_start = 1U;
    route_t.state = PHASE_VISION;
    route_t.vision_choice();

    vision.exec = 1;
    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 0.0f;
    route_t.meiling_route();

    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_wz = 0.0f;
    uint8_t active = route_t.getPathChassisTarget(9.0f, 9.0f, 9.0f, &target_vx, &target_vy, &target_wz);

    if (route_t.state != PHASE_FIND_KFS) {
        return 61;
    }
    if (active == 0U) {
        return 62;
    }
    if (floatNear(target_vx, 0.119796f) == 0) {
        return 63;
    }
    if (floatNear(target_vy, 0.195829f) == 0) {
        return 64;
    }

    return 0;
}

static int expectFirstRelocationStartsFirstTarget(void)
{
    route_t.route_reset();
    meiling.start_count = 0U;

    route_t.flag_start = 1U;
    route_t.state = FIRST_RELOCATION;
    route_t.meiling_route();

    if (meiling.start_count != 1U) {
        return 51;
    }
    if (floatNear(meiling.last_target.L_ref, 335.0f) == 0) {
        return 52;
    }
    if (floatNear(meiling.last_target.F_ref, 356.0f) == 0) {
        return 53;
    }

    return 0;
}

int main(void)
{
    int result = expectInactivePathPassesManualTargets();
    if (result != 0) {
        return result;
    }

    result = expectActivePathOverridesTargets();
    if (result != 0) {
        return result;
    }

    result = expectPathDeviationReleasesTargets();
    if (result != 0) {
        return result;
    }

    result = expectCommandZeroEntersFindKFS();
    if (result != 0) {
        return result;
    }

    result = expectPathFinishEntersFirstRelocation();
    if (result != 0) {
        return result;
    }

    result = expectFirstRelocationStartsFirstTarget();
    if (result != 0) {
        return result;
    }

    result = expectFindKFSRunsPath();
    if (result != 0) {
        return result;
    }

    return 0;
}
