#include "route_task.h"
#include "lift_step_down.h"
#include "lift_step_up.h"
#include "mieling.h"
#include "PID.h"
#include "usart_task.h"
#include <assert.h>

float yaw_target = 0.0f;
PID pid_yaw      = {};
VisionData_t vision = {};
Block_Vision block_vision_middle[13] = {};
LiftAuto lift_auto;
LiftStepDown lift_step_down;
MeilingLocator meiling;

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

static void clear_queues(void)
{
    vision_command_clear();
    vision_block_clear();
}

static void finish_turn_with_yaw(Route_state turn_state, float start_yaw_deg, float finish_yaw_deg)
{
    route_t.flag_start = 1U;
    route_t.state      = turn_state;
    vision.angle_x     = start_yaw_deg;
    pid_yaw.pid.Err    = 0.0f;

    route_t.meiling_route();
    vision.angle_x = finish_yaw_deg;
    for (int i = 1; i < 200; ++i) {
        route_t.meiling_route();
    }

    assert(route_t.state == PHASE_VISION);
}

static void configure_step_up_after_turn(void)
{
    block_vision_middle[1].x = 1.0f;
    block_vision_middle[1].y = 2.0f;
    vision_command_push(3);
    vision_block_push(1);

    route_t.vision_choice();
}

static void configure_step_down(int block_num, float x, float y)
{
    block_vision_middle[block_num].x = x;
    block_vision_middle[block_num].y = y;
    vision_command_push(5);
    vision_block_push(block_num);

    route_t.vision_choice();
}

static void test_step_up_direction_uses_radar_yaw_after_turns(void)
{
    route_t.route_reset();
    clear_queues();

    finish_turn_with_yaw(PHASE_TURN_RIGHT90, 0.0f, -90.0f);
    configure_step_up_after_turn();
    assert(lift_auto.climb_direction == -1);

    route_t.state = PHASE_VISION;
    clear_queues();

    finish_turn_with_yaw(PHASE_TURN_LEFT90, -90.0f, 0.0f);
    configure_step_up_after_turn();
    assert(lift_auto.climb_direction == 0);
}

static void test_consecutive_step_down_keeps_right_turn_direction(void)
{
    route_t.route_reset();
    clear_queues();

    finish_turn_with_yaw(PHASE_TURN_RIGHT90, 0.0f, -90.0f);
    configure_step_down(1, 1.0f, 2.0f);
    assert(lift_step_down.turn_right_90 == 1U);

    route_t.state = PHASE_VISION;
    clear_queues();

    configure_step_down(2, 3.0f, 4.0f);
    assert(lift_step_down.turn_right_90 == 1U);
    assert(lift_step_down.turn_180 == 0U);
}

int main(void)
{
    test_step_up_direction_uses_radar_yaw_after_turns();
    test_consecutive_step_down_keeps_right_turn_direction();
    return 0;
}
