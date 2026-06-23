#include "route_task.h"
#include "lift_step_down.h"
#include "lift_step_up.h"
#include "mieling.h"
#include "PID.h"
#include "usart_task.h"
#include "usart.h"
#include "arm_comm.h"
#include <assert.h>
#include <math.h>

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

static void finish_turn_with_yaw(Route_state turn_state, float start_yaw_deg, float finish_yaw_deg)
{
    route_t.flag_start = 1U;
    route_t.state      = turn_state;
    vision.angle_x     = start_yaw_deg;
    pid_yaw.pid.Err    = 0.0f;

    route_t.meiling_route();
    vision.angle_x = finish_yaw_deg;
    for (int i = 0; i < 200; ++i) {
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

static void configure_step_down_command(int cmd, int block_num, float x, float y)
{
    block_vision_middle[block_num].x = x;
    block_vision_middle[block_num].y = y;
    vision_command_push(cmd);
    vision_block_push(block_num);

    route_t.vision_choice();
}

static void configure_step_down(int block_num, float x, float y)
{
    configure_step_down_command(5, block_num, x, y);
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
    assert(lift_step_down.height_mode_mm == 200U);
    assert(lift_step_down.turn_right_90 == 1U);

    route_t.state = PHASE_VISION;
    clear_queues();

    configure_step_down(2, 3.0f, 4.0f);
    assert(lift_step_down.turn_right_90 == 1U);
    assert(lift_step_down.turn_180 == 0U);
}

static void test_case_6_configures_step_down_400(void)
{
    route_t.route_reset();
    clear_queues();

    finish_turn_with_yaw(PHASE_TURN_RIGHT90, 0.0f, -90.0f);
    configure_step_up_after_turn();
    route_t.state = PHASE_VISION;
    clear_queues();

    configure_step_down_command(6, 2, 3.0f, 4.0f);

    assert(route_t.state == PHASE_STEP_DOWN);
    assert(lift_step_down.block_num == 2);
    assert(lift_step_down.height_mode_mm == 400U);
    assert(fabsf(lift_step_down.prepare_base_x - 1.0f) < 0.001f);
    assert(fabsf(lift_step_down.prepare_base_y - 2.0f) < 0.001f);
    assert(fabsf(lift_step_down.finish_x - 3.0f) < 0.001f);
    assert(fabsf(lift_step_down.finish_y - 4.0f) < 0.001f);
    assert(lift_step_down.turn_right_90 == 1U);
    assert(lift_step_down.turn_180 == 0U);
}

static void test_step_up_skips_return_middle_before_next_step_up(void)
{
    route_t.route_reset();
    clear_queues();

    block_vision_middle[1].x = 1.0f;
    block_vision_middle[1].y = 2.0f;
    lift_auto.return_middle = 1U;
    route_t.state = PHASE_VISION;
    vision_command_push(3);
    vision_command_push(4);
    vision_block_push(1);

    route_t.vision_choice();
    assert(route_t.state == PHASE_STEP_UP);
    assert(lift_auto.return_middle == 0U);

    route_t.route_reset();
    clear_queues();

    lift_auto.return_middle = 0U;
    route_t.state = PHASE_VISION;
    vision_command_push(4);
    vision_command_push(7);
    vision_block_push(1);

    route_t.vision_choice();
    assert(route_t.state == PHASE_STEP_UP);
    assert(lift_auto.return_middle == 1U);
}

static void test_left_turn_near_180_ignores_stale_pid_error(void)
{
    route_t.route_reset();
    clear_queues();

    route_t.flag_start = 1U;
    route_t.state      = PHASE_VISION;
    vision.angle_x     = 175.0f;
    pid_yaw.pid.Err    = 0.0f;
    vision_command_push(7);

    route_t.vision_choice();
    assert(route_t.state == PHASE_TURN_LEFT90);

    for (int i = 0; i < 200; ++i) {
        route_t.meiling_route();
    }

    assert(route_t.state == PHASE_TURN_LEFT90);
}

static void test_direct_turn_keeps_immediate_yaw_target(void)
{
    route_t.route_reset();
    clear_queues();

    route_test_tick_ms = 1000U;
    route_t.flag_start = 1U;
    route_t.state      = PHASE_VISION;
    vision.angle_x     = 10.0f;
    vision_command_push(7);

    route_t.vision_choice();
    route_t.meiling_route();
    assert(fabsf(yaw_target - 100.0f) < 0.001f);

    route_test_tick_ms = 1500U;
    route_t.meiling_route();
    assert(fabsf(yaw_target - 100.0f) < 0.001f);
}

static void test_kfs_count_syncs_from_arm_feedback(void)
{
    route_t.route_reset();
    clear_queues();

    const uint8_t one_in_arm_two_in_car[ArmComm::RX_FRAME_LENGTH] = {0xBB, 0x03, 0x01, 0x01, 0x02, 0xEE};
    assert(arm_comm.parseRxFrame(one_in_arm_two_in_car, ArmComm::RX_FRAME_LENGTH) == 1U);

    route_t.state = PHASE_VISION;
    vision_command_push(10);
    route_t.update_number_KFS_by_cmd();
    route_t.vision_choice();
    assert(route_t.number_KFS == 4U);

    route_t.vision_choice();
    assert(route_t.number_KFS == 4U);

    const uint8_t none_in_arm_one_in_car[ArmComm::RX_FRAME_LENGTH] = {0xBB, 0x03, 0x01, 0x00, 0x01, 0xEE};
    assert(arm_comm.parseRxFrame(none_in_arm_one_in_car, ArmComm::RX_FRAME_LENGTH) == 1U);

    route_t.state = PHASE_VISION;
    vision_command_push(11);
    route_t.update_number_KFS_by_cmd();
    route_t.vision_choice();
    assert(route_t.number_KFS == 2U);

    const uint8_t one_in_arm_none_in_car[ArmComm::RX_FRAME_LENGTH] = {0xBB, 0x03, 0x01, 0x01, 0x00, 0xEE};
    assert(arm_comm.parseRxFrame(one_in_arm_none_in_car, ArmComm::RX_FRAME_LENGTH) == 1U);

    route_t.state = PHASE_VISION;
    vision_command_push(12);
    route_t.update_number_KFS_by_cmd();
    route_t.vision_choice();
    assert(route_t.number_KFS == 2U);
}

int main(void)
{
    test_step_up_direction_uses_radar_yaw_after_turns();
    test_consecutive_step_down_keeps_right_turn_direction();
    test_case_6_configures_step_down_400();
    test_step_up_skips_return_middle_before_next_step_up();
    test_left_turn_near_180_ignores_stale_pid_error();
    test_direct_turn_keeps_immediate_yaw_target();
    test_kfs_count_syncs_from_arm_feedback();
    return 0;
}
