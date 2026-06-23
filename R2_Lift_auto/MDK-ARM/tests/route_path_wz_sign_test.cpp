#include "route_task.h"
#include "lift_step_down.h"
#include "lift_step_up.h"
#include "mieling.h"
#include "PID.h"
#include "usart_task.h"
#include "usart.h"

float yaw_target = 0.0f;
uint32_t route_test_tick_ms = 0U;
PID pid_yaw = {};
VisionData_t vision = {};
Block_Vision block_vision_middle[13] = {};
LiftAuto lift_auto;
LiftStepDown lift_step_down;
MeilingLocator meiling;
UART_HandleTypeDef huart7 = {};

uint8_t vision_command_push(int)
{
    return 0U;
}

uint8_t vision_command_pop(int *)
{
    return 0U;
}

uint8_t vision_command_peek(int *)
{
    return 0U;
}

uint8_t vision_command_has_pending(void)
{
    return 0U;
}

void vision_command_clear(void)
{
}

uint8_t vision_block_push(int)
{
    return 0U;
}

uint8_t vision_block_pop(int *)
{
    return 0U;
}

uint8_t vision_block_has_pending(void)
{
    return 0U;
}

void vision_block_clear(void)
{
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

int main(void)
{
    route_t.route_reset();

    route_t.flag_start = 1U;
    route_t.state = PHASE_FIRST_PATH;
    vision.exec = 1;
    vision.x_diff = 0.0f;
    vision.y_diff = 0.0f;
    vision.angle_x = 10.0f;

    route_t.meiling_route();

    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_wz = 0.0f;
    uint8_t active = route_t.getPathChassisTarget(0.0f, 0.0f, 0.0f, &target_vx, &target_vy, &target_wz);

    if (active == 0U) {
        return 1;
    }

    return (target_wz > 0.0f) ? 0 : 2;
}
