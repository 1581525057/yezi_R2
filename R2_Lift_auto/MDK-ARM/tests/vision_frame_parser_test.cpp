#define VISION_FRAME_PARSER_HOST_TEST
#include "../TASK/usart_task.cpp"

#include <math.h>
#include <string.h>

static int float_near(float actual, float expected)
{
    return fabsf(actual - expected) < 0.001f;
}

static int expect_complete_frame(void)
{
    uint8_t frame[] = "S,1,1.25,-2.50,90.0,C,3,4,B,8,9,10,A,1,-1,7,E";
    VisionData_t parsed = {};
    int value = 0;

    vision_command_clear();
    vision_block_clear();
    if (parse_vision_frame_computer(frame,
                                    static_cast<uint16_t>(strlen(reinterpret_cast<char *>(frame))),
                                    &parsed) != 1)
        return 1;
    if (parsed.exec != 1 || !float_near(parsed.x_diff, 1.25f) ||
        !float_near(parsed.y_diff, -2.50f) || !float_near(parsed.angle_x, 90.0f))
        return 2;
    if (parsed.release_flag != 1 || parsed.claw_vertical_flag != -1 || parsed.unused_flag != 7)
        return 3;
    if (!vision_command_pop(&value) || value != 3)
        return 4;
    if (!vision_command_pop(&value) || value != 4 || vision_command_pop(&value))
        return 5;
    if (!vision_block_pop(&value) || value != 8)
        return 6;
    if (!vision_block_pop(&value) || value != 9)
        return 7;
    if (!vision_block_pop(&value) || value != 10 || vision_block_pop(&value))
        return 8;

    return 0;
}

static int expect_empty_queues(void)
{
    uint8_t frame[] = "S,0,0,0,0,C,B,A,0,1,0,E";
    VisionData_t parsed = {};
    int value = 0;

    vision_command_clear();
    vision_block_clear();
    if (parse_vision_frame_computer(frame,
                                    static_cast<uint16_t>(strlen(reinterpret_cast<char *>(frame))),
                                    &parsed) != 1)
        return 11;
    if (vision_command_pop(&value) || vision_block_pop(&value))
        return 12;
    if (parsed.release_flag != 0 || parsed.claw_vertical_flag != 1 || parsed.unused_flag != 0)
        return 13;

    return 0;
}

static int expect_missing_a_fails(void)
{
    uint8_t frame[] = "S,1,1,2,3,C,4,B,5,E";
    VisionData_t parsed = {};

    vision_command_clear();
    vision_block_clear();
    return parse_vision_frame_computer(frame,
                                       static_cast<uint16_t>(strlen(reinterpret_cast<char *>(frame))),
                                       &parsed) == 0
               ? 0
               : 21;
}

static int expect_incomplete_a_fails(void)
{
    uint8_t frame[] = "S,1,1,2,3,C,4,B,5,A,1,0,E";
    VisionData_t parsed = {};

    vision_command_clear();
    vision_block_clear();
    return parse_vision_frame_computer(frame,
                                       static_cast<uint16_t>(strlen(reinterpret_cast<char *>(frame))),
                                       &parsed) == 0
               ? 0
               : 31;
}

int main(void)
{
    int result = expect_complete_frame();
    if (result != 0)
        return result;

    result = expect_empty_queues();
    if (result != 0)
        return result;

    result = expect_missing_a_fails();
    if (result != 0)
        return result;

    return expect_incomplete_a_fails();
}
