#include "usart_task.h"
#include "usart.h"
#include "spi.h"
#include "cmsis_os.h"
#include "main.h"
// #include "AS5047.h"
#include "DT35.h"
#include "MiniPC.h"
#include "bsp_dwt.h"
#include "chassis_task.h"
#include "FTMTask.h"
#include "PID.h"
#include "route_task.h"
#include <string.h>
#include <stdio.h>
#include "conbat_task.h"
/* ========================== 全局变量 ========================== */

/* USB 串口接收缓冲区 */
uint8_t data_usb[USB_RX_BUFFER_SIZE];
uint16_t usb_rx_idx = 0;

static uint8_t posi_conputer[40]; // usb单片机发送给电脑

/* 视觉数据，由 parse_vision_frame_computer() 写入 */
VisionData_t vision;

/* ========================== 静态变量 ========================== */

/* 动作指令环形队列：缓存视觉帧中 C 后面的不定长动作 */
#define VISION_COMMAND_QUEUE_SIZE 32U
int vision_command_queue[VISION_COMMAND_QUEUE_SIZE];
uint8_t vision_command_head = 0U;
uint8_t vision_command_tail = 0U;
static int vision_last_command_plan[VISION_COMMAND_QUEUE_SIZE];
static uint8_t vision_last_command_count = 0U;
static uint8_t vision_last_plan_valid = 0U;
static uint8_t vision_plan_locked = 0U;

/* 方块指令环形队列：缓存视觉帧中 B 后面的不定长方块编号 */
#define VISION_BLOCK_QUEUE_SIZE 32U
int vision_block_queue[VISION_BLOCK_QUEUE_SIZE];
uint8_t vision_block_head = 0U;
uint8_t vision_block_tail = 0U;
static int vision_last_block_plan[VISION_BLOCK_QUEUE_SIZE];
static uint8_t vision_last_block_count = 0U;

// 每个方块的中心坐标
Block_Vision block_vision_middle[16];
// 第二个方块的中心坐标
float block_middle_x = 3.45f;
float block_middle_y = -1.57f;

// 通过第2个方块来计算得到其他8个的坐标位置

static void Block_claulate_Middle(void)
{
    float Block_Size = 1.2f;
    float x = block_middle_x;
    float y = block_middle_y;
    block_vision_middle[0] = {0.0, 0.0};

    block_vision_middle[1] = {x, y - Block_Size};

    block_vision_middle[2] = {x, y};

    block_vision_middle[3] = {x, y + Block_Size};

    block_vision_middle[4] = {x + Block_Size, y - Block_Size};

    block_vision_middle[5] = {x + Block_Size, y};

    block_vision_middle[6] = {x + Block_Size, y + Block_Size};

    block_vision_middle[7] = {x + Block_Size * 2.0f, y - Block_Size};

    block_vision_middle[8] = {x + Block_Size * 2.0f, y};

    block_vision_middle[9] = {x + Block_Size * 2.0f, y + Block_Size};

    block_vision_middle[10] = {x + Block_Size * 3.0f, y - Block_Size};

    block_vision_middle[11] = {x + Block_Size * 3.0f, y};

    block_vision_middle[12] = {x + Block_Size * 3.0f, y + Block_Size};

    block_vision_middle[13] = {x + Block_Size * 4.0f, y - Block_Size};

    block_vision_middle[14] = {x + Block_Size * 4.0f, y};

    block_vision_middle[15] = {x + Block_Size * 4.0f, y + Block_Size};
}

/* ===================== 内部工具函数 ===================== */

/**
 * @brief  从字节流中解析一个有符号浮点数（轻量级，不依赖标准库 atof）
 *
 * 支持格式：[-][整数部分][.小数部分]，遇到非数字字符即停止。
 * 解析结束后 *pp 自动后移到下一个待解析位置。
 *
 * @param  pp   当前解析位置的二级指针，解析后自动前进
 * @param  end  字节流末尾（不含）
 * @return 解析得到的 float 值
 */
static float fast_atof(const uint8_t **pp, const uint8_t *end)
{
    const uint8_t *p = *pp;

    /* 处理负号 */
    float sign = 1.0f;
    if (p < end && *p == '-')
    {
        sign = -1.0f;
        ++p;
    }

    /* 整数部分 */
    float v = 0.0f;
    while (p < end && *p >= '0' && *p <= '9')
    {
        v = v * 10.0f + static_cast<float>(*p - '0');
        ++p;
    }

    /* 小数部分 */
    if (p < end && *p == '.')
    {
        ++p;
        float frac = 0.1f;
        while (p < end && *p >= '0' && *p <= '9')
        {
            v += static_cast<float>(*p - '0') * frac;
            frac *= 0.1f;
            ++p;
        }
    }

    *pp = p;
    return v * sign;
}

static int fast_atoi_field(const uint8_t **pp, const uint8_t *end)
{
    return static_cast<int>(fast_atof(pp, end));
}

static uint8_t clamp_uint8_field(int value)
{
    if (value <= 0)
    {
        return 0U;
    }

    if (value >= 255)
    {
        return 255U;
    }

    return static_cast<uint8_t>(value);
}

static uint8_t vision_int_list_equal(const int *left, uint8_t left_count, const int *right, uint8_t right_count)
{
    if (left_count != right_count)
        return 0U;

    for (uint8_t i = 0U; i < left_count; ++i)
    {
        if (left[i] != right[i])
            return 0U;
    }

    return 1U;
}

static uint8_t vision_plan_is_same(const int *commands,
                                   uint8_t command_count,
                                   const int *blocks,
                                   uint8_t block_count)
{
    if (vision_last_plan_valid == 0U)
        return 0U;

    if (vision_int_list_equal(commands, command_count, vision_last_command_plan, vision_last_command_count) == 0U)
        return 0U;

    return vision_int_list_equal(blocks, block_count, vision_last_block_plan, vision_last_block_count);
}

static void vision_save_last_plan(const int *commands,
                                  uint8_t command_count,
                                  const int *blocks,
                                  uint8_t block_count)
{
    for (uint8_t i = 0U; i < command_count; ++i)
    {
        vision_last_command_plan[i] = commands[i];
    }
    vision_last_command_count = command_count;

    for (uint8_t i = 0U; i < block_count; ++i)
    {
        vision_last_block_plan[i] = blocks[i];
    }
    vision_last_block_count = block_count;
    vision_last_plan_valid = 1U;
}

static void vision_load_new_plan(const int *commands,
                                 uint8_t command_count,
                                 const int *blocks,
                                 uint8_t block_count)
{
    vision_command_clear();
    vision_block_clear();

    for (uint8_t i = 0U; i < command_count; ++i)
    {
        (void)vision_command_push(commands[i]);
    }

    for (uint8_t i = 0U; i < block_count; ++i)
    {
        (void)vision_block_push(blocks[i]);
    }

    vision_save_last_plan(commands, command_count, blocks, block_count);
}

/* 向视觉指令环形队列尾部压入一条 B 指令 */
uint8_t vision_command_push(int cmd)
{
    /* 计算 tail 前进一步后的位置，取模实现环形回绕 */
    const uint8_t next_tail = static_cast<uint8_t>((vision_command_tail + 1U) % VISION_COMMAND_QUEUE_SIZE);
    /* 若 next_tail 追上 head，说明队列已满，拒绝写入 */
    if (next_tail == vision_command_head)
        return 0U;

    /* 在当前 tail 位置写入 B 值，然后推进 tail */
    vision_command_queue[vision_command_tail] = cmd;
    vision_command_tail = next_tail;
    return 1U;
}

/* 从视觉指令环形队列头部取出一条 B 指令 */
uint8_t vision_command_pop(int *out)
{
    /* 输出指针为空，或 head == tail 表示队列空，无数据可取 */
    if (out == nullptr || vision_command_head == vision_command_tail)
        return 0U;

    /* 读取 head 位置的 B 值，然后推进 head */
    *out = vision_command_queue[vision_command_head];
    vision_command_head = static_cast<uint8_t>((vision_command_head + 1U) % VISION_COMMAND_QUEUE_SIZE);
    vision_plan_locked = 1U;
    return 1U;
}

/* 查看队头视觉指令但不弹出，用于路线层判断下一步动作。 */
uint8_t vision_command_peek(int *out)
{
    if (out == nullptr || vision_command_head == vision_command_tail)
        return 0U;

    *out = vision_command_queue[vision_command_head];
    return 1U;
}

/* 检查队列中是否有待处理的视觉指令：head != tail 即非空 */
uint8_t vision_command_has_pending(void)
{
    return (vision_command_head != vision_command_tail) ? 1U : 0U;
}

/* 清空整个视觉指令队列，重置 head 和 tail 到初始位置 */
void vision_command_clear(void)
{
    vision_command_head = 0U;
    vision_command_tail = 0U;
    vision_last_plan_valid = 0U;
    vision_plan_locked = 0U;
}

/* ===================== 方块队列 ===================== */

/* 向方块队列尾部压入一个值 */
uint8_t vision_block_push(int val)
{
    const uint8_t next_tail = static_cast<uint8_t>((vision_block_tail + 1U) % VISION_BLOCK_QUEUE_SIZE);
    if (next_tail == vision_block_head)
        return 0U;

    vision_block_queue[vision_block_tail] = val;
    vision_block_tail = next_tail;
    return 1U;
}

/* 从方块队列头部取出一个值 */
uint8_t vision_block_pop(int *out)
{
    if (out == nullptr || vision_block_head == vision_block_tail)
        return 0U;

    *out = vision_block_queue[vision_block_head];
    vision_block_head = static_cast<uint8_t>((vision_block_head + 1U) % VISION_BLOCK_QUEUE_SIZE);
    return 1U;
}

/* 检查方块队列中是否有待处理数据 */
uint8_t vision_block_has_pending(void)
{
    return (vision_block_head != vision_block_tail) ? 1U : 0U;
}

/* 清空方块队列 */
void vision_block_clear(void)
{
    vision_block_head = 0U;
    vision_block_tail = 0U;
    vision_last_plan_valid = 0U;
    vision_plan_locked = 0U;
}

void vision_plan_mark_consumed_if_empty(void)
{
    if (vision_command_has_pending() == 0U && vision_block_has_pending() == 0U)
    {
        vision_plan_locked = 0U;
    }
}

/**
 * @brief  解析视觉帧
 *
 * 帧格式：S,<exec>,<x>,<y>,<yaw>,C,<action...>,B,<block...>,A,<release>,<claw_vertical>,<unused>,P,<if_go>,<can_up>,E
 * - exec：是否前往第二区标志（整数）
 * - x、y、yaw：坐标和角度（浮点数）
 * - C：后接不定长梅花林动作，存入动作队列
 * - B：后接不定长梅花林格子编号，存入方块队列
 * - A：后接是否松手、夹爪上下和无用标定位三个整数
 * - P：后接 if_go 继续标志和 CAN 上升标定位
 *
 * @param  data  输入字节数组
 * @param  len   数组长度
 * @param  out   输出结构体指针
 * @return 1 成功，0 失败
 */
int parse_vision_frame_computer(uint8_t *data, uint16_t len, VisionData_t *out)
{
    if (data == nullptr || out == nullptr || len == 0)
        return 0;

    VisionData_t parsed = {};
    int parsed_commands[VISION_COMMAND_QUEUE_SIZE];
    uint8_t parsed_command_count = 0U;
    int parsed_blocks[VISION_BLOCK_QUEUE_SIZE];
    uint8_t parsed_block_count = 0U;

    /* 查找帧头 'S' */
    const uint8_t *s = nullptr;
    for (uint16_t i = 0; i < len; ++i)
    {
        if (data[i] == 'S')
        {
            s = &data[i];
            break;
        }
    }
    if (!s)
        return 0;

    /* 查找帧尾 'E' */
    const uint8_t *e = nullptr;
    for (const uint8_t *p = s + 1; p < data + len; ++p)
    {
        if (*p == 'E')
        {
            e = p;
            break;
        }
    }
    if (!e)
        return 0;

    const uint8_t *p = s + 1;

    /* 跳过 'S' 后的第一个逗号 */
    if (p < e && *p == ',')
        ++p;
    else
        return 0;

    /* 解析 exec（整数） */
    parsed.exec = static_cast<int>(fast_atof(&p, e));
    if (p < e && *p == ',')
        ++p;
    else
        return 0;

    /* 解析 x_diff */
    parsed.x_diff = fast_atof(&p, e);
    if (p < e && *p == ',')
        ++p;
    else
        return 0;

    /* 解析 y_diff */
    parsed.y_diff = fast_atof(&p, e);
    if (p < e && *p == ',')
        ++p;
    else
        return 0;

    /* 解析 angle_x (yaw) */
    parsed.angle_x = fast_atof(&p, e);

    /* 跳过逗号，准备解析 C 后面的动作 */
    if (p >= e || *p != ',')
        return 0;
    ++p;

    /* 期望 'C' 标记动作序列开始 */
    if (p >= e || *p != 'C')
        return 0;
    ++p;

    /* 解析 C 后面的不定长动作数字，压入动作队列 */
    while (p < e)
    {
        if (*p == ',')
        {
            ++p;
            /* 如果逗号后面是 'B'，跳出动作循环，进入方块解析 */
            if (p < e && *p == 'B')
                break;
            /* 否则继续解析动作数字 */
            if (p >= e)
                return 0;
            int action = static_cast<int>(fast_atof(&p, e));
            if (parsed_command_count >= (VISION_COMMAND_QUEUE_SIZE - 1U))
                return 0;
            parsed_commands[parsed_command_count] = action;
            ++parsed_command_count;
        }
        else
        {
            return 0;
        }
    }

    /* 现在 p 应该指向 'B' */
    if (p >= e || *p != 'B')
        return 0;
    ++p;

    /* 解析 B 后面的不定长方块编号，压入方块队列 */
    while (p < e)
    {
        if (*p != ',')
            return 0;

        ++p;
        /* A 标记表示格子序列结束 */
        if (p < e && *p == 'A')
            break;
        if (p >= e)
            return 0;

        int block = static_cast<int>(fast_atof(&p, e));
        if (parsed_block_count >= (VISION_BLOCK_QUEUE_SIZE - 1U))
            return 0;
        parsed_blocks[parsed_block_count] = block;
        ++parsed_block_count;
    }

    /* 解析 A 后面的三个固定标定位 */
    if (p >= e || *p != 'A')
        return 0;
    ++p;

    if (p >= e || *p != ',')
        return 0;
    ++p;
    if (p >= e)
        return 0;
    parsed.release_flag = static_cast<int>(fast_atof(&p, e));

    if (p >= e || *p != ',')
        return 0;
    ++p;
    if (p >= e)
        return 0;
    parsed.claw_vertical_flag = static_cast<int>(fast_atof(&p, e));

    if (p >= e || *p != ',')
        return 0;
    ++p;
    if (p >= e)
        return 0;
    parsed.claw_vertical_adjust_count = static_cast<int>(fast_atof(&p, e));

    if (p >= e || *p != ',')
        return 0;
    ++p;
    if (p >= e || *p != 'P')
        return 0;
    ++p;

    if (p >= e || *p != ',')
        return 0;
    ++p;
    if (p >= e)
        return 0;
    parsed.if_go = static_cast<int>(fast_atoi_field(&p, e));

    if (p >= e || *p != ',')
        return 0;
    ++p;
    if (p >= e)
        return 0;
    parsed.can_up = static_cast<int16_t>(fast_atoi_field(&p, e));

    if (p >= e || *p != ',')
        return 0;
    ++p;
    if (p != e)
        return 0;

    if (vision_plan_locked == 0U &&
        vision_plan_is_same(parsed_commands, parsed_command_count, parsed_blocks, parsed_block_count) == 0U)
    {
        vision_load_new_plan(parsed_commands, parsed_command_count, parsed_blocks, parsed_block_count);
    }

    g_ftm_minipc_claw_release_cmd = clamp_uint8_field(parsed.release_flag);
    g_ftm_minipc_lift_dock_adjust_cmd = clamp_uint8_field(parsed.claw_vertical_flag);
    g_ftm_minipc_claw_vertical_adjust_count = static_cast<int16_t>(parsed.claw_vertical_adjust_count);
    ++g_ftm_minipc_control_seq;

    *out = parsed;
    return 1;
}

#ifndef VISION_FRAME_PARSER_HOST_TEST
/**
 * @brief  通过 USB 向上位机发送位置信息
 *
 * 帧格式（ASCII 文本）：L,behavior,posi,x,y,yawE
 * 使用静态缓冲区 posi_conputer，通过 USB CDC 发送。
 *
 */
void send_position_to_pc(int16_t behaivor, uint8_t p_diff, float X_diff, float Y_diff, float yaw)
{
    /* 格式化为 L,x,y,aE 文本帧，%.2f 保留两位小数 */
    int n = snprintf(reinterpret_cast<char *>(posi_conputer),
                     sizeof(posi_conputer),
                     "L,%d,%d,%.2f,%.2f,%.2f,Y",
                     behaivor, p_diff, X_diff, Y_diff, yaw);

    if (n > 0 && n < static_cast<int>(sizeof(posi_conputer)))
        MiniPC_Transmit_Info(posi_conputer, static_cast<uint16_t>(n));
}

/* ===================== FreeRTOS 任务入口 ===================== */
int Flag1 = 0;

/**
 * @brief  USART 任务主循环（1ms 周期）
 *
 * 职责：
 *   1. 更新 AS5047 磁编码器和 DT35 激光传感器
 *   2. 解析 USB 串口收到的视觉帧
 */
uint16_t flag_bottom = 0;
extern "C" void usart_task(void *argument)
{
    // as5047.init(&hspi1);
    dt35.init(&hspi3);
    Block_claulate_Middle();
    for (;;)
    {
        if (Yellow == 1)
        {
            conbat_t.conbat_start = 1;
        }
        if (Blue == 1)
        {
            g_ftm_main_state = 3; // 先执行视觉置零
            flag_bottom = 1;
        }
        if (Green == 1)
        {
            static uint8_t red_step = 1; // 1=待触发9, 2=已完成
            if (red_step == 1)
            {
                g_ftm_main_state = 10; // 再执行完整自动流程
                red_step = 2;          // 不再触发
            }
        }
        if (Orange == 1)
        {
            conbat_t.conbat_start = 2;
        }

        if (Red == 1)
        {
            kfs_num = 1;
        }

        if (Whihe == 1)
        {
        }

        if (Red2 == 0)
        {
        }

        if (Blue2 == 0)
        {
        }

        /* 更新传感器数据 */
        // as5047.updata();
        dt35.update();

        /* USB 视觉帧处理：每轮主循环只处理一次，处理完立即清零缓冲区 */
        /* 第一步：检查是否有数据且包含帧头 'S'，没有则跳过解析 */
        if (usb_rx_idx > 0 && memchr(data_usb, 'S', usb_rx_idx) != nullptr)
        {
            /* 第二步：帧头存在，尝试完整解析（使用实际数据长度而非整个缓冲区大小）；
               成功（返回1）则更新 vision 结构体 */
            if (parse_vision_frame_computer(data_usb, usb_rx_idx, &vision) == 1)
            {
                /* 第三步：解析过程中若发现非零 B 字段，会被压入动作队列；
                   此处检查队列是否有待处理指令，有则通知路由任务 */
                route_t.flag_vision = vision_command_has_pending();
            }
            /* 第四步：无论解析成功与否，都清零缓冲区和写指针，防止下一轮重复处理旧数据 */
            memset(data_usb, 0, sizeof(data_usb));
            usb_rx_idx = 0;
        }

        if (Flag1 == 1)
        {
            send_position_to_pc(0, 1, 0, 0, 0.0f);
            Flag1 = 0;
        }

        osDelay(1);
    }
}
#endif
