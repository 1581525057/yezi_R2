#include "usart_task.h"
#include "usart.h"
#include "spi.h"
#include "cmsis_os.h"
#include "main.h"
// #include "AS5047.h"
#include "DT35.h"
#include "MiniPC.h"
#include <string.h>
#include <stdio.h>
#include "bsp_dwt.h"
#include "chassis_task.h"
#include "PID.h"
#include "omni_chassis.h"
#include "route_task.h"
/* ========================== 全局变量 ========================== */

/* USB 串口接收缓冲区 */
uint8_t data_usb[USB_RX_BUFFER_SIZE];
static uint8_t posi_conputer[40]; // usb单片机发送给电脑
/* 视觉数据，由 parse_vision_frame_computer() 写入 */
VisionData_t vision;
pid_data yaw_data;

/* ========================== 静态变量 ========================== */

/* LoRa 发送缓冲区：float 数据区 + 4 字节帧结束标志 */
static uint8_t lora_tx_buf[CURVE_TX_MAX_FLOATS * 4 + 4];
static uint8_t lora_rx_buf[40];

/* 动作指令环形队列：缓存视觉帧中 C 后面的不定长动作 */
#define VISION_COMMAND_QUEUE_SIZE 16U
int vision_command_queue[VISION_COMMAND_QUEUE_SIZE];
uint8_t vision_command_head = 0U;
uint8_t vision_command_tail = 0U;

/* 方块指令环形队列：缓存视觉帧中 B 后面的不定长方块编号 */
#define VISION_BLOCK_QUEUE_SIZE 16U
int vision_block_queue[VISION_BLOCK_QUEUE_SIZE];
uint8_t vision_block_head = 0U;
uint8_t vision_block_tail = 0U;
// 每个方块的中心坐标
Block_Vision block_vision_middle[10];
float block_middle_x = 3.47f;
float block_middle_y = 1.56f;
// 每个方块的爬升坐标
Block_Vision block_vision_climb[10];
float block_climb_x = 3.16f;
float block_climb_y = 1.57f;

// 通过第2个方块来计算得到其他8个的坐标位置
static void Block_claulate_Middle(void)
{
    float Block_Size       = 1.2f;
    float x                = block_middle_x;
    float y                = block_middle_y;
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
}
static void Block_claulate_Climb(void)
{
    float Block_Size      = 1.2f;
    float x               = block_climb_x;
    float y               = block_climb_y;
    block_vision_climb[0] = {0.0, 0.0};

    block_vision_climb[1] = {x, y - Block_Size};

    block_vision_climb[2] = {x, y};

    block_vision_climb[3] = {x, y + Block_Size};

    block_vision_climb[4] = {x + Block_Size, y - Block_Size};

    block_vision_climb[5] = {x + Block_Size, y - Block_Size};

    block_vision_climb[6] = {x + Block_Size, y + Block_Size};

    block_vision_climb[7] = {x + Block_Size * 2.0f, y - Block_Size};

    block_vision_climb[8] = {x + Block_Size * 2.0f, y};

    block_vision_climb[9] = {x + Block_Size * 2.0f, y + Block_Size};
}

static void lora_rx_dma_init(void)
{
    memset(lora_rx_buf, 0, sizeof(lora_rx_buf));
    HAL_UART_Receive_DMA(&huart8, lora_rx_buf, sizeof(lora_rx_buf));
    if (huart8.hdmarx != nullptr) {
        __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
    }
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
    if (p < end && *p == '-') {
        sign = -1.0f;
        ++p;
    }

    /* 整数部分 */
    float v = 0.0f;
    while (p < end && *p >= '0' && *p <= '9') {
        v = v * 10.0f + static_cast<float>(*p - '0');
        ++p;
    }

    /* 小数部分 */
    if (p < end && *p == '.') {
        ++p;
        float frac = 0.1f;
        while (p < end && *p >= '0' && *p <= '9') {
            v += static_cast<float>(*p - '0') * frac;
            frac *= 0.1f;
            ++p;
        }
    }

    *pp = p;
    return v * sign;
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
    vision_command_tail                       = next_tail;
    return 1U;
}

/* 从视觉指令环形队列头部取出一条 B 指令 */
uint8_t vision_command_pop(int *out)
{
    /* 输出指针为空，或 head == tail 表示队列空，无数据可取 */
    if (out == nullptr || vision_command_head == vision_command_tail)
        return 0U;

    /* 读取 head 位置的 B 值，然后推进 head */
    *out                = vision_command_queue[vision_command_head];
    vision_command_head = static_cast<uint8_t>((vision_command_head + 1U) % VISION_COMMAND_QUEUE_SIZE);
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
}

/* ===================== 方块队列 ===================== */

/* 向方块队列尾部压入一个值 */
uint8_t vision_block_push(int val)
{
    const uint8_t next_tail = static_cast<uint8_t>((vision_block_tail + 1U) % VISION_BLOCK_QUEUE_SIZE);
    if (next_tail == vision_block_head)
        return 0U;

    vision_block_queue[vision_block_tail] = val;
    vision_block_tail                     = next_tail;
    return 1U;
}

/* 从方块队列头部取出一个值 */
uint8_t vision_block_pop(int *out)
{
    if (out == nullptr || vision_block_head == vision_block_tail)
        return 0U;

    *out              = vision_block_queue[vision_block_head];
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
}

/**
 * @brief  解析视觉帧
 *
 * 帧格式：S,<exec>,<x>,<y>,<yaw>,C,<action...>,B,<block...>,E
 * - exec：执行程（整数）
 * - x, y, yaw：坐标和角度（浮点数）
 * - C：固定字母，后面是不定长的动作数字，存入动作队列
 * - B：固定字母，后面是不定长的方块编号，存入方块队列
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

    VisionData_t parsed = {0, 0.0f, 0.0f, 0.0f, 0};

    /* 查找帧头 'S' */
    const uint8_t *s = nullptr;
    for (uint16_t i = 0; i < len; ++i) {
        if (data[i] == 'S') {
            s = &data[i];
            break;
        }
    }
    if (!s)
        return 0;

    /* 查找帧尾 'E' */
    const uint8_t *e = nullptr;
    for (const uint8_t *p = s + 1; p < data + len; ++p) {
        if (*p == 'E') {
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

    *out = parsed;

    /* 如果已经到帧尾，直接返回 */
    if (p >= e)
        return 1;

    /* 跳过逗号，准备解析 C 后面的动作 */
    if (*p != ',')
        return 0;
    ++p;

    /* 期望 'C' 标记动作序列开始 */
    if (p >= e || *p != 'C')
        return 0;
    ++p;

    /* 解析 C 后面的不定长动作数字，压入动作队列 */
    while (p < e) {
        if (*p == ',') {
            ++p;
            /* 如果逗号后面是 'B'，跳出动作循环，进入方块解析 */
            if (p < e && *p == 'B')
                break;
            /* 否则继续解析动作数字 */
            if (p >= e)
                return 0;
            int action = static_cast<int>(fast_atof(&p, e));
            if (action != 0)
                vision_command_push(action);
        } else {
            return 0;
        }
    }

    /* 现在 p 应该指向 'B' */
    if (p >= e || *p != 'B')
        return 0;
    ++p;

    /* 解析 B 后面的不定长方块编号，压入方块队列 */
    while (p < e) {
        if (*p == ',') {
            ++p;
            /* 逗号后面可能是数字或者直接到 E */
            if (p >= e)
                return 0;
            int block = static_cast<int>(fast_atof(&p, e));
            if (block != 0)
                vision_block_push(block);
        } else {
            return 0;
        }
    }

    return 1;
}

int parse_vision_frame_pid(uint8_t *data, uint16_t len, pid_data *out)
{
    if (data == nullptr || out == nullptr || len == 0)
        return -1;

    pid_data parsed = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    const uint8_t *s = nullptr;
    for (uint16_t i = 0; i < len; ++i) {
        if (data[i] == 'S') {
            s = &data[i];
            break;
        }
    }
    if (!s)
        return -1;

    const uint8_t *e = nullptr;
    for (const uint8_t *p = s + 1; p < data + len; ++p) {
        if (*p == 'E') {
            e = p;
            break;
        }
    }
    if (!e)
        return -1;

    const uint8_t *p = s + 1;
    if (p < e && *p == ',')
        ++p;
    else
        return -1;

    parsed.kp = fast_atof(&p, e);
    if (p < e && *p == ',')
        ++p;
    else
        return -1;

    parsed.ki = fast_atof(&p, e);
    if (p < e && *p == ',')
        ++p;
    else
        return -1;

    parsed.kd = fast_atof(&p, e);
    if (p < e && *p == ',')
        ++p;
    else
        return -1;

    parsed.limit_inter = fast_atof(&p, e);
    if (p < e && *p == ',')
        ++p;
    else
        return -1;

    parsed.outputmax = fast_atof(&p, e);
    if (p != e)
        return -1;

    *out = parsed;
    return 0;
}

/* ===================== LoRa 发送 ===================== */

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

/**
 * @brief  通过 LoRa（huart10）以 DMA 方式发送 float 数组
 *
 * 帧格式：[float 原始字节 × len] [END_0][END_1][END_2][END_3]
 * 使用静态缓冲区 lora_tx_buf，非阻塞 DMA 发送。
 *
 * @param  data  float 数组指针
 * @param  len   float 个数（不超过 CURVE_TX_MAX_FLOATS）
 */
static void send_curve_lora(const float *data, uint16_t len)
{
    if (data == nullptr || len == 0 || len > CURVE_TX_MAX_FLOATS)
        return;

    /* 将 float 数组的原始字节拷贝到静态缓冲区 */
    memcpy(lora_tx_buf, data, len * 4);

    /* 追加 4 字节帧结束标志 */
    const uint16_t offset   = len * 4;
    lora_tx_buf[offset + 0] = CURVE_END_0;
    lora_tx_buf[offset + 1] = CURVE_END_1;
    lora_tx_buf[offset + 2] = CURVE_END_2;
    lora_tx_buf[offset + 3] = CURVE_END_3;

    HAL_UART_Transmit_DMA(&huart8, lora_tx_buf, offset + 4);
}

/* ===================== FreeRTOS 任务入口 ===================== */
int Flag1 = 0;

/**
 * @brief  USART 任务主循环（1ms 周期）
 *
 * 职责：
 *   1. 更新 AS5047 磁编码器和 DT35 激光传感器
 *   2. 将 DT35 调试数据通过 LoRa 发送给上位机
 *   3. 解析 USB 串口收到的视觉帧
 */

extern "C" void usart_task(void *argument)
{
    // as5047.init(&hspi1);
    dt35.init(&hspi3);
    lora_rx_dma_init();
    Block_claulate_Middle();
    Block_claulate_Climb();
    for (;;) {
        /* 更新传感器数据 */
        // as5047.updata();
        dt35.update();

        /* 组装调试数据并通过 LoRa 发送 */
        float debug_data[4] = {
            0, omni_chassis.now.rpm[1], omni_chassis.now.rpm[2], omni_chassis.now.rpm[3]};
        send_curve_lora(debug_data, 4);

        parse_vision_frame_pid(lora_rx_buf, sizeof(lora_rx_buf), &yaw_data);

        /* USB 视觉帧处理：每轮主循环只处理一次，处理完立即清零缓冲区 */
        /* 第一步：快速扫描缓冲区是否包含帧头 'S'，没有则跳过解析 */
        if (memchr(data_usb, 'S', sizeof(data_usb)) != nullptr) {
            /* 第二步：帧头存在，尝试完整解析；成功（返回1）则更新 vision 结构体 */
            if (parse_vision_frame_computer(data_usb, sizeof(data_usb), &vision) == 1) {
                /* 第三步：解析过程中若发现非零 B 字段，会被压入动作队列；
                   此处检查队列是否有待处理指令，有则通知路由任务 */
                route_t.flag_vision = vision_command_has_pending();
            }
            /* 第四步：无论解析成功与否，都清零缓冲区，防止下一轮重复处理同一帧 */
            memset(data_usb, 0, sizeof(data_usb));
        }

        if (Flag1 == 1) {
            send_position_to_pc(0, 1, 0, 0, 0.0f);
            Flag1 = 0;
        }

        if (Flag1 == 2) {
            send_position_to_pc(1, 0, 0, 0, 0);
            Flag1 = 0;
        }

        if (Flag1 == 3) {
            dt35.init(&hspi3);
            Flag1 = 0;
        }
        osDelay(1);
    }
}
