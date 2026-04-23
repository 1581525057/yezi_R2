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

/* ========================== 全局变量 ========================== */

/* USB 串口接收缓冲区 */
uint8_t data_usb[30];

/* 视觉数据，由 parse_vision_frame_computer() 写入 */
VisionData_t vision;

/* ========================== 静态变量 ========================== */

/* LoRa 发送缓冲区：float 数据区 + 4 字节帧结束标志 */
static uint8_t lora_tx_buf[CURVE_TX_MAX_FLOATS * 4 + 4];
static uint8_t posi_conputer[40]; // 单片机发送给电脑

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
   
/**
 * @brief  解析视觉帧，提取三个字段写入 VisionData_t
 *
 * 帧格式：S,<x_diff>,<y_diff>,<angle_x>,<B>E
 * 注意帧头 'S' 后紧跟一个逗号，B 为整数。
 *
 * @param  data  输入字节数组
 * @param  len   数组长度
 * @param  out   输出结构体指针
 * @return 1 成功，0 失败（空指针、帧不完整、格式错误）
 */
int parse_vision_frame_computer(uint8_t *data, uint16_t len, VisionData_t *out)
{
    if (data == nullptr || out == nullptr || len == 0)
        return 0;

    /* 清零输出 */
    out->B       = 0;
    out->x_diff  = 0.0f;
    out->y_diff  = 0.0f;
    out->angle_x = 0.0f;

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

    /* 依次解析四个字段，每个字段之间以逗号分隔 */
    out->x_diff = fast_atof(&p, e);
    if (p < e && *p == ',')
        ++p;
    else
        return 0;

    out->y_diff = fast_atof(&p, e);
    if (p < e && *p == ',')
        ++p;
    else
        return 0;

    out->angle_x = fast_atof(&p, e);
    if (p < e && *p == ',')
        ++p;
    else
        return 0;

    /* B 是整数，用 fast_atof 解析后截断为 int */
    out->B = static_cast<int>(fast_atof(&p, e));

    return 1;
}

/* ===================== LoRa 发送 ===================== */

/**
 * @brief  通过 USB 向上位机发送位置信息
 *
 * 帧格式（ASCII 文本）：L,<x_diff>,<y_diff>,<angle_x>E
 * 使用静态缓冲区 posi_conputer，通过 USB CDC 发送。
 *
 */
static void send_position_to_pc(int16_t behaivor,uint8_t  p_diff, float X_diff, float Y_diff,float yaw)
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
    const uint16_t offset = len * 4;
    lora_tx_buf[offset + 0] = CURVE_END_0;
    lora_tx_buf[offset + 1] = CURVE_END_1;
    lora_tx_buf[offset + 2] = CURVE_END_2;
    lora_tx_buf[offset + 3] = CURVE_END_3;

    HAL_UART_Transmit_DMA(&huart10, lora_tx_buf, offset + 4);
}

/* ===================== FreeRTOS 任务入口 ===================== */
int Flag1 = 0;
uint8_t beh = 0;
uint8_t p_diff = 0;
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

    for (;;)
    {
        /* 更新传感器数据 */
        // as5047.updata();
        dt35.update();

        /* 组装调试数据并通过 LoRa 发送 */
        float debug_data[4] = {
            dt35.ch0.voltage_V,
            dt35.ch1.voltage_V,
            dt35.ch0.distance_mm,
            dt35.ch1.distance_mm,
        };
        send_curve_lora(debug_data, 4);

		 parse_vision_frame_computer(data_usb, sizeof(data_usb), &vision);

        /* 每 2 秒向上位机发送一次位置信息（2000ms / 1ms 周期 = 2000 次） */
        static uint16_t pc_send_cnt = 0;
        if (++pc_send_cnt >= 2000)
        {
            pc_send_cnt = 0;
            send_position_to_pc(beh, p_diff, 1.0f, 2.0f, 3.0f);
        }
			
		
      

        /* 解析上位机视觉帧 */
       

        osDelay(1);
    }
}
