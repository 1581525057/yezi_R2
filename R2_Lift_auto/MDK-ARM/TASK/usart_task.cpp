#include "usart_task.h"
#include "usart.h"
#include "spi.h"
#include "cmsis_os.h"
#include "MiniPC.h"
#include "main.h"
#include "PID.h"
#include "dji_motor.h"
#include <string.h>
#include "dm_imu.h"
#include "laser_distance.h"
#include "AS5047.h"
#include "DT35.h"
/* 内部函数前向声明 */
static void SendCurveArray_Float_LORA(const float *data, uint16_t len);
static void SendCurveArray_Float(const float *data, uint16_t len);
static float fast_atof_simple_computer(const uint8_t **pp, const uint8_t *end);

/* LoRa 发送静态缓冲区：float 数据区 + 4 字节结束标志 */
static uint8_t lora_tx_buf[CURVE_TX_MAX_FLOATS * 4 + 4];

uint8_t data_usb[30];  /* USB 串口接收缓冲区 */
uint8_t data_lora[30]; /* LoRa 串口接收缓冲区 */

/**
 * @brief USART 任务入口（FreeRTOS 任务）
 *        每 1ms 将 IMU yaw 角和激光测距数据通过 LoRa 发送出去，用于上位机曲线调试。
 */
extern "C" void usart_task(void *argument)
{
    as5047.init(&hspi1);
    dt35.init(&hspi3);
    for (;;) {
        /* 组装调试数据：[0]=CH0电压(V), [1]=CH1电压(V), [2]=CH0距离(mm), [3]=CH1距离(mm) */
        float data_debug[4] = {
            dt35.ch0.voltage_V,
            dt35.ch1.voltage_V,
            dt35.ch0.distance_mm,
            dt35.ch1.distance_mm
        };

        SendCurveArray_Float_LORA(data_debug, 4);

        as5047.updata();
        dt35.update();

        osDelay(1); /* 1ms 周期 */
    }
}

/**
 * @brief 通过 LoRa（huart10）以 DMA 方式发送 float 数组
 * @param data  float 数组指针
 * @param len   float 个数（不超过 CURVE_TX_MAX_FLOATS）
 *
 * 帧格式：[float 原始字节 × len] [END_0][END_1][END_2][END_3]
 */
static void SendCurveArray_Float_LORA(const float *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return;

    if (len > CURVE_TX_MAX_FLOATS)
        return;

    /* 将 float 数组的原始字节拷贝到静态缓冲区 */
    memcpy(lora_tx_buf, data, len * 4);

    /* 追加 4 字节结束标志，供接收端帧同步 */
    lora_tx_buf[len * 4 + 0] = CURVE_END_0;
    lora_tx_buf[len * 4 + 1] = CURVE_END_1;
    lora_tx_buf[len * 4 + 2] = CURVE_END_2;
    lora_tx_buf[len * 4 + 3] = CURVE_END_3;

    /* DMA 发送，非阻塞 */
    HAL_UART_Transmit_DMA(&huart10, lora_tx_buf, len * 4 + 4);
}

/**
 * @brief 通过 MiniPC 串口发送不定长 float 数组（栈上 VLA 缓冲）
 * @param data  float 数组指针
 * @param len   float 个数
 *
 * 帧格式：[float 原始字节 × len] [END_0][END_1][END_2][END_3]
 * 注意：buf 为 VLA（变长数组），需 C99 支持；len 过大会消耗较多栈空间。
 */
static void SendCurveArray_Float(const float *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return;

    uint16_t total = (uint16_t)(len * 4u + 4u); /* 总字节数 */
    uint8_t buf[4u * (uint32_t)len + 4u];       /* VLA：float 区 + 结束标志区 */

    /* 将 float 数组原始字节拷入缓冲 */
    memcpy(buf, data, (size_t)len * 4u);

    /* 追加 4 字节结束标志 */
    buf[len * 4u + 0u] = CURVE_END_0;
    buf[len * 4u + 1u] = CURVE_END_1;
    buf[len * 4u + 2u] = CURVE_END_2;
    buf[len * 4u + 3u] = CURVE_END_3;

    MiniPC_Transmit_Info(buf, total);
}

/**
 * @brief 从字节流中快速解析一个有符号浮点数（不依赖标准库 atof）
 * @param pp  指向当前解析位置的指针（解析后自动向后移动）
 * @param end 字节流末尾（不含）
 * @return    解析到的 float 值
 *
 * 支持格式：[-][整数部分][.小数部分]，遇到非数字字符停止。
 */
static float fast_atof_simple(const uint8_t **pp, const uint8_t *end)
{
    const uint8_t *p = *pp;

    /* 处理负号 */
    int sign = 1;
    if (p < end && *p == '-') {
        sign = -1;
        p++;
    }

    /* 解析整数部分 */
    float v = 0.0f;
    while (p < end && *p >= '0' && *p <= '9') {
        v = v * 10.0f + (float)(*p - '0');
        p++;
    }

    /* 解析小数部分 */
    if (p < end && *p == '.') {
        p++;
        float base = 0.1f;
        while (p < end && *p >= '0' && *p <= '9') {
            v += (float)(*p - '0') * base;
            base *= 0.1f;
            p++;
        }
    }

    *pp = p;
    return v * (float)sign;
}

/**
 * @brief 解析 data[0..len-1] 中的第一帧 S...E，提取逗号分隔的 float 列表
 * @param data    输入字节数组
 * @param len     数组长度
 * @param out     输出 float 数组
 * @param out_max out 数组最大容量
 * @return 成功解析到的 float 个数，0 表示失败
 *
 * 帧格式：S<f0>,<f1>,...,<fn>E
 */
int parse_SE_simple(uint8_t *data, uint16_t len, float out[], uint8_t out_max)
{
    /* 初始化输出数组 */
    for (uint8_t i = 0; i < out_max; i++)
        out[i] = 0.0f;

    const uint8_t *s = 0, *e = 0;

    /* 查找帧头 'S' */
    for (uint16_t i = 0; i < len; i++) {
        if (data[i] == 'S') {
            s = &data[i];
            break;
        }
    }
    if (!s)
        return 0; /* 未找到帧头 */

    /* 查找帧尾 'E' */
    for (const uint8_t *p = s + 1; p < data + len; p++) {
        if (*p == 'E') {
            e = p;
            break;
        }
    }
    if (!e)
        return 0; /* 未找到帧尾 */

    uint8_t n        = 0;
    const uint8_t *p = s + 1;

    /* 依次解析逗号分隔的 float 值 */
    while (p < e && n < out_max) {
        out[n++] = fast_atof_simple(&p, e);
        if (p < e && *p == ',')
            p++; /* 跳过分隔符 */
        else
            break;
    }

    return n;
}

/**
 * @brief 从字节流中快速解析一个有符号浮点数（computer 版本，逻辑同 fast_atof_simple）
 * @param pp  指向当前解析位置的指针（解析后自动向后移动）
 * @param end 字节流末尾（不含）
 * @return    解析到的 float 值
 */
static float fast_atof_simple_computer(const uint8_t **pp, const uint8_t *end)
{
    const uint8_t *p = *pp;

    /* 处理负号 */
    int sign = 1;
    if (p < end && *p == '-') {
        sign = -1;
        p++;
    }

    /* 解析整数部分 */
    float v = 0.0f;
    while (p < end && *p >= '0' && *p <= '9') {
        v = v * 10.0f + (float)(*p - '0');
        p++;
    }

    /* 解析小数部分 */
    if (p < end && *p == '.') {
        p++;
        float base = 0.1f;
        while (p < end && *p >= '0' && *p <= '9') {
            v += (float)(*p - '0') * base;
            base *= 0.1f;
            p++;
        }
    }

    *pp = p;
    return v * (float)sign;
}

/**
 * @brief 解析视觉帧，格式：S,<x_diff>,<y_diff>,<angle_x>E
 * @param data  输入字节数组
 * @param len   数组长度
 * @param out   输出结构体（x_diff、y_diff、angle_x）
 * @return 1=成功, 0=失败（指针为空、帧不完整或格式错误）
 */
int parse_vision_frame_computer(uint8_t *data, uint16_t len, VisionData_t *out)
{
    if (data == 0 || out == 0 || len == 0)
        return 0;

    /* 清零输出 */
    out->x_diff  = 0.0f;
    out->y_diff  = 0.0f;
    out->angle_x = 0.0f;

    const uint8_t *s = 0;
    const uint8_t *e = 0;

    /* 查找帧头 'S' */
    for (uint16_t i = 0; i < len; i++) {
        if (data[i] == 'S') {
            s = &data[i];
            break;
        }
    }
    if (!s)
        return 0;

    /* 查找帧尾 'E' */
    for (const uint8_t *p = s + 1; p < data + len; p++) {
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
        p++;
    else
        return 0;

    /* 解析 x_diff */
    out->x_diff = fast_atof_simple(&p, e);
    if (p < e && *p == ',')
        p++;
    else
        return 0;

    /* 解析 y_diff */
    out->y_diff = fast_atof_simple(&p, e);
    if (p < e && *p == ',')
        p++;
    else
        return 0;

    /* 解析 angle_x */
    out->angle_x = fast_atof_simple(&p, e);

    return 1;
}
