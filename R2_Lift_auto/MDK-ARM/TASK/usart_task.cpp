#include "usart_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "MiniPC.h"
#include "main.h"
#include "PID.h"
#include "dji_motor.h"
#include <string.h>
#include "dm_imu.h"
#include "laser_distance.h"
static void SendCurveArray_Float_LORA(const float *data, uint16_t len);
static void SendCurveArray_Float(const float *data, uint16_t len);
static float fast_atof_simple_computer(const uint8_t **pp, const uint8_t *end);


static uint8_t lora_tx_buf[CURVE_TX_MAX_FLOATS * 4 + 4];

uint8_t data_usb[30];

uint8_t data_lora[30];

extern "C" void usart_task(void *argument)
{

    for (;;) {

        float data_debug[2] = {dm_imu.imu.yaw,laser.data.distance_m,};

        SendCurveArray_Float_LORA(data_debug, 2);

        osDelay(1);
    }
}

static void SendCurveArray_Float_LORA(const float *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return;

    if (len > CURVE_TX_MAX_FLOATS)
        return;

    memcpy(lora_tx_buf, data, len * 4);

    lora_tx_buf[len * 4 + 0] = CURVE_END_0;
    lora_tx_buf[len * 4 + 1] = CURVE_END_1;
    lora_tx_buf[len * 4 + 2] = CURVE_END_2;
    lora_tx_buf[len * 4 + 3] = CURVE_END_3;

     HAL_UART_Transmit_DMA(&huart10, lora_tx_buf, len * 4 + 4);
}

// 发送不定长 float 数据：data[0..len-1]
static void SendCurveArray_Float(const float *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return;

    // 总字节数 = len * 4 + 4(结束标志)
    // 注意：这是VLA(变长数组)，需要编译器支持C99；len很大时会占用较多栈
    uint16_t total = (uint16_t)(len * 4u + 4u);
    uint8_t buf[/*VLA*/ 4u * (uint32_t)len + 4u];

    // 拷贝 float 数组的原始字节到 buf
    memcpy(buf, data, (size_t)len * 4u);

    // 追加结束标志
    buf[len * 4u + 0u] = CURVE_END_0;
    buf[len * 4u + 1u] = CURVE_END_1;
    buf[len * 4u + 2u] = CURVE_END_2;
    buf[len * 4u + 3u] = CURVE_END_3;

    // 发出去
    MiniPC_Transmit_Info(buf, total);
}

static float fast_atof_simple(const uint8_t **pp, const uint8_t *end)
{
    const uint8_t *p = *pp;

    int sign = 1;
    if (p < end && *p == '-') {
        sign = -1;
        p++;
    }

    // 整数部分
    float v = 0.0f;
    while (p < end && *p >= '0' && *p <= '9') {
        v = v * 10.0f + (float)(*p - '0');
        p++;
    }

    // 小数部分
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
 * @brief 解析 data[0..len-1] 中的第一帧 S...E
 * @return 解析到的 float 个数（0=失败）
 */
int parse_SE_simple(uint8_t *data, uint16_t len, float out[], uint8_t out_max)
{
    // 关键：先清零
    for (uint8_t i = 0; i < out_max; i++)
        out[i] = 0.0f;

    const uint8_t *s = 0, *e = 0;

    for (uint16_t i = 0; i < len; i++) {
        if (data[i] == 'S') {
            s = &data[i];
            break;
        }
    }
    if (!s)
        return 0;

    for (const uint8_t *p = s + 1; p < data + len; p++) {
        if (*p == 'E') {
            e = p;
            break;
        }
    }
    if (!e)
        return 0;

    uint8_t n        = 0;
    const uint8_t *p = s + 1;

    while (p < e && n < out_max) {
        out[n++] = fast_atof_simple(&p, e);
        if (p < e && *p == ',')
            p++;
        else
            break;
    }

    return n;
}

static float fast_atof_simple_computer(const uint8_t **pp, const uint8_t *end)
{
    const uint8_t *p = *pp;

    int sign = 1;
    if (p < end && *p == '-') {
        sign = -1;
        p++;
    }

    // 整数部分
    float v = 0.0f;
    while (p < end && *p >= '0' && *p <= '9') {
        v = v * 10.0f + (float)(*p - '0');
        p++;
    }

    // 小数部分
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
 * @brief 解析一帧: S,x_diff,y_diff,angle_x,E
 * @param data    输入字节数组
 * @param len     数组长度
 * @param out     输出结构体
 * @return 1=成功, 0=失败
 */
int parse_vision_frame_computer(uint8_t *data, uint16_t len, VisionData_t *out)
{
    if (data == 0 || out == 0 || len == 0)
        return 0;

    // 先清零
    out->x_diff  = 0.0f;
    out->y_diff  = 0.0f;
    out->angle_x = 0.0f;

    const uint8_t *s = 0;
    const uint8_t *e = 0;

    // 找帧头 S
    for (uint16_t i = 0; i < len; i++) {
        if (data[i] == 'S') {
            s = &data[i];
            break;
        }
    }
    if (!s)
        return 0;

    // 找帧尾 E
    for (const uint8_t *p = s + 1; p < data + len; p++) {
        if (*p == 'E') {
            e = p;
            break;
        }
    }
    if (!e)
        return 0;

    const uint8_t *p = s + 1;

    // 跳过 S 后面的第一个逗号
    if (p < e && *p == ',')
        p++;
    else
        return 0;

    // 解析 x_diff
    out->x_diff = fast_atof_simple(&p, e);
    if (p < e && *p == ',')
        p++;
    else
        return 0;

    // 解析 y_diff
    out->y_diff = fast_atof_simple(&p, e);
    if (p < e && *p == ',')
        p++;
    else
        return 0;

    // 解析 angle_x
    out->angle_x = fast_atof_simple(&p, e);

    return 1;
}
