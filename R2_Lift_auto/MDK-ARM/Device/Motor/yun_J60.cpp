/* USER CODE BEGIN Header */
// 负责：
// 1、J60 电机对象定义
// 2、J60 协议数据打包 / 解包
// 3、J60 电机使能命令发送
// 4、J60 电机控制命令发送
// 5、J60 电机反馈解析
/* USER CODE END Header */

#include "yun_j60.h"
#include "fdcan.h"
#include "bsp_can.h"

Yun_J60_Class yun_j60_motor;

/* 电机对象 ------------------------------------------------------------------*/
Dog_j60_motor_t Yun_J60_Class::motor[8] = {0};


/* 电机使能状态 --------------------------------------------------------------*/
uint8_t Yun_J60_Class::motor_enabled[32] = {0};

/**
 * @brief  浮点数转无符号整型，用在发送电机控制数据
 * @param  x     : 输入浮点值
 * @param  x_min : 最小值
 * @param  x_max : 最大值
 * @param  bits  : 位宽
 * @retval uint16_t
 */
uint16_t Yun_J60_Class::float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (x < x_min)
        x = x_min;

    if (x > x_max)
        x = x_max;

    float span = x_max - x_min;
    float offset = x - x_min;

    return (uint16_t)(offset * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  无符号整型转浮点数，用在接收反馈数据
 * @param  x     : 输入整数
 * @param  x_min : 最小值
 * @param  x_max : 最大值
 * @param  bits  : 位宽
 * @retval float
 */
float Yun_J60_Class::uint_to_float(uint32_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    uint32_t max_int = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);

    return ((float)x) * span / ((float)max_int) + x_min;
}

/**
 * @brief  解析 J60 电机反馈数据
 * @param  id     : 电机 id
 * @param  rxData : 接收数据
 * @retval None
 */
void Yun_J60_Class::DecodeFeedback(uint8_t id, uint8_t *rxData)
{
    /* J60 反馈协议为小端字节序，这里先拼成 64 位数据 */
    uint64_t raw = 0;

    for (int i = 0; i < 8; i++)
    {
        raw |= ((uint64_t)rxData[i]) << (8 * i);
    }

    /* 按协议字段拆包 */
    uint32_t p_int = (uint32_t)(raw & 0xFFFFF);         // bit 0  ~ 19
    uint32_t v_int = (uint32_t)((raw >> 20) & 0xFFFFF); // bit 20 ~ 39
    uint32_t t_int = (uint32_t)((raw >> 40) & 0xFFFF);  // bit 40 ~ 55
    uint8_t temp_flag = (uint8_t)((raw >> 56) & 0x01);  // bit 56
    uint8_t temp_raw = (uint8_t)((raw >> 57) & 0x7F);   // bit 57 ~ 63

    (void)temp_flag; // 目前先不使用这个标志位

    /* 转换回物理量 */
    float position = uint_to_float(p_int, -40.0f, 40.0f, 20);
    float velocity = uint_to_float(v_int, -40.0f, 40.0f, 20);
    float torque = uint_to_float(t_int, -40.0f, 40.0f, 16);
    float temperature = uint_to_float((uint32_t)temp_raw, -20.0f, 200.0f, 7);

    /* 协议里 real_id 范围是低 4 位 */
    uint8_t real_id = id & 0x0F;

 
    if (real_id >= 1 && real_id <= 8)
    {
        motor[real_id - 1].position = position;
        motor[real_id - 1].velocity = velocity;
        motor[real_id - 1].torque = torque;
        motor[real_id - 1].temperature = temperature;
    }
}

/**
 * @brief  使能 J60 电机（通过 FDCAN1）
 * @param  id : 电机 id
 * @retval None
 */
void Yun_J60_Class::EnableMotor(uint8_t id)
{
    uint8_t txData[8] = {0};

    /* Identifier = (cmd << 5) | id, 这里 cmd = 2 */
    BSP_CAN::FDCAN1_TxFrame.Header.Identifier = (2U << 5) | (id & 0x1F);

    /* 使能帧是 0 字节数据 */
    BSP_CAN::FDCAN1_TxFrame.Header.DataLength = FDCAN_DLC_BYTES_0;

    HAL_FDCAN_AddMessageToTxFifoQ(BSP_CAN::FDCAN1_TxFrame.hcan,
                                  &BSP_CAN::FDCAN1_TxFrame.Header,
                                  txData);
}

/**
 * @brief  发送 J60 控制帧（通过 FDCAN1）
 * @param  id    : 电机 id
 * @param  p_des : 目标位置
 * @param  v_des : 目标速度
 * @param  kp    : 刚度
 * @param  kd    : 阻尼
 * @param  t_ff  : 前馈力矩
 * @retval None
 */
void Yun_J60_Class::SendControl(uint8_t id,
                                float p_des,
                                float v_des,
                                float kp,
                                float kd,
                                float t_ff)
{
    uint8_t txData[8] = {0};

    /* Identifier = (cmd << 5) | id, 这里 cmd = 4 */
    BSP_CAN::FDCAN1_TxFrame.Header.Identifier = (4U << 5) | (id & 0x1F);

    /* 数据长度 8 字节 */
    BSP_CAN::FDCAN1_TxFrame.Header.DataLength = FDCAN_DLC_BYTES_8;

    /* 将物理量压缩打包 */
    uint16_t p_int = float_to_uint(p_des, -40.0f, 40.0f, 16);
    uint16_t v_int = float_to_uint(v_des, -40.0f, 40.0f, 14);
    uint16_t kp_int = float_to_uint(kp, 0.0f, 1023.0f, 10);
    uint16_t kd_int = float_to_uint(kd, 0.0f, 51.0f, 8);
    uint16_t t_int = float_to_uint(t_ff, -40.0f, 40.0f, 16);

    /* 按协议打包 */
    txData[0] = p_int & 0xFF;
    txData[1] = (p_int >> 8) & 0xFF;

    txData[2] = v_int & 0xFF;
    txData[3] = ((v_int >> 8) & 0x3F) | ((kp_int & 0x03) << 6);
    txData[4] = (kp_int >> 2) & 0xFF;

    txData[5] = kd_int & 0xFF;

    txData[6] = t_int & 0xFF;
    txData[7] = (t_int >> 8) & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(BSP_CAN::FDCAN1_TxFrame.hcan,
                                  &BSP_CAN::FDCAN1_TxFrame.Header,
                                  txData);
}

/**
 * @brief  将 FDCAN Header.DataLength 转成真实字节数
 * @param  DataLength : DLC 宏值
 * @retval uint8_t
 */
uint8_t Yun_J60_Class::GetDataLengthBytes(uint32_t DataLength)
{
    switch (DataLength)
    {
    case FDCAN_DLC_BYTES_0:
        return 0;
    case FDCAN_DLC_BYTES_1:
        return 1;
    case FDCAN_DLC_BYTES_2:
        return 2;
    case FDCAN_DLC_BYTES_3:
        return 3;
    case FDCAN_DLC_BYTES_4:
        return 4;
    case FDCAN_DLC_BYTES_5:
        return 5;
    case FDCAN_DLC_BYTES_6:
        return 6;
    case FDCAN_DLC_BYTES_7:
        return 7;
    case FDCAN_DLC_BYTES_8:
        return 8;
    default:
        return 0;
    }
}

/**
 * @brief  J60 CAN 接收处理函数
 * @param  fdcan_rxframe : 接收帧
 * @param  Data          : 接收数据
 * @retval None
 */
void Yun_J60_Class::RxHandler(FDCAN_RxFrame_TypeDef *fdcan_rxframe, uint8_t Data[8])
{
    uint8_t raw_id = fdcan_rxframe->Header.Identifier & 0x1F;
    uint8_t real_id = raw_id & 0x0F;
    uint8_t is_reply = (raw_id & 0x10) ? 1 : 0;
    uint8_t cmd = (fdcan_rxframe->Header.Identifier >> 5) & 0x3F;
    uint8_t dlc = GetDataLengthBytes(fdcan_rxframe->Header.DataLength);

    /* 启停反馈 */
    if (is_reply && dlc >= 1)
    {
        if (cmd == 2)
        {
            motor_enabled[real_id] = (Data[0] == 0) ? 1 : 0;
        }
        else if (cmd == 1)
        {
            motor_enabled[real_id] = 0;
        }
    }

    /* 控制反馈：cmd == 4 且 8 字节 */
    if (is_reply && cmd == 4 && dlc == 8)
    {
        DecodeFeedback(real_id, Data);
    }
}