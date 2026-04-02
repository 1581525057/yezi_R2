#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "main.h"

#define UART_BUFNUM_DM (19u * 3u)

enum class ParseResult {
    Ok,
    HeaderError,
    CrcError,
    UnknownType
};

typedef struct
{
    float pitch;
    float roll;
    float yaw;

    float gyro_gx;
    float gyro_gy;
    float gyro_gz;

    float accel_ax;
    float accel_ay;
    float accel_az;

    float q[4];

    float cur_temp;

} imu_t;

class DM_IMU
{
public:
    uint8_t dmimu_data[2][UART_BUFNUM_DM]; // DMA缓存区，从DMA那里接收到数据

    imu_t imu; // imu数据存储

public:
    DM_IMU();
    void ParseIMUStream(uint8_t *streamData); // 处理数据

private:
    ParseResult ParseSingleIMUPacket(uint8_t *data); // 单个

    static const uint16_t CRC16_table[256]; // crc校验表

    static uint16_t Get_CRC16(uint8_t *ptr, uint16_t len); // 得到crc校验码

    static float bytesToFloat(uint8_t *bytes); // 转为浮点数
};

extern DM_IMU dm_imu;

#endif
