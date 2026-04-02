#ifndef BSP_REMOVE_H
#define BSP_REMOVE_H

#include "main.h"
#include <stdint.h>
#include <cstring>

// SBUS 一帧接收缓冲区大小
#define SBUS_RX_BUF_NUM 18

// 遥控器通道中值偏移量（大疆遥控器常见是 1024）
#define RC_CH_VALUE_OFFSET 1024

// 遥控器拨杆通道信息
struct RC_Info_t {
    int16_t ch[5];
    uint8_t s[2];
};

// 鼠标信息
struct Mouse_Info_t {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
};

// 键盘信息
struct Key_Info_t {
    uint16_t v;
};

// 底盘速度信息
struct Chassis_Cmd_t {
    float Vx;
    float Vy;
    float Vz;
    float Vl;
};


class Remote
{
public:
    uint8_t SBUS_MultiRx_Buff[2][SBUS_RX_BUF_NUM];
    Chassis_Cmd_t chassis_;
    RC_Info_t rc_;

public:
    Remote();

    void parseSBUS(const uint8_t *sbus_buf);

    void monitor();

    void updateChassosCommand();

    void reset();

    const RC_Info_t &getRC() const;

    const Mouse_Info_t &getMouse() const;

    const Key_Info_t &getKey() const;

    const Chassis_Cmd_t &getChassosCmd() const;

    bool isLost() const;
    uint16_t getOnlineCount() const;

private:
    Mouse_Info_t mouse_;
    Key_Info_t key_;

    uint16_t online_cnt_; // 在线计数器
    bool rc_lost_;        // 遥控器是否在线

private:
    int16_t normalizeChannel(uint16_t raw_value) const;
};

extern Remote remove_dji;

#endif
