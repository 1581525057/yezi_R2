#ifndef ARM_COMM_H
#define ARM_COMM_H

#include <stdint.h>

// 机械臂通信命令类。
// 当前只负责维护一帧 5 位 ASCII 命令，串口发送由 send() 后续接入。
// 命令格式：
// 第 1 位：启动位，0 关机，1 开机并进入一区预备姿态。
// 第 2 位：二区吸取次数，0 未吸取，1 吸第一个，2 吸第二个，3 吸第三个。
// 第 3 位：前方 kfs 高度，1 高 200，2 高 400，3 矮 200。
// 第 4 位：三区放置状态，1 回预备位置，2 放下手上的 kfs。
// 第 5 位：三区车内取货状态，1 取最上面，2 取下面。
class ArmComm
{
public:
    // kfs 高度类型，数值直接对应命令第 3 位。
    enum KfsHeight {
        KFS_HIGH_200 = 1,
        KFS_HIGH_400 = 2,
        KFS_LOW_200  = 3,
    };

    ArmComm();

    // 清空命令缓存，回到 "00000"。
    void reset(void);

    // 设置第 1 位启动状态：0 关机，1 开机。
    void setPower(uint8_t power);

    // 同时设置第 2 位吸取次数和第 3 位 kfs 高度。
    // 机械臂需要这两位组合判断是否执行二区吸取动作。
    void setPickup(uint8_t pickup_count, KfsHeight height);

    // 设置第 4 位三区放置状态。
    void setZone3Place(uint8_t place_state);

    // 设置第 5 位三区车内取货状态。
    void setZone3Fetch(uint8_t fetch_state);

    // 返回当前 5 位 ASCII 命令字符串，末尾带 '\0'，可直接交给串口发送。
    const char *getCommandString(void) const;

    // 串口发送入口。串口实现未接入前保持空函数。
    void send(void);

private:
    // command_ 数组各位置的含义，顺序必须和机械臂协议保持一致。
    enum CommandIndex {
        INDEX_POWER = 0,
        INDEX_PICKUP_COUNT,
        INDEX_KFS_HEIGHT,
        INDEX_ZONE3_PLACE,
        INDEX_ZONE3_FETCH,
        COMMAND_LENGTH,
    };

    // 5 位 ASCII 命令缓存，额外 1 位用于字符串结束符。
    char command_[COMMAND_LENGTH + 1];

    // 将 0~9 的数值转换成对应 ASCII 数字字符。
    static char toDigit(uint8_t value);

    // 简单范围检查，非法命令不更新缓存，避免发出半更新状态。
    static uint8_t isInRange(uint8_t value, uint8_t min, uint8_t max);
};

extern ArmComm arm_comm;

#endif
