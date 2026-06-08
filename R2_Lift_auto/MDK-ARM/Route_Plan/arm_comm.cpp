#include "arm_comm.h"

// 全局机械臂通信对象，和 lift_auto、meiling 等模块保持同样的使用方式。
ArmComm arm_comm;

ArmComm::ArmComm()
{
    reset();
}

void ArmComm::reset(void)
{
    // 默认全 0：机械臂关机、未吸取、未指定高度、三区不动作、车内不取货。
    // 最后一位固定写 '\0'，保证 getCommandString() 返回的是标准 C 字符串。
    command_[INDEX_POWER]        = '0';
    command_[INDEX_PICKUP_COUNT] = '0';
    command_[INDEX_KFS_HEIGHT]   = '0';
    command_[INDEX_ZONE3_PLACE]  = '0';
    command_[INDEX_ZONE3_FETCH]  = '0';
    command_[COMMAND_LENGTH]     = '\0';
}

void ArmComm::setPower(uint8_t power)
{
    // 启动位只允许 0 或 1，非法值直接忽略，保留上一帧有效命令。
    if (isInRange(power, 0U, 1U) == 0U) {
        return;
    }

    command_[INDEX_POWER] = toDigit(power);
}

void ArmComm::setPickup(uint8_t pickup_count, KfsHeight height)
{
    // 吸取次数只允许 0~3，分别表示未吸、第一块、第二块、第三块。
    if (isInRange(pickup_count, 0U, 3U) == 0U) {
        return;
    }

    // 高度位只允许协议定义的三种 kfs 类型。
    if (isInRange((uint8_t)height, (uint8_t)KFS_HIGH_200, (uint8_t)KFS_LOW_200) == 0U) {
        return;
    }

    // 第二位和第三位必须成对更新，避免机械臂读到吸取次数和高度不匹配。
    command_[INDEX_PICKUP_COUNT] = toDigit(pickup_count);
    command_[INDEX_KFS_HEIGHT]   = toDigit((uint8_t)height);
}

void ArmComm::setZone3Place(uint8_t place_state)
{
    // 三区放置位：0 不动作，1 回预备位置，2 放下手上的 kfs。
    if (isInRange(place_state, 0U, 2U) == 0U) {
        return;
    }

    command_[INDEX_ZONE3_PLACE] = toDigit(place_state);
}

void ArmComm::setZone3Fetch(uint8_t fetch_state)
{
    // 三区取车内 kfs：0 不动作，1 取最上面，2 取下面。
    if (isInRange(fetch_state, 0U, 2U) == 0U) {
        return;
    }

    command_[INDEX_ZONE3_FETCH] = toDigit(fetch_state);
}

const char *ArmComm::getCommandString(void) const
{
    // 返回内部缓存指针，不复制数据；调用方需要在下一次 set/reset 前使用。
    return command_;
}

void ArmComm::send(void)
{
    // 串口发送由后续接入，这里只保留调用入口。
    // 后续可在这里发送 command_ 的前 COMMAND_LENGTH 个 ASCII 字符。
}

char ArmComm::toDigit(uint8_t value)
{
    // 当前所有协议位都在 0~3 范围内，调用前已经完成合法性检查。
    return (char)('0' + value);
}

uint8_t ArmComm::isInRange(uint8_t value, uint8_t min, uint8_t max)
{
    // 使用 uint8_t 返回值，方便在嵌入式代码里保持现有 0/1 判断风格。
    return (value >= min && value <= max) ? 1U : 0U;
}
