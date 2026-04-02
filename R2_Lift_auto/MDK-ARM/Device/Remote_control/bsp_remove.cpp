#include "bsp_remove.h"

Remote remove_dji;

Remote::Remote()
{
    reset();
    online_cnt_ = 0XFAU;
    rc_lost_    = true;
}

void Remote::reset()
{
    memset(&rc_, 0, sizeof(rc_));
    memset(&mouse_, 0, sizeof(rc_));
    memset(&key_, 0, sizeof(rc_));
    memset(&chassis_, 0, sizeof(rc_));

    online_cnt_ = 0;
    rc_lost_    = true;
}

int16_t Remote::normalizeChannel(uint16_t raw_value) const
{
    return static_cast<int16_t>(raw_value) - RC_CH_VALUE_OFFSET;
}

// 解析一帧 SBUS 数据
void Remote::parseSBUS(const uint8_t *sbus_buf)
{
    // 防止空指针
    if (sbus_buf == nullptr) {
        return;
    }

    // -------------------- 解析 5 个通道 --------------------
    rc_.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07FF;
    rc_.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07FF;
    rc_.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07FF;
    rc_.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07FF;
    rc_.ch[4] = (sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF;

    // -------------------- 解析左右拨杆开关 --------------------
    rc_.s[0] = (sbus_buf[5] >> 4) & 0x0003;        // 左开关
    rc_.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2; // 右开关

    // -------------------- 解析鼠标数据 --------------------
    mouse_.x = sbus_buf[6] | (sbus_buf[7] << 8);
    mouse_.y = sbus_buf[8] | (sbus_buf[9] << 8);
    mouse_.z = sbus_buf[10] | (sbus_buf[11] << 8);

    mouse_.press_l = sbus_buf[12];
    mouse_.press_r = sbus_buf[13];

    // -------------------- 解析键盘数据 --------------------
    key_.v = sbus_buf[14] | (sbus_buf[15] << 8);

    // -------------------- 通道归一化 --------------------
    rc_.ch[0] = normalizeChannel(rc_.ch[0]);
    rc_.ch[1] = normalizeChannel(rc_.ch[1]);
    rc_.ch[2] = normalizeChannel(rc_.ch[2]);
    rc_.ch[3] = normalizeChannel(rc_.ch[3]);
    rc_.ch[4] = normalizeChannel(rc_.ch[4]);



    // -------------------- 收到数据，刷新在线状态 --------------------
    online_cnt_ = 0xFAU;
    rc_lost_    = false;
}

// 掉线监测函数
void Remote::monitor()
{
    // 如果计数器已经很小，认为遥控器丢失
    if (online_cnt_ <= 0x32U) {
        // 注意：掉线后清空所有控制量，防止底盘乱跑
        std::memset(&rc_, 0, sizeof(rc_));
        std::memset(&mouse_, 0, sizeof(mouse_));
        std::memset(&key_, 0, sizeof(key_));
        std::memset(&chassis_, 0, sizeof(chassis_));

        rc_lost_ = true;
    } else if (online_cnt_ > 0U) {
        online_cnt_--;
    }
}

void Remote::updateChassosCommand()
{
    if(rc_lost_)
    {
        chassis_.Vx = 0.0f;
        chassis_.Vy = 0.0f;
        chassis_.Vz = 0.0f;
        chassis_.Vl = 0.0f;
        return;
    }

    // 这里保留你原来的比例关系
    chassis_.Vx = static_cast<float>(rc_.ch[2]) / 660.0f * 2.0f;
    chassis_.Vy = static_cast<float>(rc_.ch[3]) / 660.0f * 2.0f;
    chassis_.Vz = static_cast<float>(rc_.ch[0]) / 660.0f * 3.0f;
    chassis_.Vl = static_cast<float>(rc_.ch[1]) / 660.0f * 0.5f;
}

// ===================== Getter 访问接口 =====================
const RC_Info_t& Remote::getRC() const
{
    return rc_;
}

const Mouse_Info_t& Remote::getMouse() const
{
    return mouse_;
}

const Key_Info_t& Remote::getKey() const
{
    return key_;
}

const Chassis_Cmd_t& Remote::getChassosCmd() const
{
    return chassis_;
}

bool Remote::isLost() const
{
    return rc_lost_;
}

uint16_t Remote::getOnlineCount() const
{
    return online_cnt_;
}


