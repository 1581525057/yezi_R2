#include "bsp_remove.h"

// 全局遥控器对象。
// 通常在其他模块中通过 extern Remote remove_dji; 访问该实例。
Remote remove_dji;

Remote::Remote()
{
    // 构造时先将所有数据清零，进入安全初始状态。
    reset();

    // 将在线计数器初始化为较大的值（0xFA == 250），
    // 表示刚启动时先认为链路可用，留出足够时间等待第一帧数据。
    online_cnt_ = 0XFAU;

    // 初始状态下仍标记为"遥控器丢失"，
    // 直到真正收到一帧有效 SBUS 数据后才会清除该标志。
    rc_lost_    = true;
}

void Remote::reset()
{
    // 清空遥控通道数据。
    memset(&rc_, 0, sizeof(rc_));

    // 清空鼠标数据。
    // 注意：原代码此处使用 sizeof(rc_)，看起来是拷贝时留下的问题，逻辑上应为 sizeof(mouse_)。
    memset(&mouse_, 0, sizeof(rc_));

    // 清空键盘数据（同上，逻辑上应为 sizeof(key_)）。
    memset(&key_, 0, sizeof(rc_));

    // 清空底盘控制指令（逻辑上应为 sizeof(chassis_)）。
    memset(&chassis_, 0, sizeof(rc_));

    // 复位后在线计数归零。
    online_cnt_ = 0;

    // 复位后默认判定为遥控器丢失，避免系统误动作。
    rc_lost_    = true;
}

int16_t Remote::normalizeChannel(uint16_t raw_value) const
{
    // DJI 遥控器的通道原始值以 1024 为中心值（RC_CH_VALUE_OFFSET == 1024）。
    // 减去中值后将通道值转换为以 0 为中心的有符号量，
    // 范围约为 [-660, +660]，便于后续速度映射和控制计算。
    return static_cast<int16_t>(raw_value) - RC_CH_VALUE_OFFSET;
}

// 解析一帧 SBUS 数据。
// SBUS 协议：每个通道占 11 bit，多个通道跨字节紧密打包存储，需要手动位操作提取。
void Remote::parseSBUS(const uint8_t *sbus_buf)
{
    // 防止空指针导致非法内存访问。
    if (sbus_buf == nullptr) {
        return;
    }

    // -------------------- 解析 5 个遥控通道 --------------------
    // 每个通道 11 bit，按照 SBUS 格式从字节流中提取，掩码 0x07FF 保留低 11 位。
    rc_.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07FF;                            // 通道 0
    rc_.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07FF;                     // 通道 1
    rc_.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07FF; // 通道 2
    rc_.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07FF;                     // 通道 3
    rc_.ch[4] = (sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF;                           // 通道 4（扩展通道）

    // -------------------- 解析左右拨杆开关状态 --------------------
    // 每个开关占 2 bit，取值范围 0~3，分别对应上/中/下三档。
    rc_.s[0] = (sbus_buf[5] >> 4) & 0x0003;        // 左拨杆开关
    rc_.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2; // 右拨杆开关

    // -------------------- 解析鼠标数据 --------------------
    // 鼠标 x/y/z 轴速度各占 2 字节（小端序有符号整数）。
    mouse_.x = sbus_buf[6] | (sbus_buf[7] << 8);   // 鼠标 X 轴速度
    mouse_.y = sbus_buf[8] | (sbus_buf[9] << 8);   // 鼠标 Y 轴速度
    mouse_.z = sbus_buf[10] | (sbus_buf[11] << 8); // 鼠标 Z 轴速度（滚轮）

    mouse_.press_l = sbus_buf[12]; // 鼠标左键状态（1=按下，0=释放）
    mouse_.press_r = sbus_buf[13]; // 鼠标右键状态（1=按下，0=释放）

    // -------------------- 解析键盘数据 --------------------
    // 键盘按键状态以 16 bit 位域存储，每一位对应一个按键。
    key_.v = sbus_buf[14] | (sbus_buf[15] << 8);

    // -------------------- 通道值归一化（以 0 为中心）--------------------
    // 将原始 [0, 2047] 范围转换为 [-1024, +1023]，以中立位为零点。
    rc_.ch[0] = normalizeChannel(rc_.ch[0]);
    rc_.ch[1] = normalizeChannel(rc_.ch[1]);
    rc_.ch[2] = normalizeChannel(rc_.ch[2]);
    rc_.ch[3] = normalizeChannel(rc_.ch[3]);
    rc_.ch[4] = normalizeChannel(rc_.ch[4]);

    // -------------------- 收到有效数据，刷新在线状态 --------------------
    online_cnt_ = 0xFAU; // 重置倒计时计数器（250 次）
    rc_lost_    = false;  // 清除丢失标志
}

// 遥控器掉线监测函数。
// 建议在固定周期任务中调用（如 1ms / 2ms / 5ms），
// 每调用一次计数器减 1，若长时间无新数据则触发掉线保护。
void Remote::monitor()
{
    // 计数器降至 0x32（50）及以下时，判定遥控器已掉线。
    if (online_cnt_ <= 0x32U) {
        // 掉线后立即清零所有控制量，防止执行机构保持上一次指令造成失控。
        std::memset(&rc_, 0, sizeof(rc_));
        std::memset(&mouse_, 0, sizeof(mouse_));
        std::memset(&key_, 0, sizeof(key_));
        std::memset(&chassis_, 0, sizeof(chassis_));

        rc_lost_ = true; // 置位丢失标志
    } else if (online_cnt_ > 0U) {
        // 仍在线时每次递减计数器。
        // 若后续没有新 SBUS 数据通过 parseSBUS() 刷新，计数器将逐步归零，最终触发掉线。
        online_cnt_--;
    }
}

// 将遥控器摇杆通道值转换为底盘速度指令。
// 遥控器丢失时输出全零，防止底盘失控运动。
void Remote::updateChassosCommand()
{
    if(rc_lost_)
    {
        // 遥控器丢失，所有速度指令清零。
        chassis_.Vx = 0.0f;
        chassis_.Vy = 0.0f;
        chassis_.Vz = 0.0f;
        chassis_.Vl = 0.0f;
        return;
    }

    // 将摇杆通道值映射到底盘速度指令。
    // 660.0f 为摇杆满量程近似值（归一化后范围约 ±660）。
    // 后面的系数为各方向速度上限缩放比例：
    //   ch[2] -> Vx（前后平移），最大 2.0
    //   ch[3] -> Vy（左右平移），最大 2.0
    //   ch[0] -> Vz（旋转），最大 3.0
    //   ch[1] -> Vl（升降/附加轴），最大 0.5
    chassis_.Vx = static_cast<float>(rc_.ch[2]) / 660.0f * 2.5f;
    chassis_.Vy = static_cast<float>(rc_.ch[3]) / 660.0f * 2.5f;
    chassis_.Vz = static_cast<float>(rc_.ch[0]) / 660.0f * 3.0f;
    chassis_.Vl = static_cast<float>(rc_.ch[1]) / 660.0f * 0.5f;
}

// ===================== Getter 访问接口 =====================

// 返回遥控通道原始数据（归一化后）。
const RC_Info_t& Remote::getRC() const
{
    return rc_;
}

// 返回鼠标数据。
const Mouse_Info_t& Remote::getMouse() const
{
    return mouse_;
}

// 返回键盘数据。
const Key_Info_t& Remote::getKey() const
{
    return key_;
}

// 返回底盘速度指令。
const Chassis_Cmd_t& Remote::getChassosCmd() const
{
    return chassis_;
}

// 返回遥控器是否丢失（true = 丢失）。
bool Remote::isLost() const
{
    return rc_lost_;
}

// 返回当前在线计数器值，可用于调试观察链路质量。
uint16_t Remote::getOnlineCount() const
{
    return online_cnt_;
}
