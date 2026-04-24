#ifndef __M2006FOLLOWERMOTOR_H__
#define __M2006FOLLOWERMOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"

#ifdef __cplusplus
}
#endif

#include <stdint.h>

/* M2006 随动电机控制类 */
class M2006FollowerMotor
{
public:
    M2006FollowerMotor(FDCAN_HandleTypeDef *hfdcan, uint16_t rx_id, uint8_t tx_slot);

    void Init();
    void ResetControl();
    void Stop();

    void UpdateFeedback(const uint8_t data[8], uint32_t now_ms);
    void SetStepReference(uint8_t step_dir, float step_speed_rpm);
    void ControlTick(uint32_t now_ms);

    bool IsFeedbackAlive(uint32_t now_ms) const;
    int16_t GetCurrentCommand() const;
    int16_t GetSpeedRpmRaw() const;
    float GetMeasuredSpeedRpm() const;
    float GetTargetSpeedRpm() const;
    bool IsDescendingActive() const;

    void SetCommandSign(int8_t sign);
    void SetFeedbackSign(int8_t sign);
    void SetDescendDirection(uint8_t dir);

private:
    void UpdateTargetSpeed(void);
    void SendCurrent(int16_t current);
    float GetMeasuredSpeedRpmInternal(void) const;

private:
    FDCAN_HandleTypeDef *hfdcan_;   // FDCAN 句柄
    uint16_t rx_id_;            // 接收 CAN ID
    uint16_t tx_std_id_;        // 发送标准 ID
    uint8_t tx_slot_;           // 发送槽位
    uint8_t step_descend_dir_;  // 步进下降方向
    int8_t cmd_sign_;           // 电流命令符号
    int8_t feedback_sign_;      // 反馈方向符号
    float step_to_rotor_ratio_; // 步进轴到转子转速换算比
    float kff_;                 // 前馈系数
    float kp_;                  // 比例系数
    float ki_;                  // 积分系数
    int16_t i_start_min_;       // 最小启动电流，单位：mA
    int16_t i_max_assist_;      // 最大辅助电流，单位：mA
    uint32_t can_timeout_ms_;   // CAN 超时时间，单位：ms

    uint16_t ecd_;              // 编码器值
    int16_t speed_rpm_raw_;     // 原始转速，单位：RPM
    int16_t given_current_;     // 给定电流，单位：mA
    uint32_t last_rx_tick_;     // 上次接收时刻
    uint8_t feedback_received_; // 已收到反馈标志

    uint8_t step_dir_;          // 步进方向
    float step_speed_rpm_;      // 步进速度，单位：RPM
    float target_speed_rpm_;    // 目标速度，单位：RPM
    float integral_;            // 积分项
    int16_t current_cmd_;       // 电流命令，单位：mA
    uint8_t descend_active_;    // 下降跟随激活标志
};

extern M2006FollowerMotor g_m2006;

#endif
