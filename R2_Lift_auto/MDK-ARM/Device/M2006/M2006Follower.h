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
    FDCAN_HandleTypeDef *hfdcan_;
    uint16_t rx_id_;
    uint16_t tx_std_id_;
    uint8_t tx_slot_;
    uint8_t step_descend_dir_;
    int8_t cmd_sign_;
    int8_t feedback_sign_;
    float step_to_rotor_ratio_;
    float kff_;
    float kp_;
    float ki_;
    int16_t i_start_min_;
    int16_t i_max_assist_;

    uint16_t ecd_;
    int16_t speed_rpm_raw_;
    int16_t given_current_;
    uint8_t step_dir_;
    float step_speed_rpm_;
    float target_speed_rpm_;
    float integral_;
    int16_t current_cmd_;
    uint8_t descend_active_;
};

extern M2006FollowerMotor g_m2006;

#endif
