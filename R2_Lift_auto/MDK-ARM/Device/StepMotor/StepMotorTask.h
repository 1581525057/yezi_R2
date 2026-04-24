#ifndef __MOTORTASK_H__
#define __MOTORTASK_H__

#include "EmmV5.h"

#include <stdint.h>

/* 圆周率，用于距离转脉冲计算 */
#define PI_VALUE 3.14159265358979323846f

/* Z 轴电机参数 */
#define Z_MOTOR_ADDR 0x01
#define Z_MOTOR_DIR 0x00
#define Z_MOTOR_KP 1.0f
#define Z_MOTOR_KI 0.0f
#define Z_ROLLER_DIAMETER 18.0f
#define Z_PULSE_SCALE 3200.0f

/* 电机控制参数 */
#define MAX_SPEED_RPM 50.0f
#define MAX_I_SUM_LIMIT (MAX_SPEED_RPM * 10.0f)
#define MOTOR_ACC 0
#define STEP_MOTOR_TX_IDLE_WAIT_MS 1U

/* 单个步进电机轴控制类 */
class StepMotorAxis
{
public:
    StepMotorAxis(uint8_t addr,
                  uint8_t default_dir,
                  float kp,
                  float ki,
                  float roller_diameter_mm,
                  float pulse_scale);

    void ResetState();
    void SetError(int16_t error);
    int16_t GetError() const;
    uint8_t GetAddr() const;
    uint8_t GetDirection() const;
    float GetSpeedRpm() const;

    void UpdateSpeedFromError();
    void QueueVelocityControl(bool sync = false) const;
    void QueueEnable(bool enable, bool sync = false) const;
    void QueueResetCurrentPosition() const;
    void QueueResetClogProtection() const;
    void PositionControl(uint8_t dir, uint16_t vel, uint8_t acc, float distance_mm, uint8_t motion_mode) const;

private:
    uint32_t DistanceToPulse(float distance_mm) const;

private:
    uint8_t addr_;               // 电机地址
    uint8_t default_dir_;        // 默认方向
    uint8_t dir_;                // 当前方向
    uint8_t acc_;                // 加速度
    float speed_;                // 当前速度，单位：RPM
    float kp_;                   // 比例系数
    float ki_;                   // 积分系数
    int16_t error_;              // 控制误差
    int32_t i_sum_;              // 积分累计值
    float roller_diameter_mm_;   // 滚轮直径，单位：mm
    float pulse_scale_;          // 脉冲比例，单位：脉冲/mm
};

/* Z 轴电机实例 */
extern StepMotorAxis StepMotor_Z;
extern volatile float g_grip_distance_mm;

void StepMotor_Init(void);
void Motor_Ctrl(StepMotorAxis &zmotor);
void FineTuneLiftForWeaponGrip(void);
void ReturnLiftToZero(void);
void ReturnLiftToPosition(float target_distance_mm, float reference_distance_mm, uint16_t speed_rpm = 60U);
void StepMotor_ServiceRecovery(void);
bool StepMotor_IsRecoveryActive(void);
uint8_t StepMotor_GetRecoveryDirection(void);
float StepMotor_GetRecoverySpeedRpm(void);
void StepMotorCommandDelay(void);
bool StepMotorCommandDelayTimeout(uint32_t timeout_ms);

#endif
