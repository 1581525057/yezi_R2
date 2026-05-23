#include "StepMotorTask.h"
#include "cmsis_os.h"

constexpr uint8_t kGripDirection = static_cast<uint8_t>(!Z_MOTOR_DIR); // 抓取抬升方向
constexpr uint16_t kGripVelocityRpm = 50U;                             // 抓取抬升速度，单位：RPM
constexpr uint16_t kReleaseVelocityRpm = 60U;                          // 释放下降速度，单位：RPM
constexpr uint8_t kAbsolutePositionMode = 1U;                          // 绝对位置模式标志
constexpr uint32_t kZAxisRecoveryPeriodMs = 100U;                      // Z 轴恢复周期，单位：ms
constexpr uint32_t kZAxisRecoveryMarginMs = 300U;                      // Z 轴恢复容差时间，单位：ms

/* Z 轴防掉落堵转保护配置 */
constexpr EmmV5MotorConfParams_t kZAxisNoDropClogConfig = {
    0x19U,
    0x02U,
    0x02U,
    0x02U,
    0x00U,
    0x10U,
    0x01U,
    0x00U,
    1200U,
    3000U,
    4000U,
    0x05U,
    0x07U,
    Z_MOTOR_ADDR,
    0x00U,
    0x01U,
    0x02U,
    3000U,
    5000U,
    65535U,
    8U
};

/**
 * @brief 估算 Z 轴恢复动作的执行时间
 *
 * @param distance_mm 参考距离，单位：mm
 * @param velocity_rpm 目标速度，单位：RPM
 * @return uint32_t 估算时间，单位：ms
 */
uint32_t EstimateZAxisMoveTimeMs(float distance_mm, uint16_t velocity_rpm)
{
    if ((distance_mm <= 0.0f) || (velocity_rpm == 0U))
    {
        return kZAxisRecoveryMarginMs;
    }

    const float revolutions = distance_mm / (PI_VALUE * Z_ROLLER_DIAMETER);
    const float move_time_minutes = revolutions / static_cast<float>(velocity_rpm);
    const float move_time_ms = move_time_minutes * 60000.0f;

    return static_cast<uint32_t>(move_time_ms + 0.5f) + kZAxisRecoveryMarginMs;
}

/**
 * @brief 配置 Z 轴驱动器保护参数
 *
 * @return 无
 */
void ConfigureZAxisDriverProtection(void)
{
    Emm_V5_Modify_Motor_Conf_Params(Z_MOTOR_ADDR, false, &kZAxisNoDropClogConfig);
    StepMotorCommandDelay();
    osDelay(20);
}

volatile float g_grip_distance_mm = 322.0f;

/* Z 轴位置恢复状态 */
typedef struct
{
    uint8_t active;             // 恢复任务激活标志
    uint8_t dir;                // 目标方向
    uint16_t vel;               // 目标速度，单位：RPM
    uint8_t acc;                // 加速度
    float target_distance_mm;   // 目标距离，单位：mm
    uint8_t motion_mode;        // 运动模式
    uint32_t start_tick;        // 启动时刻
    uint32_t window_ms;         // 执行时间窗口
    uint32_t last_service_tick; // 上次服务时刻
} ZAxisRecoveryState_t;

static ZAxisRecoveryState_t g_z_axis_recovery = {0U, 0U, 0U, 0U, 0.0f, 0U, 0U, 0U, 0U};

/**
 * @brief 等待步进电机发送队列完全空闲
 *
 * @return 无
 */
void StepMotorCommandDelay(void)
{
    while (StepMotorTx_IsIdle() == false)
    {
        osDelay(STEP_MOTOR_TX_IDLE_WAIT_MS);
    }
}

/**
 * @brief 在超时时间内等待发送队列空闲
 *
 * @param timeout_ms 超时时间，单位：ms
 * @return bool 超时返回 false
 */
bool StepMotorCommandDelayTimeout(uint32_t timeout_ms)
{
    const uint32_t start_tick = HAL_GetTick();

    while (StepMotorTx_IsIdle() == false)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            return false;
        }
        osDelay(STEP_MOTOR_TX_IDLE_WAIT_MS);
    }

    return true;
}

StepMotorAxis StepMotor_Z(Z_MOTOR_ADDR,
                          Z_MOTOR_DIR,
                          Z_MOTOR_KP,
                          Z_MOTOR_KI,
                          Z_ROLLER_DIAMETER,
                          Z_PULSE_SCALE);

/**
 * @brief 构造单个步进电机轴对象
 *
 * @param addr 电机地址
 * @param default_dir 默认方向
 * @param kp 比例系数
 * @param ki 积分系数
 * @param roller_diameter_mm 滚轮直径，单位：mm
 * @param pulse_scale 脉冲比例，单位：脉冲/mm
 * @return 无
 */
StepMotorAxis::StepMotorAxis(uint8_t addr,
                             uint8_t default_dir,
                             float kp,
                             float ki,
                             float roller_diameter_mm,
                             float pulse_scale)
    : addr_(addr),
      default_dir_(default_dir),
      dir_(default_dir),
      acc_(MOTOR_ACC),
      speed_(0.0f),
      kp_(kp),
      ki_(ki),
      error_(0),
      i_sum_(0),
      roller_diameter_mm_(roller_diameter_mm),
      pulse_scale_(pulse_scale)
{
}

/**
 * @brief 将电机轴状态恢复到初始值
 *
 * @return 无
 */
void StepMotorAxis::ResetState()
{
    dir_ = default_dir_;
    speed_ = 0.0f;
    error_ = 0;
    i_sum_ = 0;
}

/**
 * @brief 设置当前控制误差
 *
 * @param error 误差值
 * @return 无
 */
void StepMotorAxis::SetError(int16_t error)
{
    error_ = error;
}

/**
 * @brief 获取当前控制误差
 *
 * @return int16_t 当前误差
 */
int16_t StepMotorAxis::GetError() const
{
    return error_;
}

/**
 * @brief 获取电机地址
 *
 * @return uint8_t 电机地址
 */
uint8_t StepMotorAxis::GetAddr() const
{
    return addr_;
}

/**
 * @brief 获取当前方向
 *
 * @return uint8_t 当前方向
 */
uint8_t StepMotorAxis::GetDirection() const
{
    return dir_;
}

/**
 * @brief 获取当前速度命令
 *
 * @return float 当前速度，单位：RPM
 */
float StepMotorAxis::GetSpeedRpm() const
{
    return speed_;
}

/**
 * @brief 根据误差更新速度和方向
 *
 * @return 无
 */
void StepMotorAxis::UpdateSpeedFromError()
{
    float signed_speed = 0.0f;

    i_sum_ += error_;
    if (i_sum_ > static_cast<int32_t>(MAX_I_SUM_LIMIT))
    {
        i_sum_ = static_cast<int32_t>(MAX_I_SUM_LIMIT);
    }
    else if (i_sum_ < -static_cast<int32_t>(MAX_I_SUM_LIMIT))
    {
        i_sum_ = -static_cast<int32_t>(MAX_I_SUM_LIMIT);
    }

    signed_speed = kp_ * static_cast<float>(error_) + ki_ * static_cast<float>(i_sum_);

    if (signed_speed > MAX_SPEED_RPM)
    {
        signed_speed = MAX_SPEED_RPM;
    }
    else if (signed_speed < -MAX_SPEED_RPM)
    {
        signed_speed = -MAX_SPEED_RPM;
    }

    if (signed_speed < 0.0f)
    {
        dir_ = static_cast<uint8_t>(!default_dir_);
        speed_ = -signed_speed;
    }
    else
    {
        dir_ = default_dir_;
        speed_ = signed_speed;
    }
}

/**
 * @brief 发送速度控制命令
 *
 * @param sync 是否同步执行
 * @return 无
 */
void StepMotorAxis::QueueVelocityControl(bool sync) const
{
    const uint16_t velocity_rpm = static_cast<uint16_t>(speed_ + 0.5f);
    Emm_V5_Vel_Control(addr_, dir_, velocity_rpm, acc_, sync);
}

/**
 * @brief 发送使能控制命令
 *
 * @param enable true 表示使能
 * @param sync 是否同步执行
 * @return 无
 */
void StepMotorAxis::QueueEnable(bool enable, bool sync) const
{
    Emm_V5_En_Control(addr_, enable, sync);
}

/**
 * @brief 发送当前位置清零命令
 *
 * @return 无
 */
void StepMotorAxis::QueueResetCurrentPosition() const
{
    Emm_V5_Reset_CurPos_To_Zero(addr_);
}

/**
 * @brief 发送堵转保护复位命令
 *
 * @return 无
 */
void StepMotorAxis::QueueResetClogProtection() const
{
    Emm_V5_Reset_Clog_Pro(addr_);
}

/**
 * @brief 发送位置控制命令
 *
 * @param dir 方向
 * @param vel 速度，单位：RPM
 * @param acc 加速度
 * @param distance_mm 目标距离，单位：mm
 * @param motion_mode 运动模式
 * @return 无
 */
void StepMotorAxis::PositionControl(uint8_t dir, uint16_t vel, uint8_t acc, float distance_mm, uint8_t motion_mode) const
{
    Emm_V5_Pos_Control(addr_, dir, vel, acc, DistanceToPulse(distance_mm), motion_mode, false);
}

/**
 * @brief 将距离换算成脉冲数
 *
 * @param distance_mm 位移距离，单位：mm
 * @return uint32_t 脉冲数
 */
uint32_t StepMotorAxis::DistanceToPulse(float distance_mm) const
{
    const float pulses = distance_mm / (PI_VALUE * roller_diameter_mm_) * pulse_scale_;
    return static_cast<uint32_t>(pulses > 0.0f ? pulses : 0.0f);
}

/**
 * @brief 恢复 Z 轴堵转保护状态
 *
 * @return 无
 */
void RecoverZAxisFromClog(void)
{
    StepMotor_Z.QueueResetClogProtection();
    StepMotor_Z.QueueEnable(true, false);
    StepMotorCommandDelay();
}

/**
 * @brief 挂起一条 Z 轴位置恢复任务
 *
 * @param dir 方向
 * @param vel 速度，单位：RPM
 * @param acc 加速度
 * @param target_distance_mm 目标距离，单位：mm
 * @param reference_distance_mm 参考距离，单位：mm
 * @param motion_mode 运动模式
 * @return 无
 */
void ArmZAxisPositionRecovery(uint8_t dir,
                              uint16_t vel,
                              uint8_t acc,
                              float target_distance_mm,
                              float reference_distance_mm,
                              uint8_t motion_mode)
{
    g_z_axis_recovery.active = 1U;
    g_z_axis_recovery.dir = dir;
    g_z_axis_recovery.vel = vel;
    g_z_axis_recovery.acc = acc;
    g_z_axis_recovery.target_distance_mm = target_distance_mm;
    g_z_axis_recovery.motion_mode = motion_mode;
    g_z_axis_recovery.start_tick = HAL_GetTick();
    g_z_axis_recovery.window_ms = EstimateZAxisMoveTimeMs(reference_distance_mm, vel);
    g_z_axis_recovery.last_service_tick = 0U;
}

/**
 * @brief 初始化步进电机
 *
 * @return 无
 */
void StepMotor_Init(void)
{
    osDelay(1500);
    ConfigureZAxisDriverProtection();

    StepMotor_Z.ResetState();

    StepMotor_Z.QueueEnable(true, false);
    osDelay(10);
}

/**
 * @brief 根据误差更新 Z 轴速度并发送命令
 *
 * @param zmotor Z 轴对象
 * @return 无
 */
void Motor_Ctrl(StepMotorAxis &zmotor)
{
    zmotor.UpdateSpeedFromError();

    zmotor.QueueVelocityControl(false);
}

/**
 * @brief 执行抓取前的抬升动作
 *
 * @return 无
 */
void FineTuneLiftForWeaponGrip(void)
{
    ArmZAxisPositionRecovery(kGripDirection,
                             kGripVelocityRpm,
                             MOTOR_ACC,
                             g_grip_distance_mm,
                             g_grip_distance_mm,
                             kAbsolutePositionMode);
    StepMotor_ServiceRecovery();
}

/**
 * @brief 将抬升后的 Z 轴恢复到指定位置
 *
 * @param target_distance_mm 目标位置，单位：mm
 * @param reference_distance_mm 参考距离，单位：mm
 * @return 无
 */
void ReturnLiftToPosition(float target_distance_mm, float reference_distance_mm, uint16_t speed_rpm)
{
    if (target_distance_mm < 0.0f)
    {
        target_distance_mm = 0.0f;
    }

    if (reference_distance_mm < 0.0f)
    {
        reference_distance_mm = -reference_distance_mm;
    }

    ArmZAxisPositionRecovery(Z_MOTOR_DIR,
                             speed_rpm,
                             MOTOR_ACC,
                             target_distance_mm,
                             reference_distance_mm,
                             kAbsolutePositionMode);
    StepMotor_ServiceRecovery();
}

/**
 * @brief 将抬升后的 Z 轴恢复到零点
 *
 * @return 无
 */
void ReturnLiftToZero(void)
{
    ReturnLiftToPosition(0.0f, g_grip_distance_mm);
}

/**
 * @brief 周期性维护 Z 轴恢复任务
 *
 * @return 无
 */
void StepMotor_ServiceRecovery(void)
{
    const uint32_t now_tick = HAL_GetTick();

    if (g_z_axis_recovery.active == 0U)
    {
        return;
    }

    if ((now_tick - g_z_axis_recovery.last_service_tick) < kZAxisRecoveryPeriodMs)
    {
        return;
    }

    RecoverZAxisFromClog();
    StepMotor_Z.PositionControl(g_z_axis_recovery.dir,
                                g_z_axis_recovery.vel,
                                g_z_axis_recovery.acc,
                                g_z_axis_recovery.target_distance_mm,
                                g_z_axis_recovery.motion_mode);
    StepMotorCommandDelay();
    g_z_axis_recovery.last_service_tick = now_tick;

    if ((now_tick - g_z_axis_recovery.start_tick) >= g_z_axis_recovery.window_ms)
    {
        g_z_axis_recovery.active = 0U;
        RecoverZAxisFromClog();
    }
}

/**
 * @brief 查询 Z 轴恢复任务是否仍在运行
 *
 * @return bool 仍在运行返回 true
 */
bool StepMotor_IsRecoveryActive(void)
{
    return g_z_axis_recovery.active != 0U;
}

/**
 * @brief 获取当前恢复任务方向
 *
 * @return uint8_t 方向值
 */
uint8_t StepMotor_GetRecoveryDirection(void)
{
    return g_z_axis_recovery.dir;
}

/**
 * @brief 获取当前恢复任务速度
 *
 * @return float 速度，单位：RPM
 */
float StepMotor_GetRecoverySpeedRpm(void)
{
    return static_cast<float>(g_z_axis_recovery.vel);
}
