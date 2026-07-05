#include "arm_comm.h"
#include "usart.h"
#include <math.h>

// 全局机械臂通信对象，和其他路线模块保持同样的使用方式。
ArmComm arm_comm;

// 取 KFS 前底盘预走阶段的最大速度，单位 m/s。
float ARM_COMM_PICK_KFS_CHASSIS_SPEED_MPS = 0.5f;
// 取 KFS 前底盘预走阶段的制动包络参数。
float ARM_COMM_PICK_KFS_CHASSIS_ACC_SPEED = 0.5f;
// 取 KFS 前底盘到位判定阈值，单位 m。
float ARM_COMM_PICK_KFS_POSITION_TOL_M = 0.02f;
// 取 KFS 前底盘到位需要连续满足的周期数。
uint8_t ARM_COMM_PICK_KFS_STABLE_COUNT = 10U;

// 未上台阶取 KFS 前沿当前 X 轴预走距离，单位 cm。
float PICK_KFS_BEFORE_STEP_ADVANCE_CM = 42.0f;
// 已上台阶取 KFS 前按当前 yaw 方向预走距离，单位 cm。
float PICK_KFS_AFTER_STEP_ADVANCE_CM = 38.0f;

namespace
{
    static void pick_kfs_world_error_to_body_error(float x_world, float y_world, float yaw_deg, float *x_body, float *y_body)
    {
        const float deg_to_rad = 0.01745329251994329577f;
        const float yaw_rad = yaw_deg * deg_to_rad;
        const float cos_yaw = cosf(yaw_rad);
        const float sin_yaw = sinf(yaw_rad);

        // 将世界系误差转换为车体系误差，底盘速度接口使用车体系坐标。
        *x_body = cos_yaw * x_world + sin_yaw * y_world;
        *y_body = -sin_yaw * x_world + cos_yaw * y_world;
    }

    static float normalize_yaw_deg(float yaw_deg)
    {
        while (yaw_deg > 180.0f)
        {
            yaw_deg -= 360.0f;
        }
        while (yaw_deg < -180.0f)
        {
            yaw_deg += 360.0f;
        }
        return yaw_deg;
    }
}

ArmComm::ArmComm()
{
    rx_data_.event = 0x00U;
    rx_data_.sys_mode = 0x00U;
    rx_data_.arm_kfs = 0x00U;
    rx_data_.car_kfs = 0x00U;
    resetPickKFS();
    reset();
}

void ArmComm::reset(void)
{
    setFrame(0x00U, 0x00U, 0x00U, 0x00U, 0x00U);
}

uint8_t ArmComm::executeAction(uint8_t action_code, uint8_t num_KFS)
{
    switch (action_code)
    {
    case ACTION_POWER_ON_INIT: // 初始化
        setFrame(0x01U, 0x00U, 0x00U, 0x00U, 0x00U);
        break;

    case ACTION_PICK_HIGH_200:
        setFrame(0x01U, num_KFS, 0x01U, 0x00U, 0x00U);
        break;

    case ACTION_PICK_HIGH_400:
        setFrame(0x01U, num_KFS, 0x02U, 0x00U, 0x00U);
        break;

    case ACTION_PICK_LOW_200:
        setFrame(0x01U, num_KFS, 0x03U, 0x00U, 0x00U);
        break;

    case ACTION_ZONE3_READY: // 去九宫格预备位
        setFrame(0x01U, 0x02U, 0x04U, 0x01U, 0x00U);
        break;

    case ACTION_ZONE3_PLACE_HAND: // 放手持KFS
        setFrame(0x01U, 0x02U, 0x04U, 0x02U, 0x00U);
        break;

    case ACTION_ZONE3_FETCH_UPPER:
        setFrame(0x01U, num_KFS, 0x00U, 0x00U, 0x00U);
        break;

    case ACTION_ZONE3_PLACE_UPPER:
        setFrame(0x01U, 0x00U, 0x00U, 0x02U, 0x00U);
        break;

    case ACTION_ZONE3_FETCH_LOWER: // 取车底层KFS
        setFrame(0x01U, 0x02U, 0x04U, 0x03U, 0x00U);
        break;

    case ACTION_ZONE3_PLACE_LOWER: // 放车底层KFS
        setFrame(0x01U, 0x02U, 0x04U, 0x04U, 0x00U);
        break;
    case ACTION_ZONE3_RESET:
        setFrame(0x01U, 0x00U, 0x00U, 0x00U, 0x00U);
        break;

    case ACTION_POWER_OFF:
        setFrame(0x00U, 0x00U, 0x00U, 0x00U, 0x00U);
        break;

    case ACTION_PICK_FIRST_KFS:
        setFrame(0x01U, 0x01U, 0x04U, 0x00U, 0x00U);
        break;

    case ACTION_PICK_SECOND_KFS:
        setFrame(0x01U, 0x02U, 0x04U, 0x00U, 0x00U);
        break;

    default:
        return 0U;
    }

    return 1U;
}

const uint8_t *ArmComm::getFrame(void) const
{
    return frame_;
}

uint8_t ArmComm::getFrameLength(void) const
{
    return (uint8_t)FRAME_LENGTH;
}

uint8_t ArmComm::parseRxFrame(const uint8_t *data, uint8_t length)
{
    if (data == 0 || length != RX_FRAME_LENGTH)
    {
        return 0U;
    }

    if (data[RX_INDEX_HEAD] != 0xBBU || data[RX_INDEX_TAIL] != 0xEEU)
    {
        return 0U;
    }

    rx_data_.event = data[RX_INDEX_EVENT];
    rx_data_.sys_mode = data[RX_INDEX_SYS_MODE];
    rx_data_.arm_kfs = data[RX_INDEX_ARM_KFS];
    rx_data_.car_kfs = data[RX_INDEX_CAR_KFS];

    return 1U;
}

const ArmComm::RxData &ArmComm::getRxData(void) const
{
    return rx_data_;
}

uint8_t ArmComm::pickKFS(uint8_t action_code,
                         uint8_t num_KFS,
                         uint8_t already_step_up,
                         float current_x_m,
                         float current_y_m,
                         float yaw_deg,
                         uint8_t finish_at_center,
                         float center_x_m,
                         float center_y_m)
{
    const float before_step_x_m = PICK_KFS_BEFORE_STEP_ADVANCE_CM * 0.01f;
    const float after_step_x_m = PICK_KFS_AFTER_STEP_ADVANCE_CM * 0.01f;

    if (pick_kfs_state_ == PICK_KFS_IDLE)
    {
        pick_kfs_action_code_ = action_code;
        pick_kfs_num_ = num_KFS;
        pick_kfs_stable_count_ = 0U;
        pick_kfs_vx_target_ = 0.0f;
        pick_kfs_vy_target_ = 0.0f;
        pick_kfs_zero_yaw_move_ = 0U;
        rx_data_.event = 0U;

        if (executeAction(pick_kfs_action_code_, pick_kfs_num_) == 0U)
        {
            resetPickKFS();
            return 0U;
        }

        send();

        if (already_step_up == 0U)
        {
            if (before_step_x_m == 0.0f)
            {
                pick_kfs_target_x_m_ = current_x_m;
                pick_kfs_target_y_m_ = current_y_m;
            }
            else
            {
                pick_kfs_target_x_m_ = current_x_m + before_step_x_m;
                pick_kfs_target_y_m_ = current_y_m;
                pick_kfs_zero_yaw_move_ = 1U;
            }
        }
        else if (after_step_x_m == 0.0f)
        {
            pick_kfs_target_x_m_ = current_x_m;
            pick_kfs_target_y_m_ = current_y_m;
        }
        else
        {
            const float yaw = normalize_yaw_deg(yaw_deg);

            pick_kfs_target_x_m_ = center_x_m;
            pick_kfs_target_y_m_ = center_y_m;

            if (yaw >= 45.0f && yaw < 135.0f)
            {
                pick_kfs_target_y_m_ = center_y_m + after_step_x_m;
            }
            else if (yaw <= -45.0f && yaw > -135.0f)
            {
                pick_kfs_target_y_m_ = center_y_m - after_step_x_m;
            }
            else
            {
                pick_kfs_target_x_m_ = center_x_m + after_step_x_m;
            }
        }

        pick_kfs_state_ = PICK_KFS_SEND;
    }

    if (pick_kfs_state_ == PICK_KFS_SEND)
    {
        pick_kfs_vx_target_ = 0.0f;
        pick_kfs_vy_target_ = 0.0f;

        if (rx_data_.event == 5U)
        {
            rx_data_.event = 0U;
            pick_kfs_stable_count_ = 0U;
            pick_kfs_state_ = PICK_KFS_MOVE;
        }
    }

    if (pick_kfs_state_ == PICK_KFS_MOVE || pick_kfs_state_ == PICK_KFS_RETURN_CENTER)
    {
        const float x_err_world = pick_kfs_target_x_m_ - current_x_m;
        const float y_err_world = pick_kfs_target_y_m_ - current_y_m;
        float x_err_body = 0.0f;
        float y_err_body = 0.0f;
        const uint8_t return_center = (pick_kfs_state_ == PICK_KFS_RETURN_CENTER) ? 1U : 0U;

        pick_kfs_world_error_to_body_error(x_err_world,
                                           y_err_world,
                                           (return_center == 0U && pick_kfs_zero_yaw_move_ != 0U) ? 0.0f : yaw_deg,
                                           &x_err_body,
                                           &y_err_body);

        pick_kfs_vx_target_ = trapezoid_speed(x_err_body,
                                              ARM_COMM_PICK_KFS_CHASSIS_ACC_SPEED,
                                              ARM_COMM_PICK_KFS_CHASSIS_SPEED_MPS);
        pick_kfs_vy_target_ = trapezoid_speed(y_err_body,
                                              ARM_COMM_PICK_KFS_CHASSIS_ACC_SPEED,
                                              ARM_COMM_PICK_KFS_CHASSIS_SPEED_MPS);

        if (pick_kfs_stable_confirm((fabsf(x_err_world) < ARM_COMM_PICK_KFS_POSITION_TOL_M &&
                                     fabsf(y_err_world) < ARM_COMM_PICK_KFS_POSITION_TOL_M)
                                        ? 1U
                                        : 0U) != 0U)
        {
            pick_kfs_vx_target_ = 0.0f;
            pick_kfs_vy_target_ = 0.0f;
            if (return_center != 0U)
            {
                resetPickKFS();
                return 1U;
            }
            else
            {
                pick_kfs_stable_count_ = 0U;
                pick_kfs_state_ = PICK_KFS_WAIT_DONE;
            }
        }
    }

    if (pick_kfs_state_ == PICK_KFS_WAIT_DONE)
    {
        pick_kfs_vx_target_ = 0.0f;
        pick_kfs_vy_target_ = 0.0f;
    }

    if (rx_data_.event == 1U)
    {
        if (finish_at_center != 0U)
        {
            pick_kfs_target_x_m_ = center_x_m;
            pick_kfs_target_y_m_ = center_y_m;
            pick_kfs_vx_target_ = 0.0f;
            pick_kfs_vy_target_ = 0.0f;
            pick_kfs_stable_count_ = 0U;
            pick_kfs_zero_yaw_move_ = 0U;
            rx_data_.event = 0U;
            pick_kfs_state_ = PICK_KFS_RETURN_CENTER;
            return 0U;
        }

        resetPickKFS();
        return 1U;
    }

    return 0U;
}

float ArmComm::getChassisVxTarget(float manual_target) const
{
    if (pick_kfs_state_ == PICK_KFS_IDLE)
    {
        return manual_target;
    }

    return pick_kfs_vx_target_;
}

float ArmComm::getChassisVyTarget(float manual_target) const
{
    if (pick_kfs_state_ == PICK_KFS_IDLE)
    {
        return manual_target;
    }

    return pick_kfs_vy_target_;
}

void ArmComm::resetPickKFS(void)
{
    pick_kfs_state_ = PICK_KFS_IDLE;
    pick_kfs_action_code_ = 0U;
    pick_kfs_num_ = 0U;
    pick_kfs_stable_count_ = 0U;
    pick_kfs_target_x_m_ = 0.0f;
    pick_kfs_target_y_m_ = 0.0f;
    pick_kfs_vx_target_ = 0.0f;
    pick_kfs_vy_target_ = 0.0f;
    pick_kfs_zero_yaw_move_ = 0U;
}

void ArmComm::send(void)
{
    HAL_UART_Transmit_DMA(&huart7, frame_, (uint16_t)FRAME_LENGTH);
}

void ArmComm::setFrame(uint8_t boot,
                       uint8_t pick_count,
                       uint8_t kfs_height,
                       uint8_t zone3_cmd,
                       uint8_t r2r1_fused)
{
    frame_[INDEX_HEAD] = 0xAAU;
    frame_[INDEX_BOOT] = boot;
    frame_[INDEX_PICK_COUNT] = pick_count;
    frame_[INDEX_KFS_HEIGHT] = kfs_height;
    frame_[INDEX_ZONE3_CMD] = zone3_cmd;
    frame_[INDEX_R2R1_FUSED] = r2r1_fused;
    frame_[INDEX_TAIL] = 0x55U;
}

float ArmComm::speed_limit(float speed, float max) const
{
    if (speed > max)
    {
        speed = max;
    }
    if (speed < -max)
    {
        speed = -max;
    }
    return speed;
}

float ArmComm::trapezoid_speed(float error, float acc, float max) const
{
    if (error == 0.0f || acc <= 0.0f || max <= 0.0f)
    {
        return 0.0f;
    }

    float speed = sqrtf(2.0f * fabsf(error) * acc);
    if (error < 0.0f)
    {
        speed = -speed;
    }

    return speed_limit(speed, max);
}

uint8_t ArmComm::pick_kfs_stable_confirm(uint8_t condition)
{
    if (condition == 0U)
    {
        pick_kfs_stable_count_ = 0U;
        return 0U;
    }

    if (pick_kfs_stable_count_ < ARM_COMM_PICK_KFS_STABLE_COUNT)
    {
        pick_kfs_stable_count_++;
    }

    return (pick_kfs_stable_count_ >= ARM_COMM_PICK_KFS_STABLE_COUNT) ? 1U : 0U;
}
