#include "lift_auto.h"
#include "DT35.h"
#include "mieling.h"
#include "usart_task.h"
#include "lift_class.h"

extern VisionData_t vision;
extern uint8_t vision_block_pop(int *out);

// 靠近阶段最大底盘速度 (m/s)
float STEP_UP_AUTO_APPROACH_MPS = 0.8f;
// 爬升阶段升降最大线速度 (m/s)
float STEP_UP_AUTO_CLIMB_SPEED_MPS = 0.6f;
// 爬升阶段升降最大加速度 (m/s)
float STEP_UP_LIFT_ACC_SPEED = 0.3f;
// 底盘靠近阶段升降最大加速度 (m/s)
float STEP_UP_CHASSIS_ACC_SPEED = 0.8f;
// 靠近目标距离 (mm)，到达后停止前进
uint32_t STEP_UP_AUTO_PREPARE_MM = 25U;
// 爬升完成判定高度 (mm)，低于此值说明已过台阶
uint32_t STEP_UP_AUTO_FINISH_MM = 580U;
// 到达中间台阶距离
uint16_t STEP_UP_AUTO_MIDDLE_MM = 300U;
// 横向目标参考值 (mm)
float STEP_UP_AUTO_LATERAL_REF = 285.0f;
// 激光有效阈值 (mm)，超过此距离认为激光不可用
uint32_t STEP_UP_AUTO_LASER_MAX_MM = 1700U;
// 高度稳定所需连续确认次数，防抖用
uint8_t STEP_UP_AUTO_STABLE_COUNT = 10U;

LiftAuto lift_auto;

float LiftAuto::speed_limit(float speed, float max)
{
    if (speed > max) {
        speed = max;
    }
    if (speed < -max) {
        speed = -max;
    }
    return speed;
}

float LiftAuto::trapezoid_speed(float error, float acc, float max)
{
    if (error == 0.0f || acc <= 0.0f || max <= 0.0f) {
        return 0.0f;
    }

    float speed = sqrtf(2.0f * fabsf(error) * acc);
    if (error < 0.0f) {
        speed = -speed;
    }

    return speed_limit(speed, max);
}

uint8_t LiftAuto::step_up_stable_confirm(uint8_t condition)
{
    // 依旧为0返回
    if (condition == 0U) {
        step_up_stable_count_ = 0U;
        return 0U;
    }

    // 这里开始计数
    if (step_up_stable_count_ < STEP_UP_AUTO_STABLE_COUNT) {
        step_up_stable_count_++;
    }

    // 大于时返回1
    return (step_up_stable_count_ >= STEP_UP_AUTO_STABLE_COUNT) ? 1U : 0U;
}

LiftAuto::LiftAuto()
{
    resetStepUp();
}

// 启动上台阶流程
void LiftAuto::startStepUp(void)
{
    step_up_started_ = 1U;
}

// 停止并复位上台阶流程
void LiftAuto::stopStepUp(void)
{
    resetStepUp();
}

uint8_t LiftAuto::isStepUpFinished(void) const
{
    return (step_up_state_ == STEP_UP_FINISHED) ? 1U : 0U;
}

// 清零上台阶流程状态，回到空闲
void LiftAuto::resetStepUp(void)
{
    step_up_started_                 = 0U;
    step_up_state_                   = STEP_UP_IDLE;
    lift_switch_target_              = 0U;
    lift_linear_speed_target_        = 0.0f;
    chassis_vy_override_             = 0U;
    chassis_vy_target_               = 0.0f;
    step_up_stable_count_            = 0U;
    step_up_crossed_finish_height_   = 0U;
    chassis_vx_target_               = 0.0f;
    step_up_use_radar_               = 0U;
    step_up_block_num_               = 0;
    step_up_radar_x_ref_middle_      = 0.0f;
    step_up_radar_x_ref_climb_       = 0.0f;
    step_up_radar_y_ref_middle_      = 0.0f;
    step_up_lateral_ref_mm_          = STEP_UP_AUTO_LATERAL_REF;
    step_up_laser_max_mm_            = STEP_UP_AUTO_LASER_MAX_MM;
    step_up_middle_lift_command_seq_ = 0U;
    step_up_middle_lift_finished_    = 0U;
}

void LiftAuto::update(void)
{
    // 读取DT35激光测距
    const uint32_t laser_mm   = (uint32_t)dt35.ch0.distance_filtered;
    const uint8_t laser_valid = dt35.ch0.valid;

    // 未启动则持续复位
    if (step_up_started_ == 0U) {
        resetStepUp();
        return;
    }

    // 首次进入自动流程
    if (step_up_state_ == STEP_UP_IDLE) {
        // 使用外部已设置的 step_up_block_num_ 决定传感器模式
        if (step_up_block_num_ == 1 || step_up_block_num_ == 2) {
            step_up_use_radar_ = 0U;
        } else {
            step_up_use_radar_ = 1U;
        }
        step_up_state_ = STEP_UP_APPROACH_Y;
    }

    switch (step_up_state_) {
        case STEP_UP_APPROACH_Y:
            // 先靠近台阶，靠近到位后才触发升降，防侧翻
            chassis_vy_override_      = 1U;
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;

            // 梯形速度曲线靠近，距离越近速度越慢，到位自动停止
            if (laser_valid != 0U) {
                float err          = ((float)laser_mm - (float)STEP_UP_AUTO_PREPARE_MM) * 0.001f;
                chassis_vx_target_ = trapezoid_speed(err, STEP_UP_CHASSIS_ACC_SPEED, STEP_UP_AUTO_APPROACH_MPS);
                chassis_vy_target_ = 0.0f;
            }

            // 到位后需连续N次稳定确认，防误触发
            if (step_up_stable_confirm((laser_valid != 0U && laser_mm <= STEP_UP_AUTO_PREPARE_MM) ? 1U : 0U) != 0U) {
                chassis_vx_target_ = 0.0f;
                chassis_vy_target_ = 0.0f;
                step_up_stable_count_ = 0U;
                step_up_state_        = STEP_UP_CLIMB_FORWARD;
            }

            break;

        case STEP_UP_CLIMB_FORWARD:
            chassis_vy_override_ = 1U;
            chassis_vx_target_   = 0.0f;
            chassis_vy_target_   = 0.0f;
            lift_switch_target_  = 2U;

            if (step_up_use_radar_ != 0U) {
                // 雷达模式：用 vision.x_diff 走到目标 x 点
                float x_err               = step_up_radar_x_ref_climb_ - vision.x_diff;
                lift_linear_speed_target_ = trapezoid_speed(x_err, STEP_UP_LIFT_ACC_SPEED, STEP_UP_AUTO_CLIMB_SPEED_MPS);

                // 到位判定
                if (step_up_stable_confirm((fabsf(x_err) < 0.050f) ? 1U : 0U) != 0U) {
                    lift_switch_target_              = 1U;
                    lift_linear_speed_target_        = 0.0f;
                    step_up_stable_count_            = 0U;
                    step_up_middle_lift_command_seq_ = lift_calulate.command_seq;
                    step_up_middle_lift_finished_    = 0U;
                    step_up_state_                   = STEP_UP_APPROACH_MIDDLE;
                }
            } else {
                // 激光模式：原有逻辑不变
                // 激光读数先升到高处（超过FINISH_MM），标记已爬升
                if (laser_valid != 0U && laser_mm > STEP_UP_AUTO_FINISH_MM && step_up_crossed_finish_height_ != 1) {
                    step_up_crossed_finish_height_ = 1;
                }

                // 梯形速度曲线升降
                if (laser_valid != 0U && step_up_crossed_finish_height_ == 1U) {
                    float err                 = ((float)laser_mm - (float)STEP_UP_AUTO_FINISH_MM) * 0.001f;
                    lift_linear_speed_target_ = trapezoid_speed(err, STEP_UP_LIFT_ACC_SPEED, STEP_UP_AUTO_CLIMB_SPEED_MPS);
                }

                // 必须先爬升到高处，再降回FINISH_MM以下才算完成
                if (step_up_stable_confirm((step_up_crossed_finish_height_ != 0U && laser_valid != 0U && laser_mm <= STEP_UP_AUTO_FINISH_MM) ? 1U : 0U) != 0U) {
                    lift_switch_target_              = 1U;
                    lift_linear_speed_target_        = 0.0f;
                    step_up_stable_count_            = 0U;
                    step_up_middle_lift_command_seq_ = lift_calulate.command_seq;
                    step_up_middle_lift_finished_    = 0U;
                    step_up_state_                   = STEP_UP_APPROACH_MIDDLE;
                }
            }
            break;

        case STEP_UP_APPROACH_MIDDLE: {
            chassis_vy_override_      = 1U;
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;

            if (step_up_use_radar_ != 0U) {
                // ========== 雷达模式 ==========
                // Vx: vision.x_diff 走到目标 x 点（前为正）
                float x_err        = step_up_radar_x_ref_middle_ - vision.x_diff;
                float y_err        = step_up_radar_y_ref_middle_ - vision.y_diff; //15 - 10 = 5 误差为正
                chassis_vx_target_ = trapezoid_speed(x_err, STEP_UP_CHASSIS_ACC_SPEED, STEP_UP_AUTO_APPROACH_MPS);

                // Vy: vision.y_diff 走到目标 y 点（左为正）
                chassis_vy_target_ = trapezoid_speed(y_err, STEP_UP_CHASSIS_ACC_SPEED, STEP_UP_AUTO_APPROACH_MPS);

                // 到位判定：x 和 y 误差都在容差内
                if (step_up_stable_confirm((fabsf(x_err) < 0.050f && fabsf(y_err) < 0.050f && fabsf(chassis_vx_target_) < 0.2f && fabsf(chassis_vy_target_) < 0.2f) ? 1U : 0U) != 0U) {
                    chassis_vx_target_    = 0.0f;
                    chassis_vy_target_    = 0.0f;
                    step_up_stable_count_ = 0U;
                    step_up_state_        = STEP_UP_FINISHED;
                }
            } else {
                // ========== 激光模式 ==========
                // Vx: 前激光 ch0 控制（前为正）
                if (step_up_middle_lift_finished_ == 0U &&
                    lift_calulate.command_seq != step_up_middle_lift_command_seq_ &&
                    lift_calulate.finished == 1U) {
                    // 确认升降任务已经切到新的收回动作，并且该动作已经完成后，才放行底盘。
                    step_up_middle_lift_finished_ = 1U;
                }

                if (laser_valid != 0U && step_up_middle_lift_finished_ != 0U) {
                    float err          = ((float)laser_mm - (float)STEP_UP_AUTO_MIDDLE_MM) * 0.001f;
                    chassis_vx_target_ = trapezoid_speed(err, STEP_UP_CHASSIS_ACC_SPEED, STEP_UP_AUTO_APPROACH_MPS);

                    // Vy: 左右激光横向修正（左为正）
                    float lat_err      = 0.0f;
                    uint8_t lateral_ok = 0U;

                    // 优先左激光 ch1
                    if (dt35.ch1.valid != 0U && dt35.ch1.distance_filtered < (float)step_up_laser_max_mm_) {
                        lat_err    = (dt35.ch1.distance_filtered - step_up_lateral_ref_mm_) * 0.001f;
                        lateral_ok = 1U;
                    }
                    // 左激光超限，尝试右激光 ch2
                    else if (dt35.ch2.valid != 0U && dt35.ch2.distance_filtered < (float)step_up_laser_max_mm_) {
                        lat_err    = (step_up_lateral_ref_mm_ - dt35.ch2.distance_filtered) * 0.001f;
                        lateral_ok = 1U;
                    }

                    if (lateral_ok != 0U) {
                        // 左距和右距的物理方向相反，分支内已统一成 Vy 误差方向。
                        chassis_vy_target_ = trapezoid_speed(lat_err, STEP_UP_CHASSIS_ACC_SPEED, STEP_UP_AUTO_APPROACH_MPS);
                    } else {
                        chassis_vy_target_ = 0.0f;
                    }
                    // 到位判定：Vx 到达中间距离 + Vy 横向收敛
                    if (step_up_stable_confirm((laser_valid != 0U && laser_mm <= (STEP_UP_AUTO_MIDDLE_MM + 10) && laser_mm >= (STEP_UP_AUTO_MIDDLE_MM - 10) && fabsf(chassis_vy_target_) < 0.1f && fabsf(chassis_vx_target_) < 0.1f) ? 1U : 0U) != 0U) {
                        chassis_vx_target_    = 0.0f;
                        chassis_vy_target_    = 0.0f;
                        step_up_stable_count_ = 0U;
                        step_up_state_        = STEP_UP_FINISHED;
                    }
                } else {
                    // 等待升降动作完成期间，底盘保持不动。
                    chassis_vx_target_ = 0.0f;
                    chassis_vy_target_ = 0.0f;
                }

                break;
            }
            break;
        }

        case STEP_UP_FINISHED:
            // 释放底盘控制权，升降回1档，交还手动
            chassis_vy_override_      = 0U;
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;
            chassis_vx_target_        = 0.0f;
            chassis_vy_target_        = 0.0f;
            step_up_use_radar_             = 0U;
            step_up_crossed_finish_height_ = 0;
            break;

        default:
            resetStepUp();
            break;
    }
}

// IDLE时透传手动档位，否则返回自动档位
uint8_t LiftAuto::getLiftSwitch(uint8_t manual_switch) const
{
    if (step_up_state_ == STEP_UP_IDLE) {
        return manual_switch;
    }

    return lift_switch_target_;
}

// IDLE时透传手动速度，否则返回自动速度
float LiftAuto::getLiftLinearSpeedTarget(float manual_target) const
{
    if (step_up_state_ == STEP_UP_IDLE) {
        return manual_target;
    }

    return lift_linear_speed_target_;
}

// 未接管底盘时透传手动Vy，否则返回自动Vy
float LiftAuto::getChassisVyTarget(float manual_target) const
{
    if (chassis_vy_override_ == 0U) {
        return manual_target;
    }

    return chassis_vy_target_;
}

// 未接管底盘时透传手动Vx，否则返回自动Vx
float LiftAuto::getChassisVxTarget(float manual_target) const
{
    if (chassis_vy_override_ == 0U) {
        return manual_target;
    }

    return chassis_vx_target_;
}

// 配置上台阶雷达模式的目标坐标
void LiftAuto::setStepUpRadarTarget(float x_ref_middle, float x_ref_climb, float y_ref_middle)
{
    step_up_radar_x_ref_middle_ = x_ref_middle;
    step_up_radar_x_ref_climb_  = x_ref_climb;
    step_up_radar_y_ref_middle_ = y_ref_middle;
}

void LiftAuto::setStepUpBlockNum(int num)
{
    step_up_block_num_ = num;
}
