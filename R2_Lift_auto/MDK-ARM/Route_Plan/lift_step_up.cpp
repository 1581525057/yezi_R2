#include "lift_step_up.h"
#include "DT35.h"
#include "laser_distance.h"
#include "reolcation.h"
#include "usart_task.h"
#include "lift_class.h"
#include <math.h>

// 上400：1.抬升抬到正200 2.放气缸 3.抬升抬到-200  4.2006开始向前 5.收气缸 6.抬升抬到+100 7.回到中心点

extern VisionData_t vision;
extern uint8_t vision_block_pop(int *out);

#ifndef STEP_UP_DEBUG_MANUAL_STEP_CMD
#define STEP_UP_DEBUG_MANUAL_STEP_CMD 0
#endif

#if STEP_UP_DEBUG_MANUAL_STEP_CMD
uint8_t step_up_debug_step_next_cmd = 0U; // 调试用：置 1 后只允许上台阶状态机切换一次。
#define STEP_UP_DEBUG_ALLOW_NEXT() ((step_up_debug_step_next_cmd != 0U) ? (step_up_debug_step_next_cmd = 0U, 1U) : 0U)
#else
#define STEP_UP_DEBUG_ALLOW_NEXT() 1U
#endif

static void step_up_world_error_to_body_error(float x_world, float y_world, float yaw_deg, float *x_body, float *y_body)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    const float deg_to_rad = 0.01745329251994329577f;
    const float yaw_rad = yaw_deg * deg_to_rad;
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);

    // 将世界系位置误差转换为车体系位置误差，yaw 左转为正、右转为负。
    *x_body = cos_yaw * x_world + sin_yaw * y_world;
    *y_body = -sin_yaw * x_world + cos_yaw * y_world;
}

// 靠近阶段最大底盘速度 (m/s)
float STEP_UP_AUTO_APPROACH_MPS = 1.5f;
// 底盘靠近阶段升降最大加速度 (m/s)
float STEP_UP_CHASSIS_ACC_SPEED = 0.7f;
// 爬升阶段未到完成区时的最小线速度，避免小误差下卡在静摩擦附近。
float STEP_UP_AUTO_CLIMB_MIN_SPEED_MPS = 0.25f;

// 爬升阶段升降最大线速度 (m/s)
float STEP_UP_AUTO_CLIMB_SPEED_MPS = 1.35f;
// 爬升阶段升降最大加速度 (m/s)
float STEP_UP_LIFT_ACC_SPEED = 1.0f;

// 配置雷达爬升阶段的前进距离 L，单位为 m。
float STEP_UP_RADAR_CLIMB_DISTANCE_M = 0.82f;
// 靠近目标距离 (mm)，到达后停止前进
uint32_t STEP_UP_AUTO_PREPARE_MM = 60U;
// 爬升完成判定高度 (mm)，低于此值说明已过台阶
uint32_t STEP_UP_AUTO_FINISH_MM = 685U;
// 到达中间台阶距离
uint16_t STEP_UP_AUTO_MIDDLE_MM = 338U;
// 横向目标参考值 (mm)
float STEP_UP_AUTO_LATERAL_REF = 250.0f;
// 激光有效阈值 (mm)，超过此距离认为激光不可用
uint32_t STEP_UP_AUTO_LASER_MAX_MM = 1700U;
// 高度稳定所需连续确认次数，防抖用
uint8_t STEP_UP_AUTO_STABLE_COUNT = 10U;
// 2 档等待时按实际电机角度换算出的左右高度判定到位，避免只看轨迹 finished 提前跳转。
float STEP_UP_LIFT_HEIGHT_TOLERANCE_MM = 40.0f;

LiftAuto lift_auto;

float LiftAuto::speed_limit(float speed, float max)
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

float LiftAuto::trapezoid_speed(float error, float acc, float max)
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

static float step_up_min_abs_speed(float speed, float min_abs, float max_abs)
{
    if (speed == 0.0f || min_abs <= 0.0f)
    {
        return speed;
    }

    if (fabsf(speed) < min_abs)
    {
        speed = (speed > 0.0f) ? min_abs : -min_abs;
    }

    return (speed > 0.0f) ? ((speed > max_abs) ? max_abs : speed)
                          : ((speed < -max_abs) ? -max_abs : speed);
}

uint8_t LiftAuto::step_up_stable_confirm(uint8_t condition)
{
    // 依旧为0返回
    if (condition == 0U)
    {
        step_up_stable_count_ = 0U;
        return 0U;
    }

    // 这里开始计数
    if (step_up_stable_count_ < STEP_UP_AUTO_STABLE_COUNT)
    {
        step_up_stable_count_++;
    }

    // 大于时返回1
    return (step_up_stable_count_ >= STEP_UP_AUTO_STABLE_COUNT) ? 1U : 0U;
}

LiftAuto::LiftAuto()
{
    step_up_radar_last_x_ref_middle_ = 0.0f;
    step_up_radar_last_y_ref_middle_ = 0.0f;
    step_up_radar_last_middle_valid_ = 0U;
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

// 只清运行状态，不清视觉命令提前配置好的本次上台阶参数。
void LiftAuto::resetStepUpRuntime(void)
{
    step_up_started_ = 0U;
    step_up_state_ = STEP_UP_IDLE;
    lift_switch_target_ = 0U;
    lift_linear_speed_target_ = 0.0f;
    chassis_vy_override_ = 0U;
    chassis_vy_target_ = 0.0f;
    step_up_stable_count_ = 0U;
    step_up_crossed_finish_height_ = 0U;
    step_up_pre_lift_command_seq_ = 0U;
    step_up_pre_lift_started_ = 0U;
    step_up_pre_lift_ready_ = 0U;
    chassis_vx_target_ = 0.0f;
    step_up_use_radar_ = 0U;
    step_up_radar_climb_target_ = 0.0f;
    step_up_radar_climb_target_valid_ = 0U;
    step_up_climb_lift_command_seq_ = 0U;
    step_up_middle_lift_command_seq_ = 0U;
}

// 清零上台阶流程状态，回到空闲
void LiftAuto::resetStepUp(void)
{
    resetStepUpRuntime();
    step_up_height_mode_mm_ = 200U;
    step_up_return_middle_ = 1U;
    step_up_block_num_ = 0;
    step_up_radar_x_ref_middle_ = 0.0f;
    step_up_radar_y_ref_middle_ = 0.0f;
    step_up_radar_x_ref_climb_base_ = 0.0f;
    step_up_radar_y_ref_climb_base_ = 0.0f;
    step_up_radar_climb_y_direction_ = 0;
    step_up_lateral_ref_mm_ = STEP_UP_AUTO_LATERAL_REF;
    step_up_laser_max_mm_ = STEP_UP_AUTO_LASER_MAX_MM;
}

// 上台阶自动流程的周期调度函数。
// 这个函数由任务循环反复调用，每调用一次就根据当前状态推进一小步：
// 1. 读取前向激光和雷达/视觉坐标，判断车是否靠近到位、爬升是否到位。
// 2. 根据 step_up_state_ 状态机输出底盘速度、升降档位、升降线速度和气缸动作。
// 3. 通过 command_seq + finished 判断升降机构是否真正接收并完成了新的高度轨迹。
// 4. 流程未启动时持续复位输出，流程完成后释放底盘和升降控制权给外部任务。
// 注意：这里不直接阻塞等待硬件动作，而是靠状态机在多个周期中逐步推进。
void LiftAuto::update(void)
{
    // 读取前向DT35激光测距
    const uint32_t laser_mm = (uint32_t)dt35.ch2.distance_filtered;
    const uint8_t laser_valid = dt35.ch2.valid;

    // 未启动则持续复位
    if (step_up_started_ == 0U)
    {
        resetStepUpRuntime();
        return;
    }

    // 首次进入自动流程
    if (step_up_state_ == STEP_UP_IDLE)
    {
        // 使用外部已设置的 step_up_block_num_ 决定传感器模式
        if (step_up_block_num_ == 2)
        {
            step_up_use_radar_ = 0U;
        }
        else
        {
            step_up_use_radar_ = 1U;
        }
        if (step_up_height_mode_mm_ == 400U)
        {
            step_up_pre_lift_command_seq_ = lift_calulate.command_seq;
            step_up_pre_lift_started_ = 0U;
            step_up_pre_lift_ready_ = 0U;
        }
        if (STEP_UP_DEBUG_ALLOW_NEXT() != 0U)
        {
            step_up_state_ = STEP_UP_APPROACH_Y;
        }
    }

    switch (step_up_state_) // 靠近Y
    {
    case STEP_UP_IDLE:
        break;

    case STEP_UP_APPROACH_Y:
        // 先靠近台阶，靠近到位后才触发升降，防侧翻
        chassis_vy_override_ = 1U;
        if (step_up_height_mode_mm_ == 400U)
        {
            // 400mm 档需要先把升降机构预抬到最高，再打开气缸。
            // 这里不阻塞底盘靠近流程：预抬升状态检测放在梯形速度曲线前执行，
            // 下面仍会继续根据激光距离计算靠近速度，实现“边靠近、边预抬升”。
            lift_switch_target_ = 1U;

            // command_seq 变化说明升降任务已经接收到新的 1 档目标。
            // 1 档目标可能本来就在当前高度，finished 会直接为 1，不能强制要求先观察到 0。
            if (lift_calulate.command_seq != step_up_pre_lift_command_seq_)
            {
                step_up_pre_lift_started_ = 1U;
                if (lift_calulate.finished != 0U)
                {
                    step_up_pre_lift_ready_ = 1U;
                }
            }

            // 预抬升确认完成后打开气缸；后续是否进入 2 档爬升等待，
            // 还要等靠近距离也稳定到位，避免未贴近台阶时提前切换动作。
            if (step_up_pre_lift_ready_ != 0U)
            {
                STEP_UP_CYLINDER_OPEN();
            }
        }
        else
        {
            lift_switch_target_ = 3U;
        }
        lift_linear_speed_target_ = 0.0f;

        // 梯形速度曲线靠近，距离越近速度越慢，到位自动停止
        if (laser_valid != 0U)
        {
            float err = ((float)laser_mm - (float)STEP_UP_AUTO_PREPARE_MM) * 0.001f;
            chassis_vx_target_ = trapezoid_speed(err, STEP_UP_CHASSIS_ACC_SPEED, STEP_UP_AUTO_APPROACH_MPS);
            chassis_vy_target_ = 0.0f;
        }

        // 到位后需连续N次稳定确认，防误触发
        if (step_up_stable_confirm((laser_valid != 0U && laser_mm <= STEP_UP_AUTO_PREPARE_MM) ? 1U : 0U) != 0U)
        {
            chassis_vx_target_ = 0.0f;
            chassis_vy_target_ = 0.0f;
            if (step_up_height_mode_mm_ == 400U)
            {
                if (step_up_pre_lift_ready_ != 0U && STEP_UP_DEBUG_ALLOW_NEXT() != 0U)
                {
                    step_up_stable_count_ = 0U;
                    step_up_climb_lift_command_seq_ = lift_calulate.command_seq;
                    step_up_state_ = STEP_UP_WAIT_CLIMB_HEIGHT;
                }
            }
            else if (STEP_UP_DEBUG_ALLOW_NEXT() != 0U)
            {
                step_up_stable_count_ = 0U;
                step_up_climb_lift_command_seq_ = lift_calulate.command_seq;
                step_up_state_ = STEP_UP_WAIT_CLIMB_HEIGHT;
            }
        }

        break;

    case STEP_UP_WAIT_CLIMB_HEIGHT:
    {
        // 等待 2 档高度轨迹生成并完成，确认抬升任务完成后再进入边走边升阶段。
        chassis_vy_override_ = 1U;
        chassis_vx_target_ = 0.0f;
        chassis_vy_target_ = 0.0f;
        lift_switch_target_ = 2U;
        lift_linear_speed_target_ = 0.0f;

        const uint8_t climb_height_reached =
            (fabsf(lift_class.left.height - lift_calulate.target_height) <= STEP_UP_LIFT_HEIGHT_TOLERANCE_MM &&
             fabsf(lift_class.right.height - lift_calulate.target_height) <= STEP_UP_LIFT_HEIGHT_TOLERANCE_MM)
                ? 1U
                : 0U;
        if (lift_calulate.command_seq != step_up_climb_lift_command_seq_ &&
            climb_height_reached != 0U &&
            STEP_UP_DEBUG_ALLOW_NEXT() != 0U)
        {
            step_up_state_ = STEP_UP_CLIMB_FORWARD;
        }
        break;
    }

    case STEP_UP_CLIMB_FORWARD:
        chassis_vy_override_ = 1U;
        chassis_vx_target_ = 0.0f;
        chassis_vy_target_ = 0.0f;
        lift_switch_target_ = 2U;

        if (step_up_use_radar_ != 0U)
        {
            // 雷达模式：首次进入爬升阶段时锁存目标，避免每帧刷新当前坐标加 L。
            if (step_up_radar_climb_target_valid_ == 0U)
            {
                if (step_up_radar_climb_y_direction_ > 0)
                {
                    step_up_radar_climb_target_ = step_up_radar_y_ref_climb_base_ + STEP_UP_RADAR_CLIMB_DISTANCE_M;
                }
                else if (step_up_radar_climb_y_direction_ < 0)
                {
                    step_up_radar_climb_target_ = step_up_radar_y_ref_climb_base_ - STEP_UP_RADAR_CLIMB_DISTANCE_M;
                }
                else if (step_up_block_num_ >= 1 &&
                         step_up_block_num_ <= 3)
                {
                    // 入口方块 1/2/3 没有上一台阶中心，X 方向爬升从本方块基准向负方向走 L。
                    step_up_radar_climb_target_ = step_up_radar_x_ref_climb_base_ - 0.37f;
                }
                else
                {
                    step_up_radar_climb_target_ = step_up_radar_x_ref_climb_base_ + STEP_UP_RADAR_CLIMB_DISTANCE_M;
                }
                step_up_radar_climb_target_valid_ = 1U;
            }

            float climb_pos = (step_up_radar_climb_y_direction_ != 0) ? vision.y_diff : vision.x_diff;
            float climb_err = step_up_radar_climb_target_ - climb_pos;
            const uint8_t climb_reached = (fabsf(climb_err) < 0.030f) ? 1U : 0U;
            if (climb_reached != 0U)
            {
                lift_linear_speed_target_ = 0.0f;
            }
            else
            {
                lift_linear_speed_target_ = trapezoid_speed(climb_err, STEP_UP_LIFT_ACC_SPEED, STEP_UP_AUTO_CLIMB_SPEED_MPS);
                if (step_up_radar_climb_y_direction_ < 0)
                {
                    lift_linear_speed_target_ = -lift_linear_speed_target_;
                }
                lift_linear_speed_target_ = step_up_min_abs_speed(lift_linear_speed_target_,
                                                                  STEP_UP_AUTO_CLIMB_MIN_SPEED_MPS,
                                                                  STEP_UP_AUTO_CLIMB_SPEED_MPS);
            }

            // 到位判定
            if (step_up_stable_confirm(climb_reached) != 0U)
            {
                lift_linear_speed_target_ = 0.0f;
                if (STEP_UP_DEBUG_ALLOW_NEXT() != 0U)
                {
                    lift_switch_target_ = 3U;
                    if (step_up_height_mode_mm_ == 400U)
                    {
                        STEP_UP_CYLINDER_CLOSE();
                    }
                    step_up_stable_count_ = 0U;
                    step_up_middle_lift_command_seq_ = lift_calulate.command_seq;
                    step_up_state_ = STEP_UP_WAIT_NEW_HEIGHT;
                }
            }
        }
        else
        {
            // 激光模式：原有逻辑不变
            // 激光读数先升到高处（超过FINISH_MM），标记已爬升
            if (laser_valid != 0U && laser_mm > STEP_UP_AUTO_FINISH_MM && step_up_crossed_finish_height_ != 1)
            {
                step_up_crossed_finish_height_ = 1;
            }

            // 梯形速度曲线升降
            const uint8_t laser_reached = (step_up_crossed_finish_height_ != 0U && laser_valid != 0U && laser_mm <= STEP_UP_AUTO_FINISH_MM) ? 1U : 0U;
            if (laser_reached != 0U)
            {
                lift_linear_speed_target_ = 0.0f;
            }
            else if (laser_valid != 0U && step_up_crossed_finish_height_ == 1U)
            {
                float err = ((float)laser_mm - (float)STEP_UP_AUTO_FINISH_MM) * 0.001f;
                lift_linear_speed_target_ = trapezoid_speed(err, STEP_UP_LIFT_ACC_SPEED, STEP_UP_AUTO_CLIMB_SPEED_MPS);
                lift_linear_speed_target_ = step_up_min_abs_speed(lift_linear_speed_target_,
                                                                  STEP_UP_AUTO_CLIMB_MIN_SPEED_MPS,
                                                                  STEP_UP_AUTO_CLIMB_SPEED_MPS);
            }

            // 必须先爬升到高处，再降回FINISH_MM以下才算完成
            if (step_up_stable_confirm(laser_reached) != 0U)
            {
                lift_linear_speed_target_ = 0.0f;
                if (STEP_UP_DEBUG_ALLOW_NEXT() != 0U)
                {
                    lift_switch_target_ = 3U;
                    if (step_up_height_mode_mm_ == 400U)
                    {
                        STEP_UP_CYLINDER_CLOSE();
                    }
                    step_up_stable_count_ = 0U;
                    step_up_middle_lift_command_seq_ = lift_calulate.command_seq;
                    step_up_state_ = STEP_UP_WAIT_NEW_HEIGHT;
                }
            }
        }
        break;

    case STEP_UP_WAIT_NEW_HEIGHT:
    {
        // 等待新的 3 档收回轨迹生成并完成，期间底盘保持不动。
        chassis_vy_override_ = 1U;
        chassis_vx_target_ = 0.0f;
        chassis_vy_target_ = 0.0f;
        lift_switch_target_ = 3U;
        lift_linear_speed_target_ = 0.0f;

        // step_up_middle_lift_command_seq_ 记录的是进入本状态前的升降轨迹序号。
        // command_seq 发生变化，说明升降控制任务已经接收到新的 3 档收回目标；
        // 同时左右实际高度都到位，说明这条新的收回动作已经执行完成。
        // 两个条件都满足后，才允许离开等待状态，避免复用上一条轨迹的完成标志误跳转。
        const uint8_t middle_height_reached =
            (fabsf(lift_class.left.height - lift_calulate.target_height) <= STEP_UP_LIFT_HEIGHT_TOLERANCE_MM &&
             fabsf(lift_class.right.height - lift_calulate.target_height) <= STEP_UP_LIFT_HEIGHT_TOLERANCE_MM)
                ? 1U
                : 0U;
        if (lift_calulate.command_seq != step_up_middle_lift_command_seq_ &&
            middle_height_reached != 0U &&
            STEP_UP_DEBUG_ALLOW_NEXT() != 0U)
        {
            // 如果后续还需要回到台阶中心，则进入 APPROACH_MIDDLE 继续用雷达靠近中心点。
            // 如果外部配置为不回中心，说明后面通常还会连续执行下一次上台阶，
            // 此时直接结束本次上台阶流程，把控制权交还给路线任务去接下一条指令。
            step_up_state_ = (step_up_return_middle_ != 0U) ? STEP_UP_APPROACH_MIDDLE : STEP_UP_FINISHED;
        }
        break;
    }

    case STEP_UP_APPROACH_MIDDLE:
    {
        chassis_vy_override_ = 1U;
        lift_switch_target_ = 3U;
        lift_linear_speed_target_ = 0.0f;

        if (step_up_use_radar_ != 0U)
        {
            // ========== 雷达模式 ==========
            // 目标点是世界系坐标，底盘速度接口使用车体系坐标。
            float x_err_world = step_up_radar_x_ref_middle_ - vision.x_diff;
            float y_err_world = step_up_radar_y_ref_middle_ - vision.y_diff; // 15 - 10 = 5 误差为正
            float x_err_body = 0.0f;
            float y_err_body = 0.0f;
            step_up_world_error_to_body_error(x_err_world, y_err_world, vision.angle_x, &x_err_body, &y_err_body);

            chassis_vx_target_ = trapezoid_speed(x_err_body, STEP_UP_CHASSIS_ACC_SPEED, STEP_UP_AUTO_APPROACH_MPS);

            // Vy: vision.y_diff 走到目标 y 点（左为正）
            chassis_vy_target_ = trapezoid_speed(y_err_body, STEP_UP_CHASSIS_ACC_SPEED, STEP_UP_AUTO_APPROACH_MPS);

            // 到位判定：x 和 y 误差都在容差内
            if (step_up_stable_confirm((fabsf(x_err_world) < 0.040f && fabsf(y_err_world) < 0.030f && fabsf(chassis_vx_target_) < 0.2f && fabsf(chassis_vy_target_) < 0.2f) ? 1U : 0U) != 0U)
            {
                chassis_vx_target_ = 0.0f;
                chassis_vy_target_ = 0.0f;
                if (STEP_UP_DEBUG_ALLOW_NEXT() != 0U)
                {
                    step_up_stable_count_ = 0U;
                    step_up_state_ = STEP_UP_FINISHED;
                }
            }
        }
        else
        {
            // ========== 激光模式 ==========
            // Vx: 前激光 ch2 控制（前为正）
            if (laser_valid != 0U)
            {
                float err = ((float)laser_mm - (float)STEP_UP_AUTO_MIDDLE_MM) * 0.001f;
                chassis_vx_target_ = trapezoid_speed(err, STEP_UP_CHASSIS_ACC_SPEED, STEP_UP_AUTO_APPROACH_MPS);

                // Vy: 左右激光横向修正（左为正）
                float lat_err = 0.0f;
                uint8_t lateral_ok = 0U;

                // 优先使用左侧串口激光
                if (laser_left.data.valid != 0U && laser_left.data.distance_mm < step_up_laser_max_mm_)
                {
                    lat_err = ((float)laser_left.data.distance_mm - step_up_lateral_ref_mm_) * 0.001f;
                    lateral_ok = 1U;
                }
                // 左侧激光不可用时，尝试右侧串口激光
                else if (laser_right.data.valid != 0U && laser_right.data.distance_mm < step_up_laser_max_mm_)
                {
                    lat_err = (step_up_lateral_ref_mm_ - (float)laser_right.data.distance_mm) * 0.001f;
                    lateral_ok = 1U;
                }

                if (lateral_ok != 0U)
                {
                    // 左距和右距的物理方向相反，分支内已统一成 Vy 误差方向。
                    chassis_vy_target_ = trapezoid_speed(lat_err, STEP_UP_CHASSIS_ACC_SPEED, STEP_UP_AUTO_APPROACH_MPS);
                }
                else
                {
                    chassis_vy_target_ = 0.0f;
                }
                // 到位判定：Vx 到达中间距离 + Vy 横向收敛
                if (step_up_stable_confirm((laser_valid != 0U && laser_mm <= (STEP_UP_AUTO_MIDDLE_MM + 10) && laser_mm >= (STEP_UP_AUTO_MIDDLE_MM - 10) && fabsf(chassis_vy_target_) < 0.1f && fabsf(chassis_vx_target_) < 0.1f) ? 1U : 0U) != 0U)
                {
                    chassis_vx_target_ = 0.0f;
                    chassis_vy_target_ = 0.0f;
                    if (STEP_UP_DEBUG_ALLOW_NEXT() != 0U)
                    {
                        step_up_stable_count_ = 0U;
                        step_up_state_ = STEP_UP_FINISHED;
                    }
                }
            }
            else
            {
                // 前向激光不可用期间，底盘保持不动。
                chassis_vx_target_ = 0.0f;
                chassis_vy_target_ = 0.0f;
            }

            break;
        }
        break;
    }

    case STEP_UP_FINISHED:
        // 释放底盘控制权，升降回1档，交还手动
        chassis_vy_override_ = 0U;
        lift_switch_target_ = 3U;
        lift_linear_speed_target_ = 0.0f;
        chassis_vx_target_ = 0.0f;
        chassis_vy_target_ = 0.0f;
        step_up_use_radar_ = 0U;
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
    if (step_up_state_ == STEP_UP_IDLE)
    {
        return manual_switch;
    }

    return lift_switch_target_;
}

// IDLE时透传手动速度，否则返回自动速度
float LiftAuto::getLiftLinearSpeedTarget(float manual_target) const
{
    if (step_up_state_ == STEP_UP_IDLE)
    {
        return manual_target;
    }

    return lift_linear_speed_target_;
}

// 未接管底盘时透传手动Vy，否则返回自动Vy
float LiftAuto::getChassisVyTarget(float manual_target) const
{
    if (chassis_vy_override_ == 0U)
    {
        return manual_target;
    }

    return chassis_vy_target_;
}

// 未接管底盘时透传手动Vx，否则返回自动Vx
float LiftAuto::getChassisVxTarget(float manual_target) const
{
    if (chassis_vy_override_ == 0U)
    {
        return manual_target;
    }

    return chassis_vx_target_;
}

// 配置上台阶雷达模式的中间目标坐标，并锁定本次爬升基准。
void LiftAuto::setStepUpRadarTarget(float x_ref_middle, float y_ref_middle)
{
    const uint8_t entry_block = (step_up_block_num_ >= 1 && step_up_block_num_ <= 3) ? 1U : 0U;

    if (step_up_radar_last_middle_valid_ != 0U && entry_block == 0U)
    {
        // 非入口方块继续使用上一轮中心作为爬升基准。
        step_up_radar_x_ref_climb_base_ = step_up_radar_last_x_ref_middle_;
        step_up_radar_y_ref_climb_base_ = step_up_radar_last_y_ref_middle_;
    }
    else
    {
        // 首次上台阶或入口方块 1/2/3 没有可用上一台阶中心，
        // 爬升基准使用本次方块自身中心坐标。
        step_up_radar_x_ref_climb_base_ = x_ref_middle;
        step_up_radar_y_ref_climb_base_ = y_ref_middle;
    }

    step_up_radar_x_ref_middle_ = x_ref_middle;
    step_up_radar_y_ref_middle_ = y_ref_middle;

    step_up_radar_last_x_ref_middle_ = x_ref_middle;
    step_up_radar_last_y_ref_middle_ = y_ref_middle;
    step_up_radar_last_middle_valid_ = 1U;
}

// 配置本次雷达爬升方向：0 为 X+L，1 为 Y+L，-1 为 Y-L。
void LiftAuto::setStepUpRadarClimbDirection(int8_t y_direction)
{
    if (y_direction > 0)
    {
        step_up_radar_climb_y_direction_ = 1;
    }
    else if (y_direction < 0)
    {
        step_up_radar_climb_y_direction_ = -1;
    }
    else
    {
        step_up_radar_climb_y_direction_ = 0;
    }
}

// 同步路线层记录的最近一次台阶中心，供下一次上台阶计算爬升基准。
void LiftAuto::setStepUpLastMiddle(float x_ref_middle, float y_ref_middle)
{
    step_up_radar_last_x_ref_middle_ = x_ref_middle;
    step_up_radar_last_y_ref_middle_ = y_ref_middle;
    step_up_radar_last_middle_valid_ = 1U;
}

void LiftAuto::setStepUpBlockNum(int num)
{
    step_up_block_num_ = num;
}

void LiftAuto::setStepUpHeightMode(uint16_t height_mm)
{
    step_up_height_mode_mm_ = (height_mm == 400U) ? 400U : 200U;
}

void LiftAuto::setStepUpReturnMiddle(uint8_t enable)
{
    step_up_return_middle_ = (enable != 0U) ? 1U : 0U;
}
