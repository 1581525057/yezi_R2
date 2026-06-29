#include "lift_step_down.h"
#include "lift_class.h"
#include "usart_task.h"
#include <math.h>

// 下四百：1.先走到一定位置 2. 抬升抬到-200，同时打开气缸 3.动2006到指定位置 4.抬升到+200，等待+200抬升完毕后，关闭气缸  5.然后到中心点 6.然后抬升到+100
extern VisionData_t vision;

static void step_down_world_error_to_body_error(float x_world, float y_world, float yaw_deg, float *x_body, float *y_body)
{
    const float deg_to_rad = 0.01745329251994329577f;
    const float yaw_rad = yaw_deg * deg_to_rad;
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);

    *x_body = cos_yaw * x_world + sin_yaw * y_world;
    *y_body = -sin_yaw * x_world + cos_yaw * y_world;
}

// 第 1、3 阶段底盘移动时允许输出的最大速度，单位为 m/s。
float STEP_DOWN_AUTO_CHASSIS_SPEED_MPS = 0.9f;
// 底盘速度计算使用的制动包络参数。数值越大，距离目标较远时允许的速度越高。
float STEP_DOWN_CHASSIS_ACC_SPEED = 0.4f;

// 第 2 阶段升降轮带动车辆离开台阶时允许输出的最大线速度，单位为 m/s。
float STEP_DOWN_AUTO_LIFT_SPEED_MPS = 1.35f;
// 升降轮速度计算使用的制动包络参数。数值越大，离开台阶时允许的速度越高。
float STEP_DOWN_LIFT_ACC_SPEED = 1.0f;

// 雷达坐标必须连续满足目标条件 10 个周期，状态机才允许进入下一阶段。
uint8_t STEP_DOWN_AUTO_STABLE_COUNT = 10U;

// 下台阶前准备阶段离方块中心点的距离，单位为 m。
float STEP_DOWN_PREPARE_DISTANCE_L = 0.35f;
// 下台阶下降阶段离开当前坐标的距离，单位为 m。
float STEP_DOWN_DESCEND_DISTANCE_D = 0.58f;

// 全局实例由任务层调用，调用方式与现有上台阶自动流程保持一致。
LiftStepDown lift_step_down;

LiftStepDown::LiftStepDown()
{
    // 上电构造时进入空闲状态，所有输出由调用方原样透传。
    resetStepDown();
}

float LiftStepDown::speed_limit(float speed, float max)
{
    // 正反方向使用相同限幅，避免雷达误差较大时给出过高速度。
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

float LiftStepDown::trapezoid_speed(float error, float acc, float max)
{
    // 参数不合法或已经到达目标点时不输出速度。
    if (error == 0.0f || acc <= 0.0f || max <= 0.0f)
    {
        return 0.0f;
    }

    /*
     * 根据剩余距离计算制动包络速度：
     * v = sqrt(2 * |error| * acc)
     *
     * 距离目标较远时速度由 max 限制；接近目标时速度会随误差减小而下降。
     * 这里的 acc 用于生成速度包络，不是逐周期限制速度变化量的加速度环。
     */
    float speed = sqrtf(2.0f * fabsf(error) * acc);
    if (error < 0.0f)
    {
        // 误差为负时需要沿坐标轴反方向运动。
        speed = -speed;
    }

    return speed_limit(speed, max);
}

uint8_t LiftStepDown::step_down_stable_confirm(uint8_t condition)
{
    // 任何一个周期不满足条件都立即清零，防止非连续命中被累计。
    if (condition == 0U)
    {
        step_down_stable_count_ = 0U;
        return 0U;
    }

    // 计数达到阈值后保持饱和，避免 uint8_t 继续累加溢出。
    if (step_down_stable_count_ < STEP_DOWN_AUTO_STABLE_COUNT)
    {
        step_down_stable_count_++;
    }

    // 连续满足指定周期数后才返回 1，降低雷达坐标抖动造成误切换的风险。
    return (step_down_stable_count_ >= STEP_DOWN_AUTO_STABLE_COUNT) ? 1U : 0U;
}

void LiftStepDown::resetStepDown(void)
{
    /*
     * 复位后回到空闲状态。除了清零输出缓存，还要清空目标坐标和方块编号。
     * 因此每次重新启动流程前，外部接线层都需要重新调用坐标配置接口。
     */
    step_down_started_ = 0U;
    step_down_state_ = STEP_DOWN_IDLE;
    lift_switch_target_ = 0U;
    lift_linear_speed_target_ = 0.0f;
    chassis_vx_target_ = 0.0f;
    chassis_vy_target_ = 0.0f;
    step_down_height_mode_mm_ = 200U;
    step_down_stable_count_ = 0U;
    step_down_block_num_ = 0;
    step_down_radar_x_ref_prepare_base_ = 0.0f;
    step_down_radar_y_ref_prepare_base_ = 0.0f;
    step_down_radar_x_ref_prepare_ = 0.0f;
    step_down_radar_y_ref_prepare_ = 0.0f;
    step_down_radar_x_ref_descend_ = 0.0f;
    step_down_radar_y_ref_descend_ = 0.0f;
    step_down_radar_x_ref_finish_ = 0.0f;
    step_down_radar_y_ref_finish_ = 0.0f;
    step_down_turn_left_90_ = 0U;
    step_down_turn_right_90_ = 0U;
    step_down_turn_180_ = 1U;
    step_down_descend_target_valid_ = 0U;
    step_down_pre_lift_command_seq_ = 0U;
    step_down_post_lift_command_seq_ = 0U;
}

void LiftStepDown::startStepDown(void)
{
    /*
     * 这里只打开状态机启动标志，不立即改写输出。
     * 外部任务需要及时调用 update()，由第一次更新正式进入准备点阶段。
     */
    step_down_started_ = 1U;
}

void LiftStepDown::stopStepDown(void)
{
    // 停止流程后恢复空闲透传，并清除本次流程使用的目标参数。
    resetStepDown();
}

uint8_t LiftStepDown::isStepDownFinished(void) const
{
    // 只有进入结束保持状态后才认为流程完成。
    return (step_down_state_ == STEP_DOWN_FINISHED) ? 1U : 0U;
}

void LiftStepDown::update(void)
{
    // 尚未启动时不接管控制目标，getter 会继续透传外部输入。
    if (step_down_started_ == 0U)
    {
        return;
    }

    // 第一次更新时从空闲状态进入“移动到准备点”阶段。
    if (step_down_state_ == STEP_DOWN_IDLE)
    {
        step_down_state_ = STEP_DOWN_MOVE_TO_PREPARE;
    }

    switch (step_down_state_)
    {
    case STEP_DOWN_MOVE_TO_PREPARE:
    {
        /*
         * 第 1 阶段：底盘移动到下台阶准备点。
         *
         * 此时车辆仍由底盘支撑，先根据上一个转向动作推导准备点。
         * 准备点误差是世界系坐标，输出底盘速度前需要转换到车体系。
         */
        lift_switch_target_ = 3U;
        lift_linear_speed_target_ = 0.0f;

        step_down_radar_x_ref_prepare_ = step_down_radar_x_ref_prepare_base_;
        step_down_radar_y_ref_prepare_ = step_down_radar_y_ref_prepare_base_;
        float x_err = 0.0f;
        float y_err = 0.0f;
        if (step_down_turn_180_ != 0U)
        {
            step_down_radar_x_ref_prepare_ = step_down_radar_x_ref_prepare_base_ + STEP_DOWN_PREPARE_DISTANCE_L;
            x_err = step_down_radar_x_ref_prepare_ - vision.x_diff;
        }
        else if (step_down_turn_right_90_ != 0U)
        {
            step_down_radar_y_ref_prepare_ = step_down_radar_y_ref_prepare_base_ + STEP_DOWN_PREPARE_DISTANCE_L;
            y_err = step_down_radar_y_ref_prepare_ - vision.y_diff;
        }
        else
        {
            step_down_radar_y_ref_prepare_ = step_down_radar_y_ref_prepare_base_ - STEP_DOWN_PREPARE_DISTANCE_L;
            y_err = step_down_radar_y_ref_prepare_ - vision.y_diff;
        }

        float x_err_body = 0.0f;
        float y_err_body = 0.0f;

        step_down_world_error_to_body_error(x_err, y_err, vision.angle_x, &x_err_body, &y_err_body);

        chassis_vx_target_ = trapezoid_speed(x_err_body,
                                             STEP_DOWN_CHASSIS_ACC_SPEED,
                                             STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);

        chassis_vy_target_ = trapezoid_speed(y_err_body,
                                             STEP_DOWN_CHASSIS_ACC_SPEED,
                                             STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);

        uint8_t prepare_done = 0U;
        if (step_down_turn_180_ != 0U)
        {
            prepare_done = (fabsf(x_err) < 0.020f) ? 1U : 0U;
        }
        else
        {
            prepare_done = (fabsf(y_err) < 0.020f) ? 1U : 0U;
        }

        if (step_down_stable_confirm(prepare_done) != 0U)
        {
            chassis_vx_target_ = 0.0f;
            chassis_vy_target_ = 0.0f;
            step_down_stable_count_ = 0U;
            step_down_descend_target_valid_ = 0U;
            if (step_down_height_mode_mm_ == 400U)
            {
                step_down_pre_lift_command_seq_ = lift_calulate.command_seq;
                STEP_DOWN_CYLINDER_OPEN();
                step_down_state_ = STEP_DOWN_WAIT_PRE_LIFT_HEIGHT;
            }
            else
            {
                step_down_state_ = STEP_DOWN_DESCEND;
            }
        }
        break;
    }

    case STEP_DOWN_WAIT_PRE_LIFT_HEIGHT:
        /*
         * 第 2 阶段：切到 2 档下降到下 400 准备高度。
         *
         * 这里学习上 400 流程，先记录进入等待前的高度命令序号。
         * 只有 lift_task 接收到新档位并完成这条高度轨迹后，才允许 2006 开始移动。
         */
        lift_switch_target_ = 2U;
        lift_linear_speed_target_ = 0.0f;
        chassis_vx_target_ = 0.0f;
        chassis_vy_target_ = 0.0f;

        if (lift_calulate.command_seq != step_down_pre_lift_command_seq_ &&
            lift_calulate.finished == 1U)
        {
            step_down_stable_count_ = 0U;
            step_down_descend_target_valid_ = 0U;
            step_down_state_ = STEP_DOWN_DESCEND;
        }
        break;

    case STEP_DOWN_DESCEND:
    {
        /*
         * 第 3 阶段：升降轮带动车辆离开当前台阶。
         *
         * 切换到 2 档后，底盘处于悬空状态，底盘轮无法可靠驱动车辆。
         * 因此强制将底盘 Vx/Vy 置零，仅使用升降轮目标线速度离开台阶。
         */
        lift_switch_target_ = 2U;
        chassis_vx_target_ = 0.0f;
        chassis_vy_target_ = 0.0f;

        if (step_down_descend_target_valid_ == 0U)
        {
            step_down_radar_x_ref_descend_ = vision.x_diff;
            step_down_radar_y_ref_descend_ = vision.y_diff;
            if (step_down_turn_180_ != 0U)
            {
                step_down_radar_x_ref_descend_ = vision.x_diff + STEP_DOWN_DESCEND_DISTANCE_D;
            }
            else if (step_down_turn_left_90_ != 0U)
            {
                step_down_radar_y_ref_descend_ = vision.y_diff - STEP_DOWN_DESCEND_DISTANCE_D;
            }
            else
            {
                step_down_radar_y_ref_descend_ = vision.y_diff + STEP_DOWN_DESCEND_DISTANCE_D;
            }
            step_down_descend_target_valid_ = 1U;
        }

        float x_err = step_down_radar_x_ref_descend_ - vision.x_diff;
        float y_err = step_down_radar_y_ref_descend_ - vision.y_diff;
        float lift_err = (step_down_turn_180_ != 0U) ? x_err : y_err;
        float lift_speed = trapezoid_speed(lift_err,
                                           STEP_DOWN_LIFT_ACC_SPEED,
                                           STEP_DOWN_AUTO_LIFT_SPEED_MPS);
        if (step_down_turn_180_ != 0U || step_down_turn_right_90_ != 0U)
        {
            lift_speed = -lift_speed;
        }
        lift_linear_speed_target_ = lift_speed;

        if (step_down_stable_confirm((fabsf(lift_err) < 0.040f) ? 1U : 0U) != 0U)
        {
            lift_linear_speed_target_ = 0.0f;
            step_down_stable_count_ = 0U;
            step_down_descend_target_valid_ = 0U;
            if (step_down_height_mode_mm_ == 400U)
            {
                step_down_post_lift_command_seq_ = lift_calulate.command_seq;
                step_down_state_ = STEP_DOWN_WAIT_POST_LIFT_HEIGHT;
            }
            else
            {
                lift_switch_target_ = 3U;
                step_down_state_ = STEP_DOWN_MOVE_TO_FINISH;
            }
        }
        break;
    }

    case STEP_DOWN_WAIT_POST_LIFT_HEIGHT:
        /*
         * 第 4 阶段：切到 1 档上升到释放高度。
         *
         * 1 档高度轨迹完成后再关闭气缸，然后立即允许底盘回中心。
         */
        lift_switch_target_ = 1U;
        lift_linear_speed_target_ = 0.0f;
        chassis_vx_target_ = 0.0f;
        chassis_vy_target_ = 0.0f;

        if (lift_calulate.command_seq != step_down_post_lift_command_seq_ &&
            lift_calulate.finished == 1U)
        {
            STEP_DOWN_CYLINDER_CLOSE();
            step_down_stable_count_ = 0U;
            step_down_state_ = STEP_DOWN_MOVE_TO_FINISH;
        }
        break;

    case STEP_DOWN_MOVE_TO_FINISH:
    {
        /*
         * 第 5 阶段：底盘移动到下一台阶终点。
         *
         * 本阶段切回 3 档后不等待高度轨迹执行完成，直接边抬升到 +100 边回中心。
         * 升降轮保持停止，底盘同时修正 X 和 Y 坐标，走到配置的终点。
         */
        lift_switch_target_ = 3U;
        lift_linear_speed_target_ = 0.0f;

        // X/Y 两个方向分别计算误差，使底盘能够同时完成纵向和横向收敛。
        float x_err = step_down_radar_x_ref_finish_ - vision.x_diff;
        float y_err = step_down_radar_y_ref_finish_ - vision.y_diff;
        float x_err_body = 0.0f;
        float y_err_body = 0.0f;
        step_down_world_error_to_body_error(x_err, y_err, vision.angle_x, &x_err_body, &y_err_body);

        chassis_vx_target_ = trapezoid_speed(x_err_body,
                                             STEP_DOWN_CHASSIS_ACC_SPEED,
                                             STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);
        chassis_vy_target_ = trapezoid_speed(y_err_body,
                                             STEP_DOWN_CHASSIS_ACC_SPEED,
                                             STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);

        // 只有 X、Y 误差都连续 10 个周期小于 5 cm，才认为下台阶流程完成。
        if (step_down_stable_confirm((fabsf(x_err) < 0.020f &&
                                      fabsf(y_err) < 0.020f)
                                         ? 1U
                                         : 0U) != 0U)
        {
            chassis_vx_target_ = 0.0f;
            chassis_vy_target_ = 0.0f;
            step_down_stable_count_ = 0U;
            step_down_state_ = STEP_DOWN_FINISHED;
        }
        break;
    }

    case STEP_DOWN_FINISHED:
        /*
         * 结束保持阶段：不立即恢复手动透传。
         *
         * 保持 3 档和全部零速度，直到外部确认流程结束并调用 stopStepDown()。
         * 这样可以避免结束瞬间重新接入手动输入，导致车辆突然运动。
         */
        lift_switch_target_ = 3U;
        lift_linear_speed_target_ = 0.0f;
        chassis_vx_target_ = 0.0f;
        chassis_vy_target_ = 0.0f;
        break;

    default:
        // 状态异常时回到空闲状态，停止本次自动流程。
        resetStepDown();
        break;
    }
}

uint8_t LiftStepDown::getLiftSwitch(uint8_t manual_switch) const
{
    // 空闲状态不接管档位；运行或结束保持阶段返回自动流程档位。
    if (step_down_state_ == STEP_DOWN_IDLE)
    {
        return manual_switch;
    }

    return lift_switch_target_;
}

float LiftStepDown::getLiftLinearSpeedTarget(float manual_target) const
{
    // 空闲状态透传外部升降轮速度；自动流程运行时返回内部缓存。
    if (step_down_state_ == STEP_DOWN_IDLE)
    {
        return manual_target;
    }

    return lift_linear_speed_target_;
}

float LiftStepDown::getChassisVxTarget(float manual_target) const
{
    // 空闲状态透传上游底盘 Vx；自动流程运行时返回内部缓存。
    if (step_down_state_ == STEP_DOWN_IDLE)
    {
        return manual_target;
    }

    return chassis_vx_target_;
}

float LiftStepDown::getChassisVyTarget(float manual_target) const
{
    // 空闲状态透传上游底盘 Vy；自动流程运行时返回内部缓存。
    if (step_down_state_ == STEP_DOWN_IDLE)
    {
        return manual_target;
    }

    return chassis_vy_target_;
}

void LiftStepDown::setStepDownRadarTarget(float x_ref_prepare_base,
                                          float y_ref_prepare_base,
                                          float x_ref_finish,
                                          float y_ref_finish,
                                          uint8_t turn_left_90,
                                          uint8_t turn_right_90,
                                          uint8_t turn_180)
{
    /*
     * 坐标由外部接线层根据方块编号查表后写入。
     * 本类只保存目标值，不读取视觉队列，也不判断方块编号是否合法。
     *
     * 本函数只保存本次下台阶需要的基准点和终点。
     * “上一次中心点”属于路线接线层的上下文，本类不再自行缓存推断，
     * 避免连续动作变化时把上一轮下台阶终点误当成本次准备基准。
     */
    step_down_radar_x_ref_prepare_base_ = x_ref_prepare_base;
    step_down_radar_y_ref_prepare_base_ = y_ref_prepare_base;

    // 保存本次下台阶最终要到达的中心点，STEP_DOWN_MOVE_TO_FINISH 阶段会用这组坐标收敛。
    step_down_radar_x_ref_finish_ = x_ref_finish;
    step_down_radar_y_ref_finish_ = y_ref_finish;

    // 三个转向标志互斥：180 度优先，其次右转 90 度，最后左转 90 度。
    step_down_turn_180_ = (turn_180 != 0U) ? 1U : 0U;
    step_down_turn_right_90_ = (turn_right_90 != 0U && step_down_turn_180_ == 0U) ? 1U : 0U;
    step_down_turn_left_90_ = (turn_left_90 != 0U && step_down_turn_180_ == 0U && step_down_turn_right_90_ == 0U) ? 1U : 0U;

    // 如果外部没有给任何转向标志，默认按 180 度下台阶处理，保持状态机有明确分支。
    if (step_down_turn_180_ == 0U && step_down_turn_right_90_ == 0U && step_down_turn_left_90_ == 0U)
    {
        step_down_turn_180_ = 1U;
    }

    // 下一次进入 DESCEND 阶段时，需要重新按当时的 vision 坐标锁存 D 距离目标。
    step_down_descend_target_valid_ = 0U;
}

void LiftStepDown::setStepDownBlockNum(int num)
{
    // 仅保存编号，方便外部调试和后续扩展；当前状态机不根据编号分支。
    step_down_block_num_ = num;
}

void LiftStepDown::setStepDownHeightMode(uint16_t height_mm)
{
    // 只有 400 需要融合气缸流程，其余输入按下 200 旧流程处理。
    step_down_height_mode_mm_ = (height_mm == 400U) ? 400U : 200U;
}
