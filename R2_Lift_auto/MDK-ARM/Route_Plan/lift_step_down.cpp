#include "lift_step_down.h"
#include "usart_task.h"
#include <math.h>

extern VisionData_t vision;

// x_finsh:-0.19 y_finsh:-1.43
// pre:0.64  descend:0.05

// 第 1、3 阶段底盘移动时允许输出的最大速度，单位为 m/s。
float STEP_DOWN_AUTO_CHASSIS_SPEED_MPS = 0.3f;
// 底盘速度计算使用的制动包络参数。数值越大，距离目标较远时允许的速度越高。
float STEP_DOWN_CHASSIS_ACC_SPEED = 0.4f;

// 第 2 阶段升降轮带动车辆离开台阶时允许输出的最大线速度，单位为 m/s。
float STEP_DOWN_AUTO_LIFT_SPEED_MPS = 0.45f;
// 升降轮速度计算使用的制动包络参数。数值越大，离开台阶时允许的速度越高。
float STEP_DOWN_LIFT_ACC_SPEED = 0.4f;

// 雷达坐标必须连续满足目标条件 10 个周期，状态机才允许进入下一阶段。
uint8_t STEP_DOWN_AUTO_STABLE_COUNT = 10U;

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
    if (speed > max) {
        speed = max;
    }
    if (speed < -max) {
        speed = -max;
    }
    return speed;
}

float LiftStepDown::trapezoid_speed(float error, float acc, float max)
{
    // 参数不合法或已经到达目标点时不输出速度。
    if (error == 0.0f || acc <= 0.0f || max <= 0.0f) {
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
    if (error < 0.0f) {
        // 误差为负时需要沿坐标轴反方向运动。
        speed = -speed;
    }

    return speed_limit(speed, max);
}

uint8_t LiftStepDown::step_down_stable_confirm(uint8_t condition)
{
    // 任何一个周期不满足条件都立即清零，防止非连续命中被累计。
    if (condition == 0U) {
        step_down_stable_count_ = 0U;
        return 0U;
    }

    // 计数达到阈值后保持饱和，避免 uint8_t 继续累加溢出。
    if (step_down_stable_count_ < STEP_DOWN_AUTO_STABLE_COUNT) {
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
    step_down_started_             = 0U;
    step_down_state_               = STEP_DOWN_IDLE;
    lift_switch_target_            = 0U;
    lift_linear_speed_target_      = 0.0f;
    chassis_vx_target_             = 0.0f;
    chassis_vy_target_             = 0.0f;
    step_down_stable_count_        = 0U;
    step_down_block_num_           = 0;
    step_down_radar_x_ref_prepare_ = 0.0f;
    step_down_radar_x_ref_descend_ = 0.0f;
    step_down_radar_x_ref_finish_  = 0.0f;
    step_down_radar_y_ref_finish_  = 0.0f;
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
    if (step_down_started_ == 0U) {
        return;
    }

    // 第一次更新时从空闲状态进入“移动到准备点”阶段。
    if (step_down_state_ == STEP_DOWN_IDLE) {
        step_down_state_ = STEP_DOWN_MOVE_TO_PREPARE;
    }

    switch (step_down_state_) {
        case STEP_DOWN_MOVE_TO_PREPARE: {
            /*
             * 第 1 阶段：底盘移动到下台阶准备点。
             *
             * 此时车辆仍由底盘支撑，因此使用底盘 Vx 沿雷达 X 轴远离台阶。
             * 升降机构保持 1 档，升降轮不转动，Vy 固定为 0，避免横向偏移。
             */
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;
            chassis_vy_target_        = 0.0f;

            // 雷达给出当前位置，目标值与当前位置之差决定底盘前后运动方向。
            float x_err        = step_down_radar_x_ref_prepare_ - vision.x_diff;
            chassis_vx_target_ = trapezoid_speed(x_err,
                                                 STEP_DOWN_CHASSIS_ACC_SPEED,
                                                 STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);

            // X 误差连续 10 个周期小于 5 cm 后，确认车辆已经稳定到达准备点。
            if (step_down_stable_confirm((fabsf(x_err) < 0.020f) ? 1U : 0U) != 0U) {
                chassis_vx_target_      = 0.0f;
                step_down_stable_count_ = 0U;
                step_down_state_        = STEP_DOWN_DESCEND;
            }
            break;
        }

        case STEP_DOWN_DESCEND: {
            /*
             * 第 2 阶段：升降轮带动车辆离开当前台阶。
             *
             * 切换到 2 档后，底盘处于悬空状态，底盘轮无法可靠驱动车辆。
             * 因此强制将底盘 Vx/Vy 置零，仅使用升降轮目标线速度沿 X 轴运动。
             */
            lift_switch_target_ = 2U;
            chassis_vx_target_  = 0.0f;
            chassis_vy_target_  = 0.0f;

            // 根据离开台阶目标点与当前雷达 X 坐标的误差生成升降轮速度。
            float x_err               = step_down_radar_x_ref_descend_ - vision.x_diff;
            lift_linear_speed_target_ = trapezoid_speed(x_err,
                                                        STEP_DOWN_LIFT_ACC_SPEED,
                                                        STEP_DOWN_AUTO_LIFT_SPEED_MPS);

            // X 误差连续稳定在 5 cm 内后，停止升降轮并立即切回 1 档。
            if (step_down_stable_confirm((fabsf(x_err) < 0.030f) ? 1U : 0U) != 0U) {
                lift_switch_target_       = 1U;
                lift_linear_speed_target_ = 0.0f;
                step_down_stable_count_   = 0U;
                step_down_state_          = STEP_DOWN_MOVE_TO_FINISH;
            }
            break;
        }

        case STEP_DOWN_MOVE_TO_FINISH: {
            /*
             * 第 3 阶段：底盘移动到下一台阶终点。
             *
             * 本阶段不等待 1 档高度轨迹执行完成，进入状态后立即允许底盘移动。
             * 升降轮保持停止，底盘同时修正 X 和 Y 坐标，走到配置的终点。
             */
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;

            // X/Y 两个方向分别计算误差，使底盘能够同时完成纵向和横向收敛。
            float x_err        = step_down_radar_x_ref_finish_ - vision.x_diff;
            float y_err        = step_down_radar_y_ref_finish_ - vision.y_diff;
            chassis_vx_target_ = trapezoid_speed(x_err,
                                                 STEP_DOWN_CHASSIS_ACC_SPEED,
                                                 STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);
            chassis_vy_target_ = trapezoid_speed(y_err,
                                                 STEP_DOWN_CHASSIS_ACC_SPEED,
                                                 STEP_DOWN_AUTO_CHASSIS_SPEED_MPS);

            // 只有 X、Y 误差都连续 10 个周期小于 5 cm，才认为下台阶流程完成。
            if (step_down_stable_confirm((fabsf(x_err) < 0.030f &&
                                          fabsf(y_err) < 0.030f)
                                             ? 1U
                                             : 0U) != 0U) {
                chassis_vx_target_      = 0.0f;
                chassis_vy_target_      = 0.0f;
                step_down_stable_count_ = 0U;
                step_down_state_        = STEP_DOWN_FINISHED;
            }
            break;
        }

        case STEP_DOWN_FINISHED:
            /*
             * 结束保持阶段：不立即恢复手动透传。
             *
             * 保持 1 档和全部零速度，直到外部确认流程结束并调用 stopStepDown()。
             * 这样可以避免结束瞬间重新接入手动输入，导致车辆突然运动。
             */
            lift_switch_target_       = 1U;
            lift_linear_speed_target_ = 0.0f;
            chassis_vx_target_        = 0.0f;
            chassis_vy_target_        = 0.0f;
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
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_switch;
    }

    return lift_switch_target_;
}

float LiftStepDown::getLiftLinearSpeedTarget(float manual_target) const
{
    // 空闲状态透传外部升降轮速度；自动流程运行时返回内部缓存。
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_target;
    }

    return lift_linear_speed_target_;
}

float LiftStepDown::getChassisVxTarget(float manual_target) const
{
    // 空闲状态透传上游底盘 Vx；自动流程运行时返回内部缓存。
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_target;
    }

    return chassis_vx_target_;
}

float LiftStepDown::getChassisVyTarget(float manual_target) const
{
    // 空闲状态透传上游底盘 Vy；自动流程运行时返回内部缓存。
    if (step_down_state_ == STEP_DOWN_IDLE) {
        return manual_target;
    }

    return chassis_vy_target_;
}

void LiftStepDown::setStepDownRadarTarget(float x_ref_prepare,
                                          float x_ref_descend,
                                          float x_ref_finish,
                                          float y_ref_finish)
{
    /*
     * 坐标由外部接线层根据方块编号查表后写入。
     * 本类只保存目标值，不读取视觉队列，也不判断方块编号是否合法。
     */
    step_down_radar_x_ref_prepare_ = x_ref_prepare;
    step_down_radar_x_ref_descend_ = x_ref_descend;
    step_down_radar_x_ref_finish_  = x_ref_finish;
    step_down_radar_y_ref_finish_  = y_ref_finish;
}

void LiftStepDown::setStepDownBlockNum(int num)
{
    // 仅保存编号，方便外部调试和后续扩展；当前状态机不根据编号分支。
    step_down_block_num_ = num;
}
