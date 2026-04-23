#include "lift_auto.h"
#include "bsp_remove.h"
#include "laser_distance.h"

// ===================== 常量定义（匿名命名空间，仅本文件可见）=====================
namespace
{
    // 自动靠近阶段：底盘向Y轴负方向移动的速度（单位：m/s，负号表示向后/向目标方向）
    static const float LIFT_AUTO_APPROACH_VY_MPS = -0.45f;

    // 爬坡阶段：抬升机构线速度目标值（单位：m/s）
    static const float LIFT_AUTO_CLIMB_SPEED_MPS = 0.27f;

    // 准备距离阈值：激光距离小于此值时停止靠近，准备抬升（单位：mm）
    static const uint32_t LIFT_AUTO_PREPARE_MM = 110U;

    // 新高度目标值：抬升到位后激光测距应读到的高度（单位：mm）
    static const uint32_t LIFT_AUTO_NEW_HEIGHT_MM = 1120U;

    // 完成距离阈值：爬坡过程中激光距离小于此值时认为已完成（单位：mm）
    static const uint32_t LIFT_AUTO_FINISH_MM = 490U;

    // 高度容差：判断是否到达目标高度的允许误差范围（单位：mm）
    static const uint32_t LIFT_AUTO_HEIGHT_TOL_MM = 30U;

    // 稳定计数阈值：连续多少帧在目标高度范围内才认为稳定到位
    static const uint8_t LIFT_AUTO_STABLE_COUNT = 10U;
}

// ===================== 全局实例 =====================
LiftAuto lift_auto;

// ===================== 构造函数 =====================
LiftAuto::LiftAuto()
{
    reset(); // 初始化时重置所有状态
}

// ===================== 重置函数：清空所有状态，回到空闲 =====================
void LiftAuto::reset(void)
{
    state_                    = STEP_IDLE;  // 状态机回到空闲状态
    lift_switch_target_       = 0U;         // 抬升档位目标清零
    lift_linear_speed_target_ = 0.0f;       // 抬升线速度目标清零
    chassis_vy_override_      = 0U;         // 关闭底盘Y轴速度接管
    chassis_vy_target_        = 0.0f;       // 底盘Y轴速度目标清零
    stable_count_             = 0U;         // 稳定计数清零
    last_laser_mm_            = UINT32_MAX; // 上次激光距离置为无效值
}

// ===================== 主更新函数：每帧调用，驱动状态机 =====================
void LiftAuto::update(void)
{
    // 读取遥控器左右拨杆状态（0/1/2 对应下/中/上）
    const uint8_t left_sw  = remove_dji.rc_.s[0];
    const uint8_t right_sw = remove_dji.rc_.s[1];

    // 读取激光测距数据
    const uint32_t laser_mm   = laser.data.distance_mm; // 当前距离（mm）
    const uint8_t laser_valid = laser.data.valid;       // 数据是否有效

    // 安全检查：只有左右拨杆都在"上"位（值为1）时才执行自动抬升
    // 否则立即重置，防止误触发
    if (right_sw != 1U || left_sw != 1U) {
        reset();
        return;
    }

    // 从空闲状态自动进入"靠近"阶段
    if (state_ == STEP_IDLE) {
        state_ = STEP_APPROACH_Y;
    }

    // ===================== 状态机主体 =====================
    switch (state_) {

        // ---------- 第一步：靠近目标（底盘向目标移动，直到激光距离足够近）----------
        case STEP_APPROACH_Y:
            chassis_vy_override_      = 1U;   // 接管底盘Y轴速度
            lift_switch_target_       = 1U;   // 抬升档位设为1（低位/待机）
            lift_linear_speed_target_ = 0.0f; // 抬升线速度为0，不动

            // 激光有效且距离大于准备阈值：继续向目标靠近
            if (laser_valid != 0U && laser_mm > LIFT_AUTO_PREPARE_MM) {
                chassis_vy_target_ = LIFT_AUTO_APPROACH_VY_MPS; // 向目标方向移动
            } else {
                chassis_vy_target_ = 0.0f; // 停止移动
            }

            // 记录最新一次有效激光距离（备用）
            if (laser_valid != 0U) {
                last_laser_mm_ = laser_mm;
            }

            // 激光有效且已靠近到准备距离以内：切换到等待抬升到位阶段
            if (laser_valid != 0U && laser_mm <= LIFT_AUTO_PREPARE_MM) {
                chassis_vy_target_  = 0.0f;                 // 停止底盘
                lift_switch_target_ = 2U;                   // 抬升档位切换为2（高位/抬升）
                stable_count_       = 0U;                   // 稳定计数清零
                state_              = STEP_WAIT_NEW_HEIGHT; // 进入下一阶段
            }
            break;

        // ---------- 第二步：等待抬升到位（检测激光高度稳定在目标范围内）----------
        case STEP_WAIT_NEW_HEIGHT:
            chassis_vy_override_      = 1U;   // 继续接管底盘Y轴（保持不动）
            chassis_vy_target_        = 0.0f; // 底盘停止
            lift_switch_target_       = 2U;   // 继续保持抬升档位2
            lift_linear_speed_target_ = 0.0f; // 抬升线速度为0（等待到位）

            // 判断激光距离是否在目标高度的容差范围内
            if (laser_valid != 0U &&
                laser_mm >= (LIFT_AUTO_NEW_HEIGHT_MM - LIFT_AUTO_HEIGHT_TOL_MM) &&
                laser_mm <= (LIFT_AUTO_NEW_HEIGHT_MM + LIFT_AUTO_HEIGHT_TOL_MM)) {
                // 在目标范围内，稳定计数累加
                if (stable_count_ < LIFT_AUTO_STABLE_COUNT) {
                    stable_count_++;
                }
            } else {
                // 不在目标范围内，稳定计数清零（防止抖动误判）
                stable_count_ = 0U;
            }

            // 连续稳定帧数达到阈值，认为已稳定到位，进入爬坡阶段
            if (stable_count_ >= LIFT_AUTO_STABLE_COUNT) {
                state_ = STEP_CLIMB_FORWARD;
            }
            break;

        // ---------- 第三步：爬坡前进（抬升机构驱动，直到激光检测到完成位置）----------
        case STEP_CLIMB_FORWARD:
            chassis_vy_override_      = 1U;                        // 继续接管底盘Y轴
            chassis_vy_target_        = 0.0f;                      // 底盘停止
            lift_switch_target_       = 2U;                        // 保持抬升档位2
            lift_linear_speed_target_ = LIFT_AUTO_CLIMB_SPEED_MPS; // 抬升机构以爬坡速度运行

            // 激光距离小于完成阈值，认为已爬坡完成
            if (laser_valid != 0U && laser_mm <= LIFT_AUTO_FINISH_MM) {
                lift_switch_target_       = 1U;            // 抬升档位切回1（低位/收起）
                lift_linear_speed_target_ = 0.0f;          // 停止抬升
                state_                    = STEP_FINISHED; // 进入完成状态
            }
            break;

        // ---------- 第四步：完成（保持静止，等待人工接管）----------
        case STEP_FINISHED:
            chassis_vy_override_      = 0U;   // 释放底盘Y轴接管，还给手动控制
            chassis_vy_target_        = 0.0f; // 底盘速度目标清零
            lift_switch_target_       = 1U;   // 抬升档位1（低位）
            lift_linear_speed_target_ = 0.0f; // 抬升线速度为0
            break;

        // ---------- 异常情况：未知状态，重置 ----------
        default:
            reset();
            break;
    }
}

// ===================== 获取抬升档位：自动模式返回自动目标，空闲时透传手动值 =====================
uint8_t LiftAuto::getLiftSwitch(uint8_t manual_switch) const
{
    if (state_ == STEP_IDLE) {
        return manual_switch; // 未激活自动模式，直接使用手动档位
    }

    return lift_switch_target_; // 自动模式，返回自动计算的档位
}

// ===================== 获取抬升线速度目标：同上，空闲时透传手动值 =====================
float LiftAuto::getLiftLinearSpeedTarget(float manual_target) const
{
    if (state_ == STEP_IDLE) {
        return manual_target; // 未激活自动模式，透传手动速度
    }

    return lift_linear_speed_target_; // 自动模式，返回自动计算的速度
}

// ===================== 获取底盘Y轴速度目标：有接管时返回自动值，否则透传手动值 =====================
float LiftAuto::getChassisVyTarget(float manual_target) const
{
    if (chassis_vy_override_ == 0U) {
        return manual_target; // 未接管，使用手动速度
    }

    return chassis_vy_target_; // 已接管，使用自动控制速度
}
