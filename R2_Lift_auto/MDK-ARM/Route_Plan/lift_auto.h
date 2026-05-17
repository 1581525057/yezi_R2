#ifndef LIFT_AUTO_H
#define LIFT_AUTO_H

#include "main.h"
#include <stdint.h>

/**
 * 自动升降爬台阶状态机
 *
 * 状态流程: IDLE → APPROACH_Y(靠近台阶) → CLIMB_FORWARD(爬升过台阶) → APPROACH_MIDDLE(移到中间) → FINISHED
 *
 * 用法:
 *   1. 调用 start() 启动
 *   2. 周期调用 update() 驱动状态机
 *   3. 通过 get*() 方法获取自动输出，IDLE时透传手动值
 *   4. isFinished() 判断是否完成
 */
class LiftAuto
{
public:
    LiftAuto();

    void start(void);
    void stop(void);
    uint8_t isFinished(void) const;

    void update(void);

    // 以下三个get函数：自动模式返回自动值，IDLE时透传手动值
    uint8_t getLiftSwitch(uint8_t manual_switch) const;
    float getLiftLinearSpeedTarget(float manual_target) const;
    float getChassisVyTarget(float manual_target) const;
    float getChassisVxTarget(float manual_target) const;
    void setRadarTarget(float x_ref, float x_ref_climb, float y_ref_middle);
    void setBlockNum(int num);

private:
    void reset(void);

    // 自动流程步骤
    enum StepState {
        STEP_IDLE = 0,        // 空闲，透传手动控制
        STEP_APPROACH_Y,      // 新Vx方向靠近台阶 + 升降到目标高度
        STEP_WAIT_NEW_HEIGHT, // 等待升降机构到达爬升高度
        STEP_CLIMB_FORWARD,   // 前进爬升过台阶
        STEP_APPROACH_MIDDLE, // 进入下一台阶的中间位置.
        STEP_FINISHED         // 完成，释放控制权
    };

    uint8_t flag_start;              // 启动标志
    StepState state_;                // 当前状态
    uint8_t lift_switch_target_;     // 升降档位输出
    float lift_linear_speed_target_; // 升降线速度输出
    uint8_t chassis_vy_override_;    // 1=接管底盘平移控制
    float chassis_vy_target_;        // 底盘Vy输出（左为正）
    float chassis_vx_target_;        // 底盘Vx输出（前为正）
    uint8_t stable_count_;           // 高度稳定计数
    uint8_t climbed_;                // 激光读数是否曾超过FINISH_MM，防误触发

    // 雷达/激光模式选择
    uint8_t use_radar_; // 1=雷达模式, 0=激光模式
    int block_num_;     // 当前方块编号

    // 雷达目标坐标
    float radar_x_ref_;        // 雷达目标X
    float radar_x_ref_climb_;  // STEP_CLIMB_FORWARD 雷达目标X
    float radar_y_ref_middle_; // STEP_APPROACH_MIDDLE 雷达目标Y

    // 激光模式参数
    float lateral_ref_mm_;  // 横向目标参考值 = 222.0mm
    uint32_t laser_max_mm_; // 激光有效阈值 = 1500mm
    uint32_t middle_lift_command_seq_; // 进入中间位阶段前的升降轨迹序号，用来等待新动作完成。
    uint8_t middle_lift_finished_;     // 中间位阶段升降动作完成后才允许底盘继续移动。

    // 速度限幅函数
    float speed_limit(float speed, float max);
    float trapezoid_speed(float error, float acc, float max);
    uint8_t stable_confirm(uint8_t condition);
};

extern LiftAuto lift_auto;

#endif
