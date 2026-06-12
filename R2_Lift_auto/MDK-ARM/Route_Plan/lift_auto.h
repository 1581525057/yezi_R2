#ifndef LIFT_AUTO_H
#define LIFT_AUTO_H

#include "main.h"
#include <stdint.h>

/**
 * 台阶自动流程模块。
 *
 * 当前只实现上台阶流程，后续可以在同一文件中继续加入下台阶流程。
 * 底盘速度和抬升速度输出接口保持通用，供上台阶/下台阶共同接入控制任务。
 */
class LiftAuto
{
public:
    LiftAuto();

    void startStepUp(void);
    void stopStepUp(void);
    uint8_t isStepUpFinished(void) const;

    void update(void);

    // 公共输出接口：未接管时透传手动目标，接管时返回自动流程目标。
    uint8_t getLiftSwitch(uint8_t manual_switch) const;
    float getLiftLinearSpeedTarget(float manual_target) const;
    float getChassisVyTarget(float manual_target) const;
    float getChassisVxTarget(float manual_target) const;

    void setStepUpRadarTarget(float x_ref_middle, float y_ref_middle);
    void setStepUpRadarClimbDirection(int8_t y_direction);
    void setStepUpBlockNum(int num);

private:
    void resetStepUp(void);

    enum StepUpState {
        STEP_UP_IDLE = 0,        // 空闲，透传手动控制
        STEP_UP_APPROACH_Y,      // 靠近台阶，准备进入上台阶动作
        STEP_UP_WAIT_CLIMB_HEIGHT, // 等待抬升机构执行 2 档到位
        STEP_UP_WAIT_NEW_HEIGHT, // 等待抬升机构收回到中间靠近所需高度
        STEP_UP_CLIMB_FORWARD,   // 上台阶动作
        STEP_UP_APPROACH_MIDDLE, // 上台阶后移动到中间位置
        STEP_UP_FINISHED         // 上台阶完成，释放控制权
    };

    uint8_t step_up_started_;
    StepUpState step_up_state_;

    // 公共输出缓存，后续下台阶流程也可以复用这些接口输出。
    uint8_t lift_switch_target_;
    float lift_linear_speed_target_;
    uint8_t chassis_vy_override_;
    float chassis_vy_target_;
    float chassis_vx_target_;

    uint8_t step_up_stable_count_;
    uint8_t step_up_crossed_finish_height_;

    // 上台阶传感器模式与目标参数。
    uint8_t step_up_use_radar_;
    int step_up_block_num_;
    float step_up_radar_x_ref_middle_;
    float step_up_radar_y_ref_middle_;
    float step_up_radar_x_ref_climb_base_;
    float step_up_radar_y_ref_climb_base_;
    float step_up_radar_last_x_ref_middle_;
    float step_up_radar_last_y_ref_middle_;
    uint8_t step_up_radar_last_middle_valid_;
    float step_up_radar_climb_target_;
    uint8_t step_up_radar_climb_target_valid_;
    int8_t step_up_radar_climb_y_direction_;
    float step_up_lateral_ref_mm_;
    uint32_t step_up_laser_max_mm_;
    uint32_t step_up_climb_lift_command_seq_;
    uint32_t step_up_middle_lift_command_seq_;

    float speed_limit(float speed, float max);
    float trapezoid_speed(float error, float acc, float max);
    uint8_t step_up_stable_confirm(uint8_t condition);
};

extern LiftAuto lift_auto;

#endif
