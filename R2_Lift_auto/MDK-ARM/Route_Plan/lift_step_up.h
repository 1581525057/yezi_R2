#ifndef LIFT_AUTO_H
#define LIFT_AUTO_H

#include "main.h"
#include <stdint.h>

#ifndef STEP_UP_CYLINDER_OPEN
#define STEP_UP_CYLINDER_OPEN() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
#endif

#ifndef STEP_UP_CYLINDER_CLOSE
#define STEP_UP_CYLINDER_CLOSE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
#endif

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
    void setStepUpHeightMode(uint16_t height_mm);
    void setStepUpReturnMiddle(uint8_t enable);

private:
    void resetStepUp(void);

    enum StepUpState
    {
        STEP_UP_IDLE = 0,   // 空闲，透传手动控制
        STEP_UP_APPROACH_Y, // 靠近台阶，准备进入上台阶动作
        STEP_UP_WAIT_CLIMB_HEIGHT, // 等待抬升机构执行 2 档到位
        STEP_UP_WAIT_NEW_HEIGHT,   // 等待抬升机构收回到中间靠近所需高度
        STEP_UP_CLIMB_FORWARD,     // 上台阶动作
        STEP_UP_APPROACH_MIDDLE,   // 上台阶后移动到中间位置
        STEP_UP_FINISHED           // 上台阶完成，释放控制权
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

    // 本次上台阶高度模式：200 表示普通 200mm 台阶，400 表示 400mm 台阶特殊流程。
    uint16_t step_up_height_mode_mm_;
    // 400mm 档预抬升开始前锁存的升降轨迹序号，用来判断 1 档预抬升是否生成了新轨迹。
    uint32_t step_up_pre_lift_command_seq_;
    // 记录 400mm 档 1 档预抬升是否已经真正启动过，避免直接沿用旧 finished=1 误判完成。
    uint8_t step_up_pre_lift_started_;
    // 记录 400mm 档 1 档预抬升是否已经完成，完成后才允许打开气缸并进入后续爬升流程。
    uint8_t step_up_pre_lift_ready_;
    // 是否在上台阶完成后回到台阶中心：1 为回中心，0 为直接结束，常用于连续上台阶。
    uint8_t step_up_return_middle_;

    // 上台阶传感器模式与目标参数。
    // 本次上台阶是否使用雷达/视觉坐标闭环：1 为雷达模式，0 为前向激光模式。
    uint8_t step_up_use_radar_;
    // 本次上台阶对应的方块编号，用来区分入口方块 1/2/3 和普通方块。
    int step_up_block_num_;
    // 本次上台阶结束后需要靠近的目标中心坐标，供 APPROACH_MIDDLE 回中心阶段使用。
    float step_up_radar_x_ref_middle_;
    float step_up_radar_y_ref_middle_;
    // 本次爬升阶段使用的基准中心坐标：通常来自上一轮中心，入口方块则用本方块中心。
    float step_up_radar_x_ref_climb_base_;
    float step_up_radar_y_ref_climb_base_;
    // 最近一次配置过的上台阶中心坐标，供下一次连续上台阶计算爬升基准使用。
    float step_up_radar_last_x_ref_middle_;
    float step_up_radar_last_y_ref_middle_;
    // 标记 last_x/last_y 是否有效；没有上一中心时不能直接拿当前方块冒充上一中心。
    uint8_t step_up_radar_last_middle_valid_;
    // 爬升阶段锁定的单轴目标位置，只在进入爬升阶段首次计算，避免每帧刷新导致目标漂移。
    float step_up_radar_climb_target_;
    // 标记 step_up_radar_climb_target_ 是否已经锁定。
    uint8_t step_up_radar_climb_target_valid_;
    // 爬升方向选择：1 表示沿 Y 正方向，-1 表示沿 Y 负方向，0 表示沿 X 方向。
    int8_t step_up_radar_climb_y_direction_;
    float step_up_lateral_ref_mm_;
    uint32_t step_up_laser_max_mm_;
    // 进入 WAIT_CLIMB_HEIGHT 前锁存的升降轨迹序号，用来等待 2 档爬升高度真正完成。
    uint32_t step_up_climb_lift_command_seq_;
    // 进入 WAIT_NEW_HEIGHT 前锁存的升降轨迹序号，用来等待 3 档收回高度真正完成。
    uint32_t step_up_middle_lift_command_seq_;

    float speed_limit(float speed, float max);
    float trapezoid_speed(float error, float acc, float max);
    uint8_t step_up_stable_confirm(uint8_t condition);
};

extern LiftAuto lift_auto;

#endif
