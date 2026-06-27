#ifndef LIFT_STEP_DOWN_H
#define LIFT_STEP_DOWN_H

#include "main.h"
#include <stdint.h>

#ifndef STEP_DOWN_CYLINDER_OPEN
#define STEP_DOWN_CYLINDER_OPEN() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
#endif

#ifndef STEP_DOWN_CYLINDER_CLOSE
#define STEP_DOWN_CYLINDER_CLOSE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
#endif

/**
 * @brief 自动下台阶流程控制类。
 *
 * 该类只负责根据雷达坐标生成下台阶过程中的控制目标，不直接驱动电机，
 * 也不负责从视觉队列中读取方块编号。外部任务应先根据方块编号查询目标
 * 坐标，再调用 setStepDownRadarTarget() 写入坐标，并周期调用 update()。
 *
 * 下 200 流程保持旧节奏：准备点、升降轮离开台阶、回中心。
 * 下 400 流程会在升降轮离开台阶前后额外等待 2/1 档高度，并控制气缸。
 *
 * 空闲状态下，各个 getter 会透传调用方传入的目标值，便于外部将该类
 * 串接到现有手动控制或其他自动流程之后。
 */
class LiftStepDown
{
public:
    LiftStepDown();

    // 启动下台阶流程。启动后需要由外部任务周期调用 update()。
    void startStepDown(void);
    // 停止并复位流程。复位会清空目标坐标，再次运行前需要重新配置坐标。
    void stopStepDown(void);
    // 查询流程是否已经进入结束保持状态。
    uint8_t isStepDownFinished(void) const;
    // 根据当前雷达坐标推进状态机，并更新底盘、升降轮和档位目标。
    void update(void);

    // 空闲时透传手动档位；自动流程运行时返回状态机要求的档位。
    uint8_t getLiftSwitch(uint8_t manual_switch) const;
    // 空闲时透传手动升降轮速度；自动流程运行时返回升降轮目标线速度。
    float getLiftLinearSpeedTarget(float manual_target) const;
    // 空闲时透传上游底盘 Vx；自动流程运行时返回状态机生成的 Vx。
    float getChassisVxTarget(float manual_target) const;
    // 空闲时透传上游底盘 Vy；自动流程运行时返回状态机生成的 Vy。
    float getChassisVyTarget(float manual_target) const;

    /**
     * @brief 配置下台阶流程使用的雷达目标坐标。
     * @param x_ref_prepare_base 本次准备点计算使用的基准 X 坐标，单位为 m。
     * @param y_ref_prepare_base 本次准备点计算使用的基准 Y 坐标，单位为 m。
     * @param x_ref_finish       本次下台阶最终要到达的 X 坐标，单位为 m。
     * @param y_ref_finish       本次下台阶最终要到达的 Y 坐标，单位为 m。
     * @param turn_left_90   上一个动作是否左转 90 度。
     * @param turn_right_90  上一个动作是否右转 90 度。
     * @param turn_180       上一个动作是否转 180 度。
     */
    void setStepDownRadarTarget(float x_ref_prepare_base,
                                float y_ref_prepare_base,
                                float x_ref_finish,
                                float y_ref_finish,
                                uint8_t turn_left_90,
                                uint8_t turn_right_90,
                                uint8_t turn_180);
    // 保存当前方块编号，供外部接线和调试使用。本类内部不负责查表。
    void setStepDownBlockNum(int num);
    // 配置下台阶高度模式：200 走旧流程，400 走带气缸的融合流程。
    void setStepDownHeightMode(uint16_t height_mm);

private:
    enum StepDownState
    {
        STEP_DOWN_IDLE = 0,              // 空闲状态：不接管控制目标，getter 透传外部输入。
        STEP_DOWN_MOVE_TO_PREPARE,       // 第 1 阶段：底盘沿 X 轴移动到下台阶准备点。
        STEP_DOWN_WAIT_PRE_LIFT_HEIGHT,  // 第 2 阶段：切 2 档下降到下 400 准备高度。
        STEP_DOWN_DESCEND,               // 第 3 阶段：底盘悬空，由升降轮沿 X 轴带动车辆离开台阶。
        STEP_DOWN_WAIT_POST_LIFT_HEIGHT, // 第 4 阶段：切 1 档上升到释放高度。
        STEP_DOWN_MOVE_TO_FINISH,        // 第 5 阶段：切回 3 档，底盘按 X/Y 坐标移动到终点。
        STEP_DOWN_FINISHED               // 结束保持：维持 3 档和零速度，等待外部复位。
    };

    // 清零流程状态和目标参数，使所有 getter 恢复透传。
    void resetStepDown(void);
    // 将速度限制在 [-max, max] 范围内。
    float speed_limit(float speed, float max);
    // 根据剩余距离生成带限幅的制动包络速度。
    float trapezoid_speed(float error, float acc, float max);
    // 连续多周期确认到位条件，避免雷达抖动造成状态误切换。
    uint8_t step_down_stable_confirm(uint8_t condition);

    uint8_t step_down_started_;     // 流程启动标志：1 表示允许状态机推进。
    StepDownState step_down_state_; // 当前下台阶状态。

    uint8_t lift_switch_target_;        // 自动流程要求的升降机构档位。
    float lift_linear_speed_target_;    // 自动流程要求的升降轮线速度，单位为 m/s。
    float chassis_vx_target_;           // 自动流程要求的底盘 X 方向速度，单位为 m/s。
    float chassis_vy_target_;           // 自动流程要求的底盘 Y 方向速度，单位为 m/s。
    uint16_t step_down_height_mode_mm_; // 下台阶高度模式，200 为旧流程，400 为融合流程。

    uint8_t step_down_stable_count_;           // 当前状态下连续满足到位条件的周期数。
    int step_down_block_num_;                  // 当前方块编号，仅保存，不在本类内部查表。
    float step_down_radar_x_ref_prepare_base_; // 第 1 阶段准备点计算使用的本次基准 X 坐标，单位为 m。
    float step_down_radar_y_ref_prepare_base_; // 第 1 阶段准备点计算使用的本次基准 Y 坐标，单位为 m。
    float step_down_radar_x_ref_prepare_;      // 第 1 阶段准备点 X 坐标，单位为 m。
    float step_down_radar_y_ref_prepare_;      // 第 1 阶段准备点 Y 坐标，单位为 m。
    float step_down_radar_x_ref_descend_;      // 第 2 阶段离开台阶目标 X 坐标，单位为 m。
    float step_down_radar_y_ref_descend_;      // 第 2 阶段离开台阶目标 Y 坐标，单位为 m。
    float step_down_radar_x_ref_finish_;       // 第 5 阶段终点 X 坐标，单位为 m。
    float step_down_radar_y_ref_finish_;       // 第 5 阶段终点 Y 坐标，单位为 m。
    uint8_t step_down_turn_left_90_;           // 下台阶前上一个动作是否左转 90 度。
    uint8_t step_down_turn_right_90_;          // 下台阶前上一个动作是否右转 90 度。
    uint8_t step_down_turn_180_;               // 下台阶前上一个动作是否转 180 度。
    uint8_t step_down_descend_target_valid_;   // 第 3 阶段目标是否已按进入时坐标锁存。
    uint32_t step_down_pre_lift_command_seq_;  // 进入 2 档等待前的高度命令序号。
    uint32_t step_down_post_lift_command_seq_; // 进入 3 档等待前的高度命令序号。
};

// 全局自动下台阶控制实例，供任务层接线使用。
extern LiftStepDown lift_step_down;

#endif
