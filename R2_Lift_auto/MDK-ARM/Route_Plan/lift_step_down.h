#ifndef LIFT_STEP_DOWN_H
#define LIFT_STEP_DOWN_H

#include "main.h"
#include <stdint.h>

/**
 * @brief 自动下台阶流程控制类。
 *
 * 该类只负责根据雷达坐标生成下台阶过程中的控制目标，不直接驱动电机，
 * 也不负责从视觉队列中读取方块编号。外部任务应先根据方块编号查询目标
 * 坐标，再调用 setStepDownRadarTarget() 写入坐标，并周期调用 update()。
 *
 * 下台阶流程分为三个运动阶段和一个结束保持阶段：
 * 1. 底盘沿 X 轴移动到下台阶准备点；
 * 2. 底盘悬空，停止底盘运动，改由升降轮沿 X 轴带动车辆离开台阶；
 * 3. 切回 1 档，底盘按 X/Y 坐标移动到终点；
 * 4. 保持 1 档和零速度，等待外部调用 stopStepDown() 复位。
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
     * @param x_ref_prepare 第 1 阶段底盘移动到的 X 坐标，单位为 m。
     * @param x_ref_descend 第 2 阶段升降轮带动车辆移动到的 X 坐标，单位为 m。
     * @param x_ref_finish  第 3 阶段底盘移动到的终点 X 坐标，单位为 m。
     * @param y_ref_finish  第 3 阶段底盘移动到的终点 Y 坐标，单位为 m。
     */
    void setStepDownRadarTarget(float x_ref_prepare,
                                float x_ref_descend,
                                float x_ref_finish,
                                float y_ref_finish);
    // 保存当前方块编号，供外部接线和调试使用。本类内部不负责查表。
    void setStepDownBlockNum(int num);

private:
    enum StepDownState {
        STEP_DOWN_IDLE = 0,        // 空闲状态：不接管控制目标，getter 透传外部输入。
        STEP_DOWN_MOVE_TO_PREPARE, // 第 1 阶段：底盘沿 X 轴移动到下台阶准备点。
        STEP_DOWN_DESCEND,         // 第 2 阶段：底盘悬空，由升降轮沿 X 轴带动车辆离开台阶。
        STEP_DOWN_MOVE_TO_FINISH,  // 第 3 阶段：切回 1 档，底盘按 X/Y 坐标移动到终点。
        STEP_DOWN_FINISHED         // 结束保持：维持 1 档和零速度，等待外部复位。
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

    uint8_t lift_switch_target_;     // 自动流程要求的升降机构档位。
    float lift_linear_speed_target_; // 自动流程要求的升降轮线速度，单位为 m/s。
    float chassis_vx_target_;        // 自动流程要求的底盘 X 方向速度，单位为 m/s。
    float chassis_vy_target_;        // 自动流程要求的底盘 Y 方向速度，单位为 m/s。

    uint8_t step_down_stable_count_;      // 当前状态下连续满足到位条件的周期数。
    int step_down_block_num_;             // 当前方块编号，仅保存，不在本类内部查表。
    float step_down_radar_x_ref_prepare_; // 第 1 阶段准备点 X 坐标，单位为 m。
    float step_down_radar_x_ref_descend_; // 第 2 阶段离开台阶目标 X 坐标，单位为 m。
    float step_down_radar_x_ref_finish_;  // 第 3 阶段终点 X 坐标，单位为 m。
    float step_down_radar_y_ref_finish_;  // 第 3 阶段终点 Y 坐标，单位为 m。
};

// 全局自动下台阶控制实例，供任务层接线使用。
extern LiftStepDown lift_step_down;

#endif
