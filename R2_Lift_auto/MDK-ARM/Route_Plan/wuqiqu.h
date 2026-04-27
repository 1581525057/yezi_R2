#ifndef WUQIQU_H
#define WUQIQU_H

#include <stdint.h>

typedef struct
{
    uint8_t state;
    uint8_t deviation_flag;
    uint8_t on_final_point_flag;
    uint16_t final_xy_stable_count;
    uint16_t final_theta_stable_count;
    int current_index;
    int deviation_index;
    float pose_x;
    float pose_y;
    float pose_yaw;
    float target_x;
    float target_y;
    float target_yaw;
    float err_x;
    float err_y;
    float err_distance;
    float safe_distance;
    float dot_product;
    float normal_vector_x;
    float normal_vector_y;
    float normal_vel_x;
    float normal_vel_y;
    float err_theta;
    float out_vx;
    float out_vy;
    float out_wz;
} WuqiquDebug_t;

class WuqiquPathPlanner
{
public:
    static const uint16_t PATH_POINT_COUNT = 14U;

    enum PlannerState
    {
        STATE_IDLE = 0,
        STATE_RUNNING,
        STATE_FINAL_POSITION,
        STATE_FINAL_YAW,
        STATE_FINISHED,
        STATE_DEVIATED
    };

    class Point
    {
    public:
        float vx;
        float vy;
        float x;
        float y;
        float yaw;
    };

    class Pose
    {
    public:
        float x;
        float y;
        float yaw;
        float yaw_360;
        float car_speed_x;
        float car_speed_y;
        float world_speed_x;
        float world_speed_y;
        float omega;
    };

    class Output
    {
    public:
        float world_vx_set;
        float world_vy_set;
        float wz_set;
    };

    WuqiquPathPlanner();

    int follow(const Pose &current_pose);
    void reset(void);

    const Output &getOutput(void) const;
    int getCurrentIndex(void) const;
    PlannerState getState(void) const;
    uint8_t getDeviationFlag(void) const;
    int getDeviationIndex(void) const;
    const Point *getPath(void) const;
    uint16_t getPathPointCount(void) const;

    float slewRateLimit(float target_vel, float current_vel, float max_acc_step, float max_dec_step) const;

private:
    static const Point action_path_[PATH_POINT_COUNT];

    int current_path_index_;
    uint8_t vel_calculate_enter_flag_;
    uint8_t final_point_xy_correct_flag_;
    uint8_t final_point_theta_correct_flag_;
    uint8_t on_final_point_flag_;
    uint16_t point_stable_count_;
    uint16_t final_xy_stable_count_;
    uint16_t final_theta_stable_count_;
    PlannerState state_;
    uint8_t deviation_flag_;
    int deviation_index_;

    float last_err_theta_normal_;
    float last_vector4_[2];
    float last_err_x_;
    float last_err_y_;
    float last_err_theta_final_;

    Output output_;

    uint8_t safeCheck(const Point &cur_tar_point, const Pose &current_pose) const;
    void pointCalculate(const Pose &current_pose);
    int finPointCalculate(const Pose &current_pose);
    float normalizeAngle(float angle) const;
    float safeSqrt(float value) const;
    float safeInvSqrt(float value) const;
};

extern WuqiquPathPlanner wuqiqu;
extern WuqiquDebug_t wuqiqu_debug;

#endif
