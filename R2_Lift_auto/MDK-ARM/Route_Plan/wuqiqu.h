#ifndef WUQIQU_H
#define WUQIQU_H

#include <stdint.h>

class WuqiquPathPlanner
{
public:
    static const uint8_t MAX_WAYPOINTS = 10;

    enum PlannerState
    {
        STATE_IDLE = 0,
        STATE_MOVING,
        STATE_YAW_CORRECTING,
        STATE_FINISHED,
        STATE_DEVIATED
    };

    class TargetPoint
    {
    public:
        float x_m;
        float y_m;
        float yaw_deg;
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
        float world_vx_mps;
        float world_vy_mps;
        float wz_radps;
    };

    WuqiquPathPlanner();

    int follow(const Pose &current_pose);
    void reset(void);
    void resetRoute(void);

    void setParams(float pos_kp, float pos_kd,
                   float yaw_kp, float yaw_kd,
                   float pos_tolerance_m, float yaw_tolerance_deg,
                   uint16_t stable_count,
                   float max_angular_speed_radps,
                   float moving_yaw_max_radps);

    void advanceToNext(void);
    bool isAllFinished(void) const;
    uint8_t getCurrentIndex(void) const;
    uint8_t getWaypointCount(void) const;

    const Output &getOutput(void) const;
    PlannerState getState(void) const;
    bool isFinished(void) const;

private:
    TargetPoint waypoints_[MAX_WAYPOINTS];
    TargetPoint target_;
    uint8_t waypoint_count_;
    uint8_t current_index_;

    Output output_;
    PlannerState state_;

    float pos_kp_;
    float pos_kd_;
    float yaw_kp_;
    float yaw_kd_;
    float pos_tolerance_m_;
    float yaw_tolerance_deg_;
    uint16_t stable_count_;
    float max_angular_speed_radps_;
    float moving_yaw_max_radps_;

    float last_err_x_;
    float last_err_y_;
    float last_err_yaw_rad_;

    uint8_t on_target_flag_;
    uint16_t xy_stable_count_;
    uint16_t theta_stable_count_;

    void loadCurrentWaypoint(void);
    float normalizeAngleDeg(float angle) const;
    float safeSqrt(float value) const;
};

extern WuqiquPathPlanner wuqiqu;

#endif
