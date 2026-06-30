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
        STATE_APPROACH,
        STATE_SLOW_APPROACH,
        STATE_SOFT_CONTACT,
        STATE_FINISHED
    };

    class TargetPoint
    {
    public:
        float x_m;
        float y_m;
        float yaw_deg;
        float yaw_kp_scale;
        float yaw_wz_max;
        float xy_tolerance_m;
        float yaw_tolerance_deg;
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

    void advanceToNext(void);
    bool isAllFinished(void) const;
    uint8_t getCurrentIndex(void) const;
    uint8_t getWaypointCount(void) const;
    float getWaypointYawDeg(uint8_t waypoint_index) const;

    TargetPoint waypoints_[MAX_WAYPOINTS];
    TargetPoint target_;
    uint8_t waypoint_count_;
    uint8_t current_index_;

    Output output_;
    PlannerState state_;

    float min_move_v_;

    float slow_dist_;
    float contact_dist_;
    float finish_dist_;

    float kp_approach_;
    float kd_approach_;
    float kp_slow_;
    float kd_slow_;
    float kp_contact_;
    float kd_contact_;

    float yaw_sign_;
    float yaw_kp_;
    float min_yaw_wz_;
    float strong_yaw_wz_;
    float strong_yaw_error_deg_;
    float moving_wz_max_;
    float settle_wz_max_;
    float yaw_tolerance_deg_;

    uint16_t stable_cycles_;
    uint32_t contact_hold_ms_;
    uint32_t contact_timeout_ms_;
    uint32_t soft_contact_start_tick_;
    uint16_t soft_contact_stable_count_;

    const Output &getOutput(void) const;
    PlannerState getState(void) const;
    bool isFinished(void) const;

private:
    void loadCurrentWaypoint(void);
    void setZeroOutput(void);
    void updateState(float distance_m, uint8_t xy_in_tolerance, uint32_t now_tick);
    void raiseVectorToMin(float &vx, float &vy, float min_speed) const;
    float limitFloat(float value, float min_value, float max_value) const;
    float normalizeAngleDeg(float angle) const;
    float safeSqrt(float value) const;
};

extern WuqiquPathPlanner wuqiqu;

#endif
