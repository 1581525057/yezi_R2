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
        STATE_FAST,
        STATE_SLOW,
        STATE_SETTLE,
        STATE_FINISHED
    };

    class TargetPoint
    {
    public:
        float x_m;
        float y_m;
        float yaw_deg;
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
    uint8_t overrideFirstWaypointWithPrelimWeaponHead(uint8_t weapon_index);

    TargetPoint waypoints_[MAX_WAYPOINTS];
    TargetPoint target_;
    uint8_t waypoint_count_;
    uint8_t current_index_;

    Output output_;
    PlannerState state_;

    float finish_dist_;
    float max_decel_mps2_;
    float brake_margin_m_;
    float finish_speed_tolerance_mps_;

    float kp_fast_;
    float kd_fast_;
    float kp_slow_;
    float kd_slow_;

    float yaw_sign_;
    float yaw_kp_;
    float yaw_kd_;
    float min_yaw_wz_;
    float strong_yaw_wz_;
    float strong_yaw_error_deg_;
    float yaw_tolerance_deg_;

    uint16_t stable_cycles_;
    uint32_t contact_hold_ms_;
    uint32_t contact_timeout_ms_;
    uint32_t settle_start_tick_;
    uint16_t settle_stable_count_;

    const Output &getOutput(void) const;
    PlannerState getState(void) const;
    bool isFinished(void) const;

private:
    void reloadDefaultWaypoints(void);
    void loadCurrentWaypoint(void);
    void setZeroOutput(void);
    void updateState(float distance_m, float speed_mps, uint8_t xy_in_tolerance, uint32_t now_tick);
    void limitVectorToMax(float &vx, float &vy, float max_speed) const;
    float limitFloat(float value, float min_value, float max_value) const;
    float normalizeAngleDeg(float angle) const;
    float safeSqrt(float value) const;
};

extern WuqiquPathPlanner wuqiqu;
void Wuqiqu_SetFirstWaypointX(float x_m);

#endif
