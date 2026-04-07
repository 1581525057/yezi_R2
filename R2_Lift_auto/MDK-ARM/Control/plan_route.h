#pragma once

#include <cstdint>

constexpr float BR_configFOLLOWING_SAFE_DISTANCE = 500.0f;

template<typename T>
inline T BR_Abs(T x)
{
    return x > T(0) ? x : -x;
}

struct BR_Path_t {
    float (*pPath)[5];
    float NormalP;
    float NormalD;
    float ThetaP;
    float ThetaD;
};

class BR_PathFollower
{
public:
    BR_PathFollower();

    void reset();
    void followPath(const BR_Path_t &path, float *vx, float *vy, float *omega);

private:
    bool safeCheck(const float *current_target_point, float current_x, float current_y) const;
    void calculateVelocity(const BR_Path_t &path, float *vx, float *vy, float *omega,
                           float current_x, float current_y, float current_theta);

private:
    int current_path_index_;
    bool vel_calc_entered_;
    float last_err_theta_;
    float last_vector4_[2];
};

BR_PathFollower &BR_GetPathFollower();

void BR_vResetPathFollowing();
void BR_vFollowPath(BR_Path_t path, float *vx, float *vy, float *omega);
