#include "plan_route.h"

#include <cmath>

#include "arm_math.h"

namespace {

BR_PathFollower g_path_follower;

}

BR_PathFollower::BR_PathFollower()
    : current_path_index_(0),
      vel_calc_entered_(false),
      last_err_theta_(0.0f),
      last_vector4_{0.0f, 0.0f}
{
}

void BR_PathFollower::reset()
{
    current_path_index_ = 0;
    vel_calc_entered_ = false;
    last_err_theta_ = 0.0f;
    last_vector4_[0] = 0.0f;
    last_vector4_[1] = 0.0f;
}

bool BR_PathFollower::safeCheck(const float *current_target_point, float current_x, float current_y) const
{
    const float safe_distance =
        (current_target_point[0] == 0.0f && current_target_point[1] == 0.0f)
            ? BR_configFOLLOWING_SAFE_DISTANCE * 4.0f
            : BR_configFOLLOWING_SAFE_DISTANCE;

    const float error_dist = BR_Abs(current_target_point[2] - current_x)
                           + BR_Abs(current_target_point[3] - current_y);

    return error_dist > safe_distance;
}

void BR_PathFollower::calculateVelocity(const BR_Path_t &path, float *vx, float *vy, float *omega,
                                        float current_x, float current_y, float current_theta)
{
    float (*pPath)[5] = path.pPath;

    const float main_vel_x = pPath[current_path_index_][0];
    const float main_vel_y = pPath[current_path_index_][1];

    const float v1x = pPath[current_path_index_][2] - current_x;
    const float v1y = pPath[current_path_index_][3] - current_y;

    const float v2x = pPath[current_path_index_ - 1][2] - pPath[current_path_index_][2];
    const float v2y = pPath[current_path_index_ - 1][3] - pPath[current_path_index_][3];

    const float seg_length = std::sqrt(v2x * v2x + v2y * v2y);
    const float projection = BR_Abs((v1x * v2x + v1y * v2y) / seg_length);

    const float v3x = v2x / seg_length * projection;
    const float v3y = v2y / seg_length * projection;

    const float v4x = v1x + v3x;
    const float v4y = v1y + v3y;

    const float main_vel_mag = std::sqrt(main_vel_x * main_vel_x + main_vel_y * main_vel_y);
    const float normal_p = path.NormalP * main_vel_mag;
    const float normal_d = path.NormalD * main_vel_mag;

    float normal_vel_x = 0.0f;
    float normal_vel_y = 0.0f;
    if (vel_calc_entered_) {
        normal_vel_x = normal_p * v4x + normal_d * (v4x - last_vector4_[0]);
        normal_vel_y = normal_p * v4y + normal_d * (v4y - last_vector4_[1]);
    } else {
        normal_vel_x = normal_p * v4x;
        normal_vel_y = normal_p * v4y;
    }
    last_vector4_[0] = v4x;
    last_vector4_[1] = v4y;

    const float err_theta = pPath[current_path_index_][4] - current_theta;
    if (vel_calc_entered_) {
        *omega = path.ThetaP * err_theta + path.ThetaD * (err_theta - last_err_theta_);
    } else {
        *omega = path.ThetaP * err_theta;
        vel_calc_entered_ = true;
    }
    last_err_theta_ = err_theta;

    *vx = main_vel_x + normal_vel_x;
    *vy = main_vel_y + normal_vel_y;
}

void BR_PathFollower::followPath(const BR_Path_t &path, float *vx, float *vy, float *omega)
{
    float (*pPath)[5] = path.pPath;

    const float current_x = 0.0f;
    const float current_y = 0.0f;
    const float current_theta = 0.0f;

    if (safeCheck(pPath[current_path_index_], current_x, current_y)) {
        reset();
        return;
    }

    if (current_path_index_ == 0) {
        ++current_path_index_;
    } else if (pPath[current_path_index_][0] != 0.0f || pPath[current_path_index_][1] != 0.0f) {
        const float target_x = pPath[current_path_index_][2];
        const float target_y = pPath[current_path_index_][3];
        const float next_target_x = pPath[current_path_index_ + 1][2];
        const float next_target_y = pPath[current_path_index_ + 1][3];

        const float dot_product = (target_x - current_x) * (next_target_x - target_x)
                                + (target_y - current_y) * (next_target_y - target_y);
        if (dot_product < 0.0f) {
            ++current_path_index_;
        }

        calculateVelocity(path, vx, vy, omega, current_x, current_y, current_theta);
    }
}

BR_PathFollower &BR_GetPathFollower()
{
    return g_path_follower;
}

void BR_vResetPathFollowing()
{
    BR_GetPathFollower().reset();
}

void BR_vFollowPath(BR_Path_t path, float *vx, float *vy, float *omega)
{
    BR_GetPathFollower().followPath(path, vx, vy, omega);
}
