#ifndef BR_MCU_BSPLINE_PATH_GENERATOR_H
#define BR_MCU_BSPLINE_PATH_GENERATOR_H

#include <cstddef>

static const float BR_PATH_DEFAULT_POINT_GAP_M = 0.1f;

struct BRPathPose {
    float x_m;
    float y_m;
    float yaw_rad;
};

struct BRPathControlPoint {
    float x_m;
    float y_m;
};

struct BRPathPoint {
    float vx_mm_s;
    float vy_mm_s;
    float x_mm;
    float y_mm;
    float yaw_rad;
};

enum BRPathStatus {
    BR_PATH_OK = 0,
    BR_PATH_INVALID_ARGUMENT = 1,
    BR_PATH_OUTPUT_TOO_SMALL = 2
};

BRPathStatus generateBsplinePath(
    const BRPathPose& start,
    const BRPathPose& goal,
    float max_vel_m_s,
    float max_acc_m_s2,
    BRPathPoint* output,
    std::size_t output_capacity,
    std::size_t* output_count,
    float point_gap_m = BR_PATH_DEFAULT_POINT_GAP_M
);

BRPathStatus generateBsplinePath(
    const BRPathPose& start,
    const BRPathControlPoint* middle_points,
    std::size_t middle_point_count,
    const BRPathPose& goal,
    float max_vel_m_s,
    float max_acc_m_s2,
    BRPathPoint* output,
    std::size_t output_capacity,
    std::size_t* output_count,
    float point_gap_m = BR_PATH_DEFAULT_POINT_GAP_M
);

#endif
