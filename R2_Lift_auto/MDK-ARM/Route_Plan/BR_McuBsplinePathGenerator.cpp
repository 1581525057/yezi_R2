#include "BR_McuBsplinePathGenerator.h"

#include <cmath>

namespace {

struct Vec2 {
    float x;
    float y;
};

static const float kPi = 3.14159265358979323846f;
static const int kMinDenseSteps = 80;
static const int kMaxDenseSteps = 800;
static const std::size_t kMaxControlPoints = 16;

bool isFiniteFloat(float value) {
    return std::isfinite(value) != 0;
}

float clampFloat(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

float distance2D(const Vec2& a, const Vec2& b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

float shortestAngleDelta(float from_rad, float to_rad) {
    const float diff = to_rad - from_rad;
    return std::atan2(std::sin(diff), std::cos(diff));
}

int splineDegree(std::size_t control_count) {
    if (control_count >= 4) {
        return 3;
    }
    return static_cast<int>(control_count - 1);
}

float knotValue(int knot_index, std::size_t control_count, int degree) {
    if (knot_index <= degree) {
        return 0.0f;
    }
    if (knot_index >= static_cast<int>(control_count)) {
        return 1.0f;
    }

    return static_cast<float>(knot_index - degree) /
        static_cast<float>(static_cast<int>(control_count) - degree);
}

int findSplineSpan(float t, std::size_t control_count, int degree) {
    const int last_control_index = static_cast<int>(control_count) - 1;
    if (t >= 1.0f) {
        return last_control_index;
    }
    if (t <= 0.0f) {
        return degree;
    }

    for (int i = degree; i <= last_control_index; ++i) {
        if (t >= knotValue(i, control_count, degree) &&
            t < knotValue(i + 1, control_count, degree)) {
            return i;
        }
    }

    return last_control_index;
}

Vec2 splinePoint(const Vec2* control, std::size_t control_count, float t) {
    if (t <= 0.0f) {
        return control[0];
    }
    if (t >= 1.0f) {
        return control[control_count - 1];
    }

    const int degree = splineDegree(control_count);
    const int span = findSplineSpan(t, control_count, degree);
    Vec2 temp[4];

    for (int j = 0; j <= degree; ++j) {
        temp[j] = control[span - degree + j];
    }

    for (int r = 1; r <= degree; ++r) {
        for (int j = degree; j >= r; --j) {
            const int knot_index = span - degree + j;
            const float left = knotValue(knot_index, control_count, degree);
            const float right = knotValue(knot_index + degree - r + 1, control_count, degree);
            const float denom = right - left;
            const float alpha = denom > 0.000001f ? (t - left) / denom : 0.0f;

            temp[j].x = (1.0f - alpha) * temp[j - 1].x + alpha * temp[j].x;
            temp[j].y = (1.0f - alpha) * temp[j - 1].y + alpha * temp[j].y;
        }
    }

    return temp[degree];
}

float controlPolygonLength(const Vec2* control, std::size_t control_count) {
    float length = 0.0f;
    for (std::size_t i = 1; i < control_count; ++i) {
        length += distance2D(control[i - 1], control[i]);
    }
    return length;
}

float estimateLength(const Vec2* control, std::size_t control_count, int dense_steps) {
    float length = 0.0f;
    Vec2 previous = splinePoint(control, control_count, 0.0f);

    for (int i = 1; i <= dense_steps; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(dense_steps);
        const Vec2 current = splinePoint(control, control_count, t);
        length += distance2D(previous, current);
        previous = current;
    }

    return length;
}

int chooseDenseSteps(float straight_distance_m, float point_gap_m) {
    int steps = static_cast<int>((straight_distance_m / point_gap_m) * 20.0f);
    if (steps < kMinDenseSteps) {
        steps = kMinDenseSteps;
    }
    if (steps > kMaxDenseSteps) {
        steps = kMaxDenseSteps;
    }
    return steps;
}

BRPathStatus appendPoint(
    BRPathPoint* output,
    std::size_t output_capacity,
    std::size_t* count,
    const Vec2& point_m,
    float yaw_rad
) {
    if (*count >= output_capacity) {
        return BR_PATH_OUTPUT_TOO_SMALL;
    }

    BRPathPoint& point = output[*count];
    point.vx_mm_s = 0.0f;
    point.vy_mm_s = 0.0f;
    point.x_mm = point_m.x * 1000.0f;
    point.y_mm = point_m.y * 1000.0f;
    point.yaw_rad = yaw_rad;
    ++(*count);
    return BR_PATH_OK;
}

float pathPointDistanceMm(const BRPathPoint& a, const BRPathPoint& b) {
    const float dx = b.x_mm - a.x_mm;
    const float dy = b.y_mm - a.y_mm;
    return std::sqrt(dx * dx + dy * dy);
}

void fillVelocity(BRPathPoint* output, std::size_t count, float max_vel_m_s, float max_acc_m_s2) {
    if (count == 0) {
        return;
    }

    output[0].vx_mm_s = 0.0f;
    output[0].vy_mm_s = 0.0f;
    if (count == 1) {
        return;
    }

    float total_length_m = 0.0f;
    for (std::size_t i = 1; i < count; ++i) {
        total_length_m += pathPointDistanceMm(output[i - 1], output[i]) * 0.001f;
    }

    float current_s_m = 0.0f;
    for (std::size_t i = 1; i + 1 < count; ++i) {
        current_s_m += pathPointDistanceMm(output[i - 1], output[i]) * 0.001f;
        const float accel_limited = std::sqrt(2.0f * max_acc_m_s2 * current_s_m);
        const float decel_limited = std::sqrt(2.0f * max_acc_m_s2 * (total_length_m - current_s_m));
        const float speed_m_s = clampFloat(
            accel_limited < decel_limited ? accel_limited : decel_limited,
            0.0f,
            max_vel_m_s
        );

        const float dx = output[i + 1].x_mm - output[i].x_mm;
        const float dy = output[i + 1].y_mm - output[i].y_mm;
        const float distance_mm = std::sqrt(dx * dx + dy * dy);
        if (distance_mm > 0.001f) {
            const float speed_mm_s = speed_m_s * 1000.0f;
            output[i].vx_mm_s = speed_mm_s * dx / distance_mm;
            output[i].vy_mm_s = speed_mm_s * dy / distance_mm;
        } else {
            output[i].vx_mm_s = 0.0f;
            output[i].vy_mm_s = 0.0f;
        }
    }

    output[count - 1].vx_mm_s = 0.0f;
    output[count - 1].vy_mm_s = 0.0f;
}

BRPathStatus generatePathFromControls(
    const Vec2* control,
    std::size_t control_count,
    float start_yaw_rad,
    float goal_yaw_rad,
    float max_vel_m_s,
    float max_acc_m_s2,
    BRPathPoint* output,
    std::size_t output_capacity,
    std::size_t* output_count,
    float point_gap_m
) {
    if (control_count < 2) {
        return BR_PATH_INVALID_ARGUMENT;
    }

    const float control_length_m = controlPolygonLength(control, control_count);
    const float angle_delta = shortestAngleDelta(start_yaw_rad, goal_yaw_rad);

    if (control_length_m < 0.001f) {
        BRPathStatus status = appendPoint(output, output_capacity, output_count, control[0], start_yaw_rad);
        if (status != BR_PATH_OK) {
            return status;
        }
        status = appendPoint(output, output_capacity, output_count, control[control_count - 1], start_yaw_rad + angle_delta);
        if (status != BR_PATH_OK) {
            return status;
        }
        fillVelocity(output, *output_count, max_vel_m_s, max_acc_m_s2);
        return BR_PATH_OK;
    }

    const int dense_steps = chooseDenseSteps(control_length_m, point_gap_m);
    const float path_length_m = estimateLength(control, control_count, dense_steps);
    const int required_points = static_cast<int>(std::ceil(path_length_m / point_gap_m)) + 1;
    if (required_points > static_cast<int>(output_capacity)) {
        return BR_PATH_OUTPUT_TOO_SMALL;
    }

    BRPathStatus status = appendPoint(output, output_capacity, output_count, control[0], start_yaw_rad);
    if (status != BR_PATH_OK) {
        return status;
    }

    Vec2 previous_curve = splinePoint(control, control_count, 0.0f);
    float distance_from_last_output_m = 0.0f;

    for (int i = 1; i <= dense_steps; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(dense_steps);
        const Vec2 current_curve = splinePoint(control, control_count, t);
        Vec2 segment_start = previous_curve;
        float segment_length_m = distance2D(segment_start, current_curve);

        while (segment_length_m > 0.000001f &&
               distance_from_last_output_m + segment_length_m >= point_gap_m) {
            const float need_m = point_gap_m - distance_from_last_output_m;
            const float ratio = need_m / segment_length_m;
            Vec2 new_point;
            new_point.x = segment_start.x + (current_curve.x - segment_start.x) * ratio;
            new_point.y = segment_start.y + (current_curve.y - segment_start.y) * ratio;

            const float t_for_yaw = clampFloat(
                static_cast<float>(*output_count) * point_gap_m / path_length_m,
                0.0f,
                1.0f
            );
            status = appendPoint(output, output_capacity, output_count, new_point, start_yaw_rad + angle_delta * t_for_yaw);
            if (status != BR_PATH_OK) {
                return status;
            }

            segment_start = new_point;
            segment_length_m = distance2D(segment_start, current_curve);
            distance_from_last_output_m = 0.0f;
        }

        distance_from_last_output_m += segment_length_m;
        previous_curve = current_curve;
    }

    const Vec2 goal_point = control[control_count - 1];
    Vec2 last_output_point;
    last_output_point.x = output[*output_count - 1].x_mm * 0.001f;
    last_output_point.y = output[*output_count - 1].y_mm * 0.001f;
    if (*output_count > 0 && distance2D(goal_point, last_output_point) < 0.001f) {
        output[*output_count - 1].x_mm = goal_point.x * 1000.0f;
        output[*output_count - 1].y_mm = goal_point.y * 1000.0f;
        output[*output_count - 1].yaw_rad = start_yaw_rad + angle_delta;
    } else {
        status = appendPoint(output, output_capacity, output_count, goal_point, start_yaw_rad + angle_delta);
        if (status != BR_PATH_OK) {
            return status;
        }
    }

    fillVelocity(output, *output_count, max_vel_m_s, max_acc_m_s2);
    return BR_PATH_OK;
}

}

BRPathStatus generateBsplinePath(
    const BRPathPose& start,
    const BRPathPose& goal,
    float max_vel_m_s,
    float max_acc_m_s2,
    BRPathPoint* output,
    std::size_t output_capacity,
    std::size_t* output_count,
    float point_gap_m
) {
    if (output_count == 0) {
        return BR_PATH_INVALID_ARGUMENT;
    }
    *output_count = 0;

    if (output == 0 || output_capacity < 2 ||
        !isFiniteFloat(start.x_m) || !isFiniteFloat(start.y_m) || !isFiniteFloat(start.yaw_rad) ||
        !isFiniteFloat(goal.x_m) || !isFiniteFloat(goal.y_m) || !isFiniteFloat(goal.yaw_rad) ||
        !isFiniteFloat(max_vel_m_s) || !isFiniteFloat(max_acc_m_s2) || !isFiniteFloat(point_gap_m) ||
        max_vel_m_s <= 0.0f || max_acc_m_s2 <= 0.0f || point_gap_m <= 0.0f) {
        return BR_PATH_INVALID_ARGUMENT;
    }

    Vec2 control[4];
    control[0].x = start.x_m;
    control[0].y = start.y_m;
    control[3].x = goal.x_m;
    control[3].y = goal.y_m;

    const float straight_distance_m = distance2D(control[0], control[3]);
    const float angle_delta = shortestAngleDelta(start.yaw_rad, goal.yaw_rad);

    if (straight_distance_m < 0.001f) {
        BRPathStatus status = appendPoint(output, output_capacity, output_count, control[0], start.yaw_rad);
        if (status != BR_PATH_OK) {
            return status;
        }
        status = appendPoint(output, output_capacity, output_count, control[3], start.yaw_rad + angle_delta);
        if (status != BR_PATH_OK) {
            return status;
        }
        fillVelocity(output, *output_count, max_vel_m_s, max_acc_m_s2);
        return BR_PATH_OK;
    }

    float control_distance_m = straight_distance_m / 3.0f;
    if (straight_distance_m >= 0.15f) {
        control_distance_m = clampFloat(control_distance_m, 0.05f, 2.0f);
    }

    control[1].x = control[0].x + std::cos(start.yaw_rad) * control_distance_m;
    control[1].y = control[0].y + std::sin(start.yaw_rad) * control_distance_m;
    control[2].x = control[3].x - std::cos(goal.yaw_rad) * control_distance_m;
    control[2].y = control[3].y - std::sin(goal.yaw_rad) * control_distance_m;

    return generatePathFromControls(
        control,
        4,
        start.yaw_rad,
        goal.yaw_rad,
        max_vel_m_s,
        max_acc_m_s2,
        output,
        output_capacity,
        output_count,
        point_gap_m
    );
}

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
    float point_gap_m
) {
    if (output_count == 0) {
        return BR_PATH_INVALID_ARGUMENT;
    }
    *output_count = 0;

    if (output == 0 || output_capacity < 2 ||
        (middle_point_count > 0 && middle_points == 0) ||
        middle_point_count + 2 > kMaxControlPoints ||
        !isFiniteFloat(start.x_m) || !isFiniteFloat(start.y_m) || !isFiniteFloat(start.yaw_rad) ||
        !isFiniteFloat(goal.x_m) || !isFiniteFloat(goal.y_m) || !isFiniteFloat(goal.yaw_rad) ||
        !isFiniteFloat(max_vel_m_s) || !isFiniteFloat(max_acc_m_s2) || !isFiniteFloat(point_gap_m) ||
        max_vel_m_s <= 0.0f || max_acc_m_s2 <= 0.0f || point_gap_m <= 0.0f) {
        return BR_PATH_INVALID_ARGUMENT;
    }

    Vec2 control[kMaxControlPoints];
    control[0].x = start.x_m;
    control[0].y = start.y_m;

    for (std::size_t i = 0; i < middle_point_count; ++i) {
        if (!isFiniteFloat(middle_points[i].x_m) || !isFiniteFloat(middle_points[i].y_m)) {
            return BR_PATH_INVALID_ARGUMENT;
        }
        control[i + 1].x = middle_points[i].x_m;
        control[i + 1].y = middle_points[i].y_m;
    }

    const std::size_t goal_index = middle_point_count + 1;
    control[goal_index].x = goal.x_m;
    control[goal_index].y = goal.y_m;

    return generatePathFromControls(
        control,
        middle_point_count + 2,
        start.yaw_rad,
        goal.yaw_rad,
        max_vel_m_s,
        max_acc_m_s2,
        output,
        output_capacity,
        output_count,
        point_gap_m
    );
}
