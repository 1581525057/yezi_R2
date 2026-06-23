#include "path_follow.h"

#include <math.h>

static int floatNear(float actual, float expected)
{
    return (fabsf(actual - expected) < 0.001f) ? 1 : 0;
}

static int expectWorldToBodyKeepsVectorWhenYawZero(void)
{
    float body_x = 0.0f;
    float body_y = 0.0f;

    PathFollower::worldToBody(100.0f, -50.0f, 0.0f, &body_x, &body_y);

    if (floatNear(body_x, 100.0f) == 0) {
        return 1;
    }
    if (floatNear(body_y, -50.0f) == 0) {
        return 2;
    }

    return 0;
}

static int expectWorldToBodyRotatesByNegativeYaw(void)
{
    float body_x = 0.0f;
    float body_y = 0.0f;

    PathFollower::worldToBody(100.0f, 0.0f, 1.57079632679f, &body_x, &body_y);

    if (floatNear(body_x, 0.0f) == 0) {
        return 1;
    }
    if (floatNear(body_y, -100.0f) == 0) {
        return 2;
    }

    return 0;
}

static int expectFinalPointFinishesImmediately(void)
{
    const PathFollower::PathPoint path[] = {
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {100.0f, 0.0f, 100.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 200.0f, 0.0f, 1.0f},
    };
    const PathFollower::Pose pose = {200.0f, 0.0f, 0.0f};
    PathFollower follower;

    follower.loadPath(path, 3U);
    follower.follow(pose);

    const PathFollower::Output &output = follower.getOutput();
    if (follower.getState() != PathFollower::STATE_FINISHED) {
        return 1;
    }
    if (floatNear(output.world_vx, 0.0f) == 0 ||
        floatNear(output.world_vy, 0.0f) == 0 ||
        floatNear(output.wz, 0.0f) == 0) {
        return 2;
    }

    return 0;
}

int main(void)
{
    int result = expectWorldToBodyKeepsVectorWhenYawZero();
    if (result != 0) {
        return result;
    }

    result = expectWorldToBodyRotatesByNegativeYaw();
    if (result != 0) {
        return 10 + result;
    }

    result = expectFinalPointFinishesImmediately();
    if (result != 0) {
        return 20 + result;
    }

    return 0;
}
