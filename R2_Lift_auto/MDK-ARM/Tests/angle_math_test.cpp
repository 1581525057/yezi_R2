#include "../Control/angle_math.h"

#include <cmath>
#include <cstdlib>

static void expect_near(float actual, float expected)
{
    if (std::fabs(actual - expected) > 0.001f) {
        std::exit(1);
    }
}

int main()
{
    expect_near(ShortestAngleErrorDeg(90.0f, 0.0f), 90.0f);
    expect_near(ShortestAngleErrorDeg(-90.0f, 0.0f), -90.0f);
    expect_near(ShortestAngleErrorDeg(-179.0f, 179.0f), 2.0f);
    expect_near(ShortestAngleErrorDeg(179.0f, -179.0f), -2.0f);
    expect_near(NormalizeAngleDeg(181.0f), -179.0f);
    expect_near(NormalizeAngleDeg(-181.0f), 179.0f);
    return 0;
}
