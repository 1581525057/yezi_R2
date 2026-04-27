#include "../TASK/chassis_auto_source.h"

#include <cstdlib>

static void expect_equal(ChassisAutoSource actual, ChassisAutoSource expected)
{
    if (actual != expected) {
        std::exit(1);
    }
}

int main()
{
    expect_equal(ChassisAuto_SelectSource(0U, 0U), CHASSIS_AUTO_NONE);
    expect_equal(ChassisAuto_SelectSource(0U, 1U), CHASSIS_AUTO_MEILING);
    expect_equal(ChassisAuto_SelectSource(1U, 0U), CHASSIS_AUTO_WUQIQU);
    expect_equal(ChassisAuto_SelectSource(1U, 1U), CHASSIS_AUTO_CONFLICT);
    return 0;
}
