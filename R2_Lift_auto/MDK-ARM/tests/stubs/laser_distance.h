#ifndef TEST_STUB_LASER_DISTANCE_H
#define TEST_STUB_LASER_DISTANCE_H

#include <stdint.h>

#define LASER_RX_LEN 32

struct Laser_data {
    float distance_m;
    uint32_t distance_mm;
    uint16_t error_code;
    uint8_t valid;
    uint8_t error;
};

class LaserDistance
{
public:
    Laser_data data;
    uint8_t rx_buf[2][LASER_RX_LEN];
};

extern LaserDistance laser_left;
extern LaserDistance laser_right;

#endif
