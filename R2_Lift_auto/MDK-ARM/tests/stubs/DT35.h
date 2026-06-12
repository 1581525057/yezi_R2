#ifndef TEST_STUB_DT35_H
#define TEST_STUB_DT35_H

#include <stdint.h>

struct DT35_Data_t {
    float distance_mm;
    float distance_filtered;
    float voltage_V;
    uint16_t adc_raw;
    uint8_t valid;
};

class DT35
{
public:
    DT35_Data_t ch0;
    DT35_Data_t ch1;
    DT35_Data_t ch2;
    DT35_Data_t ch3;
};

extern DT35 dt35;

#endif
