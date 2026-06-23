#ifndef ROUTE_TEST_STUB_MIELING_H
#define ROUTE_TEST_STUB_MIELING_H

#include <stdint.h>

#define SENSOR_FRONT 0x01U
#define SENSOR_LEFT  0x02U
#define SENSOR_RIGHT 0x04U

typedef struct {
    uint8_t preset_id;
    float L_ref;
    float R_ref;
    float F_ref;
    float tol_lat;
    float tol_lon;
    uint32_t timeout_ms;
    uint8_t sensor_mask;
} MeilingTarget_t;

class MeilingLocator
{
public:
    enum {
        SUCCESS = 1,
        TIMEOUT = 2
    };

    MeilingLocator()
        : start_count(0U),
          last_target({})
    {
    }

    void start(const MeilingTarget_t &target)
    {
        start_count++;
        last_target = target;
    }

    uint8_t update(void) { return 0U; }

    uint8_t start_count;
    MeilingTarget_t last_target;
};

extern MeilingLocator meiling;

#endif
