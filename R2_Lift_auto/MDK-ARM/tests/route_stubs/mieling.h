#ifndef ROUTE_TEST_STUB_MIELING_H
#define ROUTE_TEST_STUB_MIELING_H

#include <stdint.h>

#define SENSOR_FRONT 0x01U
#define SENSOR_LEFT  0x02U

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

    void start(const MeilingTarget_t &) {}
    uint8_t update(void) { return 0U; }
};

extern MeilingLocator meiling;

#endif
