#ifndef MIELING_H
#define MIELING_H

#include <stdint.h>
#include "main.h"

#define MEILING_STABLE_COUNT 200U
#define MEILING_TIMEOUT_MS 5000U

#define ELAT_KP 6.0f
#define ELAT_MAX 1.0f

#define SENSOR_FRONT 0x01U
#define SENSOR_LEFT 0x02U
#define SENSOR_RIGHT 0x04U
#define SENSOR_ALL 0x07U

struct MeilingTarget_t
{
    uint8_t preset_id;
    float L_ref;
    float R_ref;
    float F_ref;
    float tol_lat;
    float tol_lon;
    uint32_t timeout_ms;
    uint8_t sensor_mask;
};

struct MeilingState_t
{
    float L_meas;
    float R_meas;
    float F_meas;
    float e_lat;
    float e_lon;
    uint8_t stable_cnt;
    uint8_t result;
};

class MeilingLocator
{
public:
    static const uint8_t RUNNING = 0U;
    static const uint8_t SUCCESS = 1U;
    static const uint8_t TIMEOUT = 2U;

    MeilingTarget_t m_target = {};
    MeilingState_t m_state = {};

    void start(const MeilingTarget_t &target);
    uint8_t update(void);

    float getChassisVxTarget(float manual_target) const;
    float getChassisVyTarget(float manual_target) const;
    float getChassisVzTarget(float manual_target) const;

private:
    uint8_t m_running = 0U;
    uint32_t m_start_tick = 0U;

    void calcErrors(void);
    uint8_t allInTolerance(void) const;
    uint8_t isTimeout(void) const;
};

extern MeilingLocator meiling;

#endif
