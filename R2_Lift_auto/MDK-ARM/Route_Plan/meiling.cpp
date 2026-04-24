#include "mieling.h"
#include "DT35.h"
#include "main.h"
#include <math.h>

MeilingLocator meiling;

namespace
{
static float abs_limit(float x, float min, float max)
{
    float abs_min = (min < 0.0f) ? -min : min;
    float abs_max = (max < 0.0f) ? -max : max;

    if (abs_max < abs_min)
    {
        float temp = abs_max;
        abs_max = abs_min;
        abs_min = temp;
    }

    if (x > abs_max)
    {
        return abs_max;
    }
    if (x < -abs_max)
    {
        return -abs_max;
    }
    if (x > 0.0f && x < abs_min)
    {
        return abs_min;
    }
    if (x < 0.0f && x > -abs_min)
    {
        return -abs_min;
    }

    return x;
}
}

void MeilingLocator::start(const MeilingTarget_t &target)
{
    m_target = target;

    if (m_target.timeout_ms == 0U)
    {
        m_target.timeout_ms = MEILING_TIMEOUT_MS;
    }

    m_state = {};
    m_running = 1U;
    m_start_tick = HAL_GetTick();
}

uint8_t MeilingLocator::update(void)
{
    if (m_running == 0U)
    {
        return RUNNING;
    }

    if (isTimeout())
    {
        m_state.result = TIMEOUT;
        m_running = 0U;
        return TIMEOUT;
    }

    calcErrors();

    if (allInTolerance())
    {
        m_state.stable_cnt++;
        if (m_state.stable_cnt >= MEILING_STABLE_COUNT)
        {
            m_state.result = SUCCESS;
            m_running = 0U;
            return SUCCESS;
        }
    }
    else
    {
        m_state.stable_cnt = 0U;
    }

    return RUNNING;
}

void MeilingLocator::calcErrors(void)
{
    uint8_t has_front = (m_target.sensor_mask & SENSOR_FRONT) != 0U;
    uint8_t has_left = (m_target.sensor_mask & SENSOR_LEFT) != 0U;
    uint8_t has_right = (m_target.sensor_mask & SENSOR_RIGHT) != 0U;

    if (has_front)
    {
        m_state.F_meas = dt35.ch0.distance_mm;
    }
    if (has_left)
    {
        m_state.L_meas = dt35.ch1.distance_mm;
    }
    if (has_right)
    {
        m_state.R_meas = dt35.ch2.distance_mm;
    }

    if (has_front)
    {
        m_state.e_lon = m_state.F_meas - m_target.F_ref;
    }
    else
    {
        m_state.e_lon = 0.0f;
    }

    if (has_left && has_right)
    {
        float e_L = m_state.L_meas - m_target.L_ref;
        float e_R = m_state.R_meas - m_target.R_ref;
        m_state.e_lat = (e_L - e_R) / 2.0f;
    }
    else if (has_left)
    {
        m_state.e_lat = m_state.L_meas - m_target.L_ref;
    }
    else if (has_right)
    {
        m_state.e_lat = m_target.R_ref - m_state.R_meas;
    }
    else
    {
        m_state.e_lat = 0.0f;
    }
}

uint8_t MeilingLocator::allInTolerance(void) const
{
    float abs_lat = m_state.e_lat < 0.0f ? -m_state.e_lat : m_state.e_lat;
    float abs_lon = m_state.e_lon < 0.0f ? -m_state.e_lon : m_state.e_lon;

    if (abs_lat > m_target.tol_lat)
    {
        return 0U;
    }
    if (abs_lon > m_target.tol_lon)
    {
        return 0U;
    }
    return 1U;
}

uint8_t MeilingLocator::isTimeout(void) const
{
    return (HAL_GetTick() - m_start_tick >= m_target.timeout_ms) ? 1U : 0U;
}

float MeilingLocator::getChassisVxTarget(float manual_target) const
{
    if (m_running == 0U)
    {
        return manual_target;
    }

    float V = -ELAT_KP * m_state.e_lat * 0.001f;
    V = abs_limit(V, 0.45f, ELAT_MAX);

    if (fabsf(m_state.e_lat) < m_target.tol_lat)
    {
        V = 0.0f;
    }

    return V;
}

float ELON_KP = 5.0f;
float ELON_MAX = 1.0f;

float MeilingLocator::getChassisVyTarget(float manual_target) const
{
    if (m_running == 0U)
    {
        return manual_target;
    }

    float V = ELON_KP * m_state.e_lon * 0.001f;
    V = abs_limit(V, 0.6f, ELON_MAX);

    if (fabsf(m_state.e_lon) < m_target.tol_lon)
    {
        V = 0.0f;
    }

    return V;
}

float MeilingLocator::getChassisVzTarget(float manual_target) const
{
    return manual_target;
}
