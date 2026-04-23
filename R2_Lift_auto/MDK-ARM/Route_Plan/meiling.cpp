/* -----------------------------------------------------------------------
 * meiling.cpp
 * 简单说：这里负责“贴边/对位”。
 * 它读 DT35 测到的距离，算出车离目标还差多少，
 * 然后分两步修正：先修横向误差 Vx，再修前后误差 Vy。
 *
 * 注意：
 *   Vx、Vy 只在对应阶段自动给速度。
 *   Vz 这里不自动改，直接沿用外面给的手动值。
 * ----------------------------------------------------------------------- */

#include "mieling.h"
#include "DT35.h"
#include "main.h" /* 用 HAL_GetTick 取系统时间，判断是否超时 */

MeilingLocator meiling;

void MeilingLocator::start(const MeilingTarget_t &target)
{
    m_target = target;

    if (m_target.timeout_ms == 0U)
    {
        m_target.timeout_ms = MEILING_TIMEOUT_MS;
    }

    m_state = {};
    m_phase = PHASE_LAT;
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

    switch (m_phase)
    {
    case PHASE_LAT:
        if ((m_state.e_lat > m_target.tol_lat) || (m_state.e_lat < -m_target.tol_lat))
        {
            /* 横向未对准，停在此阶段。
             * chassis_task 每帧主动调 getChassisVxTarget 取修正速度：
             *   e_lat>0 表示车偏右 → 函数内 V=-Kp*e_lat<0 → Vx<0 向左修正，符号正确。 */
        }
        else
        {
            m_phase = PHASE_LON;
        }
        break;

    case PHASE_LON:
        if ((m_state.e_lon > m_target.tol_lon) || (m_state.e_lon < -m_target.tol_lon))
        {
            /* 纵向未对准，停在此阶段。
             * chassis_task 每帧主动调 getChassisVyTarget 取修正速度：
             *   e_lon = F_meas - F_ref，e_lon>0 表示车离前墙太远 → 函数内 V=+Kp*e_lon>0 → Vy>0 向前靠近，符号正确。 */
        }
        else
        {
            m_phase = PHASE_DONE;
        }
        break;

    case PHASE_DONE:
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

            float abs_lat = m_state.e_lat < 0.0f ? -m_state.e_lat : m_state.e_lat;
            if (abs_lat > m_target.tol_lat)
            {
                m_phase = PHASE_LAT;
            }
            else
            {
                m_phase = PHASE_LON;
            }
        }
        break;

    default:
        m_phase = PHASE_LAT;
        break;
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

        /* e_lat 是横向误差：左、右传感器一起用时，用两边误差的差值来判断车偏哪边。 */
        m_state.e_lat = (e_L - e_R) / 2.0f;
    }
    else if (has_left)
    {
        /* 只有左边传感器时，只能拿左边距离和左边目标距离相减，当作横向误差。 */
        m_state.e_lat = m_state.L_meas - m_target.L_ref;
    }
    else if (has_right)
    {
        /* 只有右边传感器时，用目标右距离减实际右距离，保证误差方向和左传感器算法一致。 */
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
    if (m_phase != PHASE_LAT)
    {
        return 0.0f;
    }

    float V = -ELAT_KP * m_state.e_lat * 0.001f;
    if (V > ELAT_MAX)
    {
        V = ELAT_MAX;
    }
    else if (V < -ELAT_MAX)
    {
        V = -ELAT_MAX;
    }
    return V;
}

float MeilingLocator::getChassisVyTarget(float manual_target) const
{
    if (m_running == 0U)
    {
        return manual_target;
    }
    if (m_phase != PHASE_LON)
    {
        return 0.0f;
    }

    float V = ELON_KP * m_state.e_lon * 0.001f;
    if (V > ELON_MAX)
    {
        V = ELON_MAX;
    }
    else if (V < -ELON_MAX)
    {
        V = -ELON_MAX;
    }
    return V;
}

float MeilingLocator::getChassisVzTarget(float manual_target) const
{
    return manual_target;
}
