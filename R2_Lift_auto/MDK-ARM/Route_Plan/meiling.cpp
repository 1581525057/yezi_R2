/* -----------------------------------------------------------------------
 * 文件：meiling.cpp
 * 功能：梅林区精定位闭环实现（上台阶前 + 上台阶后）
 *
 * 【上台阶前精定位】
 *   三路激光全开（sensor_mask = SENSOR_ALL），skip_yaw = 0。
 *   执行顺序：yaw → 横向 → 纵向 → 稳定计数。
 *
 * 【上台阶后精定位】
 *   skip_yaw = 1，跳过 yaw 阶段，只校 X/Y。
 *   sensor_mask 按实际方块填写（小电脑随帧下发）：
 *     红方：1=前左右  3=前左  4=前左  6=前右
 *     蓝方：1=前左右  4=前右  5=前右  8=前左
 *   横向偏差根据可用侧激光自动切换：
 *     左+右可用：e_lat = (e_L - e_R) / 2
 *     仅左可用 ：e_lat =  L_meas - L_ref   （正=车偏右，需左移）
 *     仅右可用 ：e_lat =  R_ref  - R_meas  （正=车偏右，需左移）
 *   纵向偏差：e_lon = F_meas - F_ref（前激光，各方块均有）  
 *
 * 外部依赖：
 *   meiling_chassis_rotate_yaw(deg)  底盘原地旋转，正值顺时针
 *   meiling_chassis_move_lat(mm)     底盘横向平移，正值向右
 *   meiling_chassis_move_lon(mm)     底盘纵向平移，正值向前
 *   meiling_dt35_get_left_mm()       左侧 DT35 实测距离
 *   meiling_dt35_get_right_mm()      右侧 DT35 实测距离
 *   meiling_dt35_get_front_mm()      前方 DT35 实测距离
 * ----------------------------------------------------------------------- */

#include "mieling.h"
#include "DT35.h"
#include "main.h" /* HAL_GetTick */

/* 全局唯一实例 */
MeilingLocator meiling;

namespace
{
    /* CH0/CH1/CH2 are mapped to front/left/right DT35 here.
       Change these three constants if the wiring is different. */
    static const uint8_t MEILING_DT35_FRONT_CH = 0U;
    static const uint8_t MEILING_DT35_LEFT_CH  = 1U;
    static const uint8_t MEILING_DT35_RIGHT_CH = 2U;

    static const float MEILING_LINEAR_KP_MPS_PER_MM = 0.001f;
    static const float MEILING_LINEAR_MAX_MPS       = 0.25f;
    static const float MEILING_YAW_KP_RPS_PER_MM    = 0.002f;
    static const float MEILING_YAW_MAX_RPS          = 0.35f;
    static const uint32_t MEILING_CHASSIS_HOLD_MS   = 20U;

    struct MeilingChassisCmd_t {
        float vx;
        float vy;
        float vz;
        uint32_t expire_tick;
    };

    static MeilingChassisCmd_t s_chassis_cmd = {};

    static float clampFloat(float value, float min_value, float max_value)
    {
        if (value < min_value) { return min_value; }
        if (value > max_value) { return max_value; }
        return value;
    }

    static float linearCmdFromError(float err_mm)
    {
        return clampFloat(err_mm * MEILING_LINEAR_KP_MPS_PER_MM,
                          -MEILING_LINEAR_MAX_MPS,
                           MEILING_LINEAR_MAX_MPS);
    }

    static float yawCmdFromError(float err_mm)
    {
        return clampFloat(err_mm * MEILING_YAW_KP_RPS_PER_MM,
                          -MEILING_YAW_MAX_RPS,
                           MEILING_YAW_MAX_RPS);
    }

    static void setChassisCmd(float vx, float vy, float vz)
    {
        s_chassis_cmd.vx = vx;
        s_chassis_cmd.vy = vy;
        s_chassis_cmd.vz = vz;
        s_chassis_cmd.expire_tick = HAL_GetTick() + MEILING_CHASSIS_HOLD_MS;
    }

    static const DT35_Data_t &selectDt35Channel(uint8_t ch)
    {
        switch (ch)
        {
        case 1U: return dt35.ch1;
        case 2U: return dt35.ch2;
        case 3U: return dt35.ch3;
        case 0U:
        default: return dt35.ch0;
        }
    }

    static float getDt35DistanceMm(uint8_t ch)
    {
        const DT35_Data_t &data = selectDt35Channel(ch);
        return (data.valid != 0U) ? data.distance_mm : 0.0f;
    }
}

/* ====================== 外部接口声明 ===================================== */
/*
 * 以下接口由底盘模块和 DT35 模块提供实现。
 *
 * 【底盘运动接口调用约定】
 *   本模块传入的是激光测距计算出的真实偏差值，底盘侧应以此作为 PID 输入。
 *
 *   meiling_chassis_move_lat(float err_mm)
 *     err_mm < 0：车偏右，需向左平移
 *     err_mm > 0：车偏左，需向右平移
 *
 *   meiling_chassis_move_lon(float err_mm)
 *     err_mm < 0：距目标过近，需后退
 *     err_mm > 0：距目标过远，需前进
 *
 *   meiling_chassis_rotate_yaw(float err_mm)
 *     传入左右激光差值偏差（mm），底盘侧自行换算为角度后做 PID
 *     err_mm > 0：车头偏左，需顺时针旋转
 *     err_mm < 0：车头偏右，需逆时针旋转
 */



/* ====================== 公开接口实现 ===================================== */

/**
 * @brief  启动一次精定位流程
 * @param  preset  小电脑下发的预设点参数包
 * @note   上台阶前：preset.skip_yaw = 0，sensor_mask = SENSOR_ALL
 *         上台阶后：preset.skip_yaw = 1，sensor_mask 按方块填写
 */
void MeilingLocator::start(const MeilingPreset_t &preset)
{
    m_preset = preset;

    /* 若上层未填超时值，使用默认值 */
    if (m_preset.timeout_ms == 0U)
    {
        m_preset.timeout_ms = MEILING_TIMEOUT_MS;
    }

    /* 清空状态；skip_yaw=1 时从横向阶段直接开始 */
    m_state      = {};
    m_phase      = (m_preset.skip_yaw != 0U) ? PHASE_LAT : PHASE_YAW;
    m_running    = 1U;
    m_start_tick = HAL_GetTick();
}

/**
 * @brief  精定位闭环推进，每个控制周期调用一次
 * @retval RUNNING=0 进行中，SUCCESS=1 成功，TIMEOUT=2 超时失败
 */
uint8_t MeilingLocator::update(void)
{
    if (m_running == 0U)
    {
        return RUNNING;
    }

    /* 超时检测：任何阶段超出时限都直接判定失败 */
    if (isTimeout())
    {
        m_state.result = TIMEOUT;
        m_running      = 0U;
        return TIMEOUT;
    }

    /* 刷新可用 DT35 实测值并计算各轴偏差 */
    calcErrors();

    /* ----------------------------------------------------------------
     * 精定位阶段状态机
     * 上台阶前：PHASE_YAW → PHASE_LAT → PHASE_LON → PHASE_DONE
     * 上台阶后：PHASE_LAT → PHASE_LON → PHASE_DONE（跳过 yaw）
     * ---------------------------------------------------------------- */
    switch (m_phase)
    {
    /* ---- 阶段一：消除角度偏差（仅上台阶前，skip_yaw=0） ---- */
    case PHASE_YAW:
        if (m_state.e_yaw_obs > m_preset.tol_yaw)
        {
            /* 车头偏左（左距偏大），顺时针旋转；传入误差值供底盘 PID 使用 */
            meiling_chassis_rotate_yaw(m_state.e_yaw_obs);
        }
        else if (m_state.e_yaw_obs < -m_preset.tol_yaw)
        {
            /* 车头偏右，逆时针旋转 */
            meiling_chassis_rotate_yaw(m_state.e_yaw_obs);
        }
        else
        {
            m_phase = PHASE_LAT;
        }
        break;

    /* ---- 阶段二：消除横向偏差 ---- */
    case PHASE_LAT:
        if (m_state.e_lat > m_preset.tol_lat)
        {
            /* 车身整体偏右，向左平移；传入误差值供底盘 PID 使用 */
            meiling_chassis_move_lat(-m_state.e_lat);
        }
        else if (m_state.e_lat < -m_preset.tol_lat)
        {
            /* 车身偏左，向右平移 */
            meiling_chassis_move_lat(-m_state.e_lat);
        }
        else
        {
            m_phase = PHASE_LON;
        }
        break;

    /* ---- 阶段三：消除纵向偏差 ---- */
    case PHASE_LON:
        if (m_state.e_lon > m_preset.tol_lon)
        {
            /* 距目标过远，向前推进；传入误差值供底盘 PID 使用 */
            meiling_chassis_move_lon(m_state.e_lon);
        }
        else if (m_state.e_lon < -m_preset.tol_lon)
        {
            /* 距目标过近，后退 */
            meiling_chassis_move_lon(m_state.e_lon);
        }
        else
        {
            m_phase = PHASE_DONE;
        }
        break;

    /* ---- 判定阶段：各有效轴同时满足容差，连续计数 ---- */
    case PHASE_DONE:
        if (allInTolerance())
        {
            m_state.stable_cnt++;
            if (m_state.stable_cnt >= MEILING_STABLE_COUNT)
            {
                /* 连续稳定达标，精定位成功 */
                m_state.result = SUCCESS;
                m_running      = 0U;
                return SUCCESS;
            }
        }
        else
        {
            /* 底盘抖动导致某轴重新超限，清零计数，按优先级退回修正阶段 */
            m_state.stable_cnt = 0U;

            float abs_yaw = m_state.e_yaw_obs < 0.0f ? -m_state.e_yaw_obs : m_state.e_yaw_obs;
            float abs_lat = m_state.e_lat      < 0.0f ? -m_state.e_lat     : m_state.e_lat;

            /* skip_yaw=0 时 yaw 优先级最高；skip_yaw=1 时直接从横向判断 */
            if ((m_preset.skip_yaw == 0U) && (abs_yaw > m_preset.tol_yaw))
            {
                m_phase = PHASE_YAW;
            }
            else if (abs_lat > m_preset.tol_lat)
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
        /* 不应进入此分支，若触发说明 m_phase 被非法修改 */
        m_phase = (m_preset.skip_yaw != 0U) ? PHASE_LAT : PHASE_YAW;
        break;
    }

    return RUNNING;
}

/* ====================== 私有函数实现 ===================================== */

/**
 * @brief  按 sensor_mask 读取可用 DT35 并计算各轴偏差
 *
 * 横向偏差符号约定（正值 = 车偏右 = 需向左移）：
 *   左+右：e_lat = (e_L - e_R) / 2
 *   仅左  ：e_lat = L_meas - L_ref
 *   仅右  ：e_lat = R_ref  - R_meas
 *
 * yaw 偏差仅在左右均可用时有意义；skip_yaw=1 时 allInTolerance 不检查它。
 */
void MeilingLocator::calcErrors(void)
{
    uint8_t has_front = (m_preset.sensor_mask & SENSOR_FRONT) != 0U;
    uint8_t has_left  = (m_preset.sensor_mask & SENSOR_LEFT)  != 0U;
    uint8_t has_right = (m_preset.sensor_mask & SENSOR_RIGHT) != 0U;

    if (has_front) { m_state.F_meas = meiling_dt35_get_front_mm(); }
    if (has_left)  { m_state.L_meas = meiling_dt35_get_left_mm();  }
    if (has_right) { m_state.R_meas = meiling_dt35_get_right_mm(); }

    /* 纵向偏差：前激光，正值表示距目标过远，需向前推进 */
    m_state.e_lon = has_front ? (m_state.F_meas - m_preset.F_ref) : 0.0f;

    /* 横向偏差 & yaw 偏差 */
    if (has_left && has_right)
    {
        float e_L = m_state.L_meas - m_preset.L_ref;
        float e_R = m_state.R_meas - m_preset.R_ref;
        /* 双侧：用差值均值消除目标点本身的不对称影响 */
        m_state.e_lat     = (e_L - e_R) / 2.0f;
        /* yaw：左右距差值偏差，仅上台阶前使用 */
        m_state.e_yaw_obs = (m_state.L_meas - m_state.R_meas)
                          - (m_preset.L_ref  - m_preset.R_ref);
    }
    else if (has_left)
    {
        /* 仅左侧：左距大于参考 → 车偏右 → e_lat 为正 */
        m_state.e_lat     = m_state.L_meas - m_preset.L_ref;
        m_state.e_yaw_obs = 0.0f;
    }
    else if (has_right)
    {
        /* 仅右侧：右距小于参考 → 车偏右 → e_lat 为正 */
        m_state.e_lat     = m_preset.R_ref - m_state.R_meas;
        m_state.e_yaw_obs = 0.0f;
    }
    else
    {
        m_state.e_lat     = 0.0f;
        m_state.e_yaw_obs = 0.0f;
    }
}

/**
 * @brief  各有效轴偏差是否同时进入容差范围
 * @note   skip_yaw=1 时不检查 yaw 容差
 */
uint8_t MeilingLocator::allInTolerance(void) const
{
    /* yaw：仅上台阶前（skip_yaw=0）检查 */
    if (m_preset.skip_yaw == 0U)
    {
        float abs_yaw = m_state.e_yaw_obs < 0.0f ? -m_state.e_yaw_obs : m_state.e_yaw_obs;
        if (abs_yaw > m_preset.tol_yaw) { return 0U; }
    }

    float abs_lat = m_state.e_lat < 0.0f ? -m_state.e_lat : m_state.e_lat;
    float abs_lon = m_state.e_lon < 0.0f ? -m_state.e_lon : m_state.e_lon;

    if (abs_lat > m_preset.tol_lat) { return 0U; }
    if (abs_lon > m_preset.tol_lon) { return 0U; }
    return 1U;
}
//实际   7.5  35.5 49.5 100.0 
//测得   3.5 33.1  47.5 100.0 
/**
 * @brief  是否已超时
 */
uint8_t MeilingLocator::isTimeout(void) const
{
    return (HAL_GetTick() - m_start_tick >= m_preset.timeout_ms) ? 1U : 0U;
}

/* ====================== 底盘 / DT35 适配接口实现 ============================ */

void meiling_chassis_rotate_yaw(float err_mm)
{
    setChassisCmd(0.0f, 0.0f, yawCmdFromError(err_mm));
}

void meiling_chassis_move_lat(float err_mm)
{
    setChassisCmd(0.0f, linearCmdFromError(err_mm), 0.0f);
}

void meiling_chassis_move_lon(float err_mm)
{
    setChassisCmd(linearCmdFromError(err_mm), 0.0f, 0.0f);
}

uint8_t meiling_chassis_get_command(float *vx, float *vy, float *vz)
{
    if ((vx == 0) || (vy == 0) || (vz == 0))
    {
        return 0U;
    }

    if ((int32_t)(HAL_GetTick() - s_chassis_cmd.expire_tick) >= 0)
    {
        *vx = 0.0f;
        *vy = 0.0f;
        *vz = 0.0f;
        return 0U;
    }

    *vx = s_chassis_cmd.vx;
    *vy = s_chassis_cmd.vy;
    *vz = s_chassis_cmd.vz;
    return 1U;
}

float meiling_dt35_get_left_mm(void)
{
    return getDt35DistanceMm(MEILING_DT35_LEFT_CH);
}

float meiling_dt35_get_right_mm(void)
{
    return getDt35DistanceMm(MEILING_DT35_RIGHT_CH);
}

float meiling_dt35_get_front_mm(void)
{
    return getDt35DistanceMm(MEILING_DT35_FRONT_CH);
}
