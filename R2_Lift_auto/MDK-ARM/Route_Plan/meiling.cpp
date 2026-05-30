/*
 * meiling.cpp — 梅林定位器实现
 *
 * 通过三路DT35测距传感器（前/左/右）计算机器人相对目标位置的横向和纵向误差，
 * 经一阶低通滤波后，生成带加减速限制的二维速度指令，驱动全向底盘闭环收敛到目标位姿。
 */

#include "mieling.h"
#include "DT35.h"
#include "main.h"
#include "omni_chassis.h"
#include <math.h>

MeilingLocator meiling;

float MeilingLocator::MEILING_V_MAX = 2.0f;          // 最大二维合速度，单位 m/s
float MeilingLocator::MEILING_ACC_MAX = 2.0f;        // 最大加速度，限制速度指令突变，单位 m/s^2
float MeilingLocator::MEILING_DEC_MAX = 1.2f;        // 最大减速度，按剩余距离计算刹车速度，单位 m/s^2
float MeilingLocator::MEILING_FILTER_ALPHA = 0.25f;  // DT35一阶低通系数，越大响应越快、滤波越弱
float MeilingLocator::MEILING_MIN_DT = 0.001f;       // 最小规划周期，防止同一节拍内重复调用
float MeilingLocator::MEILING_MAX_DT = 0.05f;        // 最大规划周期，防止任务卡顿后步长过大
float MeilingLocator::MEILING_DIST_EPS = 0.001f;     // 距离向量归一化阈值，避免除零
float MeilingLocator::MEILING_DONE_SPEED = 0.1f;     // 到位判定允许的底盘残余速度，单位 m/s

/*
 * 绝对值限幅：将 x 限制在 [-max, -min] ∪ [min, max] 区间。
 * |x| < min 时推到 ±min（死区补偿），|x| > max 时钳位到 ±max。
 * min/max 自动取绝对值并排序，传入负值也能正确处理。
 */
float abs_limit(float x, float min, float max)
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

/* 普通限幅：将 x 钳位到 [min, max] 区间。 */
float MeilingLocator::clamp(float x, float min, float max)
{
    if (x > max)
    {
        return max;
    }
    if (x < min)
    {
        return min;
    }
    return x;
}

/* 速率限制：限制 current 到 target 的单步变化量不超过 ±max_delta。 */
float MeilingLocator::rateLimit(float target, float current, float max_delta)
{
    return current + clamp(target - current, -max_delta, max_delta);
}

/* 一阶低通滤波：output = last + α * (input - last)，α 越大响应越快。 */
float MeilingLocator::lowPass(float last, float input)
{
    return last + MEILING_FILTER_ALPHA * (input - last);
}

/* 二维速度限幅：当合速度超过 max_speed 时等比缩放 vx/vy，保持方向不变。 */
void MeilingLocator::limitVectorSpeed(float *vx, float *vy, float max_speed)
{
    float speed = sqrtf((*vx) * (*vx) + (*vy) * (*vy));

    if (speed > max_speed && speed > MEILING_DIST_EPS)
    {
        float scale = max_speed / speed;
        *vx *= scale;
        *vy *= scale;
    }
}

void MeilingLocator::resetPlanState(uint32_t now_tick)
{
    /*
     * 每次重新启动定位时都清空规划状态：
     * 速度目标从0开始重新加速，滤波器等待第一帧DT35原始值初始化，避免沿用上一次定位的残留数据。
     */
    m_plan = {};
    m_plan.last_tick = now_tick;
}

float MeilingLocator::calcDeltaTime(uint32_t now_tick)
{
    /*
     * HAL_GetTick() 单位是毫秒，规划层需要秒。
     * dt 夹在固定范围内，可以避免同一计时节拍重复调用时速度不动，也避免任务阻塞后一次性给过大的加速度步长。
     */
    uint32_t dt_ms = now_tick - m_plan.last_tick;
    float dt = static_cast<float>(dt_ms) * 0.001f;

    m_plan.last_tick = now_tick;

    if (dt < MEILING_MIN_DT)
    {
        return MEILING_MIN_DT;
    }
    if (dt > MEILING_MAX_DT)
    {
        return MEILING_MAX_DT;
    }
    return dt;
}

void MeilingLocator::updatePlanVelocity(float dt)
{
    /*
     * 将横向/纵向误差合成二维距离向量，再按剩余距离计算刹车速度。
     * 这样远处可以跑到最大合速度，接近目标时会自然减速，不再靠最小速度限幅硬顶到容差范围。
     */

    // 第一部分：误差转距离向量
    float ex = 0.0f;
    float ey = 0.0f;

    if (fabsf(m_state.e_lon) > m_target.tol_lon)
    {
        ex = m_state.e_lon * 0.001f;
    }
    if (fabsf(m_state.e_lat) > m_target.tol_lat)
    {
        ey = m_state.e_lat * 0.001f;
    }

    float dist = sqrtf(ex * ex + ey * ey);
    float vx_raw = 0.0f;
    float vy_raw = 0.0f;

    //  第二部分：计算刹车速度
    if (dist > MEILING_DIST_EPS)
    {
        float v_brake = sqrtf(2.0f * MEILING_DEC_MAX * dist); // 这是物理公式 v² = 2·a·s 的反推：以最大减速度  MEILING_DEC_MAX 刹车，需要多初速度才能在 dist 距离内停下。离目标远时 v_brake 大，靠近时 v_brake 自然变小。

        /*
          取刹车速度和最大速度中较小的那个：
  -         远处：v_brake > V_MAX，被钳到 0.8 m/s，全速跑
  -         近处：v_brake < V_MAX，用刹车速度，自然减速
        */
        float v_target = (v_brake < MEILING_V_MAX) ? v_brake : MEILING_V_MAX;

        vx_raw = v_target * ex / dist;
        vy_raw = v_target * ey / dist;
    }

    /*
     * 对分解后的 Vx/Vy 分别做加速度限制，再限制二维合速度。
     * 分轴限加速度能抑制指令突变，合速度限制保证斜向运动时不会超过底盘允许速度。
     */
    m_plan.vx_ref = rateLimit(vx_raw, m_plan.vx_ref, MEILING_ACC_MAX * dt);
    m_plan.vy_ref = rateLimit(vy_raw, m_plan.vy_ref, MEILING_ACC_MAX * dt);
    limitVectorSpeed(&m_plan.vx_ref, &m_plan.vy_ref, MEILING_V_MAX);
}

/* 启动梅林定位：保存目标参数，清空状态，记录起始时刻。 */
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
    resetPlanState(m_start_tick);
}

/*
 * 定位主循环，每帧调用一次。
 * 流程：超时检查 → 计算误差 → 更新速度规划 → 判断是否稳定到位。
 * 返回 RUNNING / SUCCESS / TIMEOUT。
 */
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

    updatePlanVelocity(calcDeltaTime(HAL_GetTick()));

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

/*
 * 计算横向误差 e_lat 和纵向误差 e_lon。
 *
 * 纵向误差 = 前传感器测量值 - 前参考值（正 = 偏远，负 = 偏近）。
 * 横向误差：左右都有传感器时取两侧偏差均值；只有一侧时用单侧偏差。
 * 首次调用时用原始DT35值初始化低通滤波器，避免从零滤波导致启动冲击。
 */
void MeilingLocator::calcErrors(void)
{
    uint8_t has_front = (m_target.sensor_mask & SENSOR_FRONT) != 0U;
    uint8_t has_left = (m_target.sensor_mask & SENSOR_LEFT) != 0U;
    uint8_t has_right = (m_target.sensor_mask & SENSOR_RIGHT) != 0U;

    if (m_plan.filter_ready == 0U)
    {
        /*
         * 第一次进入定位时直接用当前DT35原始值初始化滤波器。
         * 如果从0开始滤波，第一次误差会被人为放大，速度规划会出现不必要的启动冲击。
         */
        m_plan.F_filtered = dt35.ch2.distance_mm;
        m_plan.L_filtered = dt35.ch0.distance_mm;
        m_plan.R_filtered = dt35.ch1.distance_mm;
        m_plan.filter_ready = 1U;
    }
    else
    {
        m_plan.F_filtered = lowPass(m_plan.F_filtered, dt35.ch2.distance_mm);
        m_plan.L_filtered = lowPass(m_plan.L_filtered, dt35.ch0.distance_mm);
        m_plan.R_filtered = lowPass(m_plan.R_filtered, dt35.ch1.distance_mm);
    }

    if (has_front)
    {
        m_state.F_meas = m_plan.F_filtered;
    }
    if (has_left)
    {
        m_state.L_meas = m_plan.L_filtered;
    }
    if (has_right)
    {
        m_state.R_meas = m_plan.R_filtered;
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

/*
 * 判断是否到达目标位姿：横向和纵向误差都在容差内，且底盘实际速度已收敛。
 * 读取底盘正运动学反馈的速度，避免车还在滑行时提前判定成功。
 */
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
    /*
     * 这里有意读取底盘正运动学得到的实时速度反馈。
     * 位置误差进入容差范围后，还要等实车横向和纵向速度都收住，避免车还在滑行时提前判定定位成功。
     */
    if (fabsf(omni_chassis.now.Vx) > MEILING_DONE_SPEED)
    {
        return 0U;
    }
    if (fabsf(omni_chassis.now.Vy) > MEILING_DONE_SPEED)
    {
        return 0U;
    }
    return 1U;
}

/* 判断定位是否超时：从 start() 开始计时，超过 timeout_ms 则返回 1。 */
uint8_t MeilingLocator::isTimeout(void) const
{
    return (HAL_GetTick() - m_start_tick >= m_target.timeout_ms) ? 1U : 0U;
}

/* 获取底盘X轴速度指令：定位运行中返回规划速度，否则返回手动目标值。 */
float MeilingLocator::getChassisVxTarget(float manual_target) const
{
    if (m_running == 0U)
    {
        return manual_target;
    }

    return m_plan.vx_ref;
}

/* 获取底盘Y轴速度指令：定位运行中返回规划速度，否则返回手动目标值。 */
float MeilingLocator::getChassisVyTarget(float manual_target) const
{
    if (m_running == 0U)
    {
        return manual_target;
    }

    return m_plan.vy_ref;
}

/* 获取底盘Z轴（旋转）速度指令：梅林定位不控制旋转，直接透传手动目标值。 */
float MeilingLocator::getChassisVzTarget(float manual_target) const
{
    return manual_target;
}
