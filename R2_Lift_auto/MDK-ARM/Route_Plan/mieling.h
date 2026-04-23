/* -----------------------------------------------------------------------
 * mieling.h
 * 简单说：这里定义“贴边/对位”要用的数据和接口。
 * 调用 start() 开始一次对位，循环调用 update() 推进流程，
 * 底盘任务再通过 getChassisVxTarget/getChassisVyTarget 取自动速度。
 * ----------------------------------------------------------------------- */

#ifndef MIELING_H
#define MIELING_H

#include <stdint.h>
#include "main.h"

/* ====================== 可调参数 ==================================== */

/* 误差连续达标这么多次，才算真的对准，避免刚好抖进去一次就误判成功。 */
#define MEILING_STABLE_COUNT 5U

/* 没单独设置超时时间时，默认最多跑 5000ms。 */
#define MEILING_TIMEOUT_MS 5000U

/* 横向误差 e_lat 的放大系数：误差越大，给底盘的 Vx 越大。 */
#define ELAT_KP 15.0f
#define ELAT_MAX 0.7f

/* 前后误差 e_lon 的放大系数：误差越大，给底盘的 Vy 越大。 */
#define ELON_KP 15.0f
#define ELON_MAX 0.7f

/* ====================== 传感器开关 ================================== */
#define SENSOR_FRONT 0x01U /* bit0：使用前方 DT35。 */
#define SENSOR_LEFT 0x02U  /* bit1：使用左侧 DT35。 */
#define SENSOR_RIGHT 0x04U /* bit2：使用右侧 DT35。 */
#define SENSOR_ALL 0x07U   /* 三个 DT35 全部使用。 */

/* 对位流程分阶段跑：先横向，再前后，最后确认稳定。 */
enum Phase
{
    PHASE_LAT = 0, /* 正在修横向位置，对应 Vx。 */
    PHASE_LON,     /* 正在修前后距离，对应 Vy。 */
    PHASE_DONE,    /* 位置已经进容差，等待连续稳定。 */
};

/* ====================== 数据结构 ===================================== */

struct MeilingTarget_t
{
    uint8_t preset_id;   /* 目标编号，只是方便区分当前用哪套目标距离。 */
    float L_ref;         /* 左侧目标距离，单位 mm；只有开 SENSOR_LEFT 才会用。 */
    float R_ref;         /* 右侧目标距离，单位 mm；只有开 SENSOR_RIGHT 才会用。 */
    float F_ref;         /* 前方目标距离，单位 mm；只有开 SENSOR_FRONT 才会用。 */
    float tol_lat;       /* 横向允许误差，单位 mm；小于这个值就认为横向够准。 */
    float tol_lon;       /* 前后允许误差，单位 mm；小于这个值就认为前后够准。 */
    uint32_t timeout_ms; /* 本次对位最长跑多久；填 0 就使用 MEILING_TIMEOUT_MS。 */
    uint8_t sensor_mask; /* 选择本次要用哪些 DT35，用 SENSOR_FRONT/LEFT/RIGHT 组合。 */
};

struct MeilingState_t
{
    float L_meas;       /* 左侧 DT35 当前读数，单位 mm。 */
    float R_meas;       /* 右侧 DT35 当前读数，单位 mm。 */
    float F_meas;       /* 前方 DT35 当前读数，单位 mm。 */
    float e_lat;        /* 横向误差，后面会换算成 Vx；0 表示横向刚好。 */
    float e_lon;        /* 前后误差，等于 F_meas - F_ref，后面会换算成 Vy。 */
    uint8_t stable_cnt; /* 连续达标次数，达标越久越可信。 */
    uint8_t result;     /* 结果码：0=运行中，1=成功，2=超时。 */
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
    Phase m_phase = PHASE_LAT;
    uint8_t m_running = 0U;
    uint32_t m_start_tick = 0U;

    void calcErrors(void);
    uint8_t allInTolerance(void) const;
    uint8_t isTimeout(void) const;
};

extern MeilingLocator meiling;

#endif /* 防止 mieling.h 被重复包含 */
