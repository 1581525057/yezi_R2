/* -----------------------------------------------------------------------
 * 文件：mieling.h
 * 功能：梅林区精定位模块
 *       对应设计文档 §7：DT35 三向激光闭环精定位（单片机执行侧）
 *       顺序：先消 yaw 角度偏差 → 再消横向偏差 → 最后消纵向偏差
 * ----------------------------------------------------------------------- */

#ifndef MIELING_H
#define MIELING_H

#include <stdint.h>
#include "main.h"
/* ====================== 精定位调节参数 ====================================
 * 以下阈值在联调前应结合实测结果标定，此处为保守初始值
 * ======================================================================== */

/* 连续满足三轴容差的采样次数，达到后才判定精定位完成，防止抖动误报 */
#define MEILING_STABLE_COUNT 5U

/* 精定位最大允许执行时间（ms）；超时后直接返回失败，不允许无限闭环 */
#define MEILING_TIMEOUT_MS 5000U

/* 单次底盘微调步长 */
#define MEILING_LAT_STEP_MM 5.0f  /* 横向平移步长，mm */
#define MEILING_LON_STEP_MM 5.0f  /* 纵向平移步长，mm */
#define MEILING_YAW_STEP_DEG 1.0f /* yaw 旋转步长，° */

/* ====================== 传感器掩码 =======================================
 * sensor_mask 各 bit 含义，由小电脑随精定位帧下发。
 * 上台阶前：固定 SENSOR_ALL（三路全开 + yaw 校正）。
 * 上台阶后：按实际方块可用传感器填写，skip_yaw = 1，只做 X/Y 校正。
 *
 * 各方块传感器可用性（S 型编号，最左=0）：
 *   红方：1=前左右  3=前左  4=前左  6=前右
 *   蓝方：1=前左右  4=前右  5=前右  8=前左
 * ======================================================================== */
#define SENSOR_FRONT 0x01U /* bit0：前方激光可用 */
#define SENSOR_LEFT 0x02U  /* bit1：左侧激光可用 */
#define SENSOR_RIGHT 0x04U /* bit2：右侧激光可用 */
#define SENSOR_ALL 0x07U   /* 三路全开（上台阶前默认） */

/* ====================== 数据结构 ========================================= */

/*
 * 精定位参数包：小电脑发「精定位」指令时随帧携带
 * 对应设计文档 §9 payload 的精定位部分
 */
struct MeilingPreset_t
{
    uint8_t preset_id;   /* 预设点编号，用于日志与去重 */
    float L_ref;         /* 左侧 DT35 标准距离，mm；sensor_mask 无 SENSOR_LEFT 时忽略 */
    float R_ref;         /* 右侧 DT35 标准距离，mm；sensor_mask 无 SENSOR_RIGHT 时忽略 */
    float F_ref;         /* 前方 DT35 标准距离，mm；sensor_mask 无 SENSOR_FRONT 时忽略 */
    float tol_lat;       /* 横向偏差容差，mm */
    float tol_lon;       /* 纵向偏差容差，mm */
    float tol_yaw;       /* 角度偏差容差，mm（左右差值形式）；skip_yaw=1 时忽略 */
    uint32_t timeout_ms; /* 本次精定位最大时限，0 则使用默认值 */
    uint8_t sensor_mask; /* 本次可用传感器掩码，见 SENSOR_FRONT/LEFT/RIGHT */
    uint8_t skip_yaw;    /* 1=只校 X/Y（上台阶后），0=三轴全校（上台阶前） */
};

/*
 * 精定位实时状态，供外部查询与 ACK 帧附带实测值
 */
struct MeilingState_t
{
    float L_meas;       /* 左侧 DT35 实测距离，mm */
    float R_meas;       /* 右侧 DT35 实测距离，mm */
    float F_meas;       /* 前方 DT35 实测距离，mm */
    float e_lat;        /* 横向偏差：(e_L - e_R) / 2，正值表示车偏右 */
    float e_lon;        /* 纵向偏差：F_meas - F_ref，正值表示距台阶过远 */
    float e_yaw_obs;    /* 角度偏差：(L_meas - R_meas) - (L_ref - R_ref) */
    uint8_t stable_cnt; /* 连续满足三轴容差的采样次数 */
    uint8_t result;     /* 0=进行中，1=成功，2=超时失败 */
};

/* ====================== 精定位类 ========================================= */

class MeilingLocator
{
public:
    /* 精定位结果枚举，与 result 字段对应 */
    static const uint8_t RUNNING = 0U;
    static const uint8_t SUCCESS = 1U;
    static const uint8_t TIMEOUT = 2U;

    /*
     * 启动一次精定位流程。
     * 调用时机：收到小电脑「精定位」指令后调用一次。
     * 会重置内部所有状态，以最新指令参数为准。
     */
    void start(const MeilingPreset_t &preset);

    /*
     * 精定位闭环推进，每个控制周期调用一次。
     * 返回值：RUNNING=0 进行中，SUCCESS=1 成功，TIMEOUT=2 超时失败。
     */
    uint8_t update(void);

    /* 获取实时状态，用于调试查看或 ACK 帧附带实测值 */
    const MeilingState_t &getState(void) const { return m_state; }

private:
    /* 内部阶段枚举，控制三步顺序执行 */
    enum Phase
    {
        PHASE_YAW = 0, /* 阶段一：消除角度偏差 */
        PHASE_LAT,     /* 阶段二：消除横向偏差 */
        PHASE_LON,     /* 阶段三：消除纵向偏差 */
        PHASE_DONE,    /* 三轴均达标，等待连续稳定计数 */
    };

    MeilingPreset_t m_preset = {};
    MeilingState_t m_state = {};
    Phase m_phase = PHASE_YAW;
    uint8_t m_running = 0U;
    uint32_t m_start_tick = 0U;

    /* 读取三路 DT35 并计算三轴偏差，结果写入 m_state */
    void calcErrors(void);

    /* 三轴偏差是否同时进入容差范围 */
    uint8_t allInTolerance(void) const;

    /* 是否已超时 */
    uint8_t isTimeout(void) const;
};

/* 全局唯一实例，供工程其他模块直接访问 */
extern MeilingLocator meiling;

void meiling_chassis_rotate_yaw(float err_mm);
void meiling_chassis_move_lat(float err_mm);
void meiling_chassis_move_lon(float err_mm);
uint8_t meiling_chassis_get_command(float *vx, float *vy, float *vz);
float meiling_dt35_get_left_mm(void);
float meiling_dt35_get_right_mm(void);
float meiling_dt35_get_front_mm(void);

#endif /* MIELING_H */
