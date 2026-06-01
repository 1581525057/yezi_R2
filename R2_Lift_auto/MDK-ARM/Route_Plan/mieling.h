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

    /*
     * 速度规划参数：运行时可修改，方便串口实时调参。
     * 最大速度决定效率，加减速度决定起停柔和程度，滤波系数决定抗抖和响应速度。
     */
    static float MEILING_V_MAX;          // 最大二维合速度，单位 m/s。
    static float MEILING_ACC_MAX;        // 最大加速度，限制速度指令突变，单位 m/s^2。
    static float MEILING_DEC_MAX;        // 最大减速度，用于按剩余距离计算刹车速度，单位 m/s^2。
    static float MEILING_FILTER_ALPHA;   // 测距一阶低通系数，越大响应越快、滤波越弱。
    static float MEILING_MIN_DT;         // 最小规划周期，防止同一计时节拍内调用导致加速度步长为0。
    static float MEILING_MAX_DT;         // 最大规划周期，防止任务卡顿后速度步长突然过大。
    static float MEILING_DIST_EPS;       // 距离向量归一化阈值，避免除零。
    static float MEILING_DONE_SPEED;     // 到位判定允许的底盘残余速度，单位 m/s。

    MeilingTarget_t m_target = {};
    MeilingState_t m_state = {};

    void start(const MeilingTarget_t &target);
    uint8_t update(void);

    float getChassisVxTarget(float manual_target) const;
    float getChassisVyTarget(float manual_target) const;
    float getChassisVzTarget(float manual_target) const;

private:
    /*
     * 梅林定位的速度规划状态全部放在定位器对象内部：
     * 1. 避免使用文件级全局变量，后续即使创建多个定位器实例也不会互相污染。
     * 2. vx_ref/vy_ref 是新底盘坐标系下的二维规划速度，速度读取函数只负责读取。
     * 3. F/L/R_filtered 保存测距一阶低通后的距离，避免测距抖动直接进入速度规划。
     * 4. 时间记录成员用于计算规划周期，滤波初始化标志用于首次启动时用原始距离初始化滤波器。
     */
    struct MeilingPlanState_t
    {
        float vx_ref;
        float vy_ref;
        float F_filtered;
        float L_filtered;
        float R_filtered;
        uint32_t last_tick;
        uint8_t filter_ready;
    };

    uint8_t m_running = 0U;
    uint32_t m_start_tick = 0U;
    MeilingPlanState_t m_plan = {};

    /*
     * calcErrors() 负责更新测距和位置误差；updatePlanVelocity() 负责根据误差生成平滑速度。
     * 两者分开后，定位状态判断和速度规划不会混在 update() 主流程里。
     */
    void calcErrors(void);
    void updatePlanVelocity(float dt);
    uint8_t allInTolerance(void) const;
    uint8_t isTimeout(void) const;

    /* 这两个函数会读写 m_plan，因此必须是普通成员函数，不能再做文件级静态函数。 */
    void resetPlanState(uint32_t now_tick);
    float calcDeltaTime(uint32_t now_tick);

    /* 以下工具函数只处理入参，不依赖某次定位过程的状态，因此作为私有静态成员。 */
    static float clamp(float x, float min, float max);
    static float rateLimit(float target, float current, float max_delta);
    static float lowPass(float last, float input);
    static void limitVectorSpeed(float *vx, float *vy, float max_speed);
}; 
extern float abs_limit(float x, float min, float max);
extern MeilingLocator meiling;

#endif
