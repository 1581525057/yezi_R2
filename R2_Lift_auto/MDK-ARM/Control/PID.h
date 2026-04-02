/* PID控制头文件 */
#ifndef PID_H
#define PID_H

#include "main.h"



/******************************* PID改进功能枚举 *********************************/
enum PID_Improvement_e {
    PID_IMPROVE_NONE          = 0x00,
    PID_IMPROVE_OUTPUT_FILTER = 0x01,
    PID_IMPROVE_INT_LIMIT     = 0x02,
};

/******************************* PID控制器结构体 *********************************/
struct PID_t {
    /* 基本参数 */
    float Ref; // 目标值
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数

    /* 状态变量 */
    float Measure;  // 当前测量值
    float Err;      // 当前误差
    float Last_Err; // 上次误差

    /* 输出分量 */
    float Pout;  // 比例项输出
    float Iout;  // 积分项输出
    float Dout;  // 微分项输出
    float ITerm; // 当前积分量

    /* 输出相关 */
    float Output;      // 总输出
    float Last_Output; // 上次总输出
    /* 限制参数 */
    float Deadband;            // 死区范围
    float MaxOut;              // 输出限幅值
    float IntegralLimit;       // 积分限幅值
    float output_fiter_factor; // 低通滤波参数

    /* 计时相关 */
    uint32_t DWT_CNT; // 计时器计数器
    float dt;         // 时间间隔（毫秒）

    /* 改进功能标志 */
    uint8_t Improve; // 改进功能位掩码
};

class PID
{
public:
    PID_t pid;

public:
    PID();

    void Init(float max_out, float intergral_limit, float deadband,
              float Kp, float Ki, float Kd, float output_fiter_factor, uint8_t improve);

    float PID_Calculate(float measure, float ref);

private:
    void f_Integral_Limit();

    void f_Output_Filter();

    void f_Output_Limit();
};

#endif
