#include "PID.h"
#include "main.h"
#include "string.h"
#include "bsp_dwt.h"
/******************************* PID CONTROL *********************************/

/* 构造函数：将 pid 结构体全部清零，确保初始状态干净 */
PID::PID(void)
{
    memset(&pid, 0.0f, sizeof(pid));
}

/* 初始化 PID 参数
 * max_out           : 输出限幅值
 * intergral_limit   : 积分限幅值
 * deadband          : 死区范围，误差绝对值小于此值时视为零
 * Kp/Ki/Kd          : 比例/积分/微分系数
 * output_fiter_factor: 低通滤波系数（0~1），越大滤波越强，需先钳位再赋值
 * improve           : 功能改进位掩码，参考 PID_Improvement_e
 */
void PID::Init(float max_out, float intergral_limit, float deadband,
               float Kp, float Ki, float Kd, float output_fiter_factor, uint8_t improve)
{
    /* 对传入的滤波系数做钳位，防止参数越界 */
    if (output_fiter_factor < 0.0f)
        output_fiter_factor = 0.0f;
    if (output_fiter_factor > 1.0f)
        output_fiter_factor = 1.0f;

    pid.MaxOut              = max_out;
    pid.IntegralLimit       = intergral_limit;
    pid.Deadband            = deadband;
    pid.Kp                  = Kp;
    pid.Ki                  = Ki;
    pid.Kd                  = Kd;
    pid.output_fiter_factor = output_fiter_factor;
    pid.Improve             = improve;

    /* 记录当前 DWT 计数，作为第一次计算的时间基准 */
    pid.DWT_CNT = DWT->CYCCNT;
}

/* PID 计算主函数
 * measure : 当前实测值
 * ref     : 目标设定值
 * 返回值  : 本次 PID 输出
 */
float PID::PID_Calculate(float measure, float ref)
{
    pid.Measure = measure;
    pid.Ref     = ref;

    /* 获取距上次调用的时间间隔（单位：秒） */
    pid.dt = DWT_.getDeltaT(&pid.DWT_CNT);

    /* 计算误差，若在死区内则视为零，避免微小抖动引起积分漂移 */
    pid.Err = pid.Ref - pid.Measure;
    if (fabs(pid.Err) < pid.Deadband) {
        pid.Err = 0.0f;
    }

    /* 比例项 */
    pid.Pout = pid.Kp * pid.Err;

    /* 积分项（面积法：误差 × 时间步长） */
    pid.ITerm = pid.Ki * pid.Err * pid.dt;

    /* 微分项：对 dt 做保护，避免除以零或极小值导致数值爆炸 */
    if (pid.dt > 1e-6f) {
        pid.Dout = pid.Kd * (pid.Err - pid.Last_Err) / pid.dt;
    } else {
        pid.Dout = 0.0f;
    }

    /* 累加积分 */
    pid.Iout += pid.ITerm;

    /* 积分限幅，防止积分饱和 */
    f_Integral_Limit();

    /* 汇总三路输出 */
    pid.Output = pid.Pout + pid.Iout + pid.Dout;

    /* 可选：低通滤波平滑输出，减少高频抖动 */
    if (pid.Improve & PID_IMPROVE_OUTPUT_FILTER) {
        f_Output_Filter();
    }

    /* 输出限幅，保证不超过执行器允许范围 */
    f_Output_Limit();

    /* 保存本次误差和输出，供下次微分和滤波使用 */
    pid.Last_Err    = pid.Err;
    pid.Last_Output = pid.Output;

    return pid.Output;
}

/* 积分限幅：将 Iout 钳位在 [-IntegralLimit, +IntegralLimit] 范围内 */
void PID::f_Integral_Limit()
{
    if (pid.Iout > pid.IntegralLimit) {
        pid.Iout = pid.IntegralLimit;
    } else if (pid.Iout < -pid.IntegralLimit) {
        pid.Iout = -pid.IntegralLimit;
    }
}

/* 输出低通滤波：新输出 = (1-α)*当前值 + α*上次值
 * α = output_fiter_factor，越大惯性越强，响应越慢但越平滑
 */
void PID::f_Output_Filter()
{
    pid.Output = (1.0f - pid.output_fiter_factor) * pid.Output + pid.output_fiter_factor * pid.Last_Output;
}

/* 输出限幅：将最终输出钳位在 [-MaxOut, +MaxOut] 范围内 */
void PID::f_Output_Limit()
{
    if (pid.Output > pid.MaxOut) {
        pid.Output = pid.MaxOut;
    }
    if (pid.Output < -(pid.MaxOut)) {
        pid.Output = -(pid.MaxOut);
    }
}
