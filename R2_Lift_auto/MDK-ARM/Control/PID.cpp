#include "PID.h"
#include "main.h"
#include "string.h"
#include "bsp_dwt.h"
/******************************* PID CONTROL *********************************/

PID::PID(void)
{
    memset(&pid, 0.0f, sizeof(pid));
}

void PID::Init(float max_out, float intergral_limit, float deadband,
               float Kp, float Ki, float Kd, float output_fiter_factor, uint8_t improve)
{
    if (pid.output_fiter_factor < 0.0f)
        pid.output_fiter_factor = 0.0f;
    if (pid.output_fiter_factor > 1.0f)
        pid.output_fiter_factor = 1.0f;
    pid.MaxOut              = max_out;
    pid.IntegralLimit       = intergral_limit;
    pid.Deadband            = deadband;
    pid.Kp                  = Kp;
    pid.Ki                  = Ki;
    pid.Kd                  = Kd;
    pid.output_fiter_factor = output_fiter_factor;
    pid.Improve             = improve;
    pid.DWT_CNT             = DWT->CYCCNT;
}

float PID::PID_Calculate(float measure, float ref)
{
    pid.Measure = measure;
    pid.Ref     = ref;
    pid.dt      = DWT_.getDeltaT(&pid.DWT_CNT);
    pid.Err     = pid.Ref - pid.Measure;
    if (fabs(pid.Err) < pid.Deadband) {
        pid.Err = 0.0f;
    }

    pid.Pout  = pid.Kp * pid.Err;
    pid.ITerm = pid.Ki * pid.Err * pid.dt;
    if (pid.dt > 1e-6f) {

        // 避免除以0
        pid.Dout = pid.Kd * (pid.Err - pid.Last_Err) / pid.dt;
    } else {
        /* code */
        pid.Dout = 0.0f;
    }
    pid.Iout += pid.ITerm;

    f_Integral_Limit();

    pid.Output = pid.Pout + pid.Iout + pid.Dout;

    if (pid.Improve & PID_IMPROVE_OUTPUT_FILTER) {
        f_Output_Filter();
    }

    f_Output_Limit();

    pid.Last_Err = pid.Err;

    pid.Last_Output = pid.Output;

    return pid.Output;
}

void PID::f_Integral_Limit()
{
    if (pid.Iout > pid.IntegralLimit) {
        pid.Iout = pid.IntegralLimit;
    } else if (pid.Iout < -pid.IntegralLimit) {
        pid.Iout = -pid.IntegralLimit;
    }
}

void PID::f_Output_Filter()
{

    pid.Output = (1.0f - pid.output_fiter_factor) * pid.Output + pid.output_fiter_factor * pid.Last_Output;
}

void PID::f_Output_Limit()
{
    if (pid.Output > pid.MaxOut) {
        pid.Output = pid.MaxOut;
    }
    if (pid.Output < -(pid.MaxOut)) {
        pid.Output = -(pid.MaxOut);
    }
}