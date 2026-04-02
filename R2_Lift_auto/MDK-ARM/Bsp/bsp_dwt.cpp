#include "bsp_dwt.h"

DWT_Timer DWT_;

// 初始化DWT计数器
void DWT_Timer::init(uint32_t cpu_freq_mhz)
{
    // 使能 DWT 外设（DWT = Data Watchpoint and Trace）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 将 DWT 的 CYCCNT 寄存器清零，初始化为 0
    DWT->CYCCNT = 0u;

    // 启用 CYCCNT 寄存器计数
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 将 CPU 主频从 MHz 转换为 Hz
    cpu_freq_hz    = cpu_freq_mhz * 1000000u;
    cpu_freq_hz_ms = cpu_freq_hz / 1000u;
    cpu_freq_hz_us = cpu_freq_hz / 1000000u;

    // 初始化轮次计数和最后一次读取的计数值
    cyccnt_round_count = 0;
    cyccnt_last        = DWT->CYCCNT;
    cyccnt64           = 0;

    // 初始化 sysTime 结构体
    sysTime.s  = 0;
    sysTime.ms = 0;
    sysTime.us = 0;
}

// 计算时间差，返回值单位为
float DWT_Timer::getDeltaT(uint32_t *cnt_last)
{
    // 获取当前 DWT 计数器的值
    uint32_t cnt_now = DWT->CYCCNT;

    // 计算时间差，单位为秒
    float dt = (float)(cnt_now - *cnt_last) / (float)cpu_freq_hz;

    // 更新上次计数值
    *cnt_last = cnt_now;

    // 更新计数器轮次
    cntUpdate();

    return dt;
}

// 计算时间差，返回值单位为秒，使用64位计数器
double DWT_Timer::getDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;

    //计算时间差
    double dt = (double)(cnt_now - *cnt_last) / (double)cpu_freq_hz;

    //更新上次的计数值
    *cnt_last = cnt_now;

    //更新计数器轮次
    cntUpdate();

    return dt;
}

// 更新系统时间（秒、毫秒、微秒）
void DWT_Timer::sysTimeUpdate(void)
{
    // 获取当前 DWT 计数器的值
    uint32_t cnt_now = DWT->CYCCNT;

    // 更新计数器轮次
    cntUpdate();

    // 计算总的 64 位计数器值
    cyccnt64 = (uint64_t)cyccnt_round_count * ((uint64_t)UINT32_MAX + 1ULL) + (uint64_t)cnt_now;

    // 计算秒、毫秒、微秒部分
    uint64_t sec_part  = cyccnt64 / cpu_freq_hz;
    uint64_t rem_part  = cyccnt64 - sec_part * cpu_freq_hz;
    uint64_t ms_part   = rem_part / cpu_freq_hz_ms;
    uint64_t rem_part2 = rem_part - ms_part * cpu_freq_hz_ms;
    uint64_t us_part   = rem_part2 / cpu_freq_hz_us;

    // 更新 sysTime 结构体
    sysTime.s  = (uint32_t)sec_part;
    sysTime.ms = (uint16_t)ms_part;
    sysTime.us = (uint16_t)us_part;
}

// 获取系统时间线，单位为秒
float DWT_Timer::getTimeline_s(void)
{
    // 更新系统时间
    sysTimeUpdate();

    // 将秒、毫秒、微秒转换为总秒数
    return (float)sysTime.s + (float)sysTime.ms * 0.001f + (float)sysTime.us * 0.000001f;
}

// 获取系统时间线，单位为毫秒
float DWT_Timer::getTimeline_ms(void)
{
    // 更新系统时间
    sysTimeUpdate();

    // 将秒、毫秒、微秒转换为总毫秒数
    return (float)sysTime.s * 1000.0f + (float)sysTime.ms + (float)sysTime.us * 0.001f;
}

// 获取系统时间线，单位为微秒
uint64_t DWT_Timer::getTimeline_us(void)
{
    // 更新系统时间
    sysTimeUpdate();

    // 将秒、毫秒、微秒转换为总微秒数
    return (uint64_t)sysTime.s * 1000000ULL +
           (uint64_t)sysTime.ms * 1000ULL +
           (uint64_t)sysTime.us;
}

// 延时函数，单位为秒
void DWT_Timer::delay_s(float delay_s)
{
    // 获取当前计数器值
    uint32_t tickstart = DWT->CYCCNT;

    // 计算延时的 tick 数
    uint32_t wait_ticks = (uint32_t)(delay_s * (float)cpu_freq_hz);

    // 进入延时循环，直到计数器值满足延时时间
    while ((uint32_t)(DWT->CYCCNT - tickstart) < wait_ticks) {
    }
}

void DWT_Timer::cntUpdate()
{
    // 获取当前 DWT 计数器值
    uint32_t cnt_now = DWT->CYCCNT;

    // 如果当前计数值小于上次计数值，说明计数器发生了溢出
    if (cnt_now < cyccnt_last) {
        // 更新轮次数
        cyccnt_round_count++;
    }

    // 更新上次计数值
    cyccnt_last = cnt_now;
}
