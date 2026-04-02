/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "main.h"
#include "stdint.h"



// 时间结构体，用于存储秒、毫秒和微秒
struct DWT_Time_t {
    /* data */
    uint32_t s  = 0;
    uint16_t ms = 0;
    uint16_t us = 0;
};

// DWT_Timer 类：用于操作DWT计数器、时间获取、延时和时间计算
class DWT_Timer
{
public:
    // 系统时间结构体，存储 DWT 的秒、毫秒和微秒
    DWT_Time_t sysTime;
    //  存储 CPU 的频率和时间转换系数
    uint32_t cpu_freq_hz    = 0; // CPU 主频（Hz）
    uint32_t cpu_freq_hz_ms = 0; // CPU 主频的毫秒级转换系数
    uint32_t cpu_freq_hz_us = 0; // CPU 主频的微秒级转换系数
    // DWT计数器轮次和计数值
    uint32_t cyccnt_round_count = 0; // DWT计数器溢出轮次
    uint32_t cyccnt_last        = 0; // 上次读取的计数器值
    uint64_t cyccnt64           = 0; // 64位计数器，用于计算长时间间隔

public:
    // 初始化DWT计数器
    void init(uint32_t cpu_freq_mhz);

    // 获取时间差，单位秒
    float getDeltaT(uint32_t *cnt_last);

    // 获取时间差，单位秒，使用64位计数器
    double getDeltaT64(uint32_t *cnt_last);

    // 更新时间结构体 sysTime
    void sysTimeUpdate(void);

    // 获取时间线，单位秒
    float getTimeline_s(void);

    // 获取时间线，单位毫秒
    float getTimeline_ms(void);

    // 获取时间线，单位微秒
    uint64_t getTimeline_us(void);

    // 延时函数，单位秒
    void delay_s(float delay_s);

private:
    // 更新DWT计数器轮次
    void cntUpdate(void);
};

extern DWT_Timer DWT_;

#endif /* BSP_DWT_H_ */
