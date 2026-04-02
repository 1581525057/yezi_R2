#ifndef _BSP_USART_H
#define _BSP_USART_H

#include "main.h"
#include "stdint.h"

//============================================================
// 一个串口 DMA 双缓冲通道类
// 作用：
//  1. 记录这个串口用的是哪个 UART
//  2. 记录 M0 / M1 两块缓冲区地址
//  3. 记录单块缓冲区长度
//  4. 记录收到数据后，要调用哪个处理函数
//============================================================

class UART_DMA_Channel
{

public:
    UART_DMA_Channel();

    // 初始化这个串口通道
    void Init(UART_HandleTypeDef *huart_,
              uint8_t *buf0_,
              uint8_t *buf1_,
              uint16_t buffSize_,
              void (*rxCallback_)(uint8_t *buf, uint16_t len));

    // 初始化双缓存DMA
    void InitMultiBufferDMA();

    // 串口空闲中断到来后，处理一次接收事件
    void RxEventCallback(uint16_t Size);

    // 重新使能 DMA / 空闲中断
    void ReEnable();

    // 判断传进来的 huart 是否就是当前对象绑定的 huart
    uint8_t IsThisUart(UART_HandleTypeDef *huart_);

private:
    UART_HandleTypeDef *huart; // 当前绑定的串口
    uint8_t *buf0;             // DMA双缓冲中的 M0 地址
    uint8_t *buf1;             // DMA双缓冲中的 M1 地址
    uint16_t buffSize;         // 单个缓冲区长度

    void (*rxCallback)(uint8_t *buf, uint16_t len);
};

//============================================================
// 串口中心类
// 作用：
//  1. 初始化所有串口 DMA 通道
//  2. 在 HAL 总回调里，把事件分发给正确串口对象
//============================================================
class BSP_USART
{
public:
    static void Init(void);

    static void RxEventDispatch(UART_HandleTypeDef *huart, uint16_t Size);
};

#endif
