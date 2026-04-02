#ifndef BSP_CAN_H
#define BSP_CAN_H


	
#include "stm32h7xx.h"
#include "fdcan.h"

/**
 * @brief The structure that contains the Information of FDCAN Transmit.
 */
typedef struct
{
    FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t Data[8];
} FDCAN_TxFrame_TypeDef;

/**
 * @brief The structure that contains the Information of FDCAN Receive.
 */
typedef struct
{
    FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} FDCAN_RxFrame_TypeDef;

/**
 * @brief
 *        FDCAN BSP class
 *        负责：
 *        1、初始化 3 路 FDCAN
 *        2、配置滤波器
 *        3、统一发送接口
 *        4、接收中断回调分发
 */
class BSP_CAN
{
public:
    static FDCAN_TxFrame_TypeDef FDCAN1_TxFrame;
    static FDCAN_TxFrame_TypeDef FDCAN2_TxFrame;
    static FDCAN_TxFrame_TypeDef FDCAN3_TxFrame;

    static FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
    static FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;

public:
    static void Init(void);
    static void AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame);

    static void RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);  
    static void RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);

private:
    static void FDCAN1_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8]);
    static void FDCAN2_RxFifo1RxHandler(uint32_t *Identifier, uint8_t Data[8]);
    static void FDCAN3_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8]);
};



#endif