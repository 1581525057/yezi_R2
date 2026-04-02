/* USER CODE BEGIN Header */
// 负责：
// 1、把 3 路 FDCAN 初始化好
// 2、设置滤波器
// 3、统一发送 CAN 数据
// 4、接收中断回调分发
/* USER CODE END Header */

#include "bsp_can.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "yun_j60.h"

/* 接收帧对象 ---------------------------------------------------------------*/
FDCAN_RxFrame_TypeDef BSP_CAN::FDCAN_RxFIFO0Frame;
FDCAN_RxFrame_TypeDef BSP_CAN::FDCAN_RxFIFO1Frame;

/* 发送帧对象 ---------------------------------------------------------------*/
FDCAN_TxFrame_TypeDef BSP_CAN::FDCAN1_TxFrame =
    {
        .hcan   = &hfdcan1,
        .Header = {
            .Identifier          = 0,
            .IdType              = FDCAN_STANDARD_ID,
            .TxFrameType         = FDCAN_DATA_FRAME,
            .DataLength          = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch       = FDCAN_BRS_OFF,
            .FDFormat            = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
            .MessageMarker       = 0,
        },
        .Data = {0},
};

FDCAN_TxFrame_TypeDef BSP_CAN::FDCAN2_TxFrame =
    {
        .hcan   = &hfdcan2,
        .Header = {
            .Identifier          = 0,
            .IdType              = FDCAN_STANDARD_ID,
            .TxFrameType         = FDCAN_DATA_FRAME,
            .DataLength          = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch       = FDCAN_BRS_OFF,
            .FDFormat            = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
            .MessageMarker       = 0,
        },
        .Data = {0},
};

FDCAN_TxFrame_TypeDef BSP_CAN::FDCAN3_TxFrame =
    {
        .hcan   = &hfdcan3,
        .Header = {
            .Identifier          = 0,
            .IdType              = FDCAN_STANDARD_ID,
            .TxFrameType         = FDCAN_DATA_FRAME,
            .DataLength          = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch       = FDCAN_BRS_OFF,
            .FDFormat            = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
            .MessageMarker       = 0,
        },
        .Data = {0},
};

void BSP_CAN::Init(void)
{
    FDCAN_FilterTypeDef FilterConfig;

    /* FDCAN1 --------------------------------------------------------------*/
    FilterConfig.IdType       = FDCAN_STANDARD_ID;
    FilterConfig.FilterIndex  = 0;
    FilterConfig.FilterType   = FDCAN_FILTER_MASK;
    FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FilterConfig.FilterID1    = 0x00000000;
    FilterConfig.FilterID2    = 0x00000000;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &FilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);

    /* FDCAN2 --------------------------------------------------------------*/
    FilterConfig.IdType       = FDCAN_EXTENDED_ID;
    FilterConfig.FilterIndex  = 0;
    FilterConfig.FilterType   = FDCAN_FILTER_MASK;
    FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    FilterConfig.FilterID1    = 0x00000000;
    FilterConfig.FilterID2    = 0x00000000;

    HAL_FDCAN_ConfigFilter(&hfdcan2, &FilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);

    /* FDCAN3 --------------------------------------------------------------*/
    FilterConfig.IdType       = FDCAN_STANDARD_ID;
    FilterConfig.FilterIndex  = 0;
    FilterConfig.FilterType   = FDCAN_FILTER_MASK;
    FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FilterConfig.FilterID1    = 0x00000000;
    FilterConfig.FilterID2    = 0x00000000;

    HAL_FDCAN_ConfigFilter(&hfdcan3, &FilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan3);
}

void BSP_CAN::AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame)
{
    HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame->hcan,
                                  &FDCAN_TxFrame->Header,
                                  FDCAN_TxFrame->Data);
}

void BSP_CAN::FDCAN1_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8])
{

    Yun_J60_Class::RxHandler(&FDCAN_RxFIFO0Frame, Data);
}

void BSP_CAN::FDCAN2_RxFifo1RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
    DJI_Motor_Class::RxHandler(Identifier, Data);
}

void BSP_CAN::FDCAN3_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
    DJI_Motor_Class::RxHandler(Identifier, Data);
}

void BSP_CAN::RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;

    HAL_FDCAN_GetRxMessage(hfdcan,
                           FDCAN_RX_FIFO0,
                           &FDCAN_RxFIFO0Frame.Header,
                           FDCAN_RxFIFO0Frame.Data);

    if (hfdcan == &hfdcan1) {
        FDCAN1_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,
                                FDCAN_RxFIFO0Frame.Data);
    }

    if (hfdcan == &hfdcan3) {
        FDCAN3_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,
                                FDCAN_RxFIFO0Frame.Data);
    }
}

void BSP_CAN::RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;

    HAL_FDCAN_GetRxMessage(hfdcan,
                           FDCAN_RX_FIFO1,
                           &FDCAN_RxFIFO1Frame.Header,
                           FDCAN_RxFIFO1Frame.Data);

    if (hfdcan == &hfdcan2) {
        FDCAN2_RxFifo1RxHandler(&FDCAN_RxFIFO1Frame.Header.Identifier,
                                FDCAN_RxFIFO1Frame.Data);
    }
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    BSP_CAN::RxFifo0Callback(hfdcan, RxFifo0ITs);
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    BSP_CAN::RxFifo1Callback(hfdcan, RxFifo1ITs);
}