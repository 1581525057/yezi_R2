#include "bsp_usart.h"
#include "usart.h"
#include "dma.h"
#include "bsp_remove.h"
#include "dm_imu.h"
#include "usart_task.h"
#include "laser_distance.h"

extern DM_IMU dm_imu;
extern Remote remove_dji;
//============================================================
// 全局串口 DMA 通道对象
// 一个对象对应一个串口
//============================================================
UART_DMA_Channel uart2_dma;
UART_DMA_Channel uart5_dma;
UART_DMA_Channel uart8_dma;

//============================================================
// 各串口收到数据后的处理函数
//============================================================

//------------------------------------------------------------
// USART2 -> DM_IMU
//------------------------------------------------------------

static void USART2_RxCallback(uint8_t *buf, uint16_t len)
{
    if (len = UART_BUFNUM_DM) {
        dm_imu.ParseIMUStream(buf);
    }
}

//------------------------------------------------------------
// USART5 -> SBUS 遥控器
//------------------------------------------------------------
static void USART5_RxCallback(uint8_t *buf, uint16_t len)
{
    if (len == SBUS_RX_BUF_NUM) {
        remove_dji.parseSBUS(buf);
    }
}

//------------------------------------------------------------
// USART8 -> LaserDistance
//------------------------------------------------------------
static void USART8_RxCallback(uint8_t *buf, uint16_t len)
{
    if (len >= 8) {
        laser.laser_parse_dma_data(buf, len);
    }
}

//============================================================
// UART_DMA_Channel 构造函数
//============================================================
UART_DMA_Channel::UART_DMA_Channel()
{
    huart      = 0;
    buf0       = 0;
    buf1       = 0;
    buffSize   = 0;
    rxCallback = 0;
}

//============================================================
// 初始化这个串口 DMA 通道对象
//============================================================
void UART_DMA_Channel::Init(UART_HandleTypeDef *huart_,
                            uint8_t *buf0_,
                            uint8_t *buf1_,
                            uint16_t buffSize_,
                            void (*rxCallback_)(uint8_t *buf, uint16_t len))
{
    huart = huart_;
    buf0  = buf0_;
    buf1 = buf1_;
    buffSize = buffSize_;
    rxCallback = rxCallback_;

    InitMultiBufferDMA();
}

void UART_DMA_Channel::InitMultiBufferDMA()
{
    if (huart == 0 || buf0 == 0 || buf1 == 0) {
        return;
    }

    // 设置为 接收到空闲中断 的模式
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 
    huart->RxXferSize = buffSize * 2;

    // 打开 UART 的 DMA 接收功能
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    // 打开空闲中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    // 先关闭 DMA，等待真正关掉后再改寄存器
    do {
        __HAL_DMA_DISABLE(huart->hdmarx);
    } while (((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

    //========================================================
    // 配置 DMA
    //========================================================

    // DMA 外设地址 = UART 数据寄存器地址
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;


    // M0 地址
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M0AR = (uint32_t)buf0;

    // M1 地址
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M1AR = (uint32_t)buf1;

    // 设置单块缓冲区长度
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR = buffSize;

    // 打开双缓冲模式
    SET_BIT(((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

    // 重新使能 DMA
    __HAL_DMA_ENABLE(huart->hdmarx);
}

void UART_DMA_Channel::RxEventCallback(uint16_t Size)
{
    DMA_Stream_TypeDef *dma_stream;

    if (huart == 0) {
        return;
    }

    dma_stream = (DMA_Stream_TypeDef *)huart->hdmarx->Instance;

    //--------------------------------------------------------
    // 判断当前 DMA 正在写哪块缓冲区
    //
    // CT = 0 -> 当前目标是 M0
    // CT = 1 -> 当前目标是 M1
    //
    // 这里我沿用你原来的逻辑：
    // 如果 CT=0，就处理 buf0，然后切到 buf1
    // 如果 CT=1，就处理 buf1，然后切到 buf0
    //--------------------------------------------------------
    if ((dma_stream->CR & DMA_SxCR_CT) == RESET) {
        //----------------------------------------------------
        // 当前 DMA 正在使用 M0
        //----------------------------------------------------
        __HAL_DMA_DISABLE(huart->hdmarx);

        // 切换到 M1
        dma_stream->CR |= DMA_SxCR_CT;

        // 重设接收长度
        __HAL_DMA_SET_COUNTER(huart->hdmarx, buffSize * 2);

        // 处理 buf0
        if (rxCallback != 0) {
            rxCallback(buf0, Size);
        }
    } else {
        //----------------------------------------------------
        // 当前 DMA 正在使用 M1
        //----------------------------------------------------
        __HAL_DMA_DISABLE(huart->hdmarx);

        // 切换到 M0
        dma_stream->CR &= ~DMA_SxCR_CT;

        // 重设接收长度
        __HAL_DMA_SET_COUNTER(huart->hdmarx, buffSize * 2);

        // 处理 buf1
        if (rxCallback != 0) {
            rxCallback(buf1, Size);
        }
    }

    // 最后重新使能
    ReEnable();
}

//============================================================
// 重新使能 DMA / 空闲中断
//============================================================
void UART_DMA_Channel::ReEnable()
{
    if (huart == 0) {
        return;
    }

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    // 重新打开空闲中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    // 打开 UART DMA 接收
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    // 重新使能 DMA
    __HAL_DMA_ENABLE(huart->hdmarx);
}

//============================================================
// 判断是不是这个串口
// 是返回1，不是返回0
//============================================================
uint8_t UART_DMA_Channel::IsThisUart(UART_HandleTypeDef *huart_)
{
    if (huart == huart_) {
        return 1;
    }
    return 0;
}

//============================================================
// BSP_USART::Init
// 初始化所有串口 DMA 通道
//============================================================
void BSP_USART::Init(void)
{
    //--------------------------------------------------------
    // USART2 -> IMU
    //--------------------------------------------------------
    uart2_dma.Init(&huart2,
                   dm_imu.dmimu_data[0],
                   dm_imu.dmimu_data[1],
                   UART_BUFNUM_DM,
                   USART2_RxCallback);

    //--------------------------------------------------------
    // USART5 -> SBUS 遥控器
    //--------------------------------------------------------
    uart5_dma.Init(&huart5, remove_dji.SBUS_MultiRx_Buff[0],
                   remove_dji.SBUS_MultiRx_Buff[1],
                   SBUS_RX_BUF_NUM, USART5_RxCallback);

    //--------------------------------------------------------
    // USART8 -> LaserDistance
    //--------------------------------------------------------
    uart8_dma.Init(&huart8, laser.rx_buf[0],
                   laser.rx_buf[1],
                   LASER_RX_LEN, USART8_RxCallback);
}

//============================================================
// 串口事件总分发
// HAL_UARTEx_RxEventCallback 里只需要调这个
//============================================================
void BSP_USART::RxEventDispatch(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (uart2_dma.IsThisUart(huart)) {
        uart2_dma.RxEventCallback(Size);
        return;
    }

    if(uart5_dma.IsThisUart(huart))
    {
        uart5_dma.RxEventCallback(Size);
        return;
    }

    if (uart8_dma.IsThisUart(huart)) {
        uart8_dma.RxEventCallback(Size);
        return;
    }
}

//============================================================
// HAL 串口接收事件总回调
//============================================================
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BSP_USART::RxEventDispatch(huart, Size);
}
