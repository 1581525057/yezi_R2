#include "CommData.h"
#include "M2006Follower.h"
#include "RS05.h"
#include <stdio.h>
#include <string.h>

namespace
{
constexpr uint16_t kVisualRxSize = 65U;
constexpr uint16_t kVisualPayloadSize = kVisualRxSize - 1U;
constexpr uint32_t kM2006CanId = 0x207U;
constexpr uint32_t kDjiCurrentControlId = 0x1FFU;
constexpr uint8_t kM2006CurrentSlot = 2U;

uint8_t RxData[kVisualRxSize];
int16_t X_value = 0;
int16_t Z_value = 0;
float Yaw_value = 0.0f;

void RestartVisualDma(void)
{
    memset(RxData, 0, sizeof(RxData));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart_vis, RxData, kVisualPayloadSize);
    __HAL_DMA_DISABLE_IT(&hdma_usart_vis_rx, DMA_IT_HT);
}

void RestartCmdDma(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart_cmd, RxCmd, sizeof(RxCmd));
    __HAL_DMA_DISABLE_IT(&hdma_usart_cmd_rx, DMA_IT_HT);
}
}

extern "C" volatile VisualData_t g_visual_data = {0, 0, 0.0f};

uint8_t RxCmd[255];

extern "C" void vis_init(void)
{
    RestartVisualDma();
}

extern "C" void cmd_init(void)
{
    RestartCmdDma();
}

extern "C" uint8_t ParseData_sscanf(uint8_t *rx_buffer, int16_t *X_value, int16_t *Z_value, float *Yaw_value)
{
    int16_t x = 0;
    int16_t z = 0;
    float y = 0.0f;

    if ((rx_buffer == nullptr) || (X_value == nullptr) || (Z_value == nullptr) || (Yaw_value == nullptr))
    {
        return 0U;
    }

    if (sscanf(reinterpret_cast<char *>(rx_buffer), "S,%hd,%hd,%f,E", &x, &z, &y) == 3)
    {
        if (x >= -999 && x <= 999 &&
            z >= -999 && z <= 999 &&
            y >= -180.0f && y <= 180.0f)
        {
            *X_value = x;
            *Z_value = z;
            *Yaw_value = y;
            return 1U;
        }
    }

    return 0U;
}

extern "C" uint8_t CommData_UartRxEventDispatch(UART_HandleTypeDef *huart, uint16_t Size)
{
    (void)Size;

    if (huart == &huart_vis)
    {
        RxData[kVisualPayloadSize] = 0U;
        if (ParseData_sscanf(RxData, &X_value, &Z_value, &Yaw_value) != 0U)
        {
            g_visual_data.x = X_value;
            g_visual_data.z = Z_value;
            g_visual_data.yaw = Yaw_value;
        }

        RestartVisualDma();
        return 1U;
    }

    if (huart == &huart_cmd)
    {
        RestartCmdDma();
        return 1U;
    }

    return 0U;
}

extern "C" void FTM_FdcanRxDispatch(FDCAN_HandleTypeDef *hfdcan, const FDCAN_RxHeaderTypeDef *header, uint8_t data[8])
{
    if ((hfdcan == nullptr) || (header == nullptr) || (data == nullptr))
    {
        return;
    }

    if (hfdcan != &hfdcan3)
    {
        return;
    }

    if ((header->IdType == FDCAN_STANDARD_ID) && (header->Identifier == kM2006CanId))
    {
        g_m2006.UpdateFeedback(data, HAL_GetTick());
        return;
    }

    if (header->IdType == FDCAN_EXTENDED_ID)
    {
        RS05_HandleCanMessage(header->Identifier, data);
    }
}

extern "C" void FTM_PatchDjiCurrentCommand(FDCAN_TxHeaderTypeDef *header, uint8_t data[8])
{
    if ((header == nullptr) || (data == nullptr))
    {
        return;
    }

    if ((header->IdType != FDCAN_STANDARD_ID) || (header->Identifier != kDjiCurrentControlId))
    {
        return;
    }

    const uint16_t current = static_cast<uint16_t>(g_m2006.GetCurrentCommand());
    const uint8_t data_index = static_cast<uint8_t>(kM2006CurrentSlot * 2U);
    data[data_index] = static_cast<uint8_t>(current >> 8);
    data[data_index + 1U] = static_cast<uint8_t>(current);
}
