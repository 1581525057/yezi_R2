#include "CommData.h"
#include "M2006AngleMotor.h"
#include "RS05.h"

namespace
{
constexpr uint32_t kM2006CanId = 0x207U;
constexpr uint32_t kDjiCurrentControlId = 0x1FFU;
constexpr uint8_t kM2006CurrentSlot = 2U;
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
        M2006Angle_UpdateFeedback(data);
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

    const uint16_t current = static_cast<uint16_t>(M2006Angle_GetCurrentCommand());
    const uint8_t data_index = static_cast<uint8_t>(kM2006CurrentSlot * 2U);
    data[data_index] = static_cast<uint8_t>(current >> 8);
    data[data_index + 1U] = static_cast<uint8_t>(current);
}
