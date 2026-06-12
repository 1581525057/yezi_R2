#include "arm_comm.h"
#include "usart.h"

#include <stdint.h>
#include <string.h>

UART_HandleTypeDef huart7;
extern uint8_t ARM_COMM_PICK_KFS_STABLE_COUNT;
extern float PICK_KFS_BEFORE_STEP_ADVANCE_CM;
extern float PICK_KFS_AFTER_STEP_ADVANCE_CM;

static UART_HandleTypeDef *last_uart;
static const uint8_t *last_uart_data;
static uint16_t last_uart_size;

extern "C" HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t)
{
    last_uart     = huart;
    last_uart_data = pData;
    last_uart_size = Size;
    return HAL_OK;
}

static int frameEquals(const uint8_t *actual, const uint8_t *expected)
{
    return memcmp(actual, expected, ArmComm::FRAME_LENGTH) == 0;
}

static int expectActionFrame(ArmComm::ActionCode action, uint8_t num_KFS, const uint8_t expected[ArmComm::FRAME_LENGTH])
{
    ArmComm comm;

    if (comm.executeAction(action, num_KFS) == 0U) {
        return 1;
    }

    if (comm.getFrameLength() != ArmComm::FRAME_LENGTH) {
        return 2;
    }

    if (frameEquals(comm.getFrame(), expected) == 0) {
        return 3;
    }

    return 0;
}

static int expectSendUsesUart7(void)
{
    ArmComm comm;

    last_uart     = 0;
    last_uart_data = 0;
    last_uart_size = 0;

    if (comm.executeAction(ArmComm::ACTION_PICK_HIGH_400, 0x02U) == 0U) {
        return 1;
    }

    comm.send();

    if (last_uart != &huart7) {
        return 2;
    }

    if (last_uart_data != comm.getFrame()) {
        return 3;
    }

    if (last_uart_size != ArmComm::FRAME_LENGTH) {
        return 4;
    }

    return 0;
}

static int expectReceiveFrameParses(void)
{
    ArmComm comm;
    const uint8_t rx_frame[ArmComm::RX_FRAME_LENGTH] = {0xBB, 0x01, 0x01, 0x01, 0x02, 0xEE};

    if (comm.parseRxFrame(rx_frame, ArmComm::RX_FRAME_LENGTH) == 0U) {
        return 1;
    }

    const ArmComm::RxData &rx_data = comm.getRxData();
    if (rx_data.event != 0x01U) {
        return 2;
    }
    if (rx_data.sys_mode != 0x01U) {
        return 3;
    }
    if (rx_data.arm_kfs != 0x01U) {
        return 4;
    }
    if (rx_data.car_kfs != 0x02U) {
        return 5;
    }

    return 0;
}

static int expectInvalidReceiveFrameRejected(void)
{
    ArmComm comm;
    const uint8_t valid_frame[ArmComm::RX_FRAME_LENGTH] = {0xBB, 0x03, 0x01, 0x00, 0x01, 0xEE};
    const uint8_t bad_tail[ArmComm::RX_FRAME_LENGTH] = {0xBB, 0x04, 0x01, 0x01, 0x02, 0x55};

    if (comm.parseRxFrame(valid_frame, ArmComm::RX_FRAME_LENGTH) == 0U) {
        return 1;
    }
    if (comm.parseRxFrame(bad_tail, ArmComm::RX_FRAME_LENGTH) != 0U) {
        return 2;
    }

    const ArmComm::RxData &rx_data = comm.getRxData();
    if (rx_data.event != 0x03U) {
        return 3;
    }
    if (rx_data.sys_mode != 0x01U) {
        return 4;
    }
    if (rx_data.arm_kfs != 0x00U) {
        return 5;
    }
    if (rx_data.car_kfs != 0x01U) {
        return 6;
    }

    return 0;
}

static int expectPickKFSBeforeStepSendsDirectly(void)
{
    ArmComm comm;

    last_uart      = 0;
    last_uart_data = 0;
    last_uart_size = 0;

    if (comm.pickKFS(ArmComm::ACTION_PICK_HIGH_200, 0x02U, 0U, 1.0f, 2.0f, 0.0f) != 0U) {
        return 1;
    }
    if (last_uart != &huart7) {
        return 2;
    }
    if (comm.getChassisVxTarget(1.25f) != 0.0f) {
        return 3;
    }
    if (comm.getChassisVyTarget(-1.25f) != 0.0f) {
        return 4;
    }

    comm.rx_data_.event = 1U;
    if (comm.pickKFS(ArmComm::ACTION_PICK_HIGH_200, 0x02U, 0U, 1.0f, 2.0f, 0.0f) == 0U) {
        return 5;
    }
    if (comm.getChassisVxTarget(1.25f) != 1.25f) {
        return 6;
    }
    if (comm.getChassisVyTarget(-1.25f) != -1.25f) {
        return 7;
    }

    return 0;
}

static int expectPickKFSAfterStepMovesBeforeSending(void)
{
    ArmComm comm;
    const uint8_t old_stable_count = ARM_COMM_PICK_KFS_STABLE_COUNT;
    ARM_COMM_PICK_KFS_STABLE_COUNT = 1U;
    const float old_before_advance_cm = PICK_KFS_BEFORE_STEP_ADVANCE_CM;
    const float old_after_advance_cm  = PICK_KFS_AFTER_STEP_ADVANCE_CM;
    PICK_KFS_BEFORE_STEP_ADVANCE_CM = 10.0f;
    PICK_KFS_AFTER_STEP_ADVANCE_CM  = 10.0f;

    last_uart      = 0;
    last_uart_data = 0;
    last_uart_size = 0;

    if (comm.pickKFS(ArmComm::ACTION_PICK_HIGH_400, 0x01U, 1U, 1.0f, 2.0f, 0.0f) != 0U) {
        ARM_COMM_PICK_KFS_STABLE_COUNT = old_stable_count;
        PICK_KFS_BEFORE_STEP_ADVANCE_CM = old_before_advance_cm;
        PICK_KFS_AFTER_STEP_ADVANCE_CM  = old_after_advance_cm;
        return 1;
    }
    if (last_uart != 0) {
        ARM_COMM_PICK_KFS_STABLE_COUNT = old_stable_count;
        PICK_KFS_BEFORE_STEP_ADVANCE_CM = old_before_advance_cm;
        PICK_KFS_AFTER_STEP_ADVANCE_CM  = old_after_advance_cm;
        return 2;
    }
    if (comm.getChassisVxTarget(0.0f) <= 0.0f) {
        ARM_COMM_PICK_KFS_STABLE_COUNT = old_stable_count;
        PICK_KFS_BEFORE_STEP_ADVANCE_CM = old_before_advance_cm;
        PICK_KFS_AFTER_STEP_ADVANCE_CM  = old_after_advance_cm;
        return 3;
    }
    if (comm.getChassisVyTarget(0.0f) != 0.0f) {
        ARM_COMM_PICK_KFS_STABLE_COUNT = old_stable_count;
        PICK_KFS_BEFORE_STEP_ADVANCE_CM = old_before_advance_cm;
        PICK_KFS_AFTER_STEP_ADVANCE_CM  = old_after_advance_cm;
        return 4;
    }

    if (comm.pickKFS(ArmComm::ACTION_PICK_HIGH_400, 0x01U, 1U, 1.1f, 2.0f, 0.0f) != 0U) {
        ARM_COMM_PICK_KFS_STABLE_COUNT = old_stable_count;
        PICK_KFS_BEFORE_STEP_ADVANCE_CM = old_before_advance_cm;
        PICK_KFS_AFTER_STEP_ADVANCE_CM  = old_after_advance_cm;
        return 5;
    }
    if (last_uart != &huart7) {
        ARM_COMM_PICK_KFS_STABLE_COUNT = old_stable_count;
        PICK_KFS_BEFORE_STEP_ADVANCE_CM = old_before_advance_cm;
        PICK_KFS_AFTER_STEP_ADVANCE_CM  = old_after_advance_cm;
        return 6;
    }

    comm.rx_data_.event = 1U;
    if (comm.pickKFS(ArmComm::ACTION_PICK_HIGH_400, 0x01U, 1U, 1.1f, 2.0f, 0.0f) == 0U) {
        ARM_COMM_PICK_KFS_STABLE_COUNT = old_stable_count;
        PICK_KFS_BEFORE_STEP_ADVANCE_CM = old_before_advance_cm;
        PICK_KFS_AFTER_STEP_ADVANCE_CM  = old_after_advance_cm;
        return 7;
    }

    ARM_COMM_PICK_KFS_STABLE_COUNT = old_stable_count;
    PICK_KFS_BEFORE_STEP_ADVANCE_CM = old_before_advance_cm;
    PICK_KFS_AFTER_STEP_ADVANCE_CM  = old_after_advance_cm;
    return 0;
}

int main(void)
{
    const uint8_t power_on_init[ArmComm::FRAME_LENGTH] = {0xAA, 0x01, 0x00, 0x00, 0x00, 0x00, 0x55};
    const uint8_t pick_high_200[ArmComm::FRAME_LENGTH] = {0xAA, 0x01, 0x01, 0x01, 0x00, 0x00, 0x55};
    const uint8_t pick_high_400[ArmComm::FRAME_LENGTH] = {0xAA, 0x01, 0x02, 0x02, 0x00, 0x00, 0x55};
    const uint8_t pick_low_200[ArmComm::FRAME_LENGTH] = {0xAA, 0x01, 0x03, 0x03, 0x00, 0x00, 0x55};
    const uint8_t zone3_ready[ArmComm::FRAME_LENGTH] = {0xAA, 0x01, 0x00, 0x00, 0x01, 0x00, 0x55};
    const uint8_t zone3_place[ArmComm::FRAME_LENGTH] = {0xAA, 0x01, 0x00, 0x00, 0x02, 0x00, 0x55};
    const uint8_t zone3_fetch_upper[ArmComm::FRAME_LENGTH] = {0xAA, 0x01, 0x04, 0x00, 0x00, 0x00, 0x55};
    const uint8_t zone3_fetch_lower[ArmComm::FRAME_LENGTH] = {0xAA, 0x01, 0x05, 0x00, 0x00, 0x00, 0x55};
    const uint8_t zone3_reset[ArmComm::FRAME_LENGTH] = {0xAA, 0x01, 0x00, 0x00, 0x00, 0x00, 0x55};
    const uint8_t power_off[ArmComm::FRAME_LENGTH] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55};

    if (expectActionFrame(ArmComm::ACTION_POWER_ON_INIT, 0x00U, power_on_init) != 0) {
        return 1;
    }
    if (expectActionFrame(ArmComm::ACTION_PICK_HIGH_200, 0x01U, pick_high_200) != 0) {
        return 2;
    }
    if (expectActionFrame(ArmComm::ACTION_PICK_HIGH_400, 0x02U, pick_high_400) != 0) {
        return 3;
    }
    if (expectActionFrame(ArmComm::ACTION_PICK_LOW_200, 0x03U, pick_low_200) != 0) {
        return 4;
    }
    if (expectActionFrame(ArmComm::ACTION_ZONE3_READY, 0x00U, zone3_ready) != 0) {
        return 5;
    }
    if (expectActionFrame(ArmComm::ACTION_ZONE3_PLACE_HAND, 0x00U, zone3_place) != 0) {
        return 6;
    }
    if (expectActionFrame(ArmComm::ACTION_ZONE3_FETCH_UPPER, 0x04U, zone3_fetch_upper) != 0) {
        return 7;
    }
    if (expectActionFrame(ArmComm::ACTION_ZONE3_PLACE_UPPER, 0x00U, zone3_place) != 0) {
        return 8;
    }
    if (expectActionFrame(ArmComm::ACTION_ZONE3_FETCH_LOWER, 0x05U, zone3_fetch_lower) != 0) {
        return 9;
    }
    if (expectActionFrame(ArmComm::ACTION_ZONE3_RESET, 0x00U, zone3_reset) != 0) {
        return 10;
    }
    if (expectActionFrame(ArmComm::ACTION_POWER_OFF, 0x00U, power_off) != 0) {
        return 11;
    }
    if (expectSendUsesUart7() != 0) {
        return 12;
    }
    if (expectReceiveFrameParses() != 0) {
        return 13;
    }
    if (expectInvalidReceiveFrameRejected() != 0) {
        return 14;
    }
    if (expectPickKFSBeforeStepSendsDirectly() != 0) {
        return 15;
    }
    if (expectPickKFSAfterStepMovesBeforeSending() != 0) {
        return 16;
    }

    return 0;
}
