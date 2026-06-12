#ifndef TEST_STUB_USART_H
#define TEST_STUB_USART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct {
    void *Instance;
} UART_HandleTypeDef;

extern UART_HandleTypeDef huart7;

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif
