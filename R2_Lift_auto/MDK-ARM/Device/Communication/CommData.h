#ifndef __COMMDATA_H__
#define __COMMDATA_H__

#include "fdcan.h"
#include "main.h"
#include "usart.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern DMA_HandleTypeDef hdma_uart9_rx;
extern DMA_HandleTypeDef hdma_usart10_rx;

#define huart_vis huart10
#define hdma_usart_vis_rx hdma_usart10_rx

#define huart_cmd huart9
#define hdma_usart_cmd_rx hdma_uart9_rx

typedef struct
{
    int16_t x;
    int16_t z;
    float yaw;
} VisualData_t;

extern uint8_t RxCmd[255];
extern volatile VisualData_t g_visual_data;

void vis_init(void);
void cmd_init(void);
uint8_t ParseData_sscanf(uint8_t *rx_buffer, int16_t *X_value, int16_t *Z_value, float *Yaw_value);
uint8_t CommData_UartRxEventDispatch(UART_HandleTypeDef *huart, uint16_t Size);
void FTM_FdcanRxDispatch(FDCAN_HandleTypeDef *hfdcan, const FDCAN_RxHeaderTypeDef *header, uint8_t data[8]);
void FTM_PatchDjiCurrentCommand(FDCAN_TxHeaderTypeDef *header, uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif
