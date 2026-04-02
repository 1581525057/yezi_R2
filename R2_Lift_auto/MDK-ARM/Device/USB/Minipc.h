
#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "main.h"

extern void MiniPC_Transmit_Info(uint8_t *Buff, uint16_t len);

extern void MiniPC_Recvive_Info(uint8_t* Buff,  uint32_t *Len);
#endif
