#ifndef __COMMDATA_H__
#define __COMMDATA_H__

#include "fdcan.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void FTM_FdcanRxDispatch(FDCAN_HandleTypeDef *hfdcan, const FDCAN_RxHeaderTypeDef *header, uint8_t data[8]);
void FTM_PatchDjiCurrentCommand(FDCAN_TxHeaderTypeDef *header, uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif
