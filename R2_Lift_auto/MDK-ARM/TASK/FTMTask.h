#ifndef __FTMTASK_H__
#define __FTMTASK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint8_t g_ftm_state;

void ftm_task(void *argument);
uint8_t FTM_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
