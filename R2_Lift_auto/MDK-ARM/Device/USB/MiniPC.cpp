#include "MiniPC.h"
#include "usbd_cdc_if.h"
 

void MiniPC_Transmit_Info(uint8_t *buff, uint16_t len)
{
    CDC_Transmit_HS(buff, len);
}

//usbd_cdc_if.c -> CDC_Receive_HS
void MiniPC_Recvive_Info(uint8_t* Buff,  uint32_t *Len)
{
   //处理解包数据
    
}
