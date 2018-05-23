#ifndef __CHUANKOU_H__
#define __CHUANKOU_H__

#include "main.h"
void uartinit(void);
void UARTIntHandler(void);
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
#endif 

