#ifndef __LIN_H
#define __LIN_H

#include "stm32f4xx.h"

#include "lin.h"




void UART_Init(void);

void SendMessage(LINMSG *msg);

void SendRequest(LINMSG *msg);

void sync_break(void);

void UART_PutChar(uint8_t data);

uint8_t UART_GetChar(void);

uint8_t checksum(uint8_t length, uint8_t *data);

void USART2_IRQHandler(void);



#endif /* __LIN_H */
