#ifndef __LIN_H
#define __LIN_H

#include "stm32f4xx.h"

#define SYNC_FRAME (0x55U)
#define ERR_TIMEOUT (0xFFU)

#define LIN_LEN_Pos (0)
#define LIN_LEN_Msk (0x1111 << LIN_LEN_Pos)
#define LIN_ID_Pos (4)
#define LIN_ID_Msk (0x111 << LIN_ID_Pos)
#define LIN_MODE_Pos (7)
#define LIN_MODE_Msk (0x1 << LIN_MODE_Pos)

#define myGPIO (GPIOB)
#define myUSART (USART3)

typedef struct{
	uint16_t ID;	//10bits
	uint8_t data[16];	//8bits but 10 in reality
	uint8_t size;
}LIN_MSG;

void LIN_config(void);

void LIN_send_message(LIN_MSG *msg);

void LIN_send_request(LIN_MSG *req);

void sync_break(void);

uint8_t checksum(uint8_t length, volatile uint8_t *data);

void UART_PutChar(uint8_t data);

uint8_t UART_GetChar(void);

void LIN_read_message_content(LIN_MSG* msg);

void LIN_write_message_content(LIN_MSG* msg);

#endif /* __LIN_H */
