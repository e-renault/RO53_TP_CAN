#ifndef __LIN_H
#define __LIN_H

#include "stm32f4xx.h"

#define SYNC_FRAME (0x55U)
#define ERR_TIMEOUT (0xFFU)

typedef struct{
	uint16_t PIDField;	//10bits
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

void LIN_read_message_content(volatile LIN_MSG* msg);

void USART3_IRQHandler(void);

void handle_awnser();

void handle_data(uint16_t ID);

void handle_request(uint16_t ID);

#endif /* __LIN_H */
