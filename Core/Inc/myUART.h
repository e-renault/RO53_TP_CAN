
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "string.h"

#ifndef INC_MYUSART_H_
#define INC_MYUSART_H_

/* Exported functions prototypes ---------------------------------------------*/
void USART_config(void);
void USART_send_message(char*);
void myPrintChar(char);
int __io_getchar(void);
int USART2_Receive(uint8_t * data, uint32_t len);

#endif /* INC_MYUSART_H_ */
