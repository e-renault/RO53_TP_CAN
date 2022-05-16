#ifndef __CAN_H
#define __CAN_H

#include "stm32f4xx.h"

#define CAN_MODE_STANDARD 0b0UL
#define CAN_MODE_EXTENDED 0b1UL

#define CAN_FILTER_SCALE_16BIT 0b0UL
#define CAN_FILTER_SCALE_32BIT 0b1UL

#define CAN_FILTER_MODE_MASK 0b0UL
#define CAN_FILTER_MODE_LIST 0b1UL

#define CAN_FILTER_FIFO0_ 0b0UL
#define CAN_FILTER_FIFO1_ 0b1UL

#define CAN_FILTER_INACTIVE 0b0U
#define CAN_FILTER_ACTIVE 0b1UL

typedef struct{
	uint8_t mode;//Standard or extended
	uint32_t ID;
	uint8_t RTR;
	uint8_t DLC;
	uint8_t data[8];
}CAN_MSG;

int CAN_set_filter(int index, int scale_mode, int filter_mode, int FIFO_ID, uint32_t filter_ID, uint32_t filter_Msk);

int CAN_send_msg(CAN_MSG msg);

void CAN_config();


#endif /* __CAN_H */
