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

#define CAN_MSG_RTR_DATA 0 //Trame de donnees
#define CAN_MSG_RTR_RQST 1 //Trame de requete

#define CAN_ID_BEGINNING 0x10 //Debut d'une trame CAN

#define CAN_MASTER_ID 0x01 //Notre identifiant

#define CAN_SLAVE_CODE_SATELLITE_GROUP 0x50
#define CAN_SLAVE_CODE_COMMODOS_GROUP 0x51
#define CAN_SLAVE_CODE_FRONT_LIGHTS 0x52
#define CAN_SLAVE_CODE_REAR_LIGHTS 0x53

#define CAN_SLAVE_PORT_A 0x10
#define CAN_SLAVE_PORT_B 0x11
#define CAN_SLAVE_PORT_C 0x12
#define CAN_SLAVE_PORT_D 0x13

#define CAN_LIGHT_LEFT_REAR_TURN_SIGNAL_ON 0x04
#define CAN_LIGHT_OFF 0x00


typedef struct{
	uint8_t mode;//Standard or extended
	uint32_t ID;
	uint8_t RTR;
	uint8_t DLC;
	uint8_t data[8];
}CAN_MSG;


int CAN_set_filter(int index, int scale_mode, int filter_mode, int FIFO_ID, uint32_t filter_ID, uint32_t filter_Msk);

int CAN_send_msg(uint8_t, uint32_t, uint8_t, uint8_t, uint8_t[]);

void CAN_config();

#endif /* __CAN_H */
