#include "can.h"
#include "stm32f4xx.h"
#include "cmsis_os.h"

CAN_TypeDef *CAN = CAN1;

void CAN_config(){
	//enable GPIO B
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// set up PINs PB 8 and PB 9
	GPIO_TypeDef *PB = GPIOB;
	PB->MODER 	&= ~(GPIO_MODER_MODER8_Msk 		| GPIO_MODER_MODER9_Msk);
	PB->OTYPER 	&= ~(GPIO_OTYPER_OT8_Msk 		| GPIO_OTYPER_OT9_Msk);
	PB->PUPDR 	&= ~(GPIO_PUPDR_PUPD8_Msk 		| GPIO_PUPDR_PUPD9_Msk);
	PB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED8_Msk 	| GPIO_OSPEEDR_OSPEED9_Msk);
	PB->AFR[1] 	&= ~(GPIO_AFRH_AFSEL8_Msk 		| GPIO_AFRH_AFSEL9_Msk);

	//set mode
	PB->MODER 	|= (0b10U << GPIO_MODER_MODER8_Pos 	| 0b10U << GPIO_MODER_MODER9_Pos); //mode sortie
	PB->OTYPER 	|= (0U << GPIO_OTYPER_OT8_Pos 		| 0U << GPIO_OTYPER_OT9_Pos); //mode push-pull
	PB->PUPDR 	|= (0U << GPIO_PUPDR_PUPD8_Pos 		| 0U << GPIO_PUPDR_PUPD9_Pos); //pas de resistance pull up/down
	PB->OSPEEDR |= (0b11U << GPIO_OSPEEDR_OSPEED8_Pos 	| 0b11U << GPIO_OSPEEDR_OSPEED9_Pos); //mode high speed
	PB->AFR[1]  |= (0b1001U << GPIO_AFRH_AFSEL8_Pos | 0b1001U << GPIO_AFRH_AFSEL9_Pos);

	//Enable CAN clock
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

	//Clear sleep bit to wakeup
	CAN->MCR &= ~CAN_MCR_SLEEP; //!CAN_MCR_SLEEP;

	//Wait for CAN to wakeup
	while(!(CAN->MSR & CAN_MSR_SLAK)); //0x2

	//Enter initialization mode
	CAN->MCR |= CAN_MCR_INRQ;

	//Wait for initialization mode
	while(!(CAN->MSR & CAN_MSR_INAK)); //0x1

	//Set config
	CAN->MCR &= CAN_MCR_INRQ;

	//Set bit Timing
	CAN->BTR = (0b0U << CAN_BTR_LBKM_Pos |
			0b01U << CAN_BTR_SJW_Pos |
			0b010U << CAN_BTR_TS2_Pos|
			0b101U << CAN_BTR_TS1_Pos |
			0x10 << CAN_BTR_BRP_Pos);

	//Release Mailbox
	CAN->RF0R |= CAN_RF0R_RFOM0;

	/*Interrupts*/
	//Set Interrupt RX FIFO0 (FMPIE0) and TX box empty
	CAN->IER |= CAN_IER_FMPIE0;

	//Activate Interrupt on NVIC
	uint32_t prioritygroup = 0x00U;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(CAN1_RX0_IRQn, NVIC_EncodePriority(prioritygroup, 5, 1));

	/* Enable interrupt */
	NVIC_EnableIRQ(CAN1_RX0_IRQn);

	//Go to normal mode
	CAN->MCR &= ~CAN_MCR_INRQ; //!CAN_MCR_INRQ;
	//Wait for normal mode
	while(!(CAN->MSR & CAN_MSR_INAK)); //0x1
}



int CAN_set_filter(int index, int scale_mode, int filter_mode, int FIFO_ID, uint32_t filter_ID, uint32_t filter_Msk) {
	//deactivate filter
	CAN1->FA1R &= ~(0b1UL << index);

	//enter filter init mode
	CAN1->FMR |= 0b1UL << CAN_FMR_FINIT_Pos;

	//scale mode (16 or 32 bits)
	CAN1->FS1R &= ~(0b1UL << index);
	CAN1->FS1R |= scale_mode << index;

	//filter mode (list or mask
	CAN1->FM1R &= ~(0b1UL << index);
	CAN1->FM1R |= filter_mode << index;

	//FIFO index
	CAN1->FFA1R &= ~(0b1UL << index);
	CAN1->FFA1R |= FIFO_ID << index;

	//configure filter bank
	CAN1->sFilterRegister[index].FR1 = (filter_ID<<3);
	CAN1->sFilterRegister[index].FR2 = (filter_Msk<<3);

	//reactivate filter
	int is_active = CAN_FILTER_ACTIVE;
	CAN1->FA1R |= is_active << index;

	//leave filter init mode
	CAN1->FMR &= ~CAN_FMR_FINIT_Msk;

	return 1;
}

int CAN_send_msg(uint8_t can_mode, uint32_t msg_id, uint8_t msg_rtr, uint8_t msg_dlc, uint8_t msg_data[]) {
	//Creation du message
	CAN_MSG msg;
	msg.mode = can_mode;
	msg.ID = msg_id;
	msg.RTR = msg_rtr;
	msg.DLC = msg_dlc;

	for (int i = 0; i<msg_dlc; i++) {
		msg.data[i] = msg_data[i];
	}

	//send message
	if (CAN1->TSR & CAN_TSR_TME0_Msk) {

		// set id mode (stadnard or extanded)
		if (msg.mode == CAN_MODE_STANDARD) {
			CAN1->sTxMailBox[0].TIR = (msg.ID << CAN_TI0R_STID_Pos) | (msg.RTR << CAN_TI0R_RTR_Pos);
		} else {
			CAN1->sTxMailBox[0].TIR = (msg.ID << CAN_TI0R_EXID_Pos) | (msg.RTR << CAN_TI0R_RTR_Pos) | (0b1UL << CAN_TI0R_IDE_Pos);
		}

		// set the DLS (size)
		CAN1->sTxMailBox[0].TDTR = msg.DLC;

		// put data in the good order
		CAN1->sTxMailBox[0].TDLR =
				msg.data[3] << 24 |
				msg.data[2] << 16 |
				msg.data[1] << 8 |
				msg.data[0] << 0;

		CAN1->sTxMailBox[0].TDHR =
				msg.data[7] << 24 |
				msg.data[6] << 16 |
				msg.data[5] << 8 |
				msg.data[4] << 0;

		//request send message
		CAN1->sTxMailBox[0].TIR |= 0b1UL << CAN_TI0R_TXRQ_Pos;

		return 1;
	} else {
		return 0;
	}
}
