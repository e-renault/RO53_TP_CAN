#include "can.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_rcc_ex.h"

CAN_TypeDef *CAN = CAN1;

void CAN_config(uint8_t IDE, uint16_t Filter_ID_high, uint16_t Filter_ID_low, uint16_t Filter_Mask_high, uint16_t Filter_mask_low){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	//Enable CAN clock
	__HAL_RCC_CAN1_CLK_ENABLE();

	//Clear sleep bit to wakeup
	CAN->MCR &= 0xFFFFFFFD; //!CAN_MCR_SLEEP;
	//Wait for CAN to wakeup
	while(!(CAN->MSR & CAN_MSR_SLAK)); //0x2

	//Enter initialization mode
	CAN->MCR |= CAN_MCR_INRQ; //0x00000001;
	//Wait for initialization mode
	while(!(CAN->MSR & CAN_MSR_INAK)); //0x1

	//Set config
	CAN->MCR &= CAN_MCR_INRQ; //0x00000001;

	//Set bit Timing = 250Kbauds/sec / BS1= 6TQ / BS2 = 3TQ / Resynchronization jump width = 1
	CAN->BTR = 0x01250010; //0x00250042; //41250010

	//Set Mode Loopback
	CAN->BTR |= CAN_BTR_LBKM;
	//CAN->BTR &= !CAN_BTR_LBKM;



	//Release Mailbox
	CAN->RF0R |= CAN_RF0R_RFOM0;




	/*Interrupts*/
	//Set Interrupt RX FIFO0 (FMPIE0) and TX box empty
	CAN->IER |= CAN_IER_FMPIE0;


	//Activate Interrupt on NVIC
	//HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 1); //IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority
	uint32_t prioritygroup = 0x00U;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(CAN1_RX0_IRQn, NVIC_EncodePriority(prioritygroup, 0, 1));

	//HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	/* Enable interrupt */
	NVIC_EnableIRQ(CAN1_RX0_IRQn);




	//Go to normal mode
	CAN->MCR &= 0xFFFFFFFE; //!CAN_MCR_INRQ;
	//Wait for normal mode
	while(!(CAN->MSR & CAN_MSR_INAK)); //0x1

	//Deactivate Filter 0 and 1
	CAN->FA1R &= CAN_FFA1R_FFA0 | CAN_FFA1R_FFA1; //0xFFFFFFFC;
	//Initialize mode for all filters
	CAN->FMR |= CAN_FMR_FINIT; //0x1;




	/*Filters with 32 bits mode*/
	//32 bits filter to filter 0,1
	CAN->FS1R |= 0x00000003;
	//Filter 1 in List mode
	CAN->FM1R |= 0x00000002;
	//Filter 0 in mask mode
	CAN->FM1R &= 0xFFFFFFFE;
	//Assign filter 0,1 to FIFO0
	CAN->FFA1R &= 0xFFFFFFFC;

	//Configure the filter bank
	CAN->sFilterRegister[0].FR1 = (0x100 << 21); //ID
	CAN->sFilterRegister[0].FR2 = (0x7F0 << 5); //Mask

	CAN->sFilterRegister[1].FR1 = (0x200 << 21); //ID
	CAN->sFilterRegister[1].FR2 = (0x205 << 5); //ID
	CAN->sFilterRegister[1].FR2 |= 0x2; //ID with RTR = 1

	//Activate Filter 0 and 1
	CAN->FA1R |= 0x00000003;
	//Leave filter init
	CAN->FMR &= 0x0;


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
	CAN1->sFilterRegister[index].FR1 = filter_ID;
	CAN1->sFilterRegister[index].FR2 = filter_Msk;

	//reactivate filter
	int is_active = CAN_FILTER_ACTIVE;
	CAN1->FA1R |= is_active << index;

	//leave filter init mode
	CAN1->FMR &= ~CAN_FMR_FINIT_Msk;

	return 1;
}

int CAN_send_msg(CAN_MSG msg) {
	if (CAN1->TSR & CAN_TSR_TME0_Msk) {
		if (msg.mode == CAN_MODE_STANDARD) {
			CAN1->sTxMailBox[0].TIR = (msg.ID << CAN_TI0R_STID_Pos) | (msg.RTR << CAN_TI0R_RTR_Pos);
		} else {
			CAN1->sTxMailBox[0].TIR = (msg.ID << CAN_TI0R_EXID_Pos) | (msg.RTR << CAN_TI0R_RTR_Pos);
		}

		CAN1->sTxMailBox[0].TDTR = msg.DLC;

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




/*
CAN_MSG CAN_RxMessage;
void CAN1_RX0_IRQHandler(void) {
	//Recieve CAN frame
	CAN_RxMessage.mode = (CAN1->sFIFOMailBox[0].RIR & CAN_TI0R_IDE_Msk) >> CAN_TI0R_IDE_Pos;
	if (CAN_RxMessage.mode == CAN_MODE_STANDARD) {
		CAN_RxMessage.ID = (CAN1->sFIFOMailBox[0].RIR & CAN_TI0R_STID_Msk) >> CAN_TI0R_STID_Pos;
	} else {
		CAN_RxMessage.ID = (CAN1->sFIFOMailBox[0].RIR & CAN_TI0R_EXID_Msk) >> CAN_TI0R_EXID_Pos;
	}
	CAN_RxMessage.RTR = (CAN1->sFIFOMailBox[0].RIR & CAN_TI0R_RTR_Msk) >> CAN_TI0R_RTR_Pos;
	CAN_RxMessage.DLC = CAN1->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk;
	for (int i = 0, shift = 0; i<8; i++, shift += 8) {
		CAN_RxMessage.data[i] = (CAN1->sFIFOMailBox[0].RDLR >> shift) & 0xFF;
	}
	//reset FIFO
	CAN1->RF0R |= 0b1UL << CAN_RF0R_RFOM0_Pos;
	//reset interrupt
	//HAL_CAN_IRQHandler(&hcan1);
	CAN1->IER

}
*/


