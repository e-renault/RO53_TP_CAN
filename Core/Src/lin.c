#include "lin.h"


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




/**
extern CAN_MSG CAN_RxMessage;
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
}**/
