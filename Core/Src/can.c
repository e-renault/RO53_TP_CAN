#include "can.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_rcc_ex.h"



CAN_TypeDef *CAN = CAN1;



void CAN_config(uint8_t IDE, uint16_t Filter_ID_high, uint16_t Filter_ID_low, uint16_t Filter_Mask_high, uint16_t Filter_mask_low){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	//Enable CAN clock
	__HAL_RCC_CAN1_CLK_ENABLE();

	//Clear sleep bit to wakeup
	CAN->MCR &= 0xFFFFFFFD;
	//Wait for CAN to wakeup
	while(!(CAN1->MSR & CAN_MSR_SLAK)); //0x2

	//Enter initialization mode
	CAN->MCR |= 0x00000001;
	//Wait for initialization mode
	while(!(CAN1->MSR & CAN_MSR_INAK)); //0x1


	//TODO Remplacer les 0x00000001 et autres par les constantes


	//Set config
	CAN1->MCR &= 0x00000001;

	//Set bit Timing = 250Kbauds/sec / Mode Loopback / Prescaler = 66
	// BS1= 6TQ // BS2 = 3TQ
	CAN1->BTR = 0x00250042;

	//TODO Développer le calcul précédent



	//Release Mailbox
	CAN1->RF0R |= CAN_RF0R_RFOM0;




	/*Interrupts*/
	//Set Interrupt RX FIFO0 (FMPIE0) and TX box empty
	CAN1->IER |= CAN_IER_FMPIE0 & CAN_IER_TMEIE;//0x3;

	//TODO Développer le calcul précédent



	//Activate Interrupt on NVIC
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

	HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);


	//Go to normal mode
	CAN1->MCR &= 0xFFFFFFFE;
	//Wait for normal mode
	while(!(CAN1->MSR & CAN_MSR_INAK)); //0x1

	//Deactivate Filter 0 and 1
	CAN1->FA1R &= 0xFFFFFFFC;
	//Initialize mode for all filters
	CAN1->FMR |= 0x1;




	/*Filters with 32 bits mode*/
	//32 bits filter to filter 0,1
	CAN1->FS1R |= 0x00000003;
	//Filter 1 in List mode
	CAN1->FM1R |= 0x00000002;
	//Filter 0 in mask mode
	CAN1->FM1R &= 0xFFFFFFFE;
	//Assign filter 0,1 to FIFO0
	CAN1->FFA1R &= 0xFFFFFFFC;

	//Configure the filter bank
	CAN1->sFilterRegister[0].FR1 = (0x100 << 21); //ID
	CAN1->sFilterRegister[0].FR2 = (0x7F0 << 5); //Mask

	CAN1->sFilterRegister[1].FR1 = (0x200 << 21); //ID
	CAN1->sFilterRegister[1].FR2 = (0x205 << 5); //ID
	CAN1->sFilterRegister[1].FR2 |= 0x2; //ID with RTR = 1

	//Activate Filter 0 and 1
	CAN1->FA1R |= 0x00000003;
	//Leave filter init
	CAN1->FMR &= 0x0;

	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}



