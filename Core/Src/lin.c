#include "lin.h"

volatile uint8_t request_mode_LIN;
volatile LIN_MSG* request_msg_LIN;

GPIO_TypeDef * myGPIO = GPIOB;
USART_TypeDef * myUSART = USART3;

/*--- UART INIT ---*/
void LIN_config(void){
	/* EXTI interrupt init*/
	uint32_t prioritygroup = 0x00U;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(prioritygroup, 15, 15));

	/* Enable interrupt */
	NVIC_EnableIRQ(USART3_IRQn);

	//Initialisation de PB10 comme MyUSART_TX et PB11 comme MyUSART_RX
	/*Activation du port B*/
	RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOBEN_Msk);
	RCC->AHB1ENR |= 0x1U << (RCC_AHB1ENR_GPIOBEN_Pos);

    /* Setup PB10 and PB11 as Alternate Function */
	myGPIO->MODER &= ~(GPIO_MODER_MODER10_Msk | GPIO_MODER_MODER11_Msk);
	myGPIO->MODER |= 0b10U << (GPIO_MODER_MODER10_Pos) | 0b10U << (GPIO_MODER_MODER11_Pos);

	/* Setup Alternate function as AF7 (USART1-2-3) */
	myGPIO->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL11_Msk);
	myGPIO->AFR[1] |= 0x7U << (GPIO_AFRH_AFSEL10_Pos) | 0x7U << (GPIO_AFRH_AFSEL11_Pos);

	/* Push pull output */
	myGPIO->OTYPER &= ~(GPIO_OTYPER_OT10_Msk | GPIO_OTYPER_OT11_Msk);
	myGPIO->OTYPER |= 0b1U << (GPIO_OTYPER_OT10_Pos) | 0b1U << (GPIO_OTYPER_OT11_Pos);

	/* Pull up resistor on */
	myGPIO->PUPDR &= ~(GPIO_PUPDR_PUPD10_Msk | GPIO_PUPDR_PUPD11_Msk);
	myGPIO->PUPDR |= 0b1U << (GPIO_PUPDR_PUPD10_Pos) | 0b1U << (GPIO_PUPDR_PUPD11_Pos);

	/* Output speed set to VeryHigh */
	myGPIO->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED10_Msk | GPIO_OSPEEDR_OSPEED11_Msk);
	myGPIO->OSPEEDR |= 0b11U << (GPIO_OSPEEDR_OSPEED10_Pos) | 0b11U << (GPIO_OSPEEDR_OSPEED11_Pos);


	//Activation de l'horloge de MyUSART
	RCC->APB1ENR &= ~(RCC_APB1ENR_USART3EN_Msk);
	RCC->APB1ENR |= 0x1U << (RCC_APB1ENR_USART3EN_Pos);

	/*Enable USART, no TE no RE yet, Oversampling = 8, 8bit mode, no parity
	Enable Tx and Rx*/
	myUSART->CR1 &= ~(USART_CR1_OVER8_Msk
			| USART_CR1_RXNEIE_Msk
			| USART_CR1_TE_Msk
			| USART_CR1_RE_Msk);
	myUSART->CR1 |= 0b1U << (USART_CR1_OVER8_Pos) 	// Enable USART
			| 0b0U << (USART_CR1_RXNEIE_Pos)		// Interrupt is inhibited
			| 0b0U << (USART_CR1_TE_Pos)			// Transmitter disabled
			| 0b0U << (USART_CR1_RE_Pos);			// Receiver disabled

	/* LIN mode, STOP and CLKEN cleared for LIN,  , LBDIE Break detection interrupt to 1*/
	myUSART->CR2 &= ~(USART_CR2_LINEN_Msk
			| USART_CR2_STOP_Msk
			| USART_CR2_CLKEN_Msk
			| USART_CR2_LBDL_Msk
			| USART_CR2_LBCL_Msk
			| USART_CR2_LBDIE_Msk);
	myUSART->CR2 |= 0b1U << (USART_CR2_LINEN_Pos)	// LIN mode enabled
			| 0b00U << (USART_CR2_STOP_Pos)			// 1 Stop bit
			| 0b0U << (USART_CR2_CLKEN_Pos)			// CK pin disabled
			| 0b0U << (USART_CR2_LBDL_Pos)   		// 10-bit break detection
			| 0b1U << (USART_CR2_LBCL_Pos)   		// The clock pulse of the last data bit is output to the CK pin
			| 0b1U << (USART_CR2_LBDIE_Pos);		// An interrupt is generated whenever LBD=1 in the USART_SR register

	/*No control mode, 3 sample point, SCEN, HDSEL and IREN cleared for LIN*/
	myUSART->CR3 &= ~(USART_CR3_ONEBIT_Msk
			| USART_CR3_CTSE_Msk
			| USART_CR3_SCEN_Msk
			| USART_CR3_HDSEL_Msk
			| USART_CR3_IREN_Msk);
	myUSART->CR3 |= 0b0U << (USART_CR3_ONEBIT_Pos)	// Three sample bit method
			| 0b0U << (USART_CR3_CTSE_Pos)			// CTS hardware flow control disable
			| 0b0U << (USART_CR3_SCEN_Pos)			// Smartcard Mode disabled
			| 0b0U << (USART_CR3_HDSEL_Pos)			// Half duplex mode is not selected
			| 0b0U << (USART_CR3_IREN_Pos);			// IrDA disabled

	/*19200bauds -> Mantissa = 273 ; frac = 7 */
	myUSART->BRR &= ~(USART_BRR_DIV_Mantissa_Msk | USART_BRR_DIV_Fraction_Msk);
	myUSART->BRR |= 0x111U << (USART_BRR_DIV_Mantissa_Pos) | 0x7U << (USART_BRR_DIV_Fraction_Pos);

	/*Enable UART*/
	myUSART->CR1 &= ~(USART_CR1_UE_Msk
			| USART_CR1_RXNEIE_Msk
			| USART_CR1_TE_Msk
			| USART_CR1_RE_Msk);
	myUSART->CR1 |= 0b1U << (USART_CR1_UE_Pos)	// reenable USART
			| 0b1U << (USART_CR1_RXNEIE_Pos)	// An USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_SR
			| 0b1U << (USART_CR1_TE_Pos)		// Transmitter is enabled
			| 0b1U << (USART_CR1_RE_Pos);		// Receiver is enabled and begins searching for a start bit

	HAL_Delay(1000);
}

/*--- Transmit LIN Message ---*/
void LIN_send_message(LIN_MSG *msg) {
	//disable IRQ to ensure that the transmission goes right
	NVIC_DisableIRQ(USART3_IRQn);

	// send 10 bits in low position to announce the beginning of a trame
	sync_break();

	// send 0b01010101 to sync devices
	UART_PutChar(SYNC_FRAME);

	// send PID
	UART_PutChar(msg->PIDField);

	// send datas
	for (int i = 0; i < msg->size; i++) {
		UART_PutChar(msg->data[i]);
	}

	// send checksum
	UART_PutChar(checksum(msg->size, msg->data));

	//reenable IRQ
	NVIC_EnableIRQ(USART3_IRQn);
}

/*--- Transmit LIN Request ---*/
void LIN_send_request(LIN_MSG *req) {
	//disable IRQ to ensure that the transmission goes right
	NVIC_DisableIRQ(USART3_IRQn);

	// send 10 bits in low position to announce the beginning of a trame
	sync_break();

	// send 0b01010101 to sync devices
	UART_PutChar(SYNC_FRAME);

	// send PID
	UART_PutChar(req->PIDField);

	//set request MODE (the device wait for a response)
	request_mode_LIN = 1;

	//set where the response must be stored
	request_msg_LIN = req;

	//reenable IRQ
	NVIC_EnableIRQ(USART3_IRQn);
}

/*--- Send sync field and break ---*/
void sync_break(void) {
	myUSART->CR1 |= 0x1U << USART_CR1_SBK_Pos;
}

/*--- Calculate lin checksum ---*/
uint8_t checksum(uint8_t length, volatile uint8_t *data) {
	uint8_t ix;
	uint16_t check_sum = 0;

	for(ix = 0; ix < length-1; ix++) {
		check_sum += data[ix];
		if(check_sum >= 256){
			check_sum -= 255;
		}
	}

	return (uint8_t)(0xff - check_sum);
}

/*--- Transmit char ---*/
void UART_PutChar(uint8_t data) {
	myUSART->DR = data;

	// donnee transferee au registre de decalage
	while(!(myUSART->SR & USART_SR_TXE));

	// fin de transmission
	while(!(myUSART->SR & USART_SR_TC));
}

/*--- Read char ---*/
uint8_t UART_GetChar(void) {
	uint32_t SR_RXNE;
	volatile uint32_t i = 0;
	uint32_t TIMEOUT = 1000000;
	do {
		SR_RXNE = myUSART->SR;		//copie des drapeaux
		SR_RXNE &= USART_SR_RXNE;	//isolement du drapeau RXNE
		i++;
	} while (SR_RXNE != USART_SR_RXNE && i<TIMEOUT);
	if (i<TIMEOUT) {
		return myUSART->DR;
	} else {
		return ERR_TIMEOUT;			//en cas d'attente trop longue
	}
}

void LIN_read_message_content(volatile LIN_MSG* msg) {
	for (int i = 0; i < msg->size; i++) {
		msg->data[i] = UART_GetChar();
	}
	uint8_t actual_check_sum = UART_GetChar();
	uint8_t check_sum = checksum(msg->size, msg->data);
	if (check_sum != actual_check_sum) {
		msg->size = 0;
	}
}

void USART3_IRQHandler(void) {
	// no frame
	// awnser: recieve the result of a previous request
	if (request_mode_LIN) {
		request_mode_LIN = 0;
		handle_awnser();
		return;
	}

	if (myUSART->SR & USART_SR_LBD_Msk) {
		myUSART->SR &= ~(USART_SR_LBD_Msk);

		// with frames
		uint8_t break_frame = UART_GetChar(); //void break frame
		uint8_t sync_frame = UART_GetChar(); //void sync frame
		uint8_t PID = UART_GetChar(); //retrieve PID frame

		// data: recieve only data (UID starting by 1 // arbitrary)
		if (PID & 0x80) {
			handle_data((uint16_t) PID & ~(0x80));
			return;
		}

		// request: recieve a request that have to be awnsered (UID starting by 0 // arbitrary)
		if (!(PID & 0x80)) {
			handle_request(PID & ~(0x80));
			return;
		}
	}
}

void handle_awnser() {
	LIN_read_message_content(request_msg_LIN);

	/**
	 * request_msg_LIN must be threated there
	**/
}

volatile LIN_MSG receved_msg;
volatile int receved_msg_flag;
void handle_data(uint16_t ID) {
	receved_msg_flag = 1;
	receved_msg.PIDField = ID;
	receved_msg.size = 8;	//arbitraire
	LIN_read_message_content(&receved_msg);

	/**
	 * receved_msg must be threated there
	**/
}

void handle_request(uint16_t ID) {
	uint8_t size = 8;  //arbitraire

	/** data to sent must be created there ----------------------------**/
	/** USER CODE BEGIN **/
	uint8_t data[16] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p'};

	/** USER CODE END **/

	uint8_t actual_check_sum = checksum(size, data);
	for (int i = 0; i < size; i++) {
		UART_PutChar(data[i]);
	}
	UART_PutChar(actual_check_sum);
}
