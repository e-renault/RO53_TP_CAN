#include "myUART.h"

void USART_config(void){

	/* USART2 interrupt init*/
	uint32_t prioritygroup = 0x00U;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(prioritygroup, 15, 15));

	/* Enable interrupt */
	NVIC_EnableIRQ(USART2_IRQn);

	//Initialisation de PA2 comme USART2_TX et PA3 comme USART2_RX
	/*Activation du port A*/
	RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN_Msk);
	RCC->AHB1ENR |= 0x1U << (RCC_AHB1ENR_GPIOAEN_Pos);

    /* Setup PA2 and PA3 as Alternate Function */
	GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
	GPIOA->MODER |= 0b10U << (GPIO_MODER_MODER2_Pos) | 0b10U << (GPIO_MODER_MODER3_Pos);

	/* Setup Alternate function as USART2 */
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2_Msk | GPIO_AFRL_AFSEL3_Msk);
	GPIOA->AFR[0] |= 0x7U << (GPIO_AFRL_AFSEL2_Pos) | 0x7U << (GPIO_AFRL_AFSEL3_Pos);

	/* Push pull output */
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT0_Msk | GPIO_OTYPER_OT1_Msk);
	GPIOA->OTYPER |= 0b1U << (GPIO_OTYPER_OT0_Pos) | 0b1U << (GPIO_OTYPER_OT1_Pos);

	/* Pull up resistor on */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2_Msk | GPIO_PUPDR_PUPD3_Msk);
	GPIOA->PUPDR |= 0b1U << (GPIO_PUPDR_PUPD2_Pos) | 0b1U << (GPIO_PUPDR_PUPD3_Pos);

	/* Output speed set to VeryHigh */
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED2_Msk | GPIO_OSPEEDR_OSPEED3_Msk);
	GPIOA->OSPEEDR |= 0b11U << (GPIO_OSPEEDR_OSPEED2_Pos)
			| 0b11U << (GPIO_OSPEEDR_OSPEED3_Pos);


	//Activation de l'horloge de USART2
	RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN_Msk);
	RCC->APB1ENR |= 0x1U << (RCC_APB1ENR_USART2EN_Pos);

	/*Enable USART, no TE no RE yet, Oversampling = 8, 8bit mode, no parity
	Enable Tx and Rx*/
	USART2->CR1 &= ~(USART_CR1_OVER8_Msk | USART_CR1_RXNEIE_Msk | USART_CR1_TE_Msk | USART_CR1_RE_Msk);
	USART2->CR1 |= 0b1U << (USART_CR1_OVER8_Pos)
					| 0b0U << (USART_CR1_TE_Pos)
					| 0b0U << (USART_CR1_RXNEIE_Pos)
					| 0b0U << (USART_CR1_RE_Pos);

	/* No LIN mode, No clock output (synchronous mode)*/
	USART2->CR2 &= ~(USART_CR2_LINEN_Msk | USART_CR2_LBCL_Msk);
	USART2->CR2 |= 0b0U << (USART_CR2_LINEN_Pos) | 0b0U << (USART_CR2_LBCL_Pos);

	/*No control mode, 3 sample point,*/
	USART2->CR3 &= ~(USART_CR3_ONEBIT_Msk | USART_CR3_CTSE_Msk);
	USART2->CR3 |= 0b0U << (USART_CR3_ONEBIT_Pos) | 0b0U << (USART_CR3_CTSE_Pos);

	/*19200bauds -> USARTDIV = 273.4375 -> Mantissa = 273d=0x111 , Fraction = 0.4375*16 = 7d = 0x7*/
	USART2->BRR &= ~(USART_BRR_DIV_Mantissa_Msk | USART_BRR_DIV_Fraction_Msk);
	USART2->BRR |= 0x111U << (USART_BRR_DIV_Mantissa_Pos) | 0x7U << (USART_BRR_DIV_Fraction_Pos);

	/*Enable UART*/
	USART2->CR1 &= ~(USART_CR1_UE_Msk | USART_CR1_RXNEIE_Msk| USART_CR1_TE_Msk | USART_CR1_RE_Msk);
	USART2->CR1 |= 0b1U << (USART_CR1_UE_Pos)
		| 0b1U << (USART_CR1_TE_Pos)
		| 0b1U << (USART_CR1_RXNEIE_Pos)
		| 0b1U << (USART_CR1_RE_Pos);

	HAL_Delay(1000);

	myPrintln("connection established !");
}

void myPrintChar(char c)						// affiche un caractere unique sur le terminal
{
	USART2->DR = c;
	while(!(USART2->SR & 0x00000080));		// donnee transferee au registre de decalage
	while(!(USART2->SR & 0x00000040));		//fin de transmission
}

void USART_send_message(char *mess)					// affiche une chaine de caracteres sur le terminal
{
	int i=0;
	while(mess[i]!='\0')					//tant que le i-eme caractere est different de \0
	{

		USART2->DR = mess[i];				// Affichage du i-eme caractere sur le terminal
		while(!(USART2->SR & 0x00000080));	// donnee transferee au registre de decalage
		while(!(USART2->SR & 0x00000040));	//fin de transmission
		i++;								// incrementation de i jusqu'a la fin de la chaine de caractere
	}

	myPrintChar(10);						// saut de ligne
	myPrintChar(13);						// retour chariot
}

int __io_getchar(void) {
	uint32_t SR_RXNE;
	uint32_t i = 0;
	uint32_t TIMEOUT = 10000;
	do {
		SR_RXNE = USART2->SR;		//copie des drapeaux
		SR_RXNE &= USART_SR_RXNE;	//isolement du drapeau RXNE
		i++;
	} while (SR_RXNE != USART_SR_RXNE && i<TIMEOUT);
	if (i<TIMEOUT) {
		return USART2->DR;
	} else {
		return 0;					//en cas d'attente trop longue
	}
}

int USART2_Receive(uint8_t * data, uint32_t len) {//data l'adresse d'enregistrement et len sa capacité max
	uint32_t i;
	for (i = 0; i<len; i++) {
		int tmp = __io_getchar();
		if (tmp != 0) {
			*(data+i) = tmp;
		} else {
			*(data+i) = '\0';	//si tmp renvoie '0', cela signifie qu'il y a eu timeout, je considère
			break;				//alors que la trame est fini.
		}
	}
	return i;					//retourne la taille de la chaine de caractère, i<=len
}

void USART2_IRQHandler(void) {
	uint8_t chn[30];
	uint32_t len;

	len = USART2_Receive(chn, 30);

	/** insert you code there **/
}
