/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
#include "cmsis_os.h"
#include "lin.h"
#include "can.h"
#include "myUART.h"
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */




void USART3_IRQHandler(void) {
	extern osMessageQId queue_LIN_request_modeHandle;
	extern osMessageQId queue_LIN_waiting_for_responseHandle;
	extern osMessageQId queue_LIN_message_recievedHandle;
	extern osMessageQId queue_LIN_request_reponseHandle;


	if (uxQueueMessagesWaitingFromISR(queue_LIN_request_modeHandle)) {// read queue request mode
		//queue request mode

		int dump;xQueueReceiveFromISR(queue_LIN_request_modeHandle, (uint32_t) &dump, 100);//reset queue message recieved

		LIN_MSG msg;LIN_read_message_content(&msg);//read message

		xQueueSendFromISR(queue_LIN_request_reponseHandle, &msg, 0);//write in queue request response
		return;
	}

	if (myUSART->SR & USART_SR_LBD_Msk) {//lien break detected
		myUSART->SR &= ~(USART_SR_LBD_Msk);//reset break flag
		uint8_t break_frame = UART_GetChar(); //void break frame
		uint8_t sync_frame = UART_GetChar(); //void sync frame

		LIN_MSG msg;//recieved message
		msg.ID = UART_GetChar(); //retrieve PID frame
		msg.size = msg.ID & LIN_ID_Msk >> LIN_ID_Pos;//get size

		if (!(msg.ID & LIN_MODE_Msk)) {// is request or data mode based on ID field
			//request mode
			xQueueSendFromISR(queue_LIN_waiting_for_responseHandle, &msg, 0);//add to queue waiting for response
		} else {
			//data mode
			LIN_read_message_content(&msg);//read content
			xQueueSendFromISR(queue_LIN_message_recievedHandle, &msg, 0);//add to queue message recieved
		}
		return;
	}
}

void CAN1_RX0_IRQHandler(void) {
	extern osMessageQId queue_CAN_msgHandle;
	CAN_MSG incoming_msg_CAN;

	//retrieve CAN mode (std or extanded)
	incoming_msg_CAN.mode = (CAN1->sFIFOMailBox[0].RIR & CAN_TI0R_IDE_Msk) >> CAN_TI0R_IDE_Pos;

	//retrieve UID depending on the mode
	if (incoming_msg_CAN.mode == CAN_MODE_STANDARD) {
		incoming_msg_CAN.ID = (CAN1->sFIFOMailBox[0].RIR & CAN_TI0R_STID_Msk) >> CAN_TI0R_STID_Pos;
	} else {
		incoming_msg_CAN.ID = (CAN1->sFIFOMailBox[0].RIR & CAN_TI0R_EXID_Msk) >> CAN_TI0R_EXID_Pos;
	}
	// data or request mode
	incoming_msg_CAN.RTR = (CAN1->sFIFOMailBox[0].RIR & CAN_TI0R_RTR_Msk) >> CAN_TI0R_RTR_Pos;

	// retrieve lenght
	incoming_msg_CAN.DLC = CAN1->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk;
	for (int i = 0, shift = 0; i<8; i++, shift += 8) {
		incoming_msg_CAN.data[i] = (CAN1->sFIFOMailBox[0].RDLR >> shift) & 0xFF;
	}

	// say that the tram has been red
	CAN1->RF0R |= 0b1UL << CAN_RF0R_RFOM0_Pos;

	osMessagePut(queue_CAN_msgHandle, (uint32_t) &incoming_msg_CAN, 100);
}

void USART2_IRQHandler(void) {
	uint8_t chn[30];
	uint32_t len;

	len = USART2_Receive(chn, 30);

	/** insert you code there **/
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
