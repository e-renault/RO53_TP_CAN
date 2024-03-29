/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "lin.h"
#include "myUART.h"
#include "myTime.h"
#include "stm32f4xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId taskCommodoReqHandle;
osThreadId taskBlinkerHandle;
osThreadId taskHandleLINHandle;
osThreadId taskHandleCANHandle;
osMessageQId queue_LIN_request_modeHandle;
osMessageQId queue_LIN_request_reponseHandle;
osMessageQId queue_LIN_waiting_for_responseHandle;
osMessageQId queue_LIN_message_recievedHandle;
osMessageQId queue_CAN_msgHandle;
/* USER CODE BEGIN PV */
int blink_mode_activated = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void startTaskCommodoReq(void const * argument);
void startTaskBlinker(void const * argument);
void StartTaskHandleLIN(void const * argument);
void StartTaskHandleCAN(void const * argument);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  /*Configure the CAN*/
  CAN_config();
  //Filtre sur les messages envoyes par la maquette
  CAN_set_filter(0, CAN_FILTER_SCALE_32BIT, CAN_FILTER_MODE_MASK, CAN_FILTER_FIFO0_, 0x10FF50FF, 0xFF00FF00);
  //Filtre sur les reponses qui nous sont envoyees
  CAN_set_filter(1, CAN_FILTER_SCALE_32BIT, CAN_FILTER_MODE_MASK, CAN_FILTER_FIFO0_, 0x1001FFFF, 0xFFFF0000);

  /*Congigure the LIN, the clock and the USART*/
  LIN_config();
  clock_Init();
  USART_config();
  /* USER CODE END 2 */


  /* Create the queue(s) */
  /* definition and creation of queue_LIN_request_mode */
  osMessageQDef(queue_LIN_request_mode, 1, uint8_t);
  queue_LIN_request_modeHandle = osMessageCreate(osMessageQ(queue_LIN_request_mode), NULL);

  /* definition and creation of queue_LIN_request_reponse */
  osMessageQDef(queue_LIN_request_reponse, 1, LIN_MSG);
  queue_LIN_request_reponseHandle = osMessageCreate(osMessageQ(queue_LIN_request_reponse), NULL);

  /* definition and creation of queue_LIN_waiting_for_response */
  osMessageQDef(queue_LIN_waiting_for_response, 1, LIN_MSG);
  queue_LIN_waiting_for_responseHandle = osMessageCreate(osMessageQ(queue_LIN_waiting_for_response), NULL);

  /* definition and creation of queue_LIN_message_recieved */
  osMessageQDef(queue_LIN_message_recieved, 1, LIN_MSG);
  queue_LIN_message_recievedHandle = osMessageCreate(osMessageQ(queue_LIN_message_recieved), NULL);

  /* definition and creation of queue_CAN_msg */
  osMessageQDef(queue_CAN_msg, 8, CAN_MSG);
  queue_CAN_msgHandle = osMessageCreate(osMessageQ(queue_CAN_msg), NULL);


  /* Create the thread(s) */
  /* definition and creation of taskCommodoReq */
  osThreadDef(taskCommodoReq, startTaskCommodoReq, osPriorityNormal, 0, 128);
  taskCommodoReqHandle = osThreadCreate(osThread(taskCommodoReq), NULL);

  /* definition and creation of taskBlinker */
  osThreadDef(taskBlinker, startTaskBlinker, osPriorityAboveNormal, 0, 128);
  taskBlinkerHandle = osThreadCreate(osThread(taskBlinker), NULL);

  /* definition and creation of taskHandleLIN */
  osThreadDef(taskHandleLIN, StartTaskHandleLIN, osPriorityLow, 0, 128);
  taskHandleLINHandle = osThreadCreate(osThread(taskHandleLIN), NULL);

  /* definition and creation of taskHandleCAN */
  osThreadDef(taskHandleCAN, StartTaskHandleCAN, osPriorityBelowNormal, 0, 128);
  taskHandleCANHandle = osThreadCreate(osThread(taskHandleCAN), NULL);


  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Vert_GPIO_Port, LED_Vert_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Vert_Pin */
  GPIO_InitStruct.Pin = LED_Vert_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Vert_GPIO_Port, &GPIO_InitStruct);

}


/* USER CODE BEGIN Header_startTaskCommodoReq */
/**
  * @brief  Function implementing the taskCommodoReq thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startTaskCommodoReq */
void startTaskCommodoReq(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  //Demande de l'etat de la bague de l'essuie glace arriere
	  uint32_t msg_id = (CAN_ID_BEGINNING << 24) | (CAN_SLAVE_CODE_COMMODOS_GROUP << 16) | (CAN_MASTER_ID << 8) | CAN_SLAVE_PORT_C;
	  uint8_t data[1];
	  CAN_send_msg(CAN_MODE_EXTENDED, msg_id, CAN_MSG_RTR_RQST, 0, data);

	  //Mettre la tache en attente pendant 100ms
	  osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startTaskBlinker */
/**
* @brief Function implementing the taskBlinker thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskBlinker */
void startTaskBlinker(void const * argument)
{
  /* USER CODE BEGIN startTaskBlinker */
	int light_state = 0;
	char msg_eteindre[16]="Light Off";
	char msg_allumer[16]="Light On";
	/* Infinite loop */
	for(;;){
		if(light_state){//If the rear left turn signal is on
			//Switch the rear left turn signal on
			//Display message eteindre
			USART_send_message(msg_eteindre);

			//Switch off rear left turn signal
			uint32_t msg_id = (CAN_ID_BEGINNING << 24) | (CAN_SLAVE_CODE_REAR_LIGHTS << 16) | (CAN_MASTER_ID << 8) | CAN_SLAVE_PORT_C;
			uint8_t data[1] = {CAN_LIGHT_OFF};
			CAN_send_msg(CAN_MODE_EXTENDED, msg_id, CAN_MSG_RTR_DATA, 1, data);

			light_state = 0;
		}else if (blink_mode_activated){//If the rear left turn signal is off and if it must be activated
			//Switch the rear left turn signal off
			//Display message allumer
			USART_send_message(msg_allumer);

			//Switch on rear left turn signal
			uint32_t msg_id = (CAN_ID_BEGINNING << 24) | (CAN_SLAVE_CODE_REAR_LIGHTS << 16) | (CAN_MASTER_ID << 8) | CAN_SLAVE_PORT_C;
			uint8_t data[1] = {CAN_LIGHT_LEFT_REAR_TURN_SIGNAL_ON};
			CAN_send_msg(CAN_MODE_EXTENDED, msg_id, CAN_MSG_RTR_DATA, 1, data);

			light_state = 1;
		}
		osDelay(1000);
	}
  /* USER CODE END startTaskBlinker */
}

/* USER CODE BEGIN Header_StartTaskHandleLIN */
/**
* @brief Function implementing the taskHandleLIN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskHandleLIN */
void StartTaskHandleLIN(void const * argument)
{
  /* USER CODE BEGIN StartTaskHandleLIN */
  /* Infinite loop */
  for(;;) {

	// subtask one, look for received message
	if (uxQueueMessagesWaiting(queue_LIN_message_recievedHandle)) {/** read queue message received **/

		LIN_MSG received_msg_LIN;//received message
		xQueueReceive(queue_LIN_message_recievedHandle, &received_msg_LIN, 100);//queue message received

		//variable cannot be declared in switch case statement
		int i;
		char header[32] = "Master clock received : ";
		char time_str[32];
		uint32_t CAN_msg_id;
		uint8_t CAN_data[8];

		switch(received_msg_LIN.ID & LIN_ID_Msk >> LIN_ID_Pos){
			case 0b001://Display time
				i = 0;
				//recover time data and push it into the string list
				time_str[i++] = received_msg_LIN.data[0]+'0';
				time_str[i++] = received_msg_LIN.data[1]+'0';
				time_str[i++] = ':';
				time_str[i++] = received_msg_LIN.data[2]+'0';
				time_str[i++] = received_msg_LIN.data[3]+'0';
				time_str[i++] = ':';
				time_str[i++] = received_msg_LIN.data[4]+'0';
				time_str[i++] = received_msg_LIN.data[5]+'0';
				time_str[i++] = '\0';//string end

				USART_send_message(header);
				USART_send_message(time_str);
				break;

			case 0b010://Change LED state
				HAL_GPIO_WritePin(LED_Vert_GPIO_Port, LED_Vert_Pin, received_msg_LIN.data[0]);
				break;

			case 0b100://Send a trame CAN to the rear left turn signal to switch it on or off
				for(int i=0; (i<8) & (i<received_msg_LIN.size);i++){
					CAN_data[i] = received_msg_LIN.data[i];
				}
				CAN_msg_id = (CAN_ID_BEGINNING << 24)
						| (CAN_SLAVE_CODE_REAR_LIGHTS << 16)
						| (CAN_MASTER_ID << 8)
						| CAN_SLAVE_PORT_C;
				CAN_send_msg(CAN_MODE_EXTENDED, CAN_msg_id, CAN_MSG_RTR_DATA, 1, CAN_data);
				break;
		}
	}

	// subtask 2 check if a response is requiered
	if (uxQueueMessagesWaiting(queue_LIN_waiting_for_responseHandle)) {/** read queue waiting for response **/

		LIN_MSG received_msg_LIN;//received message header (ID + size)
		xQueueReceive(queue_LIN_waiting_for_responseHandle, &received_msg_LIN, 100);//queue waiting for response

		//variable cannot be declared in switch case statement
		LIN_MSG response;
		response.size = received_msg_LIN.size;
		int i;
		Time t;

		switch(received_msg_LIN.ID & LIN_ID_Msk >> LIN_ID_Pos){
			case 0b001:// Renvoyer l'horloge
				getCurrentTime(&t);
				i = 0;

				// generate response
				response.data[i++] = t.hou.MSD;
				response.data[i++] = t.hou.LSD;
				response.data[i++] = t.min.MSD;
				response.data[i++] = t.min.LSD;
				response.data[i++] = t.sec.MSD;
				response.data[i++] = t.sec.LSD;
				break;

			case 0b010://return LED State
				response.data[i++] = HAL_GPIO_ReadPin(LED_Vert_GPIO_Port, LED_Vert_Pin);
				break;
			default:
				response.size = 0;//error invalid ID
		}

		//return response
		LIN_write_message_content(&response);
	}

	//subtask 3 check if an answer has been published
	if (uxQueueMessagesWaiting(queue_LIN_request_reponseHandle)) {/** read queue request response **/

		LIN_MSG return_msg_LIN;//response
		xQueueReceive(queue_LIN_request_reponseHandle, &return_msg_LIN, 100);//queue waiting for response

		//variable cannot be declared in switch case statement
		int i;
		char header_clock[32] = "Slave clock received : ";
		char header_LED[32] = "Slave LED state : ";
		char unknown[32] = "Unknown recieved";
		char time_str[32];
		char state_LED_ON[3] = "ON";
		char state_LED_OFF[4] = "Off";

		switch(return_msg_LIN.ID & LIN_ID_Msk >> LIN_ID_Pos){
			case 0b001://Affiche l'horloge
				i = 0;
				time_str[i++] = return_msg_LIN.data[0]+'0';
				time_str[i++] = return_msg_LIN.data[1]+'0';
				time_str[i++] = ':';
				time_str[i++] = return_msg_LIN.data[2]+'0';
				time_str[i++] = return_msg_LIN.data[3]+'0';
				time_str[i++] = ':';
				time_str[i++] = return_msg_LIN.data[4]+'0';
				time_str[i++] = return_msg_LIN.data[5]+'0';
				time_str[i++] = '\0';

				USART_send_message(header_clock);
				USART_send_message(time_str);
				break;

			case 0b010://Affiche l'état de la LED
				USART_send_message(header_LED);
				USART_send_message(return_msg_LIN.data[0] ? state_LED_ON : state_LED_OFF);
				break;
			default:
				USART_send_message(unknown);//ID not recognized
		}
	}

	//Mettre la tache en attente pendant 10ms
    osDelay(10);
  }
  /* USER CODE END StartTaskHandleLIN */
}

/* USER CODE BEGIN Header_StartTaskHandleCAN */
/**
* @brief Function implementing the taskHandleCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskHandleCAN */
void StartTaskHandleCAN(void const * argument)
{
  /* USER CODE BEGIN StartTaskHandleCAN */
  /* Infinite loop */
  for(;;) {
	if (uxQueueMessagesWaiting(queue_CAN_msgHandle)) {/** read queue message received **/

		CAN_MSG received_msg_CAN;
		xQueueReceive(queue_CAN_msgHandle, &received_msg_CAN, 100);//queue message received

		//Gestion des donnees recues
		if((received_msg_CAN.ID & 0x00110000) == (CAN_MASTER_ID << 16)){
			//Gestion des reponses a nos requetes de donnees
			switch(received_msg_CAN.data[0]){
				case 0x0E://Bague essuie glace arriere sur le premier cran
				case 0x0D:
				case 0x0B:
					//Activer le clignotement
					blink_mode_activated = 1;
					break;
				case 0x1E://Bague essuie glace arriere sur le 0
				case 0x1D:
				case 0x1B:
					//Desactiver le clignotement
					blink_mode_activated = 0;
					break;
			}
		} else {
			uint32_t msg_id = 0;
			uint8_t data[1]={CAN_LIGHT_OFF};
			//Gestion des requetes envoyees sur le CAN
			switch(received_msg_CAN.data[0]){
				case 0x88://Bouton SET+ du commodo
					//Allumer le clignotant arriere gauche
					msg_id = (CAN_ID_BEGINNING << 24) | (CAN_SLAVE_CODE_REAR_LIGHTS << 16) | (CAN_MASTER_ID << 8) | CAN_SLAVE_PORT_C;
					data[0] = CAN_LIGHT_LEFT_REAR_TURN_SIGNAL_ON;
					CAN_send_msg(CAN_MODE_EXTENDED, msg_id, CAN_MSG_RTR_DATA, 1, data);
					break;
				case 0x5D://Bouton SET- du commodo
				case 0x5E:
					//Eteindre le clignotant arriere gauche
					msg_id = (CAN_ID_BEGINNING << 24) | (CAN_SLAVE_CODE_REAR_LIGHTS << 16) | (CAN_MASTER_ID << 8) | CAN_SLAVE_PORT_C;
					data[0] = CAN_LIGHT_OFF;
					CAN_send_msg(CAN_MODE_EXTENDED, msg_id, CAN_MSG_RTR_DATA, 1, data);
					break;
			}
		}
	}
	//Mettre la tache en attente pendant 10ms
	osDelay(10);
  }
  /* USER CODE END StartTaskHandleCAN */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
