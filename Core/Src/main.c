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
#include "myUSART.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId taskTestCommandHandle;
osThreadId taskClignoterHandle;
/* USER CODE BEGIN PV */
extern CAN_MSG incoming_msg_CAN;
int activate = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void freeRTOSTestCommand(void const * argument);
void freeRTOSClignoter(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  CAN_config();
  LIN_config();
  USART_config();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of taskTestCommand */
  osThreadDef(taskTestCommand, freeRTOSTestCommand, osPriorityBelowNormal, 0, 128);
  taskTestCommandHandle = osThreadCreate(osThread(taskTestCommand), NULL);

  /* definition and creation of taskClignoter */
  osThreadDef(taskClignoter, freeRTOSClignoter, osPriorityNormal, 0, 128);
  taskClignoterHandle = osThreadCreate(osThread(taskClignoter), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  HAL_GPIO_WritePin(GPIOD, LED_Vert_Pin|LED_Orange_Pin|LED_Rouge_Pin|LED_Bleu_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Vert_Pin LED_Orange_Pin LED_Rouge_Pin LED_Bleu_Pin */
  GPIO_InitStruct.Pin = LED_Vert_Pin|LED_Orange_Pin|LED_Rouge_Pin|LED_Bleu_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_freeRTOSTestCommand */
/**
  * @brief  Function implementing the taskTestCommand thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_freeRTOSTestCommand */
void freeRTOSTestCommand(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED_Bleu_GPIO_Port, LED_Bleu_Pin);

	  //Demande de l'etat de la bague de l'essuie glace arriere
	  uint32_t msg_id = (CAN_ID_BEGINNING << 24) | (CAN_SLAVE_CODE_COMMODOS_GROUP << 16) | (CAN_MASTER_ID << 8) | CAN_SLAVE_PORT_C;
	  uint8_t data[1] = {CAN_LIGHT_OFF};
	  CAN_send_msg(CAN_MODE_EXTENDED, msg_id, CAN_MSG_RTR_RQST, 0, data);

	  osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_freeRTOSClignoter */
/**
* @brief Function implementing the taskClignoter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_freeRTOSClignoter */
void freeRTOSClignoter(void const * argument)
{
  /* USER CODE BEGIN freeRTOSClignoter */
	  osDelay(500);
	  int allume = 0;
	  /* Infinite loop */
	  for(;;)
	  {
		  HAL_GPIO_TogglePin(LED_Rouge_GPIO_Port, LED_Rouge_Pin);
		  if(allume){
			  //eteindreClignotant(0x10530112);

			  uint32_t msg_id = (CAN_ID_BEGINNING << 24) | (CAN_SLAVE_CODE_REAR_LIGHTS << 16) | (CAN_MASTER_ID << 8) | CAN_SLAVE_PORT_C;
			  uint8_t data[1] = {CAN_LIGHT_OFF};
			  CAN_send_msg(CAN_MODE_EXTENDED, msg_id, CAN_MSG_RTR_DATA, 1, data);

			  allume = 0;
		  }else if (activate){
			  //allumerClignotant(0x10530112, 0x04);

			  uint32_t msg_id = (CAN_ID_BEGINNING << 24) | (CAN_SLAVE_CODE_REAR_LIGHTS << 16) | (CAN_MASTER_ID << 8) | CAN_SLAVE_PORT_C;
			  uint8_t data[1] = {CAN_LIGHT_LEFT_REAR_TURN_SIGNAL_ON};
			  CAN_send_msg(CAN_MODE_EXTENDED, msg_id, CAN_MSG_RTR_DATA, 1, data);

			  allume = 1;
		  }
		  osDelay(1000);
	  }

  /* USER CODE END freeRTOSClignoter */
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
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
