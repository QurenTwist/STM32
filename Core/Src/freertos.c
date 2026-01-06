/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RedLEDTask */
osThreadId_t RedLEDTaskHandle;
const osThreadAttr_t RedLEDTask_attributes = {
  .name = "RedLEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for USART1Task */
osThreadId_t USART1TaskHandle;
const osThreadAttr_t USART1Task_attributes = {
  .name = "USART1Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TFTLCDTask */
osThreadId_t TFTLCDTaskHandle;
const osThreadAttr_t TFTLCDTask_attributes = {
  .name = "TFTLCDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartRedLEDTask(void *argument);
void StartUSART1Task(void *argument);
void StartTFTLCDTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RedLEDTask */
  RedLEDTaskHandle = osThreadNew(StartRedLEDTask, NULL, &RedLEDTask_attributes);

  /* creation of USART1Task */
  USART1TaskHandle = osThreadNew(StartUSART1Task, NULL, &USART1Task_attributes);

  /* creation of TFTLCDTask */
  TFTLCDTaskHandle = osThreadNew(StartTFTLCDTask, NULL, &TFTLCDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartRedLEDTask */
/**
* @brief Function implementing the RedLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRedLEDTask */
void StartRedLEDTask(void *argument)
{
  /* USER CODE BEGIN StartRedLEDTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
    osDelay(500);
  }
  /* USER CODE END StartRedLEDTask */
}

/* USER CODE BEGIN Header_StartUSART1Task */
/**
* @brief Function implementing the USART1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSART1Task */
void StartUSART1Task(void *argument)
{
  /* USER CODE BEGIN StartUSART1Task */
  char msg[]="Hello Wrold!";
  /* Infinite loop */
  for(;;)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
    osDelay(500);
  }
  /* USER CODE END StartUSART1Task */
}

/* USER CODE BEGIN Header_StartTFTLCDTask */
/**
* @brief Function implementing the TFTLCDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTFTLCDTask */
void StartTFTLCDTask(void *argument)
{
  /* USER CODE BEGIN StartTFTLCDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTFTLCDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

