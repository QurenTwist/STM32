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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
/* Definitions for RedLED */
osThreadId_t RedLEDHandle;
const osThreadAttr_t RedLED_attributes = {
  .name = "RedLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GreenLED */
osThreadId_t GreenLEDHandle;
const osThreadAttr_t GreenLED_attributes = {
  .name = "GreenLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartRedLEDTask(void *argument);
void StartGreenLEDTask(void *argument);

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

  /* creation of RedLED */
  RedLEDHandle = osThreadNew(StartRedLEDTask, NULL, &RedLED_attributes);

  /* creation of GreenLED */
  GreenLEDHandle = osThreadNew(StartGreenLEDTask, NULL, &GreenLED_attributes);

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
  
  // 1. 初始化 ESP8266
  ESP8266_Init();
  
  // 2. 连接 WiFi (请修改为真实的 SSID 和 Password)
  if (!ESP8266_JoinWiFi("Ciallo~", "zhangzc123")) {
      // WiFi 连接失败处理
      printf("WiFi Connect Failed!\n");
  }
  
  // 3. 连接服务器
  if (ESP8266_ConnectServer("45.192.106.45", 8080)) {
      // 连接成功处理
      printf("Server Connect Success!\n");
  } else {
      // 连接失败处理
      printf("Server Connect Failed!\n");
  }

  /* Infinite loop */
  for(;;)
  {
    // 构造测试数据
    static int counter = 0;
    char json_data[64];
    sprintf(json_data, "{\"device_id\":\"stm32_01\",\"value\":%d}", counter++);
    
    // 发送数据
    ESP8266_SendJSON(json_data);
    
    // 每 5 秒发送一次
    osDelay(5000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartRedLEDTask */
/**
* @brief Function implementing the RedLED thread.
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
    osDelay(1);
  }
  /* USER CODE END StartRedLEDTask */
}

/* USER CODE BEGIN Header_StartGreenLEDTask */
/**
* @brief Function implementing the GreenLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGreenLEDTask */
void StartGreenLEDTask(void *argument)
{
  /* USER CODE BEGIN StartGreenLEDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGreenLEDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

