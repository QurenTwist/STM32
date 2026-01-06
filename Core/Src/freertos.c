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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "adc.h"
#include "usart.h"
#include "tftlcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char message[50]={};
int len=0;
typedef struct {
  float temp_voltage;
  float light_voltage;
  float Temperature;
  float Illumination;
} Sensor;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for NTCTask */
osThreadId_t NTCTaskHandle;
const osThreadAttr_t NTCTask_attributes = {
  .name = "NTCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LightSensorTask */
osThreadId_t LightSensorTaskHandle;
const osThreadAttr_t LightSensorTask_attributes = {
  .name = "LightSensorTask",
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

void StartNTCTask(void *argument);
void StartLightSenserTask(void *argument);
void StartUSART1Task(void *argument);
void StartTFTLCDTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

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
  /* creation of NTCTask */
  NTCTaskHandle = osThreadNew(StartNTCTask, NULL, &NTCTask_attributes);

  /* creation of LightSensorTask */
  LightSensorTaskHandle = osThreadNew(StartLightSenserTask, NULL, &LightSensorTask_attributes);

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

/* USER CODE BEGIN Header_StartNTCTask */
/**
  * @brief  Function implementing the NTCTask thread.
  * @param  argument: Not used
  * @retval None
  */


/* USER CODE END Header_StartNTCTask */
void StartNTCTask(void *argument)
{
  /* USER CODE BEGIN StartNTCTask */
  Sensor sensordata;
  uint32_t raw_temp=0;
  uint32_t temp_voltage=0;
  uint32_t Temperature=0;
  /* Infinite loop */
  for(;;)
  {
    if (HAL_ADC_Start(&hadc1)!=HAL_OK)
    {Error_Handler();}
    if (HAL_ADC_PollForConversion(&hadc1,500)==HAL_OK)
    {
      raw_temp=HAL_ADC_GetValue(&hadc1); //获取ADC值
      HAL_ADC_Stop(&hadc1);
      temp_voltage=raw_temp*3.3/4095.0*1000; //计算电压值，单位mv
      Temperature=(uint32_t) (Calculate_NTC_Temperature(raw_temp*3.3/4095.0f)*100); //计算温度值，单位摄氏度


      // 使用的是 STM32 的标准库（非 CMSIS 或 FPU 支持），而 sprintf() 不支持浮点数！
      // 在没有启用 FPU（浮点单元）的 STM32 上，标准 printf / sprintf 默认不支持 %f 格式符！


      sprintf(message, "Temp_voltage:%lu mV\r\n", temp_voltage);
      HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 1000);

      sprintf(message, "Temperature:%ld.%02ld C\r\n",
              Temperature / 100, abs(Temperature % 100));

      HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 1000);

      //LCD_ShowString(10,10,100, 30, 16, (u8*)message);
    }
    else
    {Error_Handler();}

    osDelay(500);
  }
  /* USER CODE END StartNTCTask */
}

/* USER CODE BEGIN Header_StartLightSenserTask */
/**
* @brief Function implementing the LightSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLightSenserTask */
void StartLightSenserTask(void *argument)
{
  /* USER CODE BEGIN StartLightSenserTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLightSenserTask */
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

    //HAL_UART_Transmit(&huart1,(uint8_t*)msg,sizeof(msg),1000);
    osDelay(1);
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
    //LCD_ShowString(10, 10, 200, 30, 16, (u8*)"HELLO");
    osDelay(1);
  }
  /* USER CODE END StartTFTLCDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

