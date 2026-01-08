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

uint8_t temp_ready=0;
uint8_t light_ready=0;
uint8_t solim_ready=0;
uint8_t key0_flag=0;
uint8_t key1_flag=0;
Sensor sensordata;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TFTLCDTask */
osThreadId_t TFTLCDTaskHandle;
const osThreadAttr_t TFTLCDTask_attributes = {
  .name = "TFTLCDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SendDataTask */
osThreadId_t SendDataTaskHandle;
const osThreadAttr_t SendDataTask_attributes = {
  .name = "SendDataTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void KEY_SCAN();
/* USER CODE END FunctionPrototypes */

void StartSensorTask(void *argument);
void StartTFTLCDTask(void *argument);
void StartSendDataTask(void *argument);
void StartMotorTask(void *argument);

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
  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of TFTLCDTask */
  TFTLCDTaskHandle = osThreadNew(StartTFTLCDTask, NULL, &TFTLCDTask_attributes);

  /* creation of SendDataTask */
  SendDataTaskHandle = osThreadNew(StartSendDataTask, NULL, &SendDataTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief  Function implementing the SensorTask thread.
  * @param  argument: Not used
  * @retval None
  */


/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */

  //传感器结构体

  /* Infinite loop */

  if (HAL_ADCEx_Calibration_Start(&hadc1)!=HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2)!=HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc3)!=HAL_OK)
  {
    Error_Handler();
  }

  for(;;)
  {
    //读取温度传感器数据
    if (HAL_ADC_Start(&hadc1)!=HAL_OK)
    {Error_Handler();}
    if (HAL_ADC_Start(&hadc2)!=HAL_OK)
    {Error_Handler();}
    if (HAL_ADC_Start(&hadc3)!=HAL_OK)
    {Error_Handler();}

    if (HAL_ADC_PollForConversion(&hadc1,500)==HAL_OK)
    {
      sensordata.raw_temp=HAL_ADC_GetValue(&hadc1); //获取ADC值
      sensordata.temp_voltage=sensordata.raw_temp*3.3/4095.0*1000; //计算电压值，单位mv
      sensordata.Temperature=(uint32_t) (Calculate_NTC_Temperature(sensordata.raw_temp*3.3/4095.0f)*100); //计算温度值，单位摄氏度
      temp_ready=1;
    }
    else
    {Error_Handler();}

    //读取土壤湿度传感器数据
    if (HAL_ADC_PollForConversion(&hadc2,500)==HAL_OK)
    {
      sensordata.raw_soilm=HAL_ADC_GetValue(&hadc2); //获取ADC值
      sensordata.soilm_voltage=sensordata.raw_soilm*3.3/4095.0*1000; //计算电压值，单位mv
      sensordata.SoilMoisture = (uint32_t)(sensordata.raw_soilm * 100); //计算温度值，单位摄氏度
      solim_ready=1;
    }
    else
    {Error_Handler();}

    //读取光照传感器数据

    if (HAL_ADC_PollForConversion(&hadc3,500)==HAL_OK)
    {
      sensordata.raw_light=HAL_ADC_GetValue(&hadc3); //获取ADC值
      sensordata.light_voltage=sensordata.raw_light*3.3/4095.0*1000; //计算电压值，单位mv
      sensordata.Illumination = (uint32_t)(Calculate_Light_Intensity(sensordata.raw_light) * 100); //计算温度值，单位摄氏度
      light_ready=1;
    }
    else
    {Error_Handler();}

    osDelay(300);
  }


  /* USER CODE END StartSensorTask */
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
  static char message[50]={};
  /* Infinite loop */
  for(;;)
  {
    // 使用的是 STM32 的标准库（非 CMSIS 或 FPU 支持），而 sprintf() 不支持浮点数！
    // 在没有启用 FPU（浮点单元）的 STM32 上，标准 printf / sprintf 默认不支持 %f 格式符！

    sprintf(message, "Temperature:%ld.%02ld C\r\n",
    sensordata.Temperature / 100, abs(sensordata.Temperature % 100));

    LCD_ShowString(10,10,300, 30, 16, (u8*)message);

    sprintf(message, "Soil_Moisture:%ld %%\r\n",
        sensordata.SoilMoisture / 100);
    LCD_ShowString(10,30,300, 30, 16, (u8*)message);


    sprintf(message, "Illumination:%ld.%02ld Lux\r\n",
            sensordata.Illumination / 100, abs(sensordata.Illumination % 100));

    LCD_ShowString(10,50,300, 30, 16, (u8*)message);

    osDelay(500);
  }
  /* USER CODE END StartTFTLCDTask */
}

/* USER CODE BEGIN Header_StartSendDataTask */
/**
* @brief Function implementing the SendDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendDataTask */
void StartSendDataTask(void *argument)
{
  /* USER CODE BEGIN StartSendDataTask */
  static char message[50]={};
  /* Infinite loop */
  for(;;)
  {
    if (temp_ready) {
      //传输原始温度数据
      sprintf(message, "Raw_temp:%lu mV\r\n", sensordata.raw_temp);
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
      //传输温度传感器电压数据
      sprintf(message, "Temp_voltage:%lu mV\r\n", sensordata.temp_voltage);
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
      //传输计算后的温度数据
      sprintf(message, "Temperature:%ld.%02ld C\r\n",
    sensordata.Temperature / 100, abs(sensordata.Temperature % 100));
      HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);

      temp_ready = 0;
    }

    if (solim_ready) {
      //传输原始土壤湿度数据
      sprintf(message, "Raw_soilm:%lu mV\r\n", sensordata.raw_soilm);
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
      //传输土壤湿度传感器电压数据
      sprintf(message, "Soilm_voltage:%lu mV\r\n", sensordata.soilm_voltage);
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
      //传输计算后的土壤湿度数据
      sprintf(message, "Soil_Moisture:%ld %%\r\n",
          sensordata.SoilMoisture / 100);
      HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
      solim_ready = 0;
    }

    if (light_ready) {
      //传输原始光照数据
      sprintf(message, "Raw_light:%lu mV\r\n", sensordata.raw_light);
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
      //传输光照传感器电压数据
      sprintf(message, "Light_voltage:%lu mV\r\n", sensordata.light_voltage);
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
      //传输计算后的光照数据
      sprintf(message, "Illumination:%ld.%02ld Lux\r\n",
          sensordata.Illumination / 100, abs(sensordata.Illumination % 100));
      HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
      light_ready = 0;
    }

    osDelay(500);

  //   float vofa_data[2] = {
  //     sensordata.Temperature / 100.0f,
  //     sensordata.Illumination / 100.0f
  // };
  //
  //   HAL_UART_Transmit(&huart2, (uint8_t*)vofa_data, sizeof(vofa_data), 100);

  }
  /* USER CODE END StartSendDataTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
    KEY_SCAN();

      uint8_t moto_flag = HAL_GPIO_ReadPin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin);
      HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, !moto_flag);//电机状态翻转

    osDelay(3000);
  }
  /* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void KEY_SCAN()
{
  if(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
  {
    HAL_Delay(10);  // 消抖
    while(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET);
    key0_flag = 1;
  }
  if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
  {
    HAL_Delay(10);  // 消抖
    while(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET);
    key1_flag = 1;
  }

}
/* USER CODE END Application */

